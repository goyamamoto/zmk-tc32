/*
 * The Cidoo V75 Pro's backlight (89 RGB LEDs under the keys and three left of the
 * knob), also the V65 V3's (66 under the keys): two LED driver chips, each with 192
 * PWM channels, written over the chip's SPI master (CONFIG_LED_DRIVER_SPI).
 *
 * - The chips keep the PWM values, so nothing is kept per LED here: a frame's
 *   bytes are computed from the board's LED wiring (led_driver_table.h, led_driver_table.h:
 *   what each channel drives, and which channels are the notice LEDs) as they go
 *   out. The effects are those a channel's value follows from the setting, its
 *   colour and its key's column alone: one colour, the colours of the wheel
 *   across the columns, and one colour breathing. Breathing changes no PWM
 *   value: it steps the chips' current registers (page 4).
 * - A chip takes a command octet (0x20 | page), a register and data for
 *   consecutive registers while its select pin is high (chip 1: PA1, chip 2:
 *   PD5). Page 0 holds one enable bit per channel, page 1 the PWM octets, page
 *   3 the function registers (0x00: 1 runs the chip, 0 stops it), page 4 the
 *   currents. PD3 high lets the chips run; the board's deep sleep pads put it
 *   low.
 * - The SPI lines (PA4, PA3, PA2) and both selects are matrix columns as well.
 *   Every transfer runs on the system work queue, where the key scan runs, so
 *   the two never overlap: the pins become the SPI's for a transfer and get
 *   their GPIO state back after it, bit by bit, with interrupts locked for
 *   each register change only.
 *   The work queue is cooperative, so no other thread runs in a transfer.
 * - While lit, the chips' setup registers and the frame are sent again every
 *   second: the matrix scan moves the chips' lines between transfers.
 * - The setting (on, effect, colour, level) starts from the defaults, or with
 *   BACKLIGHT_SAVE from the settings' record "bl/v75" (two octets: bit 0
 *   on, bits 1-2 the effect, bits 3-5 the level; the colour). The record is
 *   written by the transfers' work once a change has stood for
 *   BACKLIGHT_SAVE_DELAY_MS, and before a reboot of the mode switch;
 *   a change younger than that at a deep sleep is lost. The LEDs go dark when
 *   ZMK's activity state leaves "active", while
 *   the USB bus is suspended and before a deep sleep, and come back with the
 *   next key. An SPI octet that does not finish turns the backlight off
 *   until the next boot.
 * - With the backlight toggled off nothing lights, the notice, the status
 *   display and the host's LEDs included: off is for where the keyboard must
 *   stay dark.
 * - A notice (with ZMK_USJIS, the US-JIS mode's
 *   change: green on, red off; with TLSR_USB_ON_REQUEST, blue when &usb_on
 *   brought USB up) blinks the board's notice LEDs (the V75 Pro: the three
 *   left of the knob; the V65 V3: the Tab key's) in its
 *   colour, half a second on and half off for three seconds, also while the
 *   backlight is dark (level 0, or the LEDs hidden).
 * - The status display (v75pro_status.c, v65v3_status.c; TLSR_STATUS_DISPLAY) overlays a
 *   frame while the keys are lit: a channel it names gets its value instead
 *   of the effect's, at full current (no breathing while it shows), so it
 *   shows at its own brightness also at level 0.
 * - The host's keyboard LEDs (ZMK_HID_INDICATORS: the LED output report of
 *   the endpoint in use): the LED the board's table names for an LED bit
 *   (BL_HOST_LED_BIT, _CHIP, _CHANNELS; the Caps Lock key's for bit 1, Caps
 *   Lock) is full white while the host has that bit set and the keys are
 *   lit, over the effect and at every level, 0 included: its three channels
 *   are written again after the frame. The table keeps it off the status
 *   display's and the notice's LEDs. With the breathing effect it breathes
 *   with every LED: the current registers are the chips' and not a channel's.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include "led_key_matrix_effects.h"
/* The board's LED wiring table and its part of the status display (its directory is on the include path). */
#include "led_driver_table.h"
#include "led_driver_status.h"

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS) && defined(BL_HOST_LED_BIT)
#include <zmk/hid_indicators.h>
#include <zmk/events/hid_indicators_changed.h>
#define HOST_LEDS 1
#define HOST_LED_VALUE 0xffU
#else
#define HOST_LEDS 0
#endif

#define REG(off) (0x00800000U + (off))

/* The SPI master: data, control (bit 6 reads 1 while an octet is going out), clock, mode */
#define SPI_DATA 0x08U
#define SPI_CTRL 0x09U
#define SPI_CLK  0x0aU
#define SPI_MODE 0x0bU
#define SPI_BUSY BIT(6)

/* GPIO ports A and D: input enable, output disable, output level, GPIO function */
#define PA_IE   0x581U
#define PA_OEN  0x582U
#define PA_OUT  0x583U
#define PA_GPIO 0x586U
#define PD_OEN  0x59aU
#define PD_OUT  0x59bU
#define PD_GPIO 0x59eU
/* Port A's pin functions (two bits a pin: PA0-PA3, PA4-PA7) and the SPI/I2C pin enables */
#define PA_MUX_L   0x5a8U
#define PA_MUX_H   0x5a9U
#define SPI_PIN_EN 0x5b6U
#define SPI_PIN_SEL 0x5b7U

#define PA_CS1  BIT(1)
#define PA_DO   BIT(2)
#define PA_DI   BIT(3)
#define PA_CK   BIT(4)
#define PA_SPI  (PA_DO | PA_DI | PA_CK)
#define PA_ALL  (PA_CS1 | PA_SPI)
#define PD_SDB  BIT(3)
#define PD_CS2  BIT(5)

#define CMD_WRITE 0x20U
#define PAGE_ENABLE 0U
#define PAGE_PWM    1U
#define PAGE_FUNC   3U
#define PAGE_CURRENT 4U
#define FUNC_RUN    0x00U
#define FUNC_OFF    0x1aU
#define ENABLE_OCTETS  24U
#define CURRENT_OCTETS 12U
#define CURRENT_FULL   0x70U

#define EFFECTS 3U
#define EFFECT_COLOUR  0U
#define EFFECT_COLUMNS 1U
#define EFFECT_BREATHE 2U
#define COLOURS 8U
#define LEVELS  5U
#define WHEEL   96U /* the colour wheel's steps: six segments of 16 */
#define BREATHE_STEPS 32U
#define BREATHE_MS    60
#define REFRESH_MS    1000
#define SPI_TRIES     2000U /* polls of the busy bit for one octet (16 clocks at 4 MHz) */
#define NOTICE_MS     500
#define NOTICE_STEPS  6U    /* half seconds from its start: on, off, three times */
#define NOTICE_VALUE  0x80U

BUILD_ASSERT(CONFIG_SYSTEM_WORKQUEUE_PRIORITY < 0, "the transfers must not be preempted");

static struct {
	uint8_t on : 1;      /* the setting */
	uint8_t lit : 1;     /* the chips are set up and show a frame */
	uint8_t awake : 1;   /* ZMK's activity state is active */
	uint8_t dirty : 1;   /* the setting changed since the last frame */
	uint8_t hidden : 1;  /* the USB bus is suspended */
	uint8_t fault : 1;   /* an SPI octet did not finish */
	uint8_t effect : 2;
	uint8_t colour;
	uint8_t level;       /* 0 (dark) .. LEVELS */
	uint8_t phase;       /* breathing: the step */
	uint8_t since;       /* breathing: steps since the setup was last sent */
#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
	uint8_t unsaved;     /* the setting changed and is not kept yet */
	uint16_t changed_at; /* when it changed: the low 16 bits of k_uptime_get_32() */
#endif
} bl = {.on = 1, .awake = 1, .effect = CONFIG_LED_DRIVER_SPI_EFFECT,
	.colour = CONFIG_LED_DRIVER_SPI_COLOUR, .level = CONFIG_LED_DRIVER_SPI_LEVEL};

/* red, green, blue of the preset colours */
static const uint8_t colours[COLOURS][3] = {
	{255, 255, 255}, {255, 0, 0}, {255, 96, 0}, {255, 224, 0},
	{0, 255, 0},     {0, 224, 255}, {0, 0, 255}, {224, 0, 255},
};

/* The notice: its colour (bit 0 red, 1 green, 2 blue), its start (k_uptime_get_32()), the half
 * second of it the last frame showed, and whether it runs */
static struct {
	uint32_t start;
	uint8_t rgb;
	uint8_t step;
	uint8_t on : 1;
} notice;

/* The half second of the notice now (NOTICE_STEPS once it is over) and the ms to the next one */
static uint8_t notice_step(int32_t *to_next)
{
	uint32_t elapsed = k_uptime_get_32() - notice.start;

	*to_next = (int32_t)(NOTICE_MS - elapsed % NOTICE_MS);
	return (uint8_t)MIN(elapsed / NOTICE_MS, NOTICE_STEPS);
}

static void work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(bl_work, work_handler);
static int changed(void);

/* ---------------------------------------------------------------- pins */

static void reg_set(uint32_t off, uint8_t bits, uint8_t to)
{
	unsigned int key = irq_lock();

	sys_write8((sys_read8(REG(off)) & (uint8_t)~bits) | (to & bits), REG(off));
	irq_unlock(key);
}

static struct {
	uint8_t pa_ie, pa_oen, pa_out, pa_gpio, pd_oen, pd_out, pd_gpio;
} kept;

/* The matrix's pins become the SPI's and the selects, both low. */
static void pins_take(void)
{
	kept.pa_ie = sys_read8(REG(PA_IE));
	kept.pa_oen = sys_read8(REG(PA_OEN));
	kept.pa_out = sys_read8(REG(PA_OUT));
	kept.pa_gpio = sys_read8(REG(PA_GPIO));
	kept.pd_oen = sys_read8(REG(PD_OEN));
	kept.pd_out = sys_read8(REG(PD_OUT));
	kept.pd_gpio = sys_read8(REG(PD_GPIO));

	reg_set(PA_OUT, PA_CS1, 0);
	reg_set(PD_OUT, PD_CS2, 0);
	reg_set(PA_GPIO, PA_CS1, PA_CS1);
	reg_set(PD_GPIO, PD_CS2, PD_CS2);
	reg_set(PA_OEN, PA_CS1, 0);
	reg_set(PD_OEN, PD_CS2, 0);

	reg_set(SPI_PIN_EN, 0x30U, 0x30U);
	reg_set(SPI_PIN_SEL, 0x22U, 0x02U);
	reg_set(SPI_PIN_SEL, 0x11U, 0x01U);
	reg_set(SPI_PIN_EN, 0x80U, 0x80U);
	reg_set(PA_MUX_H, 0x03U, 0);          /* PA4 */
	reg_set(PA_GPIO, PA_CK, 0);
	reg_set(PA_MUX_L, 0x30U, 0);          /* PA2 */
	reg_set(PA_GPIO, PA_DO, 0);
	reg_set(PA_MUX_L, 0xc0U, 0);          /* PA3 */
	reg_set(PA_GPIO, PA_DI, 0);
	reg_set(PA_IE, PA_DI, PA_DI);
	reg_set(PA_OEN, PA_DI, PA_DI);

	/* the SPI master, set again for every use (a chip suspend may come between
	 * two): clock divider 5, enabled; master; mode 0 */
	sys_write8(0x00, REG(SPI_CLK));
	sys_write8(0x05, REG(SPI_CLK));
	sys_write8(0x85, REG(SPI_CLK));
	sys_write8(sys_read8(REG(SPI_CTRL)) | 0x02U, REG(SPI_CTRL));
	sys_write8(0x00, REG(SPI_MODE));
}

/* The pins as the matrix had them. */
static void pins_give(void)
{
	reg_set(SPI_PIN_EN, 0x30U, 0);
	reg_set(SPI_PIN_SEL, 0x22U, 0);
	reg_set(SPI_PIN_SEL, 0x11U, 0);
	reg_set(SPI_PIN_EN, 0x80U, 0);
	/* the SPI lines first, the selects (still low) last */
	reg_set(PA_OUT, PA_SPI, kept.pa_out);
	reg_set(PA_GPIO, PA_SPI, kept.pa_gpio);
	reg_set(PA_IE, PA_SPI, kept.pa_ie);
	reg_set(PA_OEN, PA_SPI, kept.pa_oen);
	reg_set(PA_OUT, PA_CS1, kept.pa_out);
	reg_set(PA_GPIO, PA_CS1, kept.pa_gpio);
	reg_set(PA_IE, PA_CS1, kept.pa_ie);
	reg_set(PA_OEN, PA_CS1, kept.pa_oen);
	reg_set(PD_OUT, PD_CS2, kept.pd_out);
	reg_set(PD_GPIO, PD_CS2, kept.pd_gpio);
	reg_set(PD_OEN, PD_CS2, kept.pd_oen);
}

static void sdb(bool high)
{
	reg_set(PD_OUT, PD_SDB, high ? PD_SDB : 0);
	reg_set(PD_GPIO, PD_SDB, PD_SDB);
	reg_set(PD_OEN, PD_SDB, 0);
}

/* ---------------------------------------------------------------- SPI */

static void spi_octet(uint8_t v)
{
	if (bl.fault) {
		return;
	}
	sys_write8(v, REG(SPI_DATA));
	for (uint32_t i = 0; i < SPI_TRIES; i++) {
		if ((sys_read8(REG(SPI_CTRL)) & SPI_BUSY) == 0U) {
			return;
		}
	}
	bl.fault = 1;
}

/* chips: bit 0 chip 1, bit 1 chip 2 (both: the same octets to both) */
static void chip_begin(uint8_t chips, uint8_t page, uint8_t reg)
{
	if ((chips & 1U) != 0U) {
		reg_set(PA_OUT, PA_CS1, PA_CS1);
	}
	if ((chips & 2U) != 0U) {
		reg_set(PD_OUT, PD_CS2, PD_CS2);
	}
	sys_write8(sys_read8(REG(SPI_CTRL)) & 0xfbU, REG(SPI_CTRL));
	sys_write8(sys_read8(REG(SPI_CTRL)) & 0xf7U, REG(SPI_CTRL));
	spi_octet(CMD_WRITE | page);
	spi_octet(reg);
}

static void chip_end(void)
{
	reg_set(PA_OUT, PA_CS1, 0);
	reg_set(PD_OUT, PD_CS2, 0);
}

static void chip_fill(uint8_t chips, uint8_t page, uint8_t reg, uint8_t v, uint8_t n)
{
	chip_begin(chips, page, reg);
	while (n-- != 0U) {
		spi_octet(v);
	}
	chip_end();
}

/* ---------------------------------------------------------------- colours */

/* Component c (0 red, 1 green, 2 blue) of wheel position h (0 .. WHEEL - 1). */
static uint8_t wheel(uint8_t h, uint8_t c)
{
	/* each component: up over one segment, full for two, down over one, dark for two */
	uint8_t p = (uint8_t)((h + WHEEL + 32U - 32U * c) % WHEEL);

	if (p < 16U) {
		return (uint8_t)(p * 17U);
	}
	if (p < 48U) {
		return 255;
	}
	if (p < 64U) {
		return (uint8_t)((63U - p) * 17U);
	}
	return 0;
}

static uint8_t channel_value(uint8_t what)
{
	uint8_t c = what & 3U;
	uint8_t v;

	if (c == 0U) {
		return 0;
	}
	if (bl.effect == EFFECT_COLUMNS) {
		v = wheel((uint8_t)((what >> 2) * (WHEEL / 15U)), (uint8_t)(c - 1U));
	} else {
		v = colours[bl.colour][c - 1U];
	}
	return (uint8_t)((v * (uint16_t)bl.level * (256U / LEVELS)) >> 8);
}

/* ---------------------------------------------------------------- frames */

/* keys: the setting shows; stat: the status display may show; the notice's LEDs are lit in its even
 * half seconds. A channel the status display names takes its value, before the notice and the setting;
 * the host's LED goes over the frame. */
static void send_frame(bool keys, bool stat)
{
	bool note = bl.on && notice.on;
	bool shown = note && (notice.step & 1U) == 0U;

	for (uint8_t chip = 0; chip < 2U; chip++) {
		chip_begin(BIT(chip), PAGE_PWM, 0);
		for (uint8_t i = 0; i < BL_LED_CHANNELS; i++) {
			uint8_t what = bl_led_channel[chip][i];
			int v = stat ? board_status_channel((uint8_t)(chip + 1U), i) : -1;

			if (v < 0 && note && BL_NOTICE_LED(chip, i)) {
				v = shown && (notice.rgb & BIT((what & 3U) - 1U)) != 0U ? NOTICE_VALUE : 0;
			} else if (v < 0) {
				v = keys ? channel_value(what) : 0;
			}
			spi_octet((uint8_t)v);
		}
		chip_end();
	}
#if HOST_LEDS
	if (keys && (zmk_hid_indicators_get_current_profile() & BL_HOST_LED_BIT) != 0U) {
		static const uint8_t host_led[] = BL_HOST_LED_CHANNELS;

		for (uint8_t k = 0; k < sizeof(host_led); k++) {
			chip_fill(BIT(BL_HOST_LED_CHIP), PAGE_PWM, host_led[k], HOST_LED_VALUE, 1);
		}
	}
#endif
}

static uint8_t breathe_current(void)
{
	/* a triangle over BREATHE_STEPS, from a sixteenth to the full current */
	uint8_t s = bl.phase < BREATHE_STEPS / 2U ? bl.phase : (uint8_t)(BREATHE_STEPS - bl.phase);

	return (uint8_t)(CURRENT_FULL / 16U + (CURRENT_FULL - CURRENT_FULL / 16U) * s / (BREATHE_STEPS / 2U));
}

/* Each chip's setup. fresh: from stopped (the chip stopped and its channels cleared first);
 * else the registers a running chip keeps, sent again. */
static void chips_setup(bool fresh)
{
	static const uint8_t setup[5] = {0xaa, 0x00, 0x04, 0xc0, 0x00}; /* function registers 0x13-0x17 */

	sdb(true);
	if (fresh) {
		k_busy_wait(1000);
	}
	for (uint8_t chip = 1; chip <= 2U; chip++) {
		if (fresh) {
			chip_fill(chip, PAGE_FUNC, FUNC_RUN, 0, 1);
		}
		for (uint8_t i = 0; i < sizeof(setup); i++) {
			chip_fill(chip, PAGE_FUNC, (uint8_t)(0x13U + i), setup[i], 1);
		}
		if (fresh) {
			chip_fill(chip, PAGE_ENABLE, 0, 0, ENABLE_OCTETS);
			chip_fill(chip, PAGE_PWM, 0, 0, BL_LED_CHANNELS);
			chip_fill(chip, PAGE_CURRENT, 0, CURRENT_FULL, CURRENT_OCTETS);
		}
		chip_fill(chip, PAGE_ENABLE, 0, 0xff, ENABLE_OCTETS);
		chip_fill(chip, PAGE_FUNC, FUNC_OFF, 0x01, 1); /* chips_stop() left 0x02 there */
		chip_fill(chip, PAGE_FUNC, FUNC_RUN, 1, 1);
	}
}

static void chips_stop(void)
{
	chip_fill(3, PAGE_PWM, 0, 0, BL_LED_CHANNELS);
	chip_fill(3, PAGE_FUNC, FUNC_RUN, 0, 1);
	chip_fill(3, PAGE_FUNC, FUNC_OFF, 0x02, 1);
	sdb(false);
}

/* The chips follow the setting. The ms to the next run; negative: the chips need none. */
static int32_t chips_follow(void)
{
	bool keys = bl.on && bl.awake && !bl.hidden;
	bool stat = keys && board_status_active();
	bool want = (keys || (bl.on && notice.on)) && !bl.fault;
	bool breathe = keys && !stat && bl.effect == EFFECT_BREATHE;
	int32_t next = breathe ? BREATHE_MS : REFRESH_MS;

	if (!want && !bl.lit) {
		return -1;
	}
	pins_take();
	if (!want) {
		chips_stop();
		bl.lit = 0;
	} else {
		if (!bl.lit) {
			chips_setup(true);
			bl.lit = 1;
			bl.dirty = 1;
			bl.since = 0;
			bl.phase = BREATHE_STEPS / 2U;
		} else if (!breathe || ++bl.since >= REFRESH_MS / BREATHE_MS) {
			/* not breathing, every run is the second's refresh or a changed setting */
			chips_setup(false);
			bl.dirty = 1;
			bl.since = 0;
		}
		if (notice.on) {
			int32_t to_next;
			uint8_t step = notice_step(&to_next);

			if (step != notice.step) {
				notice.step = step;
				notice.on = step < NOTICE_STEPS;
				bl.dirty = 1;
			}
			if (notice.on) {
				next = MIN(next, to_next);
			}
		}
		if (bl.dirty) {
			bl.dirty = 0;
			send_frame(keys, stat);
		}
		if (breathe) {
			bl.phase = (uint8_t)((bl.phase + 1U) % BREATHE_STEPS);
		}
		for (uint8_t chip = 1; chip <= 2U; chip++) {
			chip_fill(chip, PAGE_CURRENT, 0, breathe ? breathe_current() : CURRENT_FULL,
				  CURRENT_OCTETS);
		}
	}
	if (bl.fault) {
		sdb(false);
		bl.lit = 0;
	}
	pins_give();
	return want && !bl.fault ? next : -1;
}

#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
BUILD_ASSERT(CONFIG_BACKLIGHT_SAVE_DELAY_MS <= 30000, "the change's time is kept in 16 bits");

static void save_now(void)
{
	const uint8_t v[2] = {(uint8_t)(bl.on | bl.effect << 1 | bl.level << 3), bl.colour};

	bl.unsaved = 0;
	(void)settings_save_one("bl/v75", v, sizeof(v));
}

/*
 * A change not kept yet is written now when the caller is on the system work queue (the mode switch's watch,
 * before its reboot). The link's thread on its way to a deep sleep calls this too: the settings' call chain is
 * not put on its stack, so a change made in the last BACKLIGHT_SAVE_DELAY_MS before a deep sleep is lost.
 */
void cidoo_backlight_save_pending(void)
{
	if (bl.unsaved && k_current_get() == &k_sys_work_q.thread) {
		save_now();
	}
}

static int saved_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	uint8_t v[2];

	if (settings_name_steq(name, "v75", &next) && next == NULL && len == sizeof(v) &&
	    read_cb(cb_arg, v, sizeof(v)) == (ssize_t)sizeof(v) && (v[0] >> 1 & 3U) < EFFECTS &&
	    v[0] >> 3 <= LEVELS && v[1] < COLOURS) {
		/* the work changes the other bits of the setting's octet */
		k_sched_lock();
		bl.on = v[0] & 1U;
		bl.effect = v[0] >> 1 & 3U;
		bl.level = v[0] >> 3;
		bl.colour = v[1];
		bl.dirty = 1;
		k_sched_unlock();
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(v75pro_backlight, "bl", NULL, saved_set, NULL, NULL);
#endif

/* On the system work queue: the chips, then the setting's record once the change has stood. */
static void work_handler(struct k_work *work)
{
	int32_t next = chips_follow();

	ARG_UNUSED(work);
#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
	if (bl.unsaved) {
		int32_t left = CONFIG_BACKLIGHT_SAVE_DELAY_MS -
			       (int32_t)(uint16_t)(k_uptime_get_32() - bl.changed_at);

		if (left <= 0) {
			save_now();
		} else if (next < 0 || left < next) {
			next = left;
		}
	}
#endif
	if (next >= 0) {
		(void)k_work_schedule(&bl_work, K_MSEC(next));
	}
}

static void v75pro_backlight_notice(uint8_t rgb)
{
	notice.rgb = rgb;
	notice.start = k_uptime_get_32();
	notice.step = 0;
	notice.on = 1;
	bl.dirty = 1;
	(void)k_work_reschedule(&bl_work, K_NO_WAIT);
}

#if IS_ENABLED(CONFIG_ZMK_USJIS)
#include <zmk/usjis.h>

void zmk_usjis_mode_applied(bool enabled)
{
	v75pro_backlight_notice(enabled ? BIT(1) : BIT(0));
}
#endif

#if IS_ENABLED(CONFIG_TLSR_USB_ON_REQUEST)
/* &usb_on brought USB up (src/behavior_usb_request.c): blue */
void tlsr_usb_requested(void)
{
	v75pro_backlight_notice(BIT(2));
}
#endif

void v75pro_backlight_refresh(void)
{
	(void)changed();
}

void v75pro_backlight_hide(bool hidden)
{
	if (bl.hidden != hidden) {
		bl.hidden = hidden;
		(void)k_work_reschedule(&bl_work, K_NO_WAIT);
	}
}

/* Before a deep sleep, from the thread that enters it: the chips stopped as when turned off. */
void v75pro_backlight_sleep(void)
{
	(void)k_work_cancel_delayable(&bl_work);
	if (bl.lit) {
		pins_take();
		chips_stop();
		pins_give();
		bl.lit = 0;
	}
}

static int changed(void)
{
	bl.dirty = 1;
	(void)k_work_reschedule(&bl_work, K_NO_WAIT);
	return 0;
}

/* ---------------------------------------------------------------- the setting */

/* The setting changed: shown now, kept once it has stood. */
static int setting_changed(void)
{
#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
	bl.unsaved = 1;
	bl.changed_at = (uint16_t)k_uptime_get_32();
#endif
	return changed();
}

int cidoo_backlight_toggle(void)
{
	bl.on = !bl.on;
	return setting_changed();
}

int cidoo_backlight_set_on(bool on)
{
	bl.on = on;
	return setting_changed();
}

int cidoo_backlight_cycle_effect(int direction)
{
	bl.effect = (uint8_t)((bl.effect + EFFECTS + (direction < 0 ? EFFECTS - 1U : 1U)) % EFFECTS);
	return setting_changed();
}

int cidoo_backlight_select_effect(int effect)
{
	if (effect < 0 || effect >= (int)EFFECTS) {
		return -EINVAL;
	}
	bl.effect = (uint8_t)effect;
	return setting_changed();
}

int cidoo_backlight_step_colour(int direction)
{
	bl.colour = (uint8_t)((bl.colour + (direction < 0 ? COLOURS - 1U : 1U)) % COLOURS);
	return setting_changed();
}

int cidoo_backlight_step_white(int direction)
{
	ARG_UNUSED(direction);
	return 0;
}

int cidoo_backlight_step_level(int direction)
{
	if (direction > 0 && bl.level < LEVELS) {
		bl.level++;
	} else if (direction < 0 && bl.level > 0U) {
		bl.level--;
	}
	return setting_changed();
}

int cidoo_backlight_step_speed(int direction)
{
	ARG_UNUSED(direction);
	return 0;
}

/* ---------------------------------------------------------------- start, activity */

static int on_event(const zmk_event_t *eh)
{
	const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

	if (ev != NULL) {
		bl.awake = ev->state == ZMK_ACTIVITY_ACTIVE;
		(void)changed();
	}
#if HOST_LEDS
	if (as_zmk_hid_indicators_changed(eh) != NULL) {
		(void)changed(); /* the next frame reads the endpoint's LEDs */
	}
#endif
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(v75pro_backlight, on_event);
ZMK_SUBSCRIPTION(v75pro_backlight, zmk_activity_state_changed);
#if HOST_LEDS
ZMK_SUBSCRIPTION(v75pro_backlight, zmk_hid_indicators_changed);
#endif

static int backlight_init(void)
{
	(void)k_work_schedule(&bl_work, K_MSEC(CONFIG_LED_DRIVER_SPI_START_MS));
	return 0;
}

SYS_INIT(backlight_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
