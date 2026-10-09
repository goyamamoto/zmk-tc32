/*
 * Backlight effects, wave, breathe and light, for a
 * "cidoo,led-key-matrix-backlight" LED strip, whose pixel n is LED index n:
 * matrix row * 12 + column. The V21 uses it.
 *
 * - Colours: a wheel of 192 steps, six segments of 32 in each of which one
 *   channel rises or falls along 255 * (x / 32)^1.8 (rising at x = 1 .. 32,
 *   falling at x = 31.5 .. 0.5), and 46 preset colours on it. A colour is
 *   scaled by the brightness level (0, 28, ..., 224, 255 of 256) and by an
 *   effect's curve, each as c = ((c | white) * f) >> 8, white being the
 *   white mixed in (0 to 240). tc32/scripts/backlight_curves.py
 *   prints the curves' tables.
 * - Time: an effect tick is (speed delay + 1) * 0.5 ms, the delay 0 to 4,
 *   and an effect steps every so many ticks: the wave 25 (37.5 ms at the
 *   default delay 2), breathe 20, light 40. The matrix driver's 500 us
 *   Timer1 interrupt counts the steps (led_key_matrix_tick()), so no kernel
 *   timer interrupt comes at a phase of its own between the slots.
 * - Wave: the grid's six rows in colours 7 wheel steps apart, moving 2 steps
 *   a step, downwards; with one colour, its brightness along the curve, the
 *   rows 3 of its 96 steps apart.
 * - Breathe: each row one of six colours 30 wheel steps apart (or the one
 *   colour), all along the curve's 128 steps, one a step.
 * - Light: the rings around a centre key lit one a step, then steady; each
 *   ring one of four colours 50 wheel steps apart (or the one colour).
 *
 * - Kept (CONFIG_BACKLIGHT_SAVE): on/off, the effect, the level, the speed, the colour
 *   (a preset or multicolour) and the white, BACKLIGHT_SAVE_DELAY_MS
 *   after the last change, and at once before a deep sleep. One sector, the
 *   storage partition's third (the first two hold the BLE bonds), as a log
 *   of 16-octet records, the newest valid one winning; a full sector is
 *   erased and the record written first (a power cut in between leaves the
 *   defaults at the next boot). A record whose CRC-16 or values do
 *   not check is skipped; a record counts as free only when all 16 octets
 *   are erased. Every write goes through the flash driver, which darkens the
 *   backlight first. The sector erase (every 256 changes) holds interrupts
 *   off for the erase time, and a key pressed and released within it is not
 *   seen; a key held longer is.
 *
 * - The host's Num Lock (ZMK_HID_INDICATORS, the endpoint in use): with
 *   num-lock-pixel, that pixel full white over the effect while the
 *   backlight is on, at every level, 0 included; nothing while it is off.
 *
 * Not here: the other effects of the keyboard's original firmware, its
 * power-on sweep, its other indicators (charging, the blink at a limit).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT zmk_backlight_effects

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include <zmk/workqueue.h>
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS) && DT_NODE_HAS_PROP(DT_DRV_INST(0), num_lock_pixel)
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#define HOST_LEDS 1
#else
#define HOST_LEDS 0
#endif

#include "led_key_matrix_effects.h"
#include "led_key_matrix.h"

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "one zmk,backlight-effects");
BUILD_ASSERT(IS_ENABLED(CONFIG_CIDOO_LED_KEY_MATRIX), "the matrix driver's interrupt steps the effects");

#define FX     DT_DRV_INST(0)
#define STRIP  DT_PHANDLE(FX, led_strip)
#define PIXELS DT_PROP(STRIP, chain_length)

#define GRID_ROWS 6U
#define GRID_COLS 4U
#define RINGS     4U
#define RING_LEN  (DT_PROP_LEN(FX, rings) / RINGS)

BUILD_ASSERT(DT_PROP_LEN(FX, grid) == GRID_ROWS * GRID_COLS, "grid: six rows of four");
BUILD_ASSERT(DT_PROP_LEN(FX, rings) % RINGS == 0, "rings: four of the same length");

#if HOST_LEDS
#define NUM_LOCK_PIXEL DT_PROP(FX, num_lock_pixel)
BUILD_ASSERT(NUM_LOCK_PIXEL < PIXELS, "num-lock-pixel: a pixel of the strip");
#endif

static const uint8_t grid[] = DT_PROP(FX, grid);
static const uint8_t rings[] = DT_PROP(FX, rings);

/* 255 * (x / 32)^1.8 rounded: rising at x = 1 .. 32, falling at x = 31.5 .. 0.5. */
static const uint8_t ramp_up[32] = {
	0,   2,   4,   6,   9,   13,  17,  21,  26,  31,  37,  44,  50,  58,  65,  73,
	82,  91,  100, 109, 119, 130, 141, 152, 164, 175, 188, 201, 214, 227, 241, 255,
};
static const uint8_t ramp_down[32] = {
	248, 234, 220, 207, 194, 182, 169, 158, 146, 135, 125, 114, 105, 95, 86, 77,
	69,  61,  54,  47,  40,  34,  29,  23,  19,  14,  11,  7,   5,   3,  1,  0,
};
/* max(k, 255 * (k / 63)^1.8 rounded), k = 0 .. 63: rises for breathe and the one-colour wave. */
static const uint8_t curve[64] = {
	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  13,  15,  17,  19,
	22,  24,  27,  29,  32,  35,  38,  42,  45,  48,  52,  55,  59,  63,  67,  71,
	75,  80,  84,  89,  93,  98,  103, 108, 113, 118, 123, 128, 134, 139, 145, 150,
	156, 162, 168, 174, 181, 187, 193, 200, 206, 213, 220, 227, 234, 241, 248, 255,
};
/* The preset colours as wheel steps. */
static const uint8_t presets[] = {
	0,   8,   12,  19,  20,  29,  31,  32,  34,  35,  39,  44,  50,  55,  63,  64,
	72,  76,  83,  87,  91,  95,  96,  98,  99,  103, 107, 112, 118, 127, 128, 136,
	140, 146, 151, 155, 159, 160, 161, 162, 163, 167, 172, 174, 178, 183,
};

#define WHEEL        192U
#define WAVE_ONE     96U  /* the one-colour wave's steps */
#define BREATHE_LEN  128U
#define LEVELS       10U
#define DELAYS       5U
#define WHITE_STEP   15U
#define WHITE_MAX    240U

enum effect {
	WAVE,
	BREATHE,
	LIGHT,
	EFFECTS
};

static const uint8_t step_ticks[EFFECTS] = {25U, 20U, 40U};

static const struct device *const strip = DEVICE_DT_GET(STRIP);

static struct {
	bool on;
	uint8_t effect;
	uint8_t level;
	uint8_t delay;
	bool multicolour;
	uint8_t preset;
	uint8_t white;
	struct led_rgb colour; /* the one colour: presets[preset] */
	uint8_t phase;         /* the effects share it */
	uint8_t rings_lit;     /* light: bit k for ring k, filled from bit 0 */
	bool settled;          /* light: every ring lit, nothing to step */
#if HOST_LEDS
	bool num_lock;         /* the host's Num Lock, for the endpoint in use */
#endif
	uint32_t period;       /* the step's in half milliseconds; 0: stopped */
	struct led_rgb pixels[PIXELS];
} fx;

static void fx_work_handler(struct k_work *work);

K_MUTEX_DEFINE(fx_lock);
K_WORK_DEFINE(fx_work, fx_work_handler);

#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
#define SAVE_SECTOR 4096U
#define SAVE_ADDR   (DT_REG_ADDR(DT_NODELABEL(storage_partition)) + 2U * SAVE_SECTOR)
#define SAVE_MAGIC  0xc1U

BUILD_ASSERT(DT_REG_SIZE(DT_NODELABEL(storage_partition)) >= 3U * SAVE_SECTOR,
	     "the storage partition's third sector keeps the backlight");

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define SAVE_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define SAVE_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

struct saved {
	uint8_t magic;
	uint8_t on;
	uint8_t effect;
	uint8_t level;
	uint8_t delay;
	uint8_t multicolour;
	uint8_t preset;
	uint8_t white;
	uint8_t reserved[6];
	uint16_t crc; /* CRC-16/CCITT of the 14 octets before it, little-endian */
};
BUILD_ASSERT(sizeof(struct saved) == 16U, "16-octet records");

static const struct device *const save_flash = DEVICE_DT_GET(SAVE_FLASH_NODE);
static uint32_t save_next = SAVE_SECTOR; /* the next free record; SAVE_SECTOR: erase first */
static struct saved last_saved;

static void save_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(save_work, save_work_handler);
#endif

static struct led_rgb rgb(uint8_t r, uint8_t g, uint8_t b)
{
	struct led_rgb c = {0};

	c.r = r;
	c.g = g;
	c.b = b;
	return c;
}

static struct led_rgb wheel(uint32_t step)
{
	const uint8_t up = ramp_up[step % 32U];
	const uint8_t down = ramp_down[step % 32U];

	switch (step / 32U) {
	case 0:
		return rgb(255U, up, 0U);
	case 1:
		return rgb(down, 255U, 0U);
	case 2:
		return rgb(0U, 255U, up);
	case 3:
		return rgb(0U, down, 255U);
	case 4:
		return rgb(up, 0U, 255U);
	default:
		return rgb(255U, 0U, down);
	}
}

static uint8_t scale(uint8_t c, uint8_t f)
{
	return (uint8_t)(((uint32_t)(c | fx.white) * f) >> 8);
}

static struct led_rgb scaled(struct led_rgb c, uint8_t f)
{
	return rgb(scale(c.r, f), scale(c.g, f), scale(c.b, f));
}

static uint8_t level_factor(void)
{
	return fx.level + 1U >= LEVELS ? 255U : (uint8_t)(28U * fx.level);
}

/* The phase step with the direction flag 0 (the default): backwards. */
static uint32_t advance(uint8_t period, uint8_t step)
{
	fx.phase -= step;
	if (fx.phase >= period) {
		fx.phase = period - 1U;
	}
	return fx.phase;
}

static void paint(const uint8_t *px, uint32_t len, struct led_rgb c)
{
	for (uint32_t i = 0; i < len; i++) {
		if (px[i] < PIXELS) {
			fx.pixels[px[i]] = c;
		}
	}
}

static void wave(void)
{
	if (fx.multicolour) {
		uint32_t s = advance(WHEEL, 2U);

		for (uint32_t r = 0; r < GRID_ROWS; r++) {
			paint(&grid[r * GRID_COLS], GRID_COLS, scaled(wheel(s), level_factor()));
			s = (s + 7U) % WHEEL;
		}
		return;
	}
	uint32_t s = advance(WAVE_ONE, 2U);

	for (uint32_t r = 0; r < GRID_ROWS; r++) {
		const uint8_t f = s < 48U ? curve[16U + s] : curve[111U - s];

		paint(&grid[r * GRID_COLS], GRID_COLS, scaled(scaled(fx.colour, level_factor()), f));
		s = (s + 3U) % WAVE_ONE;
	}
}

static void breathe(void)
{
	const uint32_t s = advance(BREATHE_LEN, 1U);
	const uint8_t f = s < 64U ? curve[s] : curve[127U - s];

	for (uint32_t r = 0; r < GRID_ROWS; r++) {
		const struct led_rgb c = fx.multicolour ? wheel(15U + 30U * r) : fx.colour;

		paint(&grid[r * GRID_COLS], GRID_COLS, scaled(scaled(c, f), level_factor()));
	}
}

static void light(void)
{
	for (uint32_t k = 0; k < RINGS; k++) {
		if ((fx.rings_lit & BIT(k)) != 0U) {
			const struct led_rgb c = fx.multicolour ? wheel(15U + 50U * k) : fx.colour;

			paint(&rings[k * RING_LEN], RING_LEN, scaled(c, level_factor()));
		}
	}
	fx.settled = (fx.rings_lit & BIT(RINGS - 1U)) != 0U;
	fx.rings_lit = (uint8_t)((fx.rings_lit << 1) | 1U);
}

/* The pixels to the strip, the Num Lock indicator over them while the backlight is on. */
static void show(void)
{
#if HOST_LEDS
	const struct led_rgb under = fx.pixels[NUM_LOCK_PIXEL];

	if (fx.on && fx.num_lock) {
		fx.pixels[NUM_LOCK_PIXEL] = rgb(255U, 255U, 255U);
	}
	(void)led_strip_update_rgb(strip, fx.pixels, PIXELS);
	fx.pixels[NUM_LOCK_PIXEL] = under;
#else
	(void)led_strip_update_rgb(strip, fx.pixels, PIXELS);
#endif
}

static void dark(void)
{
	memset(fx.pixels, 0, sizeof(fx.pixels));
	show();
}

/* The steps for the state: started when their period changes, stopped when off or settled. */
static void schedule(void)
{
	const uint32_t period = fx.on && !fx.settled ? (fx.delay + 1U) * step_ticks[fx.effect] : 0U;

	if (period == fx.period) {
		return;
	}
	fx.period = period;
	led_key_matrix_tick(zmk_workqueue_lowprio_work_q(), &fx_work, period);
}

static void fx_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_mutex_lock(&fx_lock, K_FOREVER);
	if (fx.on) {
		switch (fx.effect) {
		case WAVE:
			wave();
			break;
		case BREATHE:
			breathe();
			break;
		default:
			light();
			break;
		}
		show();
		schedule();
	}
	k_mutex_unlock(&fx_lock);
}

/* A change the next step shows; light, if settled, steps again to show it. */
static int changed(void)
{
	fx.settled = false;
	schedule();
	k_mutex_unlock(&fx_lock);
#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
	(void)k_work_reschedule(&save_work, K_MSEC(CONFIG_BACKLIGHT_SAVE_DELAY_MS));
#endif
	return 0;
}

#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
static uint16_t saved_crc(const struct saved *r)
{
	return crc16_ccitt(0xffffU, (const uint8_t *)r, offsetof(struct saved, crc));
}

static bool saved_valid(const struct saved *r)
{
	return r->magic == SAVE_MAGIC && r->crc == saved_crc(r) && r->on <= 1U &&
	       r->effect < EFFECTS && r->level < LEVELS && r->delay < DELAYS &&
	       r->multicolour <= 1U && r->preset < ARRAY_SIZE(presets) && r->white <= WHITE_MAX &&
	       r->white % WHITE_STEP == 0U;
}

/*
 * A free record: every octet erased. A program cut short can leave the first
 * octet at 0xff with others written; such a record is skipped, not written
 * over (it would fail its CRC-16 and the change would be lost).
 */
static bool erased(const struct saved *r)
{
	const uint8_t *b = (const uint8_t *)r;

	for (size_t i = 0; i < sizeof(*r); i++) {
		if (b[i] != 0xffU) {
			return false;
		}
	}
	return true;
}

/* The newest valid record, if any; save_next at the first free record after the log. */
static bool save_load(struct saved *out)
{
	bool found = false;

	if (!device_is_ready(save_flash)) {
		return false;
	}
	for (uint32_t off = 0U; off < SAVE_SECTOR; off += sizeof(struct saved)) {
		struct saved r;

		if (flash_read(save_flash, SAVE_ADDR + off, &r, sizeof(r)) != 0) {
			save_next = SAVE_SECTOR;
			return found;
		}
		if (erased(&r)) {
			save_next = off;
			return found;
		}
		if (saved_valid(&r)) {
			*out = r;
			found = true;
		}
	}
	save_next = SAVE_SECTOR;
	return found;
}

static void save_write(const struct saved *r)
{
	if (memcmp(r, &last_saved, sizeof(*r)) == 0) {
		return;
	}
	if (save_next + sizeof(*r) > SAVE_SECTOR) {
		if (flash_erase(save_flash, SAVE_ADDR, SAVE_SECTOR) != 0) {
			return;
		}
		save_next = 0U;
	}
	if (flash_write(save_flash, SAVE_ADDR + save_next, r, sizeof(*r)) != 0) {
		save_next = SAVE_SECTOR; /* erase before the next try */
		return;
	}
	save_next += sizeof(*r);
	last_saved = *r;
}

static void snapshot(struct saved *r)
{
	memset(r, 0, sizeof(*r));
	k_mutex_lock(&fx_lock, K_FOREVER);
	r->magic = SAVE_MAGIC;
	r->on = fx.on;
	r->effect = fx.effect;
	r->level = fx.level;
	r->delay = fx.delay;
	r->multicolour = fx.multicolour;
	r->preset = fx.preset;
	r->white = fx.white;
	k_mutex_unlock(&fx_lock);
	r->crc = saved_crc(r);
}

static void save_now(void)
{
	struct saved r;

	snapshot(&r);
	save_write(&r);
}

static void save_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	save_now();
}

void cidoo_backlight_save_pending(void)
{
	struct k_work_sync sync;

	if (k_work_cancel_delayable_sync(&save_work, &sync)) {
		save_now();
	}
}
#endif

static bool lock_on(void)
{
	k_mutex_lock(&fx_lock, K_FOREVER);
	if (fx.on) {
		return true;
	}
	k_mutex_unlock(&fx_lock);
	return false;
}

int cidoo_backlight_set_on(bool on)
{
	k_mutex_lock(&fx_lock, K_FOREVER);
	if (on == fx.on) {
		k_mutex_unlock(&fx_lock);
		return 0;
	}
	fx.on = on;
	fx.rings_lit = 0U;
	if (!on) {
		dark();
	}
	return changed();
}

int cidoo_backlight_toggle(void)
{
	return cidoo_backlight_set_on(!fx.on);
}

int cidoo_backlight_select_effect(int effect)
{
	if (effect < 0 || effect >= EFFECTS) {
		return -EINVAL;
	}
	if (!lock_on()) {
		return 0;
	}
	fx.effect = (uint8_t)effect;
	if (fx.effect == LIGHT) {
		fx.rings_lit = 0U;
		dark(); /* the rings light up from dark */
	}
	return changed();
}

int cidoo_backlight_cycle_effect(int direction)
{
	return cidoo_backlight_select_effect((fx.effect + EFFECTS + (direction > 0 ? 1 : -1)) %
					     EFFECTS);
}

/*
 * From multicolour to the first preset (or the last one, backwards); past the second to last preset forwards, or before the first,
 * back to multicolour.
 */
int cidoo_backlight_step_colour(int direction)
{
	const uint8_t last = ARRAY_SIZE(presets) - 1U;

	if (!lock_on()) {
		return 0;
	}
	fx.rings_lit = 0U;
	if (fx.multicolour) {
		fx.multicolour = false;
		fx.preset = direction > 0 ? 0U : last;
	} else if (direction > 0) {
		if (fx.preset + 1U >= last) {
			fx.multicolour = true;
			fx.preset = 0U;
		} else {
			fx.preset++;
		}
	} else if (fx.preset == 0U) {
		fx.multicolour = true;
		fx.preset = last;
	} else {
		fx.preset--;
	}
	fx.colour = wheel(presets[fx.preset]);
	return changed();
}

int cidoo_backlight_step_white(int direction)
{
	if (!lock_on()) {
		return 0;
	}
	if (direction > 0 && fx.white < WHITE_MAX) {
		fx.white += WHITE_STEP;
	} else if (direction < 0 && fx.white >= WHITE_STEP) {
		fx.white -= WHITE_STEP;
	}
	return changed();
}

int cidoo_backlight_step_level(int direction)
{
	if (!lock_on()) {
		return 0;
	}
	if (direction > 0 && fx.level + 1U < LEVELS) {
		fx.level++;
	} else if (direction < 0 && fx.level > 0U) {
		fx.level--;
	}
	return changed();
}

int cidoo_backlight_step_speed(int direction)
{
	if (!lock_on()) {
		return 0;
	}
	if (direction > 0 && fx.delay > 0U) {
		fx.delay--;
	} else if (direction < 0 && fx.delay + 1U < DELAYS) {
		fx.delay++;
	}
	return changed();
}

#if HOST_LEDS
/* The host's LED output report, from the system work queue: a change of Num Lock shown now. */
static int on_indicators(const zmk_event_t *eh)
{
	const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);

	if (ev != NULL) {
		const bool num_lock = (ev->indicators & BIT(0)) != 0U;

		k_mutex_lock(&fx_lock, K_FOREVER);
		if (num_lock != fx.num_lock) {
			fx.num_lock = num_lock;
			if (fx.on) {
				show();
			}
		}
		k_mutex_unlock(&fx_lock);
	}
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(backlight_host_leds, on_indicators);
ZMK_SUBSCRIPTION(backlight_host_leds, zmk_hid_indicators_changed);
#endif

/*
 * The defaults: on, the wave in multicolour, level 9, delay 2, no white; or
 * what was kept.
 */
static int fx_init(void)
{
	bool on = true;

	if (!device_is_ready(strip)) {
		return -ENODEV;
	}
	fx.effect = WAVE;
	fx.level = LEVELS - 1U;
	fx.delay = 2U;
	fx.multicolour = true;
	fx.colour = wheel(presets[0]);
#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
	struct saved r;

	if (save_load(&r)) {
		on = r.on != 0U;
		fx.effect = r.effect;
		fx.level = r.level;
		fx.delay = r.delay;
		fx.multicolour = r.multicolour != 0U;
		fx.preset = r.preset;
		fx.white = r.white;
		fx.colour = wheel(presets[fx.preset]);
	}
#endif
	if (on) {
		(void)cidoo_backlight_set_on(true);
	} else {
		dark();
	}
#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
	/* The state at boot is what flash holds, or the defaults: nothing to write for it. */
	(void)k_work_cancel_delayable(&save_work);
	snapshot(&last_saved);
#endif
	return 0;
}

SYS_INIT(fx_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
