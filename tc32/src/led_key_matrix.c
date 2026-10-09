/*
 * The key matrix and the RGB backlight of a TLSR8278 keyboard whose columns
 * switch both (dts/bindings/kscan/cidoo,led-key-matrix.yaml), the Cidoo V21's
 * wiring:
 *
 * - Timer1 in mode 0 interrupts every 500 us: 24000 cycles of the 48 MHz
 *   system clock. A frame is the column slots and then the key scan slot:
 *   13 slots, 6.5 ms, for the V21's 12 columns.
 * - A column slot: the PWM stopped (its outputs go low at once, DS-TLSR8278
 *   8.5.2), every column high, this column's compare values written, the
 *   channels with a value above 0 started, this column driven low. So at
 *   most one column is low while a PWM output runs. A channel with compare 0
 *   is not started (a new frame starts high, 8.4.1). The slot before works
 *   the values out, so that the column goes dark and the next one lit within
 *   a few microseconds.
 * - The PWM: 1 MHz (48 MHz / 48), no inversion. The light of a slot for the
 *   colour byte c is c + min(c, 237) us: what two 255 us PWM frames with c
 *   as the compare give in a slot, the second pulse cut where the next slot
 *   stops the PWM, about 492 us after the first started. Here that light
 *   comes as one pulse (the frame is 65535 us): at most 492 us, 7.6 % of a
 *   frame. With 255 us frames, a slot interrupt that comes late would start
 *   a third pulse; with one pulse it never adds light, but it cuts the pulse
 *   before it short when the next interrupt comes before that pulse ends
 *   (the system timer interrupt holds a slot interrupt back: the kernel's
 *   timeouts run in it, and without USB_DC_TELINK_B87_EXTERNAL_POLL the 1 ms
 *   USB poll too).
 *   A slot interrupt that waited starts the slots again from itself
 *   (follow_phase()), so that the next slot does not cut its pulse. Each
 *   interrupt measures what the last pulse had (the system timer, 16 MHz),
 *   and the column's next pulse makes up what a cut left out, once, up to
 *   492 us: over the two frames the light is the colour's.
 *   Waiting in the interrupt for the pulse to end instead made the two
 *   interrupts delay each other: up to 30 % of the CPU in this interrupt. A
 *   slot interrupt that never comes leaves at most 492 us of light in
 *   65.5 ms.
 * - The key scan slot, the PWM stopped: every column low; for each column
 *   the rows are driven low for a moment (a row that a key pulled high for
 *   the previous column would otherwise fall only through its 100 kOhm
 *   pull-down) and let go, the column driven high, the rows read, the column
 *   low again. The rows are let go before the column goes high: a column
 *   driven high while a row is still driven low would be a moment's short
 *   through a pressed key's diode. Then every column high.
 * - Without USB_DC_TELINK_B87_EXTERNAL_POLL, with nothing to light (every
 *   pixel black, or hidden while the host sleeps) the timer runs the key
 *   scan only, one interrupt a frame.
 * - With USB_DC_TELINK_B87_EXTERNAL_POLL (which this driver selects) the
 *   interrupt polls the USB controller every other slot, once a millisecond,
 *   after the column has switched, and the slots go on at 500 us when there
 *   is nothing to light. The kernel then runs no 1 ms USB timer: its timeout
 *   processing took several hundred microseconds of every millisecond in the
 *   system timer interrupt (busy CPU 34.9 % to 12.7 % in tc32emu). The USB
 *   driver's slow fallback poll keeps USB working if this timer stops.
 *
 * The interrupt's code is in .ram_code and its data in SRAM, so a column
 * slot fetches nothing from the flash itself; the dispatcher reads the
 * interrupt table and asks the scheduler from the flash on every interrupt.
 * ZMK's debouncer (in the flash) runs in the scan slot, only for keys whose
 * reading differs from their state (6.5 ms per scan); the changes go through
 * a queue to a work item, which calls ZMK's kscan callback.
 *
 * DS-TLSR8278 5.1.2 says a mode 0 timer stops counting at its capture.
 * tc32emu models the tick starting again from 0 after the match, and a unit
 * was checked to behave the same: Timer1 is set once here and interrupts at
 * its period from then on.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT cidoo_led_key_matrix

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zmk/debounce.h>

#include "led_key_matrix.h"

#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
#include <zephyr/drivers/usb/usb_dc_b87.h>
#define EXTERNAL_POLL 1
#else
#define EXTERNAL_POLL 0
#endif

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "one cidoo,led-key-matrix");

#define MATRIX DT_DRV_INST(0)
#define ROWS   DT_PROP_LEN(MATRIX, row_gpios)
#define COLS   DT_PROP_LEN(MATRIX, col_gpios)
#define KEYS   (ROWS * COLS)
#define SCAN   COLS /* the key scan's slot, after the column slots */
#define PWMS   6U

BUILD_ASSERT(KEYS <= 32, "a scan keeps the matrix in 32 bits");
BUILD_ASSERT(DT_PROP_LEN(MATRIX, pwm_gpios) == PWMS && DT_PROP_LEN(MATRIX, pwm_mux) == PWMS,
	     "pwm-gpios and pwm-mux: PWM0 to PWM5");
BUILD_ASSERT(DT_PROP_LEN(MATRIX, rgb_channels) == 3 * ROWS, "rgb-channels: three per row");

#define ACTIVE_HIGH(node, prop, idx) ((DT_GPIO_FLAGS_BY_IDX(node, prop, idx) & GPIO_ACTIVE_LOW) == 0) &&
BUILD_ASSERT(DT_FOREACH_PROP_ELEM(MATRIX, col_gpios, ACTIVE_HIGH) DT_FOREACH_PROP_ELEM(
		     MATRIX, row_gpios, ACTIVE_HIGH) true,
	     "the driver drives and reads physical levels: GPIO_ACTIVE_HIGH columns and rows");

/* reg_clk_sel 0x66 = 0x20, set by the SoC setup; a unit measured 47.995 MHz. */
#define SCLK_HZ      48000000U
#define SLOT_CYCLES  (SCLK_HZ / 2000U) /* 500 us */
#define FRAME_CYCLES ((SCAN + 1U) * SLOT_CYCLES)
#define SCAN_HALF_MS (SCAN + 1U) /* a frame in half milliseconds (a slot each): 13, 6.5 ms */

/* The GPIO registers of a port (DS-TLSR8278 7.1): in, ie, oen, out, pol, ds, gpio-function. */
#define GPIO_PORT0 0x00800580U
#define GPIO_PORTS 4U /* PA..PD */
#define GPIO_IN    0U
#define GPIO_OEN   2U /* 1: output disabled */
#define GPIO_OUT   3U
#define GPIO_FUNC  6U /* 1: GPIO, 0: the pad's function (PAD_MUX) */
#define PAD_MUX(port, pin) (0x008005a8U + 2U * (port) + (pin) / 4U) /* two bits a pin */

/* PWM (DS-TLSR8278 8.1). */
#define PWM_EN     0x00800780U /* bits 1-5: PWM1-PWM5 */
#define PWM_EN0    0x00800781U /* bit 0: PWM0 */
#define PWM_CLKDIV 0x00800782U
#define PWM_MODE   0x00800783U
#define PWM_INV    0x00800784U
#define PWM_N_INV  0x00800785U
#define PWM_POL    0x00800786U
#define PWM_TCMP(n) (0x00800794U + 4U * (n))
#define PWM_TMAX(n) (0x00800796U + 4U * (n))
#define PWM_HZ     1000000U
#define PWM_FRAME  0xffffU /* longer than any slot: one pulse a slot */
/* The light in a slot for colour byte c: two pulses of c us, the second cut at 237 us. */
#define STOCK_SECOND_US 237U
#define PULSE_US(c)     ((c) + MIN((c), STOCK_SECOND_US)) /* 492 us at 255 */
#define PULSE_MAX_US    492U /* a pulse that makes up for a cut one, at most */

/* The system timer (16 MHz), which measures how long a pulse had before the next slot. */
#define STIMER_TICK     0x00800740U
#define STIMER_US_SHIFT 4 /* 16 ticks a microsecond */

/* Timer1 (DS-TLSR8278 5.1.1). Byte 0x620 only: 0x621-0x622 hold the watchdog, 0x623 the statuses. */
#define TMR_CTRL  0x00800620U
#define TMR1_EN   BIT(3)
#define TMR1_MODE (BIT(4) | BIT(5)) /* 0: the system clock */
#define TMR_STA   0x00800623U
#define TMR1_STA  BIT(1)
#define TMR1_CAPT 0x00800628U
#define TMR1_TICK 0x00800634U
#define IRQ_SRC   0x00800648U
#define IRQ_TMR1  1 /* reg_irq_mask bit 1; the arch clears the timer's status */

/*
 * A slot interrupt that starts more than LATE_CYCLES after its match waited
 * for another interrupt; the slots then start again from that moment (see
 * follow_phase()).
 */
#define LATE_CYCLES (30U * (SCLK_HZ / 1000000U))

/* An event: bit 7 pressed, bits 0-6 the position row * COLS + column. */
#define EVENTS       32U /* a power of two: indexes are masked */
#define EV_PRESSED   BIT(7)
#define EV_POSITION  0x7fU

/* Loop turns of at least 1 us: a turn is three instructions or more at 48 MHz. */
#define SPIN_PER_US 16U

#define LKM_RAM_CODE __attribute__((noinline, section(".ram_code.led_key_matrix")))

struct lkm_pad {
	uint32_t base; /* the port's registers */
	uint8_t bit;
};

#define PAD_ENTRY(node, prop, idx)                                                                 \
	{.base = DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node, prop, idx)),                                \
	 .bit = BIT(DT_GPIO_PIN_BY_IDX(node, prop, idx))},

/* Everything the interrupt reads is here, in SRAM (not in flash: no cache miss). */
static struct {
	struct lkm_pad col[COLS];
	uint32_t col_port[GPIO_PORTS]; /* the ports with columns; 0 after the last */
	uint8_t col_mask[GPIO_PORTS];
	uint32_t row_port[GPIO_PORTS]; /* the ports with rows; 0 after the last */
	uint8_t row_mask[GPIO_PORTS];
	struct lkm_pad row[ROWS];
	uint8_t duty[COLS][PWMS]; /* each column slot's colour bytes, PWM channel order */
	uint8_t en0[COLS];        /* each slot's PWM_EN0 bits: PWM0 when its value is above 0 */
	uint8_t en[COLS];         /* each slot's PWM_EN bits: PWM1-PWM5 likewise */
	bool any;                 /* some slot lights something */
	bool hidden;
	bool scan_only;           /* the timer's period is a frame */
	bool ready;
#if IS_ENABLED(CONFIG_TLSR_BLE_LOW_POWER)
	bool asleep;              /* led_key_matrix_sleep(): Timer1 stopped, the rows wake the chip */
#endif
	uint8_t slot;             /* the next interrupt's slot */
	uint8_t half_ms;          /* the half millisecond the scans leave over */
	uint16_t tick_half_ms;    /* led_key_matrix_tick()'s period; 0: none */
	uint16_t tick_count;      /* half milliseconds since its last tick */
	bool poll_half;           /* the USB poll is due at the next interrupt */
	uint32_t pulse_start;     /* the system timer when the last column slot started its pulses */
	uint16_t pulse_us;        /* that slot's longest pulse; 0: none */
	uint8_t pulse_col;        /* that slot's column */
	uint16_t cut[COLS];       /* 1 + the window a late interrupt left a column's pulse; 0: not cut */
	uint16_t next_us[PWMS];   /* the next column slot's pulses, worked out in the slot before */
	uint16_t next_longest;    /* the longest of them */
	uint8_t next_en0;         /* its PWM_EN0 and PWM_EN bits; both 0: it lights nothing */
	uint8_t next_en;
	struct zmk_debounce_config debounce;
	struct zmk_debounce_state keys[KEYS];
	uint8_t events[EVENTS];
	volatile uint8_t ev_head; /* written by the interrupt */
	volatile uint8_t ev_tail; /* written by the work item */
} lkm;

static const struct device *lkm_dev;
static kscan_callback_t lkm_callback;
static bool lkm_enabled;
static struct k_work lkm_work;
static struct k_work_q *tick_queue;
static struct k_work *tick_work;

static ALWAYS_INLINE void reg_clear(uint32_t addr, uint8_t bits)
{
	sys_write8(sys_read8(addr) & (uint8_t)~bits, addr);
}

static ALWAYS_INLINE void reg_set(uint32_t addr, uint8_t bits)
{
	sys_write8(sys_read8(addr) | bits, addr);
}

static ALWAYS_INLINE void spin_us(uint32_t us)
{
	for (uint32_t n = us * SPIN_PER_US; n > 0U; n--) {
		arch_nop();
	}
}

static ALWAYS_INLINE void pwm_stop(void)
{
	reg_clear(PWM_EN0, BIT(0));
	reg_clear(PWM_EN, 0x3eU);
}

/* Every column high (its LEDs dark), or low (the key scan's idle level). */
static ALWAYS_INLINE void cols_level(bool high)
{
	for (uint32_t i = 0; i < GPIO_PORTS && lkm.col_port[i] != 0U; i++) {
		if (high) {
			reg_set(lkm.col_port[i] + GPIO_OUT, lkm.col_mask[i]);
		} else {
			reg_clear(lkm.col_port[i] + GPIO_OUT, lkm.col_mask[i]);
		}
	}
}

/* The rows driven low (their output bits are 0), or let go to their pull-downs. */
static ALWAYS_INLINE void rows_driven(bool driven)
{
	for (uint32_t i = 0; i < GPIO_PORTS && lkm.row_port[i] != 0U; i++) {
		if (driven) {
			reg_clear(lkm.row_port[i] + GPIO_OEN, lkm.row_mask[i]);
		} else {
			reg_set(lkm.row_port[i] + GPIO_OEN, lkm.row_mask[i]);
		}
	}
}

/*
 * The Timer1 period from now: stopped, a match that came meanwhile cleared
 * (the arch's order: the interrupt source, then the status), the tick
 * cleared, the capture set, started (as the SDK's timer1_set_mode).
 */
static ALWAYS_INLINE void timer1_period(uint32_t cycles)
{
	uint8_t ctrl = sys_read8(TMR_CTRL) & (uint8_t)~(TMR1_EN | TMR1_MODE);

	sys_write8(ctrl, TMR_CTRL);
	sys_write32(BIT(IRQ_TMR1), IRQ_SRC);
	sys_write8(TMR1_STA, TMR_STA);
	sys_write32(0U, TMR1_TICK);
	sys_write32(cycles, TMR1_CAPT);
	sys_write8(ctrl | TMR1_EN, TMR_CTRL);
}

/* Reads the matrix: bit row * COLS + column set for a key down. The PWM is stopped. */
static ALWAYS_INLINE uint32_t scan_keys(void)
{
	uint32_t down = 0U;

	spin_us(2U); /* the stopped PWM's outputs low before any column goes low */
	cols_level(false);
	for (uint32_t c = 0; c < COLS; c++) {
		const struct lkm_pad *col = &lkm.col[c];

		rows_driven(true);
		spin_us(1U);
		rows_driven(false);
		reg_set(col->base + GPIO_OUT, col->bit);
		spin_us(2U);
		for (uint32_t r = 0; r < ROWS; r++) {
			if ((sys_read8(lkm.row[r].base + GPIO_IN) & lkm.row[r].bit) != 0U) {
				down |= BIT(r * COLS + c);
			}
		}
		reg_clear(col->base + GPIO_OUT, col->bit);
	}
	cols_level(true);
	return down;
}

/* Debounces the keys whose reading or state is in motion; queues the changes. */
static ALWAYS_INLINE bool debounce_keys(uint32_t down)
{
	bool queued = false;
	int elapsed;

	lkm.half_ms += SCAN_HALF_MS;
	elapsed = lkm.half_ms >> 1;
	lkm.half_ms &= 1U;
	for (uint32_t i = 0; i < KEYS; i++) {
		struct zmk_debounce_state *st = &lkm.keys[i];
		const bool active = (down & BIT(i)) != 0U;

		if (active == st->pressed && st->counter == 0U) {
			continue;
		}
		if ((uint8_t)(lkm.ev_head - lkm.ev_tail) >= EVENTS) {
			break; /* the queue is full: these keys wait for the next scan */
		}
		zmk_debounce_update(st, active, elapsed, &lkm.debounce);
		if (st->changed) {
			lkm.events[lkm.ev_head & (EVENTS - 1U)] = (st->pressed ? EV_PRESSED : 0U) | i;
			compiler_barrier();
			lkm.ev_head++;
			queued = true;
		}
	}
	return queued;
}

/*
 * The pulses of column slot `slot`, for the next interrupt. A cut in the last
 * frame is made up in this one, once: the light over the two is the colour's.
 */
static ALWAYS_INLINE void prepare(uint32_t slot)
{
	const uint32_t cut = lkm.cut[slot];
	uint32_t longest = 0U;

	lkm.cut[slot] = 0U;
	lkm.next_en0 = 0U;
	lkm.next_en = 0U;
	if (lkm.hidden || (lkm.en0[slot] | lkm.en[slot]) == 0U) {
		return;
	}
	for (uint32_t n = 0; n < PWMS; n++) {
		uint32_t us = PULSE_US(lkm.duty[slot][n]);

		if (cut != 0U && us > cut - 1U) {
			us = MIN(us + (us - (cut - 1U)), PULSE_MAX_US);
		}
		lkm.next_us[n] = (uint16_t)us;
		longest = MAX(longest, us);
	}
	lkm.next_longest = (uint16_t)longest;
	lkm.next_en0 = lkm.en0[slot];
	lkm.next_en = lkm.en[slot];
}

/*
 * The kernel's timeouts (without USB_DC_TELINK_B87_EXTERNAL_POLL among them
 * the USB poll every 1 ms, with it the USB driver's 10 ms fallback) run in
 * the system timer's interrupt, for a few hundred microseconds when their
 * code misses the flash cache. Both timers count the same crystal, so these
 * interrupts come at a fixed phase to the slots: one that runs across a
 * slot's match holds that slot back at every poll, and the pulse the late
 * slot starts is cut short by the next slot. A slot interrupt that waited
 * therefore starts the timer's period again from now: the next slot comes
 * 500 us after this one, so its pulse is not cut, and the slots move to just
 * after the interrupt that held them back, where the next ones clear it.
 * The capture is never changed while the timer runs: a capture written
 * below the count (a slot interrupt more than a period late, as while a
 * flash write holds interrupts off) would match only after the count wraps,
 * 2^32 cycles (89 s) later, with no key scanned meanwhile.
 */
static ALWAYS_INLINE void follow_phase(uint32_t late)
{
	if (late > LATE_CYCLES && late < SLOT_CYCLES) {
		timer1_period(SLOT_CYCLES);
	}
}

static void LKM_RAM_CODE lkm_isr(const void *arg)
{
	const uint32_t late = sys_read32(TMR1_TICK); /* cycles since the match */
	const uint32_t now = sys_read32(STIMER_TICK);
	const uint32_t last_start = lkm.pulse_start;
	const uint32_t last_us = lkm.pulse_us;
	const uint8_t last_col = lkm.pulse_col;
	const uint8_t slot = lkm.slot;

	ARG_UNUSED(arg);
	pwm_stop();
	cols_level(true);
	lkm.pulse_us = 0U;
	if (slot != SCAN && !lkm.hidden && (lkm.next_en0 | lkm.next_en) != 0U) {
		for (uint32_t n = 0; n < PWMS; n++) {
			sys_write16(lkm.next_us[n], PWM_TCMP(n));
		}
		lkm.pulse_start = sys_read32(STIMER_TICK);
		reg_set(PWM_EN0, lkm.next_en0);
		reg_set(PWM_EN, lkm.next_en);
		reg_clear(lkm.col[slot].base + GPIO_OUT, lkm.col[slot].bit);
		lkm.pulse_us = lkm.next_longest;
		lkm.pulse_col = slot;
	}
	/* The last column slot's pulses: cut short if this interrupt came before they ended. */
	if (last_us != 0U) {
		const uint32_t us = (now - last_start) >> STIMER_US_SHIFT;

		if (us < last_us) {
			lkm.cut[last_col] = (uint16_t)(us + 1U);
		}
	}
	if (!lkm.scan_only) {
		follow_phase(late);
	}
	if (lkm.tick_half_ms != 0U) {
		lkm.tick_count += lkm.scan_only ? SCAN_HALF_MS : 1U;
		if (lkm.tick_count >= lkm.tick_half_ms) {
			lkm.tick_count = 0U;
			k_work_submit_to_queue(tick_queue, tick_work);
		}
	}

	if (slot == SCAN) {
		const bool dark = !EXTERNAL_POLL && (lkm.hidden || !lkm.any);

		if (debounce_keys(scan_keys())) {
			k_work_submit(&lkm_work);
		}
		if (dark != lkm.scan_only) {
			timer1_period(dark ? FRAME_CYCLES : SLOT_CYCLES);
			lkm.scan_only = dark;
		}
		lkm.slot = dark ? SCAN : 0U;
		if (!dark) {
			prepare(0U);
		}
	} else {
		lkm.slot = slot + 1U;
		if (slot + 1U < SCAN) {
			prepare(slot + 1U);
		}
	}
#if EXTERNAL_POLL
	lkm.poll_half = !lkm.poll_half;
	if (lkm.poll_half) {
		usb_dc_b87_poll();
	}
#endif
}

void led_key_matrix_blank(void)
{
	if (!lkm.ready) {
		return;
	}
	pwm_stop();
	cols_level(true);
}

void led_key_matrix_tick(struct k_work_q *queue, struct k_work *work, uint32_t half_ms)
{
	unsigned int key = irq_lock();

	tick_queue = queue;
	tick_work = work;
	lkm.tick_half_ms = (uint16_t)MIN(half_ms, UINT16_MAX);
	lkm.tick_count = 0U;
	irq_unlock(key);
}

void led_key_matrix_hide(bool hidden)
{
	unsigned int key = irq_lock();

	lkm.hidden = hidden;
	if (hidden) {
		led_key_matrix_blank();
	}
	irq_unlock(key);
}

#if IS_ENABLED(CONFIG_TLSR_BLE_LOW_POWER)
#include <tlsr8278_suspend.h>

#define GPIO_PORT0 0x00800580U
#define GPIO_IN    0U

static struct lkm_colour_pads sleep_pads;

void led_key_matrix_sleep(bool sleep)
{
	unsigned int key = irq_lock();

	if (!lkm.ready || sleep == lkm.asleep) {
		irq_unlock(key);
		return;
	}
	lkm.asleep = sleep;
	if (sleep) {
		/* Timer1 stopped and its pending match cleared (the arch's order). */
		reg_clear(TMR_CTRL, TMR1_EN);
		sys_write32(BIT(IRQ_TMR1), IRQ_SRC);
		sys_write8(TMR1_STA, TMR_STA);
		pwm_stop();
		cols_level(true);
		led_key_matrix_colour_pads_low(&sleep_pads);
		rows_driven(false);
		spin_us(10U);
		/* Each row wakes the chip at the level opposite to the one it reads now. */
		for (uint32_t i = 0; i < GPIO_PORTS && lkm.row_port[i] != 0U; i++) {
			uint8_t high = sys_read8(lkm.row_port[i] + GPIO_IN) & lkm.row_mask[i];

			tlsr8278_pad_wakeup((uint8_t)((lkm.row_port[i] - GPIO_PORT0) / 8U),
					    lkm.row_mask[i], high, true);
		}
	} else {
		for (uint32_t i = 0; i < GPIO_PORTS && lkm.row_port[i] != 0U; i++) {
			tlsr8278_pad_wakeup((uint8_t)((lkm.row_port[i] - GPIO_PORT0) / 8U),
					    lkm.row_mask[i], 0U, false);
		}
		led_key_matrix_colour_pads_back(&sleep_pads);
		/* The key scan first, then the slots as the next scan decides. */
		lkm.slot = SCAN;
		lkm.scan_only = false;
		timer1_period(SLOT_CYCLES);
	}
	irq_unlock(key);
}
#endif

static void lkm_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	while (lkm.ev_tail != lkm.ev_head) {
		uint8_t ev;

		compiler_barrier();
		ev = lkm.events[lkm.ev_tail & (EVENTS - 1U)];
		compiler_barrier(); /* read before the slot is given back to the interrupt */
		lkm.ev_tail++;
		if (lkm_enabled && lkm_callback != NULL) {
			const uint32_t pos = ev & EV_POSITION;

			lkm_callback(lkm_dev, pos / COLS, pos % COLS, (ev & EV_PRESSED) != 0U);
		}
	}
}

static int lkm_configure(const struct device *dev, kscan_callback_t callback)
{
	ARG_UNUSED(dev);
	if (callback == NULL) {
		return -EINVAL;
	}
	lkm_callback = callback;
	return 0;
}

static int lkm_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	lkm_enabled = true;
	return 0;
}

static int lkm_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	lkm_enabled = false;
	return 0;
}

static const struct lkm_pad col_pads[] = {DT_FOREACH_PROP_ELEM(MATRIX, col_gpios, PAD_ENTRY)};
static const struct lkm_pad row_pads[] = {DT_FOREACH_PROP_ELEM(MATRIX, row_gpios, PAD_ENTRY)};

struct pwm_pad {
	uint8_t port; /* 0: PA */
	uint8_t pin;
};

#define PWM_PAD_ENTRY(node, prop, idx)                                                             \
	{.port = (DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node, prop, idx)) - GPIO_PORT0) / 8U,            \
	 .pin = DT_GPIO_PIN_BY_IDX(node, prop, idx)},

static const struct pwm_pad pwm_pads[] = {DT_FOREACH_PROP_ELEM(MATRIX, pwm_gpios, PWM_PAD_ENTRY)};
static const uint8_t pwm_mux[] = DT_PROP(MATRIX, pwm_mux);

#define SPEC_ENTRY(node, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx),
static const struct gpio_dt_spec col_specs[] = {DT_FOREACH_PROP_ELEM(MATRIX, col_gpios, SPEC_ENTRY)};
static const struct gpio_dt_spec row_specs[] = {DT_FOREACH_PROP_ELEM(MATRIX, row_gpios, SPEC_ENTRY)};

/* A pad's port joined to the list of ports and its bit to the port's mask. */
static void port_add(uint32_t *ports, uint8_t *masks, const struct lkm_pad *pad)
{
	for (uint32_t i = 0; i < GPIO_PORTS; i++) {
		if (ports[i] == 0U || ports[i] == pad->base) {
			ports[i] = pad->base;
			masks[i] |= pad->bit;
			return;
		}
	}
}

static int lkm_init(const struct device *dev)
{
	unsigned int key;
	int err;

	lkm_dev = dev;
	k_work_init(&lkm_work, lkm_work_handler);
	lkm.debounce.debounce_press_ms = DT_PROP(MATRIX, debounce_press_ms);
	lkm.debounce.debounce_release_ms = DT_PROP(MATRIX, debounce_release_ms);
	lkm.slot = SCAN;
	lkm.scan_only = true;

	/* The columns high (their LEDs dark), the rows inputs on their pulls. */
	for (size_t i = 0; i < COLS; i++) {
		if (!gpio_is_ready_dt(&col_specs[i])) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&col_specs[i], GPIO_OUTPUT_HIGH);
		if (err != 0) {
			return err;
		}
		lkm.col[i] = col_pads[i];
		port_add(lkm.col_port, lkm.col_mask, &col_pads[i]);
	}
	for (size_t i = 0; i < ROWS; i++) {
		if (!gpio_is_ready_dt(&row_specs[i])) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&row_specs[i], GPIO_INPUT);
		if (err != 0) {
			return err;
		}
		lkm.row[i] = row_pads[i];
		port_add(lkm.row_port, lkm.row_mask, &row_pads[i]);
	}

	key = irq_lock();
	/* A row driven for the scan drives low. */
	for (size_t i = 0; i < ROWS; i++) {
		reg_clear(row_pads[i].base + GPIO_OUT, row_pads[i].bit);
	}
	/* The PWM set up stopped, every value 0. */
	pwm_stop();
	sys_write8(SCLK_HZ / PWM_HZ - 1U, PWM_CLKDIV);
	sys_write8(0U, PWM_MODE);
	sys_write8(0U, PWM_INV);
	sys_write8(0U, PWM_N_INV);
	sys_write8(0U, PWM_POL);
	for (uint32_t n = 0; n < PWMS; n++) {
		sys_write16(0U, PWM_TCMP(n));
		sys_write16(PWM_FRAME, PWM_TMAX(n));
	}
	/* The pads to their PWM function: the channel stopped keeps them low, as the GPIO did. */
	for (uint32_t n = 0; n < PWMS; n++) {
		const struct pwm_pad *p = &pwm_pads[n];
		const uint32_t mux = PAD_MUX(p->port, p->pin);
		const uint8_t sh = 2U * (p->pin % 4U);

		sys_write8((sys_read8(mux) & (uint8_t)~(3U << sh)) | (uint8_t)((pwm_mux[n] & 3U) << sh),
			   mux);
		reg_clear(GPIO_PORT0 + 8U * p->port + GPIO_FUNC, BIT(p->pin));
	}
	/* Timer1: the key scan alone until something is lit. */
	IRQ_CONNECT(IRQ_TMR1, 0, lkm_isr, NULL, 0);
	timer1_period(FRAME_CYCLES);
	sys_write8(TMR1_STA, TMR_STA);
	lkm.ready = true;
	irq_enable(IRQ_TMR1);
	irq_unlock(key);
	return 0;
}

static const struct kscan_driver_api lkm_api = {
	.config = lkm_configure,
	.enable_callback = lkm_enable,
	.disable_callback = lkm_disable,
};

DEVICE_DT_INST_DEFINE(0, lkm_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,
		      &lkm_api);

#if DT_HAS_COMPAT_STATUS_OKAY(cidoo_led_key_matrix_backlight) && defined(CONFIG_LED_STRIP)
#include <zephyr/drivers/led_strip.h>

#define STRIP  DT_COMPAT_GET_ANY_STATUS_OKAY(cidoo_led_key_matrix_backlight)
#define PIXELS DT_PROP(STRIP, chain_length)
#define NO_LED 0xffU

BUILD_ASSERT(DT_PROP_LEN(STRIP, led_pixels) == KEYS, "led-pixels: one per LED position");

static const uint8_t led_pixel[KEYS] = DT_PROP(STRIP, led_pixels);
static const uint8_t rgb_channel[ROWS * 3] = DT_PROP(MATRIX, rgb_channels); /* red, green, blue a row */

static int strip_update_rgb(const struct device *dev, struct led_rgb *pixels, size_t num_pixels)
{
	uint8_t duty[COLS][PWMS] = {0};
	uint8_t en0[COLS] = {0};
	uint8_t en[COLS] = {0};
	bool any = false;
	unsigned int key;

	ARG_UNUSED(dev);
	for (uint32_t r = 0; r < ROWS; r++) {
		for (uint32_t c = 0; c < COLS; c++) {
			const uint8_t p = led_pixel[r * COLS + c];

			if (p == NO_LED || p >= num_pixels) {
				continue;
			}
			duty[c][rgb_channel[3 * r]] = pixels[p].r;
			duty[c][rgb_channel[3 * r + 1]] = pixels[p].g;
			duty[c][rgb_channel[3 * r + 2]] = pixels[p].b;
		}
	}
	for (uint32_t c = 0; c < COLS; c++) {
		for (uint32_t n = 0; n < PWMS; n++) {
			if (duty[c][n] != 0U) {
				if (n == 0U) {
					en0[c] |= BIT(0);
				} else {
					en[c] |= BIT(n);
				}
				any = true;
			}
		}
	}
	key = irq_lock();
	memcpy(lkm.duty, duty, sizeof(duty));
	memcpy(lkm.en0, en0, sizeof(en0));
	memcpy(lkm.en, en, sizeof(en));
	lkm.any = any;
	irq_unlock(key);
	return 0;
}

static size_t strip_length(const struct device *dev)
{
	ARG_UNUSED(dev);
	return PIXELS;
}

static DEVICE_API(led_strip, strip_api) = {
	.update_rgb = strip_update_rgb,
	.length = strip_length,
};

DEVICE_DT_DEFINE(STRIP, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY,
		 &strip_api);
#endif
