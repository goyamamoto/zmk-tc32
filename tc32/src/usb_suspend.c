/*
 * The chip suspends while the host keeps USB suspended (the host sleeps):
 * after TLSR_USB_SUSPEND_DELAY_MS of bus suspend every matrix column is
 * driven to its active level (low on the V75 Pro, high on the V21: the
 * col-gpios flags), each row becomes a wake pad for a change of its level,
 * and the chip suspends (tlsr8278_suspend()) until a key, the host's resume
 * (USB core wake), or a timer.
 *
 * - The rows keep their pulls (the V75 Pro's 10 kOhm pull-ups, the V21's
 *   100 kOhm pull-downs), so a row with no key down stays at its idle level.
 * - The timer wakes the chip every TLSR_USB_SUSPEND_WAKE_MS. It catches a
 *   host resume that fails to wake the chip, and it lets the kernel's clock
 *   catch up: the system timer difference the timer driver announces wraps
 *   after 268 s. After a timer wake the chip suspends again 20 ms later if
 *   the bus still is; after a key or the host woke it, the delay starts again.
 *   A host resume that starts in those 20 ms can still meet a suspend; the
 *   chip then answers at the next timer wake, up to 1 s later, unless the USB
 *   core wake ends that suspend.
 * - The bus suspend is seen every 100 ms, and a key asks for a remote wakeup
 *   only on a bus seen suspended for 5 ms and at most once a second: a key in
 *   the first 100 ms of a bus suspend, or within 1 s of a request the host did
 *   not answer, asks for nothing.
 * - Nothing within TLSR_USB_SUSPEND_HOLDOFF_MS of boot, past the boot guard's
 *   healthy uptime.
 * - The mode switch pads (V75 Pro PA0 and PB6, V21 PB6 and PC4) are unused
 *   here and float with input and output off between the BLE stack's
 *   readings (board.c, src/ble/ble.c): not wake sources.
 *
 * The watchdog (Timer2) stands still while the chip is suspended. A key
 * pressed while the bus is suspended asks the host to wake up, if the host
 * allowed it (usb_dc_wakeup_request()), once a second at most.
 *
 * Not in the switch's BT position while the own BLE stack runs (TLSR_BLE):
 * the suspend for the host's USB suspend is the wired position's only.
 *
 * A backlight multiplexed on the matrix columns (src/led_key_matrix.c) goes
 * dark after 1 s of bus suspend and lights again when the bus does.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/drivers/usb/usb_dc_b87.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include <tlsr8278_suspend.h>

#include "led_key_matrix.h"
#if IS_ENABLED(CONFIG_TLSR_BLE)
#include "ble/tlsr_ble.h"
#endif

/* The timer wake's tick difference wraps after 2^32 system timer ticks (16 MHz). */
BUILD_ASSERT(CONFIG_TLSR_USB_SUSPEND_WAKE_MS <= 268000,
	     "TLSR_USB_SUSPEND_WAKE_MS: at most 268000 ms");
#ifdef CONFIG_TLSR_BOOT_GUARD
/* A boot must count as healthy (reset counter cleared) before the chip first suspends. */
BUILD_ASSERT(CONFIG_TLSR_USB_SUSPEND_HOLDOFF_MS > CONFIG_TLSR_BOOT_GUARD_HEALTHY_MS,
	     "TLSR_USB_SUSPEND_HOLDOFF_MS must be longer than TLSR_BOOT_GUARD_HEALTHY_MS");
#endif

#define KSCAN DT_CHOSEN(zmk_kscan)

#define GPIO_PORT0             0x00800580U
#define GPIO_IN                0U
#define GPIO_OEN               2U /* 1: output disabled */
#define GPIO_OUT               3U
#define GPIO_FUNC              6U /* 1: GPIO */
#define GPIO_PORTS             4U /* PA..PD */
#define IRQ_SOURCE_B0          0x00800648U
#define IRQ_SOURCE_B0_USB_PWDN BIT(3) /* usb_pwdn: the bus is suspended (DS-TLSR8278 6.2.1) */
#define WAKEUP_EN              0x0080006eU /* bit 2: wakeup from USB (DS-TLSR8278 Table 2-7) */
#define WAKEUP_EN_USB          BIT(2)

struct pad {
	uint32_t base;
	uint8_t bit;
	bool active_high;
};

#define PAD_ENTRY(node, prop, idx)                                                                 \
	{.base = DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node, prop, idx)),                                \
	 .bit = BIT(DT_GPIO_PIN_BY_IDX(node, prop, idx)),                                          \
	 .active_high = (DT_GPIO_FLAGS_BY_IDX(node, prop, idx) & GPIO_ACTIVE_LOW) == 0},

static const struct pad cols[] = {DT_FOREACH_PROP_ELEM(KSCAN, col_gpios, PAD_ENTRY)};
static const struct pad rows[] = {DT_FOREACH_PROP_ELEM(KSCAN, row_gpios, PAD_ENTRY)};

static uint8_t col_mask[GPIO_PORTS];
static uint8_t col_high[GPIO_PORTS]; /* the columns whose active level is high */
static uint8_t row_mask[GPIO_PORTS];
static int64_t suspended_since = -1; /* the bus seen suspended from then (or the chip woken by a key or the host) */
static int64_t last_wakeup = -1;     /* the last remote wakeup this bus suspend */
static struct k_work_delayable check_work;

static bool bus_suspended(void)
{
	return (sys_read8(IRQ_SOURCE_B0) & IRQ_SOURCE_B0_USB_PWDN) != 0U;
}

/*
 * A remote wakeup, at most one a second and only on a bus suspended for 5 ms
 * (USB 2.0 7.1.7.7). The pad that woke the chip and the same key's event from
 * the matrix scan both ask, one right after the other, and a second resume
 * signal can run into the host's own resume.
 */
#define WAKEUP_IDLE_MS  5
#define WAKEUP_RETRY_MS 1000

/* Bus suspend before the backlight goes dark. */
#define BACKLIGHT_HIDE_MS 1000

static void request_wakeup(void)
{
	int64_t now = k_uptime_get();

	if (!bus_suspended() || suspended_since < 0 || now - suspended_since < WAKEUP_IDLE_MS) {
		return;
	}
	if (last_wakeup >= 0 && now - last_wakeup < WAKEUP_RETRY_MS) {
		return;
	}
	if (usb_dc_wakeup_request() == 0) {
		last_wakeup = now;
	}
}

static uint32_t port_base(uint8_t port)
{
	return GPIO_PORT0 + 8U * port;
}

static uint32_t suspend_once(void)
{
	uint8_t oen[GPIO_PORTS], out[GPIO_PORTS], func[GPIO_PORTS];
	uint8_t wake_en;
	uint32_t st;
	unsigned int key;
	struct lkm_colour_pads pads;

	usb_dc_b87_chip_suspend(true);
	key = irq_lock();
	led_key_matrix_blank();
	led_key_matrix_colour_pads_low(&pads);
	/* Columns driven active (level first, then GPIO, then the output). */
	for (uint8_t p = 0; p < GPIO_PORTS; p++) {
		uint32_t b = port_base(p);

		oen[p] = sys_read8(b + GPIO_OEN);
		out[p] = sys_read8(b + GPIO_OUT);
		func[p] = sys_read8(b + GPIO_FUNC);
		if (col_mask[p] != 0U) {
			sys_write8((out[p] & (uint8_t)~col_mask[p]) | col_high[p], b + GPIO_OUT);
			sys_write8(func[p] | col_mask[p], b + GPIO_FUNC);
			sys_write8(oen[p] & (uint8_t)~col_mask[p], b + GPIO_OEN);
		}
	}
	k_busy_wait(10);
	/* Each row wakes the chip when its level changes. */
	for (uint8_t p = 0; p < GPIO_PORTS; p++) {
		if (row_mask[p] != 0U) {
			uint8_t high = sys_read8(port_base(p) + GPIO_IN) & row_mask[p];

			tlsr8278_pad_wakeup(p, row_mask[p], high, true);
		}
	}
	wake_en = sys_read8(WAKEUP_EN);
	sys_write8(wake_en | WAKEUP_EN_USB, WAKEUP_EN);

	st = tlsr8278_suspend(TLSR8278_WAKEUP_PAD | TLSR8278_WAKEUP_CORE | TLSR8278_WAKEUP_TIMER,
			      k_cycle_get_32() + (uint32_t)CONFIG_TLSR_USB_SUSPEND_WAKE_MS *
							 (sys_clock_hw_cycles_per_sec() / 1000U));

	sys_write8(wake_en, WAKEUP_EN);
	for (uint8_t p = 0; p < GPIO_PORTS; p++) {
		if (row_mask[p] != 0U) {
			tlsr8278_pad_wakeup(p, row_mask[p], 0U, false);
		}
		if (col_mask[p] != 0U) {
			uint32_t b = port_base(p);

			sys_write8(out[p], b + GPIO_OUT);
			sys_write8(func[p], b + GPIO_FUNC);
			sys_write8(oen[p], b + GPIO_OEN);
		}
	}
	led_key_matrix_colour_pads_back(&pads);
	/* The system timer jumped: set the next timer interrupt from now. */
	sys_clock_set_timeout(0, false);
	irq_unlock(key);
	usb_dc_b87_chip_suspend(false);
	return st;
}

static void check(struct k_work *work)
{
	int64_t now = k_uptime_get();

	ARG_UNUSED(work);
#if IS_ENABLED(CONFIG_TLSR_BLE)
	if (ble_running()) {
		/*
		 * The BT position: the USB suspend is the wired position's
		 * only. With no host on the cable, or none at
		 * all, the bus reads suspended, and this suspend (a second at a
		 * time) would end every BLE connection and darken the
		 * backlight. The position is read once, so this stops here.
		 */
		suspended_since = -1;
		led_key_matrix_hide(false);
		return;
	}
#endif
	if (!bus_suspended()) {
		suspended_since = -1;
		last_wakeup = -1;
		led_key_matrix_hide(false);
		k_work_schedule(&check_work, K_MSEC(100));
		return;
	}
	if (suspended_since < 0) {
		suspended_since = now;
	}
	if (now - suspended_since >= BACKLIGHT_HIDE_MS) {
		led_key_matrix_hide(true);
	}
	if (now < CONFIG_TLSR_USB_SUSPEND_HOLDOFF_MS ||
	    now - suspended_since < CONFIG_TLSR_USB_SUSPEND_DELAY_MS) {
		k_work_schedule(&check_work, K_MSEC(100));
		return;
	}
	uint32_t st = suspend_once();

	if ((st & TLSR8278_WAKEUP_STATUS_PAD) != 0U) {
		request_wakeup();
	}
	if ((st & (TLSR8278_WAKEUP_STATUS_PAD | TLSR8278_WAKEUP_STATUS_CORE)) != 0U) {
		/*
		 * A key or the host woke the chip, and the host resumes the bus
		 * now: its K lasts 20 ms or more, and the bus reads suspended
		 * until it ends. The delay starts again, so the chip does not
		 * suspend inside that resume, where a core wake that fires once
		 * per K would leave it asleep until the timer.
		 */
		suspended_since = k_uptime_get();
		k_work_schedule(&check_work, K_MSEC(100));
		return;
	}
	/* The timer only: awake for a moment (ZMK scans), then asleep again if the bus still is. */
	k_work_schedule(&check_work, K_MSEC(20));
}

static int on_position(const zmk_event_t *eh)
{
	const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

	if (ev != NULL && ev->state) {
		request_wakeup();
	}
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(cidoo_usb_suspend, on_position);
ZMK_SUBSCRIPTION(cidoo_usb_suspend, zmk_position_state_changed);

static int usb_suspend_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(cols); i++) {
		col_mask[(cols[i].base - GPIO_PORT0) / 8U] |= cols[i].bit;
		if (cols[i].active_high) {
			col_high[(cols[i].base - GPIO_PORT0) / 8U] |= cols[i].bit;
		}
	}
	for (size_t i = 0; i < ARRAY_SIZE(rows); i++) {
		row_mask[(rows[i].base - GPIO_PORT0) / 8U] |= rows[i].bit;
	}
	k_work_init_delayable(&check_work, check);
	k_work_schedule(&check_work, K_MSEC(100));
	return 0;
}

SYS_INIT(usb_suspend_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
