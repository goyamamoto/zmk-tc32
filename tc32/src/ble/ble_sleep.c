/*
 * Deep sleep while no host connects:
 *
 * - When: after CONFIG_TLSR_BLE_RECONNECT_TIMEOUT_S of advertising to a
 *   bonded host (20 s) or CONFIG_TLSR_BLE_PAIRING_TIMEOUT_S of discoverable
 *   advertising (60 s), counted from when advertising began (keys do not
 *   restart it), and at 0 % on battery (ble_battery.c). Not while ZMK's USB
 *   is connected (ZMK serves USB in every switch position): advertising
 *   then starts its count again.
 * - The pins, on the V21: the backlight dark (its colour lines GPIO outputs
 *   low), every matrix column high through its 10 kOhm pull-up (a pull, not
 *   the GPIO output, holds the level in deep sleep). The rows keep their
 *   100 kOhm pull-downs: a pressed key pulls its row high through the
 *   column's pull-up. On the V75 Pro the board sets every pad instead
 *   (CONFIG_TLSR_BOARD_DEEP_SLEEP_PADS, cidoo_v75pro/deep_sleep_pads.c): the
 *   columns at 0 with 100 kOhm pull-downs, the rows without pulls waking on
 *   low, the switch's common pin driven.
 * - Wake pads, on the V21: each row and each mode switch position pin on the
 *   level opposite to the one it reads now, and the power-in pin
 *   ("cidoo,battery" power-gpios) on its active level if it reads inactive.
 *   The switch pins get 1 MOhm pull-ups, not 10 kOhm: the pin grounded by the
 *   switch would draw about 0.3 mA through 10 kOhm for the whole sleep. On
 *   the V75 Pro (deep_sleep_pads.c): the rows, and both switch pins on the
 *   level opposite to the one each reads, so that a move wakes the chip in
 *   either position; not the power-in pin, so plugging USB in does not wake
 *   it.
 * - The boot guard: an image confirmed in RAM and in the counter sector
 *   marks the wake as a planned reset (tlsr_boot_guard_plan_wake()), so that
 *   wakes with no host in reach do not add up to a revert. Such a wake reads
 *   no power-on chord: going back to the stock from deep sleep takes a real
 *   power cycle with the chord held, or &prev_fw once awake. An unconfirmed
 *   image's wakes are counted and read the chord. On the V21 the columns'
 *   pull-ups from the sleep last until the matrix driver sets its pins after
 *   the count, so keys held in both chord rows at the wake can read as the
 *   chord (the image then goes back to the stock).
 *
 * The wake is a boot through the boot ROM; the key that woke the chip is not
 * reported.
 *
 * The low-power state while connected (CONFIG_TLSR_BLE_LOW_POWER): on an
 * encrypted link with the keyboard's notifications on, after
 * CONFIG_TLSR_BLE_IDLE_S (300 s) with no key, knob or pad wake, and not while
 * a USB host has the keyboard over the cable (ble_usb_host()). Then the matrix
 * sleeps (key_matrix_sleep(): on the V21 led_key_matrix_sleep(), Timer1
 * stopped, the backlight dark, the rows wake pads; on the V75 Pro
 * gpio_matrix_sleep.c, the key scan stopped, the columns active, the rows wake
 * pads; no USB poll, USB being only in the wired position unless &usb_on
 * asked for it), the link skips
 * events up to its peripheral latency (ble_conn_latency();
 * CONFIG_TLSR_BLE_CONN_LATENCY, 44, once the central takes the parameters
 * asked for), and the idle thread suspends the chip between the events it
 * attends (tlsr8278_idle_suspend()); after every suspend the radio's
 * registers and TX power are set again (in Suspend mode the RF transceiver
 * is powered down, DS-TLSR8278 2.5.2; tc32emu
 * keeps them unless TC32EMU_SUSPEND_RF=lost). A key wakes it through its
 * row: the state ends at once and the idle time starts again; the key itself
 * is scanned when the key scan runs again. The knob's pins are not wake pads: a
 * turn while suspended is seen only if the chip is awake for an event at the
 * time.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <tlsr8278_suspend.h>

#include "ble_internal.h"
#include "led_key_matrix_effects.h"
#include "key_matrix_sleep.h"
#include "led_key_matrix.h"
#include "tlsr_analog.h"
#include <tlsr_radio.h>
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD)
#include "tlsr_slots.h"
#endif

#if IS_ENABLED(CONFIG_TLSR_BLE_DEEP_SLEEP)
#define GPIO_PORT0 0x00800580U
#define GPIO_IN    0U
#define GPIO_OEN   2U /* 1: output disabled */
#define GPIO_OUT   3U
#define GPIO_FUNC  6U /* 1: GPIO */
#define GPIO_PORTS 4U /* PA..PD */

#define ANA_PULL0   0x0eU /* PA0-PA3; two bits a pin, four pins a register */
#define ANA_PC_IE   0xc0U /* port C input enable */
#define PULL_1M_UP  1U
#define PULL_10K_UP 3U

#define KSCAN       DT_CHOSEN(zmk_kscan)
#define MODE_SWITCH DT_COMPAT_GET_ANY_STATUS_OKAY(cidoo_mode_switch)
#define BATTERY     DT_COMPAT_GET_ANY_STATUS_OKAY(cidoo_battery)

#if IS_ENABLED(CONFIG_TLSR_BOARD_DEEP_SLEEP_PADS)
/* The board's pads for the deep sleep (its board directory); interrupts locked. */
void board_deep_sleep_pads(void);
#else
struct pad {
	uint8_t port;
	uint8_t bit;
	bool active_high;
};

#define PAD_INIT(node, prop, idx)                                                                  \
	{.port = (DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node, prop, idx)) - GPIO_PORT0) / 8U,            \
	 .bit = BIT(DT_GPIO_PIN_BY_IDX(node, prop, idx)),                                          \
	 .active_high = (DT_GPIO_FLAGS_BY_IDX(node, prop, idx) & GPIO_ACTIVE_LOW) == 0}
#define PAD_ENTRY(node, prop, idx) PAD_INIT(node, prop, idx),

static const struct pad cols[] = {DT_FOREACH_PROP_ELEM(KSCAN, col_gpios, PAD_ENTRY)};
static const struct pad rows[] = {DT_FOREACH_PROP_ELEM(KSCAN, row_gpios, PAD_ENTRY)};

#if DT_NODE_EXISTS(MODE_SWITCH) && !DT_NODE_HAS_PROP(MODE_SWITCH, common_gpios)
static const struct pad switch_pads[] = {PAD_ENTRY(MODE_SWITCH, ble_gpios, 0)
						 PAD_ENTRY(MODE_SWITCH, rf24_gpios, 0)};
#else
static const struct pad switch_pads[] = {};
#endif

#if DT_NODE_EXISTS(BATTERY)
static const struct pad power_pad = PAD_INIT(BATTERY, power_gpios, 0);
#endif

static uint32_t port_base(uint8_t port)
{
	return GPIO_PORT0 + 8U * port;
}

/* Interrupts locked: the analog port is shared with interrupt handlers. */
static void set_pull(const struct pad *p, uint8_t pull)
{
	uint8_t pin = (uint8_t)find_lsb_set(p->bit) - 1U;
	uint8_t reg = ANA_PULL0 + 2U * p->port + pin / 4U;
	uint8_t shift = 2U * (pin % 4U);

	tlsr_analog_write(reg, (tlsr_analog_read(reg) & (uint8_t)~(3U << shift)) |
				       (uint8_t)(pull << shift));
}

static bool reads_high(const struct pad *p)
{
	return (sys_read8(port_base(p->port) + GPIO_IN) & p->bit) != 0U;
}

/* Wake on the level opposite to the one the pad reads now. */
static void wake_on_change(const struct pad *p)
{
	tlsr8278_pad_wakeup(p->port, p->bit, reads_high(p) ? p->bit : 0U, true);
}

/* The input on for a pad the board left with input off (the switch pins).
 * Port C's input enable is analog 0xc0 on the TLSR8278 (DS-TLSR8278 7.1.1.1,
 * Table 7-2); the other ports' is the digital one, at port base + 1. */
static void input_on(const struct pad *p)
{
	if (p->port == 2U) {
		tlsr_analog_write(ANA_PC_IE, tlsr_analog_read(ANA_PC_IE) | p->bit);
	} else {
		uint32_t ie = port_base(p->port) + 1U;

		sys_write8(sys_read8(ie) | p->bit, ie);
	}
}

#endif /* CONFIG_TLSR_BOARD_DEEP_SLEEP_PADS */

#if IS_ENABLED(CONFIG_LED_DRIVER_SPI)
void v75pro_backlight_sleep(void);
#endif

FUNC_NORETURN void ble_deep_sleep(void)
{
	cidoo_backlight_save_pending();
#if IS_ENABLED(CONFIG_LED_DRIVER_SPI)
	v75pro_backlight_sleep();
#endif
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD)
	tlsr_boot_guard_plan_wake();
#endif
	unsigned int key = irq_lock();

	tlsr_radio_stop();
#if IS_ENABLED(CONFIG_TLSR_BOARD_DEEP_SLEEP_PADS)
	board_deep_sleep_pads();
#else
	led_key_matrix_blank();
	struct lkm_colour_pads pads;

	led_key_matrix_colour_pads_low(&pads); /* the LED colour pins low before deep sleep */
	/* Columns high: the pull-up, then the level, the GPIO function, the output. */
	for (size_t i = 0; i < ARRAY_SIZE(cols); i++) {
		uint32_t b = port_base(cols[i].port);

		set_pull(&cols[i], PULL_10K_UP);
		sys_write8(sys_read8(b + GPIO_OUT) | cols[i].bit, b + GPIO_OUT);
		sys_write8(sys_read8(b + GPIO_FUNC) | cols[i].bit, b + GPIO_FUNC);
		sys_write8(sys_read8(b + GPIO_OEN) & (uint8_t)~cols[i].bit, b + GPIO_OEN);
	}
	for (size_t i = 0; i < ARRAY_SIZE(switch_pads); i++) {
		set_pull(&switch_pads[i], PULL_1M_UP);
		input_on(&switch_pads[i]);
	}
	/* 10 ms for the levels to settle; 1 MOhm on the switch pins needs it. */
	k_busy_wait(10000);
	for (size_t i = 0; i < ARRAY_SIZE(rows); i++) {
		wake_on_change(&rows[i]);
	}
	for (size_t i = 0; i < ARRAY_SIZE(switch_pads); i++) {
		wake_on_change(&switch_pads[i]);
	}
#if DT_NODE_EXISTS(BATTERY)
	if (reads_high(&power_pad) != power_pad.active_high) {
		tlsr8278_pad_wakeup(power_pad.port, power_pad.bit,
				    power_pad.active_high ? 0U : power_pad.bit, true);
	}
#endif
#endif /* CONFIG_TLSR_BOARD_DEEP_SLEEP_PADS */
	ARG_UNUSED(key); /* the chip powers down or reboots with interrupts locked */
	tlsr8278_deep_sleep(TLSR8278_WAKEUP_PAD);
}

#endif /* CONFIG_TLSR_BLE_DEEP_SLEEP */

#if IS_ENABLED(CONFIG_TLSR_BLE_LOW_POWER)
static volatile bool pad_woke;
static volatile uint32_t last_active;
static bool low;

void ble_low_power_activity(void)
{
	last_active = k_uptime_get_32();
}

bool ble_low_power_on(void)
{
	return low;
}

/*
 * The idle thread after every suspend, interrupts still locked: the radio set
 * up again (the RF transceiver is powered down in Suspend mode), and a row's wake ends the
 * low-power state.
 */
static void resumed(uint32_t status)
{
	tlsr_radio_resume(ble_tx_level);
	if ((status & TLSR8278_WAKEUP_STATUS_PAD) != 0U) {
		pad_woke = true;
		ble_conn_kick();
	}
}

void ble_low_power_poll(bool link_ready)
{
	uint32_t now = k_uptime_get_32();
	bool want;

	if (pad_woke) {
		pad_woke = false;
		last_active = now;
	}
	want = link_ready && now - last_active >= CONFIG_TLSR_BLE_IDLE_S * 1000U;
	want = want && !ble_usb_host();
	if (want == low) {
		return;
	}
	low = want;
	if (low) {
		key_matrix_sleep(true);
		ble_conn_latency(true);
		tlsr8278_idle_suspend(TLSR8278_WAKEUP_PAD, ble_conn_between_events, resumed);
	} else {
		tlsr8278_idle_suspend(0U, NULL, NULL);
		ble_conn_latency(false);
		key_matrix_sleep(false);
		last_active = now;
	}
}
#else
void ble_low_power_activity(void)
{
}

bool ble_low_power_on(void)
{
	return false;
}

void ble_low_power_poll(bool link_ready)
{
	ARG_UNUSED(link_ready);
}
#endif
