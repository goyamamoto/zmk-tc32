/*
 * key_matrix_sleep() for a key matrix that ZMK's "zmk,kscan-gpio-matrix"
 * polls (CONFIG_ZMK_KSCAN_MATRIX_POLLING), as on the V75 Pro: the low-power
 * state stops the key scan, keeps every column at its active level, so that
 * a key pulls its row to the column's level, and wakes the suspended chip on
 * the rows.
 *
 * Asleep: the poll stopped (kscan_disable_callback() cancels its work), the
 * columns at their active level, and each row a wake pad on the level
 * opposite to the one it reads (a key going down, or a held key going up).
 * Awake: the pads off, the columns back at their inactive level as the scan
 * leaves them between columns, and the poll started with a scan
 * (kscan_enable_callback()).
 *
 * Where the USB driver polls from its kernel timer (no
 * USB_DC_TELINK_B87_EXTERNAL_POLL), that poll, every millisecond otherwise,
 * runs every 10 ms while asleep (USB_DC_TELINK_B87_SLOW_POLL): the chip's idle
 * suspend needs the next kernel timeout a few milliseconds away, and the
 * poll's would always be the next. With src/usb_poll_timer.c and USB only in
 * the wired position, nothing polls USB in these states, unless &usb_on
 * brought USB up there (TLSR_USB_ON_REQUEST): then Timer1's poll goes on
 * every millisecond until the power goes. The low-power states begin only
 * while no USB host is using the bus.
 *
 * Called from the BLE thread (ble_sleep.c, p24.c). The poll runs on the
 * system work queue, a cooperative thread, so it never stands half way
 * through a scan when this runs.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#if IS_ENABLED(CONFIG_USB_DC_TELINK_B87_SLOW_POLL)
#include <zephyr/drivers/usb/usb_dc_b87.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <tlsr8278_suspend.h>

#include "key_matrix_sleep.h"

#define KSCAN      DT_CHOSEN(zmk_kscan)
#define GPIO_PORT0 0x00800580U

BUILD_ASSERT(DT_NODE_HAS_COMPAT(KSCAN, zmk_kscan_gpio_matrix),
	     "TLSR_GPIO_MATRIX_SLEEP needs the chosen zmk,kscan to be a zmk,kscan-gpio-matrix");
/* The columns are the driven side and the rows the read side only for col2row. */
BUILD_ASSERT(DT_ENUM_HAS_VALUE(KSCAN, diode_direction, col2row),
	     "TLSR_GPIO_MATRIX_SLEEP drives the columns and wakes on the rows: diode-direction col2row");

struct row_pad {
	struct gpio_dt_spec gpio;
	uint8_t port;
	uint8_t bit;
};

#define ROW_PAD(node, prop, idx)                                                                   \
	{.gpio = GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx),                                         \
	 .port = (DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node, prop, idx)) - GPIO_PORT0) / 8U,            \
	 .bit = BIT(DT_GPIO_PIN_BY_IDX(node, prop, idx))},

static const struct gpio_dt_spec cols[] = {
	DT_FOREACH_PROP_ELEM_SEP(KSCAN, col_gpios, GPIO_DT_SPEC_GET_BY_IDX, (,))};
static const struct row_pad rows[] = {DT_FOREACH_PROP_ELEM(KSCAN, row_gpios, ROW_PAD)};
static const struct device *const kscan = DEVICE_DT_GET(KSCAN);
static bool asleep;

void key_matrix_sleep(bool sleep)
{
	if (sleep == asleep || !device_is_ready(kscan)) {
		return;
	}
	asleep = sleep;
	if (sleep) {
		(void)kscan_disable_callback(kscan);
		for (size_t i = 0; i < ARRAY_SIZE(cols); i++) {
			(void)gpio_pin_set_dt(&cols[i], 1);
		}
		/* The rows settle through their pulls (about 1 us; 10 to be sure). */
		k_busy_wait(10);
		for (size_t i = 0; i < ARRAY_SIZE(rows); i++) {
			int high = gpio_pin_get_raw(rows[i].gpio.port, rows[i].gpio.pin);

			tlsr8278_pad_wakeup(rows[i].port, rows[i].bit, high > 0 ? rows[i].bit : 0U, true);
		}
#if IS_ENABLED(CONFIG_USB_DC_TELINK_B87_SLOW_POLL)
		/* The USB poll every 10 ms, so that the chip suspends between its timeouts. */
		usb_dc_b87_slow_poll(true);
#endif
	} else {
#if IS_ENABLED(CONFIG_USB_DC_TELINK_B87_SLOW_POLL)
		usb_dc_b87_slow_poll(false);
#endif
		for (size_t i = 0; i < ARRAY_SIZE(rows); i++) {
			tlsr8278_pad_wakeup(rows[i].port, rows[i].bit, 0U, false);
		}
		for (size_t i = 0; i < ARRAY_SIZE(cols); i++) {
			(void)gpio_pin_set_dt(&cols[i], 0);
		}
		(void)kscan_enable_callback(kscan);
	}
}
