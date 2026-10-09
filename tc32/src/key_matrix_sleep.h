/*
 * The key matrix in the low-power states of the BLE and 2.4G links: the
 * V21's LED and key matrix (led_key_matrix.c) or a board's GPIO matrix
 * that ZMK's kscan-gpio-matrix polls (gpio_matrix_sleep.c).
 *
 * Asleep: no key scan, every column at its active level, and each row a
 * wake pad for the chip's suspend on the level opposite to the one it reads
 * (a key going down, or up). Awake again: the pads off and the scan running,
 * a scan first.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TC32_KEY_MATRIX_SLEEP_H_
#define TC32_KEY_MATRIX_SLEEP_H_

#include <stdbool.h>

#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_CIDOO_LED_KEY_MATRIX)
#include "led_key_matrix.h"

static inline void key_matrix_sleep(bool sleep)
{
	led_key_matrix_sleep(sleep);
}
#elif IS_ENABLED(CONFIG_TLSR_GPIO_MATRIX_SLEEP)
void key_matrix_sleep(bool sleep);
#else
static inline void key_matrix_sleep(bool sleep)
{
	(void)sleep;
}
#endif

#endif /* TC32_KEY_MATRIX_SLEEP_H_ */
