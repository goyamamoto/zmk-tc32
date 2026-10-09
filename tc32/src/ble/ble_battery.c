/*
 * The battery level for the GATT Battery service, measured in the BLE thread
 * (dts/bindings/misc/cidoo,battery.yaml):
 *
 * - A measurement: src/battery_adc.c (the ADC reading, percentage and
 *   charger pins).
 * - The level kept: the first five samples set it; on battery it only goes
 *   down, by 1 after five lower samples in a row; charging it only goes up,
 *   by 1 after six higher samples in a row; with the charge complete it is
 *   100 after four samples. This level is sent, not the last sample's
 *   percentage.
 * - The schedule: the first sample when the stack starts, then 80 ms, 80 ms,
 *   2 s and 2 s later, then every 4 s.
 * - Critical (with TLSR_BLE_DEEP_SLEEP): on battery at 0 % and at or below
 *   the empty point, after the first five samples, the chip goes into deep
 *   sleep (ble_sleep.c).
 *
 * Not done here yet: the low-battery indicator.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "ble_internal.h"
#include "battery_adc.h"

#if IS_ENABLED(CONFIG_BATTERY_ADC)

#define FIRST_RUNS 5U

BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC % 1000 == 0, "a millisecond is a whole number of kernel ticks");

/* from a sample to the next, in kernel ticks */
static const uint32_t schedule[] = {BLE_MS_TICKS(80), BLE_MS_TICKS(80), BLE_MS_TICKS(2000), BLE_MS_TICKS(2000),
				    BLE_MS_TICKS(4000)};

static bool started;
static uint8_t level;
static uint8_t samples;
static uint8_t run;
static uint8_t run_kind; /* what run counts: 0 on battery, 1 charging, 2 complete */
static uint8_t step;
static uint32_t sampled_at; /* the last sample's time, kernel ticks */
static uint32_t gap;        /* from it to the next sample */

static void sample(void)
{
	struct cidoo_battery_reading b;

	if (cidoo_battery_read(&b) != 0) {
		return;
	}
	uint8_t pct = b.percent;
	uint8_t kind = !b.power ? 0U : (b.charging ? 1U : 2U);

	if (kind != run_kind) {
		run_kind = kind;
		run = 0;
	}
	if (samples < FIRST_RUNS) {
		samples++;
		level = pct;
		run = 0;
	} else if (kind == 0U) {
		run = pct < level ? run + 1U : 0U;
		if (run >= 5U) {
			level--;
			run = 0;
		}
	} else if (kind == 1U) {
		run = pct > level ? run + 1U : 0U;
		if (run >= 6U) {
			level++;
			run = 0;
		}
	} else {
		run++;
		if (run >= 4U) {
			level = 100;
			run = 0;
		}
	}
#if IS_ENABLED(CONFIG_TLSR_BLE_DEEP_SLEEP)
	/*
	 * Critical battery: on battery, 0 %, at or below empty, after the first
	 * five; and not while a USB host has the keyboard over the cable
	 * (ble_usb_host(), the test of the advertising, 2.4G and low-power
	 * gates). Both rest on the power-in pin: on battery (kind 0) it reads no
	 * power in.
	 */
	if (kind == 0U && level == 0U && b.mv <= cidoo_battery_empty_mv() &&
	    samples >= FIRST_RUNS && !ble_usb_host()) {
		ble_deep_sleep();
	}
#endif
}

void ble_battery_poll(void)
{
	uint32_t now = (uint32_t)k_uptime_ticks();

	if (started) {
		ble_att_battery_level(level);
	}
	if (!started) {
		started = true;
	} else if (now - sampled_at < gap) {
		return;
	}
	sample();
	ble_att_battery_level(level);
	sampled_at = now;
	gap = schedule[step];
	if (step < ARRAY_SIZE(schedule) - 1U) {
		step++;
	}
}

#else

void ble_battery_poll(void)
{
}

#endif
