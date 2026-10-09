/*
 * The own BLE stack's side of ZMK's HID: the host's keyboard LED output
 * report (Num Lock, Caps Lock, ...) goes to ZMK's HID indicators when they
 * are built, as the USB output report does. The value belongs to the BLE
 * endpoint of the active profile, the one ZMK selects while BLE or the 2.4G
 * link carries the reports; the 2.4G link's ACKs repeat the byte with every
 * report, so only a change is passed on.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/kernel.h>
#include <zmk/endpoints.h>

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif

#include "ble_internal.h"

void ble_hid_leds(uint8_t leds)
{
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
	struct zmk_hid_led_report_body report = {.leds = leds};
	const struct zmk_endpoint_instance endpoint = {
		.transport = ZMK_TRANSPORT_BLE,
		.ble = {.profile_index = ble_profile_active()},
	};

	if (zmk_hid_indicators_get_profile(endpoint) != leds) {
		zmk_hid_indicators_process_report(&report, endpoint);
	}
#else
	ARG_UNUSED(leds);
#endif
}
