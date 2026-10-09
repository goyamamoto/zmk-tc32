/*
 * Passkey Entry for the own BLE stack (TLSR_BLE_SC_PASSKEY): while a
 * pairing asks for the passkey the host shows, the digits (the number row
 * or the keypad), Backspace, Enter (either one) and Escape typed on the
 * keyboard go to ble_smp.c instead of the HID reports; every other key
 * passes. ZMK's keycode events reach this listener before its HID
 * listener.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/kernel.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

#include "ble_internal.h"

/* The key of a passkey a HID keyboard usage is: a digit (the number row 0x1e-0x27 or the keypad
 * 0x59-0x62), Enter (either one), Backspace or Escape; 0xff for any other usage. */
static uint8_t passkey_key(uint32_t usage)
{
	if (usage - 0x1eU < 10U) {
		return (uint8_t)((usage - 0x1dU) % 10U);
	}
	if (usage - 0x59U < 10U) {
		return (uint8_t)((usage - 0x58U) % 10U);
	}
	switch (usage) {
	case 0x28U:
	case 0x58U:
		return BLE_PASSKEY_ENTER;
	case 0x2aU:
		return BLE_PASSKEY_BACKSPACE;
	case 0x29U:
		return BLE_PASSKEY_ESCAPE;
	default:
		return 0xffU;
	}
}

static int on_keycode(const zmk_event_t *eh)
{
	const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
	uint8_t key;

	if (ev == NULL || !ble_smp_passkey_wanted() || ev->usage_page != HID_USAGE_KEY) {
		return ZMK_EV_EVENT_BUBBLE;
	}
	key = passkey_key(ev->keycode);
	if (key == 0xffU) {
		return ZMK_EV_EVENT_BUBBLE;
	}
	if (ev->state) {
		ble_smp_passkey_key(key);
	}
	return ZMK_EV_EVENT_HANDLED;
}

ZMK_LISTENER(tlsr_ble_passkey, on_keycode);
ZMK_SUBSCRIPTION(tlsr_ble_passkey, zmk_keycode_state_changed);
