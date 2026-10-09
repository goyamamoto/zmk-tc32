/*
 * ZMK's BLE transport on the own stack: the functions ZMK's endpoints call
 * for BLE (zmk/hog.h, zmk/ble.h; app/src/endpoints.c takes them with
 * TLSR_BLE as with ZMK_BLE), and the profile calls of ZMK's &bt behaviour
 * (app/src/behaviors/behavior_bt.c, built with TLSR_BLE too).
 *
 * - The endpoint: in the BT position of the mode switch BLE is the
 *   preferred transport (ZMK falls back to USB while the host is not
 *   ready). The profile counts as connected once the link is encrypted
 *   and the host has turned the keyboard report's notifications on; each
 *   change re-selects the endpoint on the system work queue.
 * - The boot guard: with the USB healthy rule, a bonded central's encrypted
 *   link with the keyboard report's notifications on marks the boot healthy
 *   too (once per connection), as the host configuring USB does: a BLE-only
 *   session (a charger or the battery, no host on the cable) would otherwise
 *   count every boot and go back to the other slot.
 * - Keys and the knob: the time of each key press or release and of each
 *   knob step (ZMK's position and sensor events, in every position of the
 *   mode switch) goes to tc32_rng_add_event(), with the system timer.
 * - Reports: ZMK's thread queues each report (no coalescing, so a quick
 *   tap is two notifications), the BLE thread sends them in order as
 *   notifications while the link has room, and drops the queue when the
 *   connection ends.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/hog.h>

#include "ble_internal.h"
#if IS_ENABLED(CONFIG_BATTERY_ADC)
#include "battery_adc.h"
#endif
#if IS_ENABLED(CONFIG_ZMK_USB)
#include <zmk/usb.h>
#endif
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_HEALTHY_USB)
#include "tlsr_slots.h"
#endif

#define REPORT_KEYBOARD BLE_ZMK_REPORT_KEYBOARD
#define REPORT_CONSUMER BLE_ZMK_REPORT_CONSUMER

struct report {
	uint8_t kind;
	uint8_t len;
	uint8_t data[sizeof(struct zmk_hid_consumer_report_body)];
};

BUILD_ASSERT(sizeof(struct zmk_hid_keyboard_report_body) == 8, "8-octet keyboard reports");

K_MSGQ_DEFINE(reports, sizeof(struct report), CONFIG_TLSR_BLE_REPORT_QUEUE, 1);

static bool ready;
static struct report pending;
static bool have_pending;
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_HEALTHY_USB)
static bool healthy_told; /* since the link got ready, tlsr_boot_guard_healthy_soon() was called */
#endif

static void reselect(struct k_work *work)
{
	ARG_UNUSED(work);
	zmk_endpoints_reselect();
}
static K_WORK_DEFINE(reselect_work, reselect);

static void prefer_ble(struct k_work *work)
{
	ARG_UNUSED(work);
	zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_BLE);
}
static K_WORK_DEFINE(prefer_work, prefer_ble);

void ble_zmk_mode_ble(void)
{
	k_work_submit(&prefer_work);
}

bool ble_zmk_take(uint8_t *kind, uint8_t *data, uint8_t *len)
{
	struct report r;

	if (k_msgq_get(&reports, &r, K_NO_WAIT) != 0) {
		return false;
	}
	*kind = r.kind;
	*len = r.len;
	memcpy(data, r.data, r.len);
	return true;
}

void ble_zmk_p24_ready(bool now)
{
	if (now == ready) {
		return;
	}
	ready = now;
	if (!ready) {
		k_msgq_purge(&reports);
	}
	k_work_submit(&reselect_work);
}

bool zmk_ble_active_profile_is_connected(void)
{
	return ready;
}

int zmk_ble_active_profile_index(void)
{
	return ble_profile_active();
}

int zmk_ble_prof_select(uint8_t index)
{
	if (!ble_running()) {
		return -ENOTSUP;
	}
	if (index >= CONFIG_TLSR_BLE_PROFILES) {
		return -ERANGE;
	}
	if (index != ble_profile_selected()) {
		ble_profile_request((int8_t)index, 0, false);
	}
	return 0;
}

int zmk_ble_prof_next(void)
{
	return zmk_ble_prof_select((ble_profile_selected() + 1U) % CONFIG_TLSR_BLE_PROFILES);
}

int zmk_ble_prof_prev(void)
{
	return zmk_ble_prof_select((ble_profile_selected() + CONFIG_TLSR_BLE_PROFILES - 1U) %
				   CONFIG_TLSR_BLE_PROFILES);
}

void zmk_ble_clear_bonds(void)
{
	if (ble_running()) {
		/* the profile a select just asked for, as &bt's macros chain them */
		ble_profile_request(-1, BIT(ble_profile_selected()), false);
	}
}

void zmk_ble_clear_all_bonds(void)
{
	if (ble_running()) {
		ble_profile_request(-1, BIT_MASK(CONFIG_TLSR_BLE_PROFILES), false);
	}
}

int zmk_ble_prof_disconnect(uint8_t index)
{
	if (!ble_running()) {
		return -ENOTSUP;
	}
	if (index == ble_profile_selected()) {
		ble_profile_request(-1, 0, true);
	}
	return 0;
}

static int queue(uint8_t kind, const void *data, uint8_t len)
{
	struct report r = {.kind = kind, .len = len};

	if (!ready) {
		return -ENOTCONN;
	}
	memcpy(r.data, data, len);
	if (k_msgq_put(&reports, &r, K_NO_WAIT) != 0) {
		return -ENOBUFS;
	}
	ble_conn_kick();
	return 0;
}

int zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *body)
{
	return queue(REPORT_KEYBOARD, body, sizeof(*body));
}

int zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *body)
{
	return queue(REPORT_CONSUMER, body, sizeof(*body));
}

/* The BLE thread: the reports ZMK queued, as far as the link takes them. */
void ble_zmk_flush(void)
{
	bool now = ble_att_ready();

	if (now != ready) {
		ready = now;
		if (!ready) {
			k_msgq_purge(&reports);
			have_pending = false;
		}
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_HEALTHY_USB)
		healthy_told = false;
#endif
		k_work_submit(&reselect_work);
	}
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_HEALTHY_USB)
	if (ready && !healthy_told && ble_link_bonded()) {
		/* a bonded host has the HID service; at a pairing the bond can come after the service is ready */
		healthy_told = true;
		tlsr_boot_guard_healthy_soon(0U);
	}
#endif
	for (;;) {
		int err;

		if (!have_pending) {
			if (k_msgq_get(&reports, &pending, K_NO_WAIT) != 0) {
				return;
			}
			have_pending = true;
		}
		err = pending.kind == REPORT_KEYBOARD ? ble_att_notify_keyboard(pending.data)
						      : ble_att_notify_consumer(pending.data, pending.len);
		if (err == -ENOBUFS) {
			return; /* no room now (the TX ring's report slots full, an encryption pause): a later pass */
		}
		have_pending = false;
	}
}

/* Power in from the cable: the charger's power-in pin, or, on a board without one, ZMK's USB state. */
bool ble_cable_power(void)
{
#if IS_ENABLED(CONFIG_BATTERY_ADC)
	return cidoo_battery_power_in();
#elif IS_ENABLED(CONFIG_ZMK_USB)
	return zmk_usb_is_hid_ready();
#else
	return false;
#endif
}

/*
 * A USB host has the keyboard over the cable: ZMK's HID ready and power in. ZMK keeps its HID ready through a bus
 * suspend, and a pulled cable leaves the bus looking suspended (the controller reports no disconnect), so the
 * power-in pin tells a sleeping host from a pulled cable.
 */
bool ble_usb_host(void)
{
#if IS_ENABLED(CONFIG_ZMK_USB)
	return zmk_usb_is_hid_ready() && ble_cable_power();
#else
	return false;
#endif
}

/* Keys and the knob: the low-power state's idle time starts again (ble_sleep.c), and a link that has stopped
 * looking for its host while a USB host has the keyboard looks again (ble.c, p24.c); the time goes to tc32_rng. */
static int on_activity(const zmk_event_t *eh)
{
	tc32_rng_add_event(k_cycle_get_32(), as_zmk_position_state_changed(eh) != NULL ? TC32_RNG_SRC_KEY
										      : TC32_RNG_SRC_KNOB);
	ble_low_power_activity();
	ble_adv_activity();
#if IS_ENABLED(CONFIG_TLSR_P24)
	p24_activity();
#endif
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(tlsr_ble_activity, on_activity);
ZMK_SUBSCRIPTION(tlsr_ble_activity, zmk_position_state_changed);
ZMK_SUBSCRIPTION(tlsr_ble_activity, zmk_sensor_event);
