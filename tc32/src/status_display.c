/*
 * The status display's state (status_display.h): gathered when the &sts key
 * is pressed and handed to the board's part. The battery comes from one
 * measurement (battery_adc.c), the link and profile state from the own
 * BLE and 2.4G stacks (ble.c, p24.c), and the CPU left over from a 100 ms
 * measurement in the USB OTA receiver's thread, which otherwise waits for
 * reports (tlsr_cpu_left.c, tlsr_usb_ota.c), shown once it is in. The display goes off CONFIG_TLSR_STATUS_DISPLAY_MS after the
 * press; a press while it shows starts the time again.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/kernel.h>

#include "status_display.h"
#include "tlsr_cpu_left.h"
#if IS_ENABLED(CONFIG_BATTERY_ADC)
#include "battery_adc.h"
#endif
#if IS_ENABLED(CONFIG_TLSR_BLE)
#include "ble/ble_internal.h"
#endif
#ifdef CONFIG_TLSR_KEPT_LAYER
#include <zmk/keymap.h>
#endif

static struct status_snapshot snap;
static bool shown;

static void hide(struct k_work *work)
{
	ARG_UNUSED(work);
	shown = false;
	status_board_hide();
}
static K_WORK_DELAYABLE_DEFINE(hide_work, hide);

/* From the receiver's thread, once measured. */
static void measured(uint8_t left)
{
	if (shown) {
		snap.cpu_left = left;
		status_board_show(&snap);
	}
}

static void gather(void)
{
	memset(&snap, 0, sizeof(snap));
	snap.cpu_left = 0xffU;
#if IS_ENABLED(CONFIG_BATTERY_ADC)
	{
		struct cidoo_battery_reading b;

		if (cidoo_battery_read(&b) == 0) {
			snap.battery_known = true;
			snap.battery_percent = b.percent;
			snap.power_in = b.power;
			snap.charging = b.charging;
		} else {
			snap.power_in = cidoo_battery_power_in();
		}
	}
#endif
#if IS_ENABLED(CONFIG_TLSR_BLE)
	{
		struct ble_status st;

		ble_status(&st);
		snap.link = st.p24 ? STATUS_LINK_P24 : (st.ble ? STATUS_LINK_BLE : STATUS_LINK_WIRED);
		snap.ble_active = st.active;
		snap.ble_bonded = st.bonded;
		snap.ble_connected = st.connected;
		snap.ble_ready = st.ready;
		snap.ble_pairing = st.pairing;
		snap.p24_linked = st.p24_linked;
		snap.p24_pairing = st.p24_pairing;
	}
#endif
#ifdef CONFIG_TLSR_KEPT_LAYER
	snap.mode_known = true;
	snap.mac_mode = zmk_keymap_layer_active((zmk_keymap_layer_id_t)CONFIG_TLSR_KEPT_LAYER);
#endif
}

void status_display_show(void)
{
	gather();
	shown = true;
	status_board_show(&snap);
#if IS_ENABLED(CONFIG_TLSR_USB_OTA)
	tlsr_usb_ota_cpu_left_async(measured);
#endif
	(void)k_work_reschedule(&hide_work, K_MSEC(CONFIG_TLSR_STATUS_DISPLAY_MS));
}
