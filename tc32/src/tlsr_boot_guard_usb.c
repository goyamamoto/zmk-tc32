/*
 * The boot guard's healthy mark from USB: the boot is healthy once the host
 * has configured the USB device, which proves the USB driver, the descriptors
 * and the control transfers on this hardware. That is zmk_usb_is_hid_ready():
 * configured, and neither reset nor disconnected since. ZMK's connection state
 * alone also reads HID on a bus suspend, which an idle bus with no host (a
 * charger) gives too.
 *
 * The mark is set at each USB state event while the device is configured:
 * the clearing writes and erases nothing once the count is clear (the counter
 * sector's "nothing to clear" case), so after the first mark of a boot the
 * later ones (a suspend, a resume) only read, and a clearing that failed is
 * tried again at the next event. On a boot that was counted, the first mark
 * erases the counter sector with interrupts off (up to 500 ms by the
 * datasheet, typically tens) while the host may still be sending its HID
 * requests; it is not delayed past the enumeration, since a delay widens the
 * window in which power cycles in quick succession (a KVM, a cable put in and
 * out) count as boots without a healthy one. The clearing
 * runs on the system work queue; ZMK raises the event from the USB stack's
 * status callback, which the TLSR8278 driver calls from its poll timer.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

#include "tlsr_slots.h"

static int on_usb_state(const zmk_event_t *eh)
{
	ARG_UNUSED(eh);

	if (zmk_usb_is_hid_ready()) {
		tlsr_boot_guard_healthy_soon(0U);
	}
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(tlsr_boot_guard_usb, on_usb_state);
ZMK_SUBSCRIPTION(tlsr_boot_guard_usb, zmk_usb_conn_state_changed);
