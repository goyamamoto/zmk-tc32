/*
 * The Telink TLSR8278 USB device controller driver's board interface.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_USB_DC_B87_H_
#define ZEPHYR_INCLUDE_DRIVERS_USB_USB_DC_B87_H_

#include <stdbool.h>

/*
 * One poll of the controller, for CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL: the
 * board calls it every millisecond from its own timer interrupt. Call it
 * from an interrupt only: the USB stack's handlers it runs can sleep or
 * yield when not in an interrupt. It does nothing before usb_dc_attach() and
 * while the chip is in suspend.
 */
void usb_dc_b87_poll(void);

/*
 * For CONFIG_USB_DC_TELINK_B87_SLOW_POLL: true polls every
 * CONFIG_USB_DC_TELINK_B87_SLOW_POLL_MS instead of every millisecond, for a
 * board's low-power state while no host uses the bus; false polls every
 * millisecond again. Call it from a thread.
 */
void usb_dc_b87_slow_poll(bool slow);

/*
 * The driver stops its poll (its k_timer, and the board's with
 * CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL) while the chip is suspended
 * (entering true) and polls at once on the wake (false).
 */
void usb_dc_b87_chip_suspend(bool entering);

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_USB_DC_B87_H_ */
