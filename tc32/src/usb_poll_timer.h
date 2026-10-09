/*
 * The USB controller's 1 ms poll from Timer1's interrupt (usb_poll_timer.c).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TC32_USB_POLL_TIMER_H_
#define TC32_USB_POLL_TIMER_H_

#include <stdbool.h>

/* Timer1 started (the next poll a millisecond from now) or stopped. */
void usb_poll_timer_run(bool run);

#endif /* TC32_USB_POLL_TIMER_H_ */
