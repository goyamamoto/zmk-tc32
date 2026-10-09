/*
 * The CPU the lowest-priority thread gets, measured in the calling thread.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_TLSR_CPU_LEFT_H_
#define TC32_TLSR_CPU_LEFT_H_

#include <stdint.h>

#define TLSR_CPU_LEFT_MS 100 /* the measurement's length */

/* Percent of the CPU the lowest-priority thread got over TLSR_CPU_LEFT_MS; 0xff when it
 * cannot be measured (native_sim). The calling thread runs at that priority meanwhile,
 * interrupts on; the full rate was taken once at boot. */
uint8_t tlsr_cpu_left_percent(void);

/* The measurement in the USB OTA receiver's thread (tlsr_usb_ota.c, CONFIG_TLSR_USB_OTA), which
 * otherwise waits for reports: done(percent) is called from that thread. */
void tlsr_usb_ota_cpu_left_async(void (*done)(uint8_t percent));

#endif /* TC32_TLSR_CPU_LEFT_H_ */
