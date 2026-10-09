/*
 * What the rest of the module may ask the own BLE stack (src/ble).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_TLSR_BLE_H_
#define TC32_TLSR_BLE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The stack runs: the mode switch is in the BT position (read once, at boot). */
bool ble_running(void);

/*
 * The links' counters for a host tool: two little-endian 16-bit sizes, then the BLE link's counters of that
 * size and the 2.4G link's (32-bit words each, as in RAM). n octets from at; the number copied.
 */
size_t ble_link_stats_read(size_t at, uint8_t *out, size_t n);

/*
 * TLSR_USB_ON_REQUEST: USB from now until the power goes, in a position of the mode switch that has none at
 * boot. False when USB is up already.
 */
bool ble_usb_request(void);

/*
 * The flash test behind the host's confirm (ble_bond.c): the bond log's two
 * sectors read, erased, written and read back, a compaction of its records.
 * Built with TLSR_BOND_LOG (the BLE stack, or the OTA receiver alone, which
 * rewrites whatever the log holds). 0 on a pass; -EILSEQ when the headers
 * and the sector in use do not read as the load left RAM, or a read back differs from what
 * was erased or written; else the flash call's error (the flash driver
 * reports none, so a fault shows as -EILSEQ). Refused with nothing erased
 * when the headers disagree with the load.
 */
int ble_bond_flash_test(void);

#endif /* TC32_TLSR_BLE_H_ */
