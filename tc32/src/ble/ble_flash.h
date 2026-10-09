/*
 * The own BLE stack's gate for flash writes (flash_tlsr_spi.c): the
 * radio is kept idle while the flash is programmed or erased.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TLSR_BLE_FLASH_H_
#define TLSR_BLE_FLASH_H_

#include <stdbool.h>

/*
 * hold true: no connection or advertising event starts until the matching
 * false, and it returns once none is on air (connection events skipped
 * meanwhile count as missed; the central keeps the connection up to its
 * supervision timeout). Nests. Does nothing before the kernel runs or while
 * the stack is not running.
 */
void ble_flash_radio_hold(bool hold);

#endif /* TLSR_BLE_FLASH_H_ */
