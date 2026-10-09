/*
 * The SPI flash routines: the SPI transport is tlsr_spi_flash_io.S,
 * and these wrappers call it page by page. Plain functions (address, length,
 * buffer), usable before the kernel runs; the Zephyr flash driver
 * (flash_tlsr_spi.c) is a layer on top.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TLSR_SPI_FLASH_H_
#define TLSR_SPI_FLASH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TLSR_SPI_FLASH_PAGE   256U
#define TLSR_SPI_FLASH_SECTOR 4096U

/* Read: command 0x03. */
void tlsr_spi_flash_read(uint32_t addr, uint32_t len, uint8_t *buf);
/* Page program: 0x06 then 0x02 per 256-byte page, each with the busy wait. */
void tlsr_spi_flash_page_program(uint32_t addr, uint32_t len, const uint8_t *buf);
/* Sector erase: 0x06 then 0x20, with the busy wait. */
void tlsr_spi_flash_erase_sector(uint32_t addr);
/* Status read: one byte after the command (0x05 or 0x35). */
uint8_t tlsr_spi_flash_read_status(uint8_t cmd);
/* Status write: 0x06, then 0x01 with one or two bytes, the busy wait. */
void tlsr_spi_flash_write_status(const uint8_t *data, uint32_t len);
/*
 * For GD25LD40C/80C and ZB25WD40B/80B by JEDEC ID: the status register read
 * (0x05), the block-protect bits (mask 0x1c) cleared with a one-byte write
 * when any is set, the register read again. *before and *after get the
 * register (*after equals *before when nothing was written). Returns 1 when
 * it was locked and is unlocked now, 0 when it was not locked, -ENOTSUP for
 * any other ID (nothing written), -EIO when the bits are still set after
 * three writes, -EACCES when the register read as busy or write-enabled
 * before a write (a misread; that write not made).
 */
int tlsr_spi_flash_unlock(uint16_t *before, uint16_t *after);
/* A busy wait on the 16 MHz system timer. */
void tlsr_spi_flash_delay_us(uint32_t us);
/* The JEDEC ID (0x9f): manufacturer, type, capacity << 16. Read once. */
uint32_t tlsr_spi_flash_jedec_id(void);
/* A Zbit flash (mid 0x11325e, 0x12325e, 0x13325e or 0x14325e). */
bool tlsr_spi_flash_is_zbit(void);
/*
 * The two calibration bytes at 0x771c0 set analog 0x09 bits 6:4 and 0x0c
 * bits 2:0 (neither register is documented in DS-TLSR8278); when they are
 * blank or invalid, a Zbit flash gets fixed values (7 and 6) and any other
 * flash nothing. Runs once; later calls do nothing. Call it before the first
 * flash write of a boot (the early boot guard).
 */
void tlsr_spi_flash_trim_calib(void);
/*
 * When an OTA starts, on a Zbit flash only: analog 0x09 |= 0x70, analog
 * 0x0c = (0x0c & 0xf8) | 6, left so until the reboot at the end of the OTA.
 * On any other flash it does nothing.
 */
void tlsr_spi_flash_trim_raise(void);
/*
 * The fields tlsr_spi_flash_trim_raise() found, put back: for an OTA
 * test (tlsr_usb_ota.c), which ends without a reboot. Does nothing when they
 * were not raised.
 */
void tlsr_spi_flash_trim_restore(void);

#endif /* TLSR_SPI_FLASH_H_ */
