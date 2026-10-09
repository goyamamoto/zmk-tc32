/*
 * The places of the SPI flash transport's functions in tlsr_spi_flash_io.S
 * (496 bytes, SHA-256 in tlsr_spi_flash_io.json) and their C prototypes.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#include <stdint.h>

extern const uint8_t tlsr_spi_flash_io[496];
#define TLSR_SPI_FLASH_IO_SIZE 496u

#define TLSR_SPI_FLASH_IO_OFF_SEND_ADDR 0x0u
#define TLSR_SPI_FLASH_IO_OFF_SEND_CMD 0x40u
#define TLSR_SPI_FLASH_IO_OFF_WAIT_READY 0x6cu
#define TLSR_SPI_FLASH_IO_OFF_READ_CMD 0xb0u
#define TLSR_SPI_FLASH_IO_OFF_WRITE_CMD 0x160u
#define TLSR_SPI_FLASH_IO_OFF_DELAY_US 0x1dcu

/* The prototypes (arguments as the .S takes them). Call through a pointer with bit 0 set:
 * (fn)((uintptr_t)tlsr_spi_flash_io + TLSR_SPI_FLASH_IO_OFF_x + 1). */
typedef void (*tlsr_spi_flash_io_read_t)(unsigned char cmd, unsigned long addr, unsigned char with_addr,
				       unsigned char zeros, unsigned char *buf, unsigned long len);
typedef void (*tlsr_spi_flash_io_write_t)(unsigned char cmd, unsigned long addr, unsigned char with_addr,
					unsigned char *buf, unsigned long len);
typedef void (*tlsr_spi_flash_io_delay_t)(unsigned long us);
#define TLSR_SPI_FLASH_IO_FN(type, off) ((type)((uintptr_t)tlsr_spi_flash_io + (off) + 1u))
