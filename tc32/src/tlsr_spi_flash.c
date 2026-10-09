/*
 * Wrappers over the SPI flash transport (tlsr_spi_flash_io.S):
 * read, page program, sector erase, status read and write; and the fields
 * of analog 0x09 and 0x0c that tlsr_spi_flash_trim_calib() and
 * tlsr_spi_flash_trim_raise() set. Every SPI transaction goes to
 * the transport.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "led_key_matrix.h"
#include "tlsr_spi_flash.h"
#include "tlsr_spi_flash_io.h"

#define FLASH_CMD_READ         0x03U
#define FLASH_CMD_PAGE_PROG    0x02U
#define FLASH_CMD_SECT_ERASE   0x20U
#define FLASH_CMD_WRITE_STATUS 0x01U
#define FLASH_CMD_READ_STATUS  0x05U

#define ANALOG_PORT_ADDR  0x008000b8U
#define ANALOG_PORT_DATA  0x008000b9U
#define ANALOG_PORT_CTRL  0x008000baU
#define ANALOG_PORT_BUSY  BIT(0)
#define ANALOG_PORT_WRITE BIT(5)
#define ANALOG_PORT_START BIT(6)

static inline tlsr_spi_flash_io_read_t io_read(void)
{
	return TLSR_SPI_FLASH_IO_FN(tlsr_spi_flash_io_read_t, TLSR_SPI_FLASH_IO_OFF_READ_CMD);
}

static inline tlsr_spi_flash_io_write_t io_write(void)
{
	return TLSR_SPI_FLASH_IO_FN(tlsr_spi_flash_io_write_t, TLSR_SPI_FLASH_IO_OFF_WRITE_CMD);
}

void tlsr_spi_flash_read(uint32_t addr, uint32_t len, uint8_t *buf)
{
	io_read()(FLASH_CMD_READ, addr, 1, 0, buf, len);
}

/*
 * A write through the transport keeps interrupts off (0x800643 = 0) until the
 * flash reads ready, and a sector erase takes up to hundreds of milliseconds.
 * A backlight multiplexed from an interrupt would keep the column lit at that
 * moment lit throughout, so it is blanked first, with interrupts already off
 * so that its interrupt cannot light it again in between (src/led_key_matrix.c).
 */
static void io_write_blanked(uint8_t cmd, uint32_t addr, uint8_t addr_en, uint8_t *data,
			     uint32_t len)
{
#ifdef CONFIG_CIDOO_LED_KEY_MATRIX
	unsigned int key = irq_lock();

	led_key_matrix_blank();
	io_write()(cmd, addr, addr_en, data, len);
	irq_unlock(key);
#else
	io_write()(cmd, addr, addr_en, data, len);
#endif
}

/*
 * The data to program is fetched while the SPI transaction to the flash is
 * open, and a read of the flash itself cannot be served then: a buffer that
 * resides in flash (below the SRAM at 0x840000, DS-TLSR8278 2.1.1) is copied
 * to the stack a piece at a time first, for every caller (both boot guard
 * stages, the OTA receiver, the Zephyr driver).
 */
#define RAM_ADDR_BASE 0x840000U

static bool in_flash(const void *p)
{
	return (uintptr_t)p < RAM_ADDR_BASE;
}

/* A flash-resident buffer goes in pieces of this size. */
#define FLASH_DATA_PIECE 32U

void tlsr_spi_flash_page_program(uint32_t addr, uint32_t len, const uint8_t *buf)
{
	/* The first write ends at the page boundary, the rest are whole pages. */
	uint32_t ns = TLSR_SPI_FLASH_PAGE - (addr & (TLSR_SPI_FLASH_PAGE - 1U));
	const bool copy = in_flash(buf);

	do {
		uint32_t nw = len > ns ? ns : len;

		if (copy) {
			/* Each piece its own page program (the same page): 32 bytes of stack, no more. */
			for (uint32_t off = 0; off < nw; off += FLASH_DATA_PIECE) {
				uint8_t ram[FLASH_DATA_PIECE];
				uint32_t n = MIN(FLASH_DATA_PIECE, nw - off);

				memcpy(ram, buf + off, n);
				io_write_blanked(FLASH_CMD_PAGE_PROG, addr + off, 1, ram, n);
			}
		} else {
			io_write_blanked(FLASH_CMD_PAGE_PROG, addr, 1, (uint8_t *)buf, nw);
		}
		ns = TLSR_SPI_FLASH_PAGE;
		addr += nw;
		buf += nw;
		len -= nw;
	} while (len > 0U);
}

void tlsr_spi_flash_erase_sector(uint32_t addr)
{
	io_write_blanked(FLASH_CMD_SECT_ERASE, addr, 1, NULL, 0);
}

uint8_t tlsr_spi_flash_read_status(uint8_t cmd)
{
	uint8_t status = 0;

	io_read()(cmd, 0, 0, 0, &status, 1);
	return status;
}

void tlsr_spi_flash_write_status(const uint8_t *data, uint32_t len)
{
	uint8_t ram[2] = {data[0], len > 1U ? data[1] : 0U}; /* never fetched from flash mid-transaction */

	io_write_blanked(FLASH_CMD_WRITE_STATUS, 0, 0, ram, len > 2U ? 2U : len);
}

/*
 * The parts handled, by JEDEC ID: GD25LD40C, GD25LD80C, ZB25WD40B and
 * ZB25WD80B (0x13325e and 0x14325e are taken as the Zbit B parts). Each has
 * an 8-bit status register with the block-protect bits in mask 0x1c, written
 * with one byte. A 128 KB part cannot hold the layout (slot B at 0x20000,
 * settings from 0x40000, the calibration at 0x771c0), so none is listed.
 * Any other ID: nothing is written.
 */
static bool status_layout(uint32_t mid, uint16_t *mask)
{
	switch (mid) {
	case 0x1360c8U: /* GD25LD40C */
	case 0x1460c8U: /* GD25LD80C */
	case 0x13325eU: /* ZB25WD40B */
	case 0x14325eU: /* ZB25WD80B */
		*mask = 0x1cU;
		return true;
	default:
		return false;
	}
}

static uint16_t read_status(void)
{
	return tlsr_spi_flash_read_status(FLASH_CMD_READ_STATUS);
}

int tlsr_spi_flash_unlock(uint16_t *before, uint16_t *after)
{
	uint16_t mask;
	uint8_t data[1];
	uint16_t status;

	if (!status_layout(tlsr_spi_flash_jedec_id(), &mask)) {
		*before = read_status();
		*after = *before;
		return -ENOTSUP;
	}
	status = read_status();
	*before = status;
	*after = status;
	if ((status & mask) == 0U) {
		return 0;
	}
	/*
	 * Up to three times: the register written with the mask's bits cleared,
	 * then read back. Before each write: WIP (bit 0) or WEL (bit 1) read as
	 * set means a misread (0xff from a flash that did not answer), and
	 * writing that back would set SRP and, on the Puya parts, the OTP lock
	 * bits; nothing is written then.
	 */
	for (int i = 0; i < 3; i++) {
		if ((*after & 0x03U) != 0U) {
			return -EACCES;
		}
		status = *after & (uint16_t)~mask;
		data[0] = (uint8_t)status;
		tlsr_spi_flash_write_status(data, 1U);
		*after = read_status();
		if ((*after & mask) == 0U) {
			return 1;
		}
	}
	return -EIO;
}

void tlsr_spi_flash_delay_us(uint32_t us)
{
	TLSR_SPI_FLASH_IO_FN(tlsr_spi_flash_io_delay_t, TLSR_SPI_FLASH_IO_OFF_DELAY_US)(us);
}

/*
 * Analog 0x09 bits 6:4 and 0x0c bits 2:0. DS-TLSR8278 documents neither
 * register, so nothing is known to keep the flash the CPU fetches its code
 * from readable while they change: the whole read-modify-write of each
 * register runs from .ram_code with interrupts off across it. No code is
 * fetched from flash between a write and the return (a hang there would come
 * before the boot guard counts the boot), and no handler takes the analog
 * port between an address and its data.
 *
 * The analog port (0x8000b8-0x8000ba) is not documented in DS-TLSR8278
 * either: the accessors write the address, the data and the control byte,
 * and wait for bit 0 of the control byte to clear.
 */
#define TRIM_RAM_CODE __attribute__((noinline, section(".ram_code.tlsr_flash_trim")))

static TRIM_RAM_CODE uint8_t trim_analog_read(uint8_t addr)
{
	uint8_t v;

	sys_write8(addr, ANALOG_PORT_ADDR);
	sys_write8(ANALOG_PORT_START, ANALOG_PORT_CTRL);
	while ((sys_read8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	v = sys_read8(ANALOG_PORT_DATA);
	sys_write8(0U, ANALOG_PORT_CTRL);
	return v;
}

static TRIM_RAM_CODE void trim_analog_write(uint8_t addr, uint8_t value)
{
	sys_write8(addr, ANALOG_PORT_ADDR);
	sys_write8(value, ANALOG_PORT_DATA);
	sys_write8(ANALOG_PORT_START | ANALOG_PORT_WRITE, ANALOG_PORT_CTRL);
	while ((sys_read8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	sys_write8(0U, ANALOG_PORT_CTRL);
}

/* 0x09 bits 6:4 from ldo, then 0x0c bits 2:0 from dcdc, each read, masked and written. */
static TRIM_RAM_CODE void trim_set(uint8_t ldo, uint8_t dcdc)
{
	unsigned int key = irq_lock();

	trim_analog_write(0x09, (trim_analog_read(0x09) & 0x8fU) | (uint8_t)((ldo & 0x07U) << 4));
	trim_analog_write(0x0c, (trim_analog_read(0x0c) & 0xf8U) | (dcdc & 0x07U));
	irq_unlock(key);
}

/* The two registers as they are, read together (no writer between them). */
static TRIM_RAM_CODE void trim_get(uint8_t *v09, uint8_t *v0c)
{
	unsigned int key = irq_lock();

	*v09 = trim_analog_read(0x09);
	*v0c = trim_analog_read(0x0c);
	irq_unlock(key);
}

#define FLASH_CMD_JEDEC_ID 0x9fU
#define ZBIT_TRIM_09 0x07U /* for 0x09 bits 6:4 */
#define ZBIT_TRIM_0C  0x06U /* for 0x0c bits 2:0 */

static uint32_t jedec_id;
static bool jedec_id_read;
static bool trim_calibrated;

uint32_t tlsr_spi_flash_jedec_id(void)
{
	if (!jedec_id_read) {
		uint8_t id[3] = {0, 0, 0};

		/* The JEDEC ID: three bytes into a little-endian word. */
		io_read()(FLASH_CMD_JEDEC_ID, 0, 0, 0, id, 3);
		jedec_id = (uint32_t)id[0] | ((uint32_t)id[1] << 8) | ((uint32_t)id[2] << 16);
		jedec_id_read = true;
	}
	return jedec_id;
}

/* True for every Zbit JEDEC ID: 0x11325e, 0x12325e, 0x13325e, 0x14325e. */
bool tlsr_spi_flash_is_zbit(void)
{
	uint32_t mid = tlsr_spi_flash_jedec_id();

	return mid == 0x13325eU || mid == 0x14325eU || mid == 0x11325eU || mid == 0x12325eU;
}

/*
 * The unit's calibration bytes: two bytes at 0x771c0, a fixed address
 * whatever the flash ID says; these keyboards' flash holds them there.
 */
#define CALIB_SECTOR 0x77000U

static uint16_t trim_calib_value(void)
{
	uint8_t v[2] = {0, 0};

	tlsr_spi_flash_read(CALIB_SECTOR + 0x1c0U, 2, v);
	return (uint16_t)v[0] | ((uint16_t)v[1] << 8);
}

void tlsr_spi_flash_trim_calib(void)
{
	if (trim_calibrated) {
		return;
	}
	trim_calibrated = true;
	/* Blank or invalid bytes: fixed values on a Zbit flash, nothing on any other. */
	uint16_t calib = trim_calib_value();

	if (calib == 0xffffU || (calib & 0xf8f8U) != 0U) {
		if (tlsr_spi_flash_is_zbit()) {
			trim_set(ZBIT_TRIM_09, ZBIT_TRIM_0C);
		}
	} else {
		/* The byte at 0x771c1 into 0x09's field, 0x771c0's into 0x0c's (both 0-7 here). */
		trim_set((uint8_t)(calib >> 8), (uint8_t)calib);
	}
}

static bool trim_raised;
static uint8_t trim_saved[2];

void tlsr_spi_flash_trim_raise(void)
{
	/* Only on a Zbit flash; the fields as found are saved once for the restore. */
	if (!tlsr_spi_flash_is_zbit()) {
		return;
	}
	if (!trim_raised) {
		trim_get(&trim_saved[0], &trim_saved[1]);
		trim_raised = true;
	}
	/* 0x09 | 0x70 and 0x0c's field at 6. */
	trim_set(0x07U, 0x06U);
}

void tlsr_spi_flash_trim_restore(void)
{
	if (!trim_raised) {
		return;
	}
	/* The trim fields only (the masks the calibration writes with); the other bits stay. */
	trim_set((uint8_t)(trim_saved[0] >> 4), trim_saved[1]);
	trim_raised = false;
}
