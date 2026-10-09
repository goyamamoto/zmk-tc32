/*
 * tc32_rng's hooks (zephyr-tc32 include/zephyr/random/tc32_rng.h) on the
 * keyboard's flash (CONFIG_TLSR_RNG_STORE).
 *
 * tc32_rng_device_id(): the 128-bit unique ID the flash is preloaded with
 * (DS-TLSR8278 2.1.3), read with the flash's command 0x4b, four dummy
 * octets, then the 16 octets, through the SPI flash transport's read
 * (src/tlsr_spi_flash_io.S: code in RAM, the interrupt enable 0x800643
 * cleared for the transaction and written back). The GD25LD and ZB25WD
 * parts of these boards answer 0x4b so.
 *
 * tc32_rng_seed_load() and tc32_rng_seed_store(): a log of records in the
 * board's rng_seed_partition, two 4 KB sectors, so that a store is a page
 * program and an erase comes once every 127 stores:
 * - A sector: a header slot (magic, then the sequence number, in its first
 *   8 octets), then 127 slots of 32 octets. The sector in use is the one
 *   with the magic; with both, the one whose sequence number is the newer
 *   (compared as serial numbers).
 * - A slot: the 20-octet record, octets 20-30 left erased and octet 31 the
 *   commit mark, 0x00, programmed only once the record reads back as
 *   written. Slots are used in order: a store takes the first one whose 32
 *   octets read 0xff after the last one that does not, so one a power cut
 *   left half written is skipped, never written over.
 * - A load gives the record of the last used slot of the sector in use if
 *   that slot is committed, and none otherwise. The record of an earlier
 *   slot is never given: it was loaded once already. (tc32_rng checks the
 *   record against its own check value, which covers the device ID.)
 * - A full sector (or none in use): the other sector's magic programmed to
 *   0, the sector erased, the record written to its first slot and
 *   committed, its header written (the sequence number, then the magic)
 *   and read back, then the old sector's magic programmed to 0.
 * A store succeeds only when everything it wrote reads back as written. A
 * power cut at any point of a store leaves one of three: the slot still
 * erased or the new sector's magic not yet in (the next load gives the
 * record the cut boot loaded, which gave nothing out before its store), the
 * slot used but its mark not in (none), or the new record whole (its octets
 * read back before the mark, the record before the header). A sector whose
 * erase was cut has no magic, cleared before the erase, so a load never
 * reads it. tc32_rng credits the record it loads only after it stored the
 * next one.
 *
 * tc32_rng loads at init (POST_KERNEL, TC32_RNG_INIT_PRIORITY), then
 * stores at init and once more when it becomes ready, from the thread that
 * asked; in this firmware that is the BLE thread.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/util.h>

#include "tlsr_spi_flash_io.h"

#define FLASH_CMD_READ_UID 0x4bU

#define SEED_PART DT_NODELABEL(rng_seed_partition)
#define BASE      DT_REG_ADDR(SEED_PART)
#define SECTOR    4096U
#define SLOT      32U
#define SLOTS     (SECTOR / SLOT) /* slot 0: the header */
#define COMMIT    (SLOT - 1U)
#define MAGIC     0x44454553U /* "SEED" */
#define NONE      2U          /* no sector in use */
#define UNKNOWN   3U          /* not looked at yet */

BUILD_ASSERT(DT_REG_SIZE(SEED_PART) == 2U * SECTOR && BASE % SECTOR == 0U,
	     "rng_seed_partition: two sectors");

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define SEED_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define SEED_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

static const struct device *const flash = DEVICE_DT_GET(SEED_FLASH_NODE);
static uint8_t cur = UNKNOWN; /* the sector in use */
static uint8_t next;          /* its first slot after the last used one */
static uint32_t seq;          /* its sequence number */

int tc32_rng_device_id(uint8_t id[16])
{
	TLSR_SPI_FLASH_IO_FN(tlsr_spi_flash_io_read_t, TLSR_SPI_FLASH_IO_OFF_READ_CMD)(
		FLASH_CMD_READ_UID, 0U, 0U, 4U, id, 16U);
	return 0;
}

static uint32_t at(uint8_t sec, uint32_t slot)
{
	return BASE + sec * SECTOR + slot * SLOT;
}

static __noinline int rd(uint32_t a, void *buf, size_t len)
{
	return flash_read(flash, a, buf, len);
}

static __noinline int wr(uint32_t a, const void *data, size_t len)
{
	return flash_write(flash, a, data, len);
}

static bool erased(uint8_t sec, uint32_t slot)
{
	uint32_t w[SLOT / 4U];

	if (rd(at(sec, slot), w, sizeof(w)) != 0) {
		return false;
	}
	for (size_t i = 0; i < ARRAY_SIZE(w); i++) {
		if (w[i] != 0xffffffffU) {
			return false;
		}
	}
	return true;
}

/* The sector in use, its sequence number and its first slot after the used ones. */
static void scan(void)
{
	uint32_t h[2][2];
	bool valid[2];
	uint32_t lo = 1U, hi = SLOTS;

	for (uint8_t s = 0U; s < 2U; s++) {
		valid[s] = rd(at(s, 0U), h[s], sizeof(h[s])) == 0 && h[s][0] == MAGIC;
	}
	cur = valid[1] && (!valid[0] || (int32_t)(h[1][1] - h[0][1]) > 0) ? 1U : (valid[0] ? 0U : NONE);
	if (cur == NONE) {
		return;
	}
	seq = h[cur][1];
	/* The used slots come first: the first erased one by bisection. */
	while (lo < hi) {
		uint32_t mid = (lo + hi) / 2U;

		if (erased(cur, mid)) {
			hi = mid;
		} else {
			lo = mid + 1U;
		}
	}
	next = (uint8_t)lo;
}

int tc32_rng_seed_load(uint8_t record[TC32_RNG_SEED_SIZE])
{
	uint8_t b[SLOT];

	scan();
	if (cur == NONE || next < 2U || rd(at(cur, next - 1U), b, sizeof(b)) != 0 || b[COMMIT] != 0U) {
		return -ENOENT;
	}
	memcpy(record, b, TC32_RNG_SEED_SIZE);
	return 0;
}

/* Programs len octets (at most a record's) at a and reads them back: 0 when they read as given. */
static int prog(uint32_t a, const void *data, size_t len)
{
	uint8_t r[TC32_RNG_SEED_SIZE];

	return wr(a, data, len) == 0 && rd(a, r, len) == 0 && memcmp(r, data, len) == 0 ? 0 : -EIO;
}

/* The record, then the commit mark. */
static int put(uint8_t sec, uint32_t slot, const uint8_t record[TC32_RNG_SEED_SIZE])
{
	static const uint8_t mark;

	return prog(at(sec, slot), record, TC32_RNG_SEED_SIZE) == 0 &&
			       prog(at(sec, slot) + COMMIT, &mark, 1U) == 0
		       ? 0
		       : -EIO;
}

int tc32_rng_seed_store(const uint8_t record[TC32_RNG_SEED_SIZE])
{
	static const uint32_t zero;
	uint8_t to;
	uint32_t hdr[2];

	if (cur == UNKNOWN) {
		scan();
	}
	if (cur != NONE && next < SLOTS) {
		return put(cur, next++, record); /* a slot tried is used, written or not */
	}
	/* The other sector (or the first): no magic while it is erased, the header after the record. */
	to = cur == 0U ? 1U : 0U;
	hdr[0] = MAGIC;
	hdr[1] = cur == NONE ? 1U : seq + 1U;
	if (wr(at(to, 0U), &zero, sizeof(zero)) != 0 ||
	    flash_erase(flash, at(to, 0U), SECTOR) != 0 || put(to, 1U, record) != 0 ||
	    prog(at(to, 0U) + 4U, &hdr[1], 4U) != 0 || prog(at(to, 0U), &hdr[0], 4U) != 0) {
		return -EIO;
	}
	if (cur != NONE) {
		(void)wr(at(cur, 0U), &zero, sizeof(zero));
	}
	cur = to;
	seq = hdr[1];
	next = 2U;
	return 0;
}
