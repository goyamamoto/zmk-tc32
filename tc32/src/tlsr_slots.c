/*
 * Firmware slots: which one runs, image checks, and going back to the image in
 * the other slot.
 *
 * The revert, step by step (each one can be cut by a reset):
 *   1. Copy the other slot's first sector to the journal copy sector, with
 *      bytes 8..11 set to "KNLT". The other image, with the copy as its first
 *      sector, must give back the CRC taken before the copy; otherwise the
 *      revert stops here, with nothing in either slot changed.
 *   2. Write the journal record: magic, other slot, the CRCs of both images.
 *   3. Rewrite the other slot's first sector from the copy (erase, write),
 *      its flag left 0xff; the other image must then check with the recorded
 *      CRC before its flag is written. If it does not, the revert stops with
 *      the journal kept, the other slot not bootable and this one still
 *      bootable. If the flag does not read back once its byte 8 holds 0x4b,
 *      the revert stops with both slots bootable, each image checked.
 *   4. Erase the journal.
 *   5. Clear the running slot's "KNLT" word.
 *   6. Reboot.
 * At boot, a journal record whose CRCs still match the two slots means a
 * revert stopped between 2 and 4: steps 3 onwards run again. A record that
 * does not match when read and compared twice (the images changed since) is
 * dropped; one read that went wrong does not drop it.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_io.h>

#include "tlsr_slots.h"
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
#include "tlsr_spi_flash.h"
#endif

LOG_MODULE_REGISTER(tlsr_slots, CONFIG_TLSR_SLOTS_LOG_LEVEL);

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define SLOTS_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define SLOTS_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

/* Not documented in DS-TLSR8278: the code takes a nonzero value as a run from slot B. */
#define TLSR_REG_63E 0x0080063eU

#define JOURNAL_RECORD CONFIG_TLSR_SLOTS_JOURNAL_OFFSET
#define JOURNAL_COPY   (CONFIG_TLSR_SLOTS_JOURNAL_OFFSET + TLSR_SECTOR)
#define JOURNAL_MAGIC  0x54525652U /* "RVRT" */

BUILD_ASSERT(CONFIG_TLSR_SLOTS_JOURNAL_OFFSET % TLSR_SECTOR == 0U);

struct journal {
	uint32_t magic;
	uint32_t other;
	uint32_t other_crc;
	uint32_t own_crc;
};

static const uint8_t knlt[4] = {'K', 'N', 'L', 'T'};
static const struct device *const flash = DEVICE_DT_GET(SLOTS_FLASH_NODE);

/*
 * The planned-reboot mark: analog register 0x3c, a buffer register that a
 * watchdog or software reset keeps and only a power-on reset sets back to
 * 0x0f (DS-TLSR8278-E v1.0.4, Table 2-2, page 45: "0x3a ~ 0x3c are
 * non-volatile even when chip ... is reset by watchdog or software"). The
 * firmware in the other slot does not write it. A word in RAM would sit at
 * each build's own link
 * address, so a reboot between two builds whose layouts differ would go
 * unseen. The mark names the image the reboot is for, by a byte of its tag
 * (the CRC-32 word at its end): a mark left by a reboot into another image
 * (the stock, which never clears it) is not taken for this image's.
 */
#define PLANNED_REG     0x3cU
#define PLANNED_CLEARED 0x0fU /* the register's power-on value */

BUILD_ASSERT(IS_ENABLED(CONFIG_SOC_TLSR8278) || IS_ENABLED(CONFIG_TLSR_SLOTS_SIM),
	     "the planned-reboot mark is the TLSR8278's analog 0x3c (other parts use it otherwise)");

#ifdef CONFIG_TLSR_SLOTS_SIM
static uint8_t sim_retained = PLANNED_CLEARED;

__weak uint8_t tlsr_slots_sim_retained_read(void)
{
	return sim_retained;
}

__weak void tlsr_slots_sim_retained_write(uint8_t value)
{
	sim_retained = value;
}

static uint8_t planned_read(void)
{
	return tlsr_slots_sim_retained_read();
}

static void planned_write(uint8_t value)
{
	tlsr_slots_sim_retained_write(value);
}
#else
/*
 * The analog port, not documented in DS-TLSR8278: the address, the data and
 * the control byte written, then bit 0 of the control byte waited on.
 */
#define ANALOG_PORT_ADDR  0x008000b8U
#define ANALOG_PORT_DATA  0x008000b9U
#define ANALOG_PORT_CTRL  0x008000baU
#define ANALOG_PORT_BUSY  BIT(0)
#define ANALOG_PORT_WRITE BIT(5)
#define ANALOG_PORT_START BIT(6)

static uint8_t planned_read(void)
{
	unsigned int key = irq_lock();
	uint8_t v;

	sys_write8(PLANNED_REG, ANALOG_PORT_ADDR);
	sys_write8(ANALOG_PORT_START, ANALOG_PORT_CTRL);
	while ((sys_read8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	v = sys_read8(ANALOG_PORT_DATA);
	sys_write8(0U, ANALOG_PORT_CTRL);
	irq_unlock(key);
	return v;
}

static void planned_write(uint8_t value)
{
	unsigned int key = irq_lock();

	sys_write8(PLANNED_REG, ANALOG_PORT_ADDR);
	sys_write8(value, ANALOG_PORT_DATA);
	sys_write8(ANALOG_PORT_START | ANALOG_PORT_WRITE, ANALOG_PORT_CTRL);
	while ((sys_read8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	sys_write8(0U, ANALOG_PORT_CTRL);
	irq_unlock(key);
}
#endif

uint8_t tlsr_planned_mark(uint32_t tag)
{
	uint8_t m = (uint8_t)(tag ^ (tag >> 8) ^ (tag >> 16) ^ (tag >> 24));

	/* Never the power-on value (0x0f; 0 in the emulator's model). */
	return m == PLANNED_CLEARED || m == 0U ? (uint8_t)(m ^ 0xa5U) : m;
}

static enum tlsr_boot_mark boot_mark = TLSR_BOOT_MARK_NONE;
static bool mark_stuck; /* the clear did not read back at this boot */

bool tlsr_reboot_was_planned(uint32_t tag)
{
	const uint8_t mark = planned_read();
	bool planned = mark == tlsr_planned_mark(tag);

	mark_stuck = false;
	if (mark != PLANNED_CLEARED) {
		planned_write(PLANNED_CLEARED);
		if (planned_read() != PLANNED_CLEARED) {
			/*
			 * A mark that stays would make every later reset look
			 * planned, not counted and with the chord not read, until
			 * a power cycle: this boot is taken as unplanned instead,
			 * and the version answer reports it.
			 */
			mark_stuck = true;
			planned = false;
		}
	}
	if (planned) {
		boot_mark = TLSR_BOOT_MARK_PLANNED;
	} else if (mark == PLANNED_CLEARED || mark == 0U || mark == tlsr_planned_mark(tag)) {
		boot_mark = TLSR_BOOT_MARK_NONE;
	} else {
		boot_mark = TLSR_BOOT_MARK_OTHER;
	}
	return planned;
}

enum tlsr_boot_mark tlsr_boot_mark(void)
{
	return boot_mark;
}

bool tlsr_boot_mark_stuck(void)
{
	return mark_stuck;
}

const struct device *tlsr_slot_flash(void)
{
	return flash;
}

/*
 * The flash operations. Before the kernel runs (the early boot guard, from
 * the board's early-init hook) no device is ready, so they go straight to the
 * flash routines of tlsr_spi_flash.h; afterwards through the
 * flash driver, which serializes them with the OTA receiver. On native_sim
 * (the tests) the flash simulator is the driver in both cases.
 */
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
#define PRE_KERNEL_DIRECT() k_is_pre_kernel()
#else
#define PRE_KERNEL_DIRECT() false
#endif

#ifdef CONFIG_TLSR_SLOTS_SIM
__weak void tlsr_slots_sim_read_done(uint32_t off, void *buf, size_t len)
{
	ARG_UNUSED(off);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}
#endif

int tlsr_slot_read(uint32_t off, void *buf, size_t len)
{
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	if (PRE_KERNEL_DIRECT()) {
		tlsr_spi_flash_read(off, len, buf);
		return 0;
	}
#endif
#ifdef CONFIG_TLSR_SLOTS_SIM
	int err = flash_read(flash, off, buf, len);

	if (err == 0) {
		tlsr_slots_sim_read_done(off, buf, len);
	}
	return err;
#else
	return flash_read(flash, off, buf, len);
#endif
}

#ifdef CONFIG_TLSR_SLOTS_SIM
__weak bool tlsr_slots_sim_protected(uint32_t off, size_t len)
{
	ARG_UNUSED(off);
	ARG_UNUSED(len);
	return false;
}
#define SIM_PROTECTED(off, len) tlsr_slots_sim_protected(off, len)
#else
#define SIM_PROTECTED(off, len) false
#endif

static bool write_failed; /* a write or erase of this boot did not read back */
/*
 * tlsr_slot_generation(): bumped by the OTA receiver's writes and by a
 * running image's revert that stopped (&prev_fw), not by the writes of this
 * file, which also run in the early boot stage before the boot is counted
 * (left as proven). The version reply's check of the other slot is made anew
 * at every boot anyway. Two increments at once may make one; the number still
 * changes, which is all a reader looks for.
 */
static volatile uint32_t generation;

uint32_t tlsr_slot_generation(void)
{
	return generation;
}

void tlsr_slot_touched(void)
{
	generation = generation + 1U;
}

bool tlsr_slot_write_failed(void)
{
	return write_failed;
}

#ifdef CONFIG_TLSR_SLOTS_SIM
void tlsr_slots_sim_new_boot(void)
{
	write_failed = false;
}
#endif

/*
 * Every write and erase is read back (a write into a locked or failing
 * flash fails silently). -EIO when the flash does not hold
 * what was written, and tlsr_slot_write_failed() stays true for the boot.
 */
static int read_back(uint32_t off, const void *buf, size_t len)
{
	uint8_t back[64];
	const uint8_t *want = buf;

	for (size_t done = 0; done < len; done += sizeof(back)) {
		size_t n = MIN(sizeof(back), len - done);
		int err = tlsr_slot_read(off + done, back, n);

		if (err != 0) {
			return err;
		}
		for (size_t i = 0; i < n; i++) {
			if (back[i] != (want != NULL ? want[done + i] : 0xffU)) {
				return -EIO;
			}
		}
	}
	return 0;
}

int tlsr_slot_write(uint32_t off, const void *buf, size_t len)
{
	int err = 0;

	if (SIM_PROTECTED(off, len)) {
		/* Dropped, as a locked flash drops it; the read-back below notices. */
	} else {
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
		if (PRE_KERNEL_DIRECT()) {
			tlsr_spi_flash_page_program(off, len, buf);
		} else
#endif
		{
			err = flash_write(flash, off, buf, len);
		}
	}
	if (err == 0) {
		err = read_back(off, buf, len);
	}
	if (err != 0) {
		write_failed = true;
		LOG_ERR("write of %u bytes at 0x%05x did not take: %d", (unsigned int)len,
			(unsigned int)off, err);
	}
	return err;
}

int tlsr_slot_erase(uint32_t off, size_t len)
{
	int err = 0;

	if (SIM_PROTECTED(off, len)) {
		/* Dropped, as a locked flash drops it; the read-back below notices. */
	} else {
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
		if (PRE_KERNEL_DIRECT()) {
			for (uint32_t a = off; a < off + len; a += TLSR_SECTOR) {
				tlsr_spi_flash_erase_sector(a);
			}
		} else
#endif
		{
			err = flash_erase(flash, off, len);
		}
	}
	if (err == 0) {
		err = read_back(off, NULL, len);
	}
	if (err != 0) {
		write_failed = true;
		LOG_ERR("erase of %u bytes at 0x%05x did not take: %d", (unsigned int)len,
			(unsigned int)off, err);
	}
	return err;
}

uint32_t tlsr_slot_running(void)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	return tlsr_slots_sim_running_slot();
#else
	/* Nonzero: running from slot B. */
	return sys_read16(TLSR_REG_63E) != 0U ? TLSR_SLOT_B : TLSR_SLOT_A;
#endif
}

static int read_cpu_view(uint32_t off, uint8_t *buf, size_t len)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	return tlsr_slots_sim_cpu_view(off, buf, len);
#else
	const volatile uint8_t *p =
		(const volatile uint8_t *)(uintptr_t)(CONFIG_FLASH_BASE_ADDRESS + off);

	for (size_t i = 0; i < len; i++) {
		buf[i] = p[i];
	}
	return 0;
#endif
}

int tlsr_slot_running_checked(uint32_t *slot)
{
	const uint32_t running = tlsr_slot_running();
	uint8_t a[64];
	uint8_t b[64];
	int err;

	err = tlsr_slot_read(running + TLSR_SLOT_FLAG, a, 1);
	if (err != 0) {
		return err;
	}
	if (a[0] != TLSR_SLOT_FLAG_OK) {
		LOG_ERR("slot 0x%05x is not marked bootable; refusing to write",
			(unsigned int)running);
		return -EIO;
	}
	for (uint32_t off = 0x100U; off < 0x300U; off += sizeof(a)) {
		err = read_cpu_view(off, a, sizeof(a));
		if (err == 0) {
			err = tlsr_slot_read(running + off, b, sizeof(b));
		}
		if (err != 0) {
			return err;
		}
		if (memcmp(a, b, sizeof(a)) != 0) {
			LOG_ERR("the running code is not slot 0x%05x; refusing to write",
				(unsigned int)running);
			return -EIO;
		}
	}
	*slot = running;
	return 0;
}

void tlsr_plan_reset(uint32_t tag)
{
	planned_write(tlsr_planned_mark(tag));
}

void tlsr_reboot(uint32_t tag)
{
	planned_write(tlsr_planned_mark(tag));
#ifdef CONFIG_TLSR_SLOTS_SIM
	tlsr_slots_sim_reboot();
#else
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

static int stop_at(int step)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	if (tlsr_slots_sim_step(step) != 0) {
		return -EINTR;
	}
#else
	ARG_UNUSED(step);
#endif
	return 0;
}

/* Reads [addr, addr + len) of a slot image whose first sector is at sector0. */
static int read_image(uint32_t slot, uint32_t sector0, uint32_t addr, uint8_t *buf, size_t len)
{
	size_t n = 0;
	int err = 0;

	if (addr < TLSR_SECTOR) {
		n = MIN(len, TLSR_SECTOR - addr);
		err = tlsr_slot_read(sector0 + addr, buf, n);
	}
	if (err == 0 && n < len) {
		err = tlsr_slot_read(slot + addr + n, &buf[n], len - n);
	}
	return err;
}

static int check_image(uint32_t slot, uint32_t sector0, uint32_t *crc)
{
	uint8_t buf[64];
	uint32_t size;
	uint32_t c = 0U;
	int err;

	err = read_image(slot, sector0, TLSR_SLOT_SIZE_WORD, buf, 4);
	if (err != 0) {
		return err;
	}
	size = sys_get_le32(buf);
	if ((size & 0xfU) != 4U || size < 0x24U || size > TLSR_SLOT_SIZE) {
		return -ENOENT;
	}
	for (uint32_t off = 0; off < size - 4U; off += sizeof(buf)) {
		size_t n = MIN(sizeof(buf), size - 4U - off);

		if (off % TLSR_SECTOR == 0U) {
			/* Up to 128 KB read from the flash over SPI: seconds on hardware. */
			tlsr_boot_guard_feed();
		}

		err = read_image(slot, sector0, off, buf, n);
		if (err != 0) {
			return err;
		}
		if (off == 0U) {
			memcpy(&buf[TLSR_SLOT_FLAG], knlt, sizeof(knlt));
		}
		c = crc32_ieee_update(c, buf, n);
	}
	err = read_image(slot, sector0, size - 4U, buf, 4);
	if (err != 0) {
		return err;
	}
	if (sys_get_le32(buf) != ~c) {
		return -ENOENT;
	}
	if (crc != NULL) {
		*crc = ~c;
	}
	return 0;
}

int tlsr_slot_check(uint32_t slot, uint32_t *crc)
{
	return check_image(slot, slot, crc);
}

/*
 * Copies a sector, putting "KNLT" at 8..11 when fix_flag is set, then verifies.
 * When dst starts a slot, the boot flag (bytes 8..11) stays 0xff until the
 * rest of the sector is written and verified: a cut or a write error on the
 * way leaves the slot not bootable, never bootable with a partial first
 * sector. With write_flag the flag is then written; without it, it stays 0xff
 * for the caller to write once the whole image checks.
 */
static int copy_sector(uint32_t dst, uint32_t src, bool fix_flag, bool write_flag)
{
	uint8_t buf[64];
	uint8_t back[64];
	uint8_t flag[4];
	bool flag_last = dst == TLSR_SLOT_A || dst == TLSR_SLOT_B;
	int err;

	tlsr_boot_guard_feed();
	err = tlsr_slot_erase(dst, TLSR_SECTOR);
	for (uint32_t off = 0; err == 0 && off < TLSR_SECTOR; off += sizeof(buf)) {
		err = tlsr_slot_read(src + off, buf, sizeof(buf));
		if (err == 0 && off == 0U && fix_flag) {
			memcpy(&buf[TLSR_SLOT_FLAG], knlt, sizeof(knlt));
		}
		if (err == 0 && off == 0U && flag_last) {
			memcpy(flag, &buf[TLSR_SLOT_FLAG], sizeof(flag));
			memset(&buf[TLSR_SLOT_FLAG], 0xff, sizeof(flag));
		}
		if (err == 0) {
			err = tlsr_slot_write(dst + off, buf, sizeof(buf));
		}
		if (err == 0) {
			err = tlsr_slot_read(dst + off, back, sizeof(back));
		}
		if (err == 0 && memcmp(buf, back, sizeof(buf)) != 0) {
			err = -EIO;
		}
		if (err == 0) {
			err = stop_at(TLSR_STEP_CHUNK);
		}
	}
	if (err == 0 && flag_last && write_flag) {
		err = tlsr_slot_write(dst + TLSR_SLOT_FLAG, flag, sizeof(flag));
		if (err == 0) {
			err = tlsr_slot_read(dst + TLSR_SLOT_FLAG, back, sizeof(flag));
		}
		if (err == 0 && memcmp(flag, back, sizeof(flag)) != 0) {
			err = -EIO;
		}
	}
	return err;
}

static int sectors_equal(uint32_t a, uint32_t b, bool *equal)
{
	uint8_t x[64];
	uint8_t y[64];

	*equal = true;
	for (uint32_t off = 0; off < TLSR_SECTOR; off += sizeof(x)) {
		int err = tlsr_slot_read(a + off, x, sizeof(x));

		if (err == 0) {
			err = tlsr_slot_read(b + off, y, sizeof(y));
		}
		if (err != 0) {
			return err;
		}
		if (memcmp(x, y, sizeof(x)) != 0) {
			*equal = false;
			return 0;
		}
	}
	return 0;
}

static uint32_t own_crc(uint32_t slot)
{
	uint32_t crc;

	/* An image without the size word (not installed by OTA) has no CRC to compare. */
	return tlsr_slot_check(slot, &crc) == 0 ? crc : 0U;
}

/* Steps 3 to 6; other_crc: the tag of the image the reboot is for. */
static int revert_finish(uint32_t other, uint32_t running, uint32_t other_crc)
{
	static const uint8_t cleared[4] = {0};
	uint8_t flag[4];
	bool same;
	int err;

	err = stop_at(3);
	if (err == 0) {
		err = sectors_equal(other, JOURNAL_COPY, &same);
	}
	if (err == 0 && !same) {
		/* The rewritten sector's flag stays 0xff until the image checks (below). */
		err = copy_sector(other, JOURNAL_COPY, false, false);
	}
	if (err == 0) {
		/*
		 * The other slot as rewritten checks, with the CRC the revert
		 * recorded (tlsr_slot_check() takes bytes 8..11 as "KNLT"), before
		 * it gets its flag and before this slot gives up its own. Otherwise
		 * the other slot is left without a flag, so that only this slot is
		 * bootable, and the journal stays: the next boot copies the sector
		 * again.
		 */
		uint32_t crc;

		err = tlsr_slot_check(other, &crc);
		if (err == 0 && crc != other_crc) {
			err = -EIO;
		}
		if (err != 0) {
			/*
			 * After a copy the flag is still 0xff; a sector found equal to
			 * the copy has it set already. Cleared either way.
			 */
			(void)tlsr_slot_write(other + TLSR_SLOT_FLAG, cleared, sizeof(cleared));
		} else {
			/*
			 * Written whatever it reads now, so that a read that went
			 * wrong cannot skip the write, then read back. A flag the
			 * write cannot set (bits already 0, as an earlier failed check
			 * leaves it) fails the read-back and stops the revert here.
			 */
			err = tlsr_slot_write(other + TLSR_SLOT_FLAG, knlt, sizeof(knlt));
			if (err == 0) {
				err = tlsr_slot_read(other + TLSR_SLOT_FLAG, flag, sizeof(flag));
			}
			if (err == 0 && memcmp(flag, knlt, sizeof(knlt)) != 0) {
				err = -EIO;
			}
		}
	}
	if (err == 0) {
		err = stop_at(4);
	}
	if (err == 0) {
		err = tlsr_slot_erase(JOURNAL_RECORD, 2U * TLSR_SECTOR);
	}
	if (err == 0) {
		err = stop_at(5);
	}
	if (err == 0) {
		err = tlsr_slot_read(running + TLSR_SLOT_FLAG, flag, sizeof(flag));
	}
	if (err == 0 && memcmp(flag, cleared, sizeof(flag)) != 0) {
		err = tlsr_slot_write(running + TLSR_SLOT_FLAG, cleared, sizeof(cleared));
	}
	if (err == 0) {
		err = stop_at(6);
	}
	if (err != 0) {
		LOG_ERR("revert stopped: %d", err);
		return err;
	}
	LOG_WRN("slot 0x%05x boots next", (unsigned int)other);
	tlsr_reboot(other_crc);
	return 0;
}

int tlsr_slot_revert(void)
{
	uint32_t running;
	uint32_t other;
	struct journal j = {.magic = JOURNAL_MAGIC};
	struct journal back;
	int err;

	err = tlsr_slot_running_checked(&running);
	if (err != 0) {
		return err;
	}
	other = running == TLSR_SLOT_A ? TLSR_SLOT_B : TLSR_SLOT_A;
	j.other = other;
	j.own_crc = own_crc(running);
	err = tlsr_slot_check(other, &j.other_crc);
	if (err != 0) {
		LOG_ERR("no image to go back to in slot 0x%05x: %d", (unsigned int)other, err);
		return err;
	}
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	/* A Zbit flash: analog 0x09 and 0x0c raised before programming; put back if this stops. */
	tlsr_spi_flash_trim_raise();
#endif
	err = stop_at(1);
	if (err == 0) {
		err = copy_sector(JOURNAL_COPY, other, true, false);
	}
	if (err == 0) {
		/*
		 * The copy, as the image's first sector, gives back the CRC just
		 * taken: a read that went wrong while copying would otherwise be
		 * written into the other slot later. Nothing in a slot has changed
		 * yet, so stopping here leaves both as they were.
		 */
		uint32_t copy_crc;

		err = check_image(other, JOURNAL_COPY, &copy_crc);
		if (err == 0 && copy_crc != j.other_crc) {
			err = -EIO;
		}
	}
	if (err == 0) {
		err = stop_at(2);
	}
	if (err == 0) {
		err = tlsr_slot_erase(JOURNAL_RECORD, TLSR_SECTOR);
	}
	if (err == 0) {
		err = tlsr_slot_write(JOURNAL_RECORD, &j, sizeof(j));
	}
	if (err == 0) {
		err = tlsr_slot_read(JOURNAL_RECORD, &back, sizeof(back));
	}
	if (err == 0 && memcmp(&j, &back, sizeof(j)) != 0) {
		err = -EIO;
	}
	if (err != 0) {
		LOG_ERR("revert journal: %d", err);
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
		tlsr_spi_flash_trim_restore();
#endif
		return err;
	}
	/*
	 * The counted resets belong to this image. Forget them now that the
	 * journal is written (from here the next boot finishes the revert):
	 * after it, a later install of the same build must not revert at once.
	 * Earlier, and a revert that stopped before the journal would have left
	 * count 0 and the image unconfirmed.
	 */
	tlsr_boot_guard_forget();
	err = revert_finish(other, running, j.other_crc);
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	if (err != 0) {
		tlsr_spi_flash_trim_restore();
	}
#endif
	return err;
}

/* Whether a journal record still matches the two slots: the revert it was written for. */
static bool journal_matches(const struct journal *j, uint32_t running)
{
	uint32_t crc;

	return j->other == (running == TLSR_SLOT_A ? TLSR_SLOT_B : TLSR_SLOT_A) &&
	       check_image(j->other, JOURNAL_COPY, &crc) == 0 && crc == j->other_crc &&
	       own_crc(running) == j->own_crc;
}

/*
 * Steps 3 to 6 from a record that matches the slots: 1 (the simulated reboot)
 * or an error. The same as the end of tlsr_slot_revert_resume(), which keeps
 * its own copy (a call there would change the code it runs at every boot):
 * a change to one is made to the other.
 */
static int finish_journal(const struct journal *j, uint32_t running)
{
	int err;

	LOG_WRN("finishing an interrupted revert");
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	/* As tlsr_slot_revert(): a Zbit flash programs with analog 0x09 and 0x0c raised. */
	tlsr_spi_flash_trim_raise();
#endif
	err = revert_finish(j->other, running, j->other_crc);
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	if (err != 0) {
		tlsr_spi_flash_trim_restore();
	}
#endif
	return err == 0 ? 1 : err;
}

/*
 * Drops a journal record that did not match the slots (erases off..off+len),
 * unless it matches when read and compared once more; then the revert is
 * finished here. After a reset inside step 3 the journal copy is the only
 * intact first sector of the other image, and one read that went wrong must
 * not lose it. It takes the region to erase, as tlsr_slot_erase() does, is
 * never inlined and is not static (so its arguments are kept):
 * tlsr_slot_revert_resume(), whose first instructions run at every boot
 * before the count, then compiles as with a plain erase there. Its only
 * caller passes the journal (JOURNAL_RECORD, two sectors), whose record it
 * reads again; on a match it does not return (it reboots).
 */
int tlsr_slot_drop_journal(uint32_t off, size_t len);

__noinline int tlsr_slot_drop_journal(uint32_t off, size_t len)
{
	uint32_t running;
	struct journal j;
	int err;

	err = tlsr_slot_read(JOURNAL_RECORD, &j, sizeof(j));
	if (err != 0 || j.magic != JOURNAL_MAGIC) {
		/* The record read differently this time: kept for the next boot. */
		return err;
	}
	err = tlsr_slot_running_checked(&running);
	if (err != 0) {
		return err;
	}
	if (journal_matches(&j, running)) {
		return finish_journal(&j, running);
	}
	LOG_WRN("dropping a stale revert journal");
	return tlsr_slot_erase(off, len);
}

int tlsr_slot_revert_resume(void)
{
	uint32_t running;
	struct journal j;
	uint32_t crc;
	int err;

	err = tlsr_slot_read(JOURNAL_RECORD, &j, sizeof(j));
	if (err != 0 || j.magic != JOURNAL_MAGIC) {
		return err;
	}
	err = tlsr_slot_running_checked(&running);
	if (err != 0) {
		return err;
	}
	if (j.other != (running == TLSR_SLOT_A ? TLSR_SLOT_B : TLSR_SLOT_A) ||
	    check_image(j.other, JOURNAL_COPY, &crc) != 0 ||
	    crc != j.other_crc || own_crc(running) != j.own_crc) {
		return tlsr_slot_drop_journal(JOURNAL_RECORD, 2U * TLSR_SECTOR);
	}
	LOG_WRN("finishing an interrupted revert");
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	/* As tlsr_slot_revert(): a Zbit flash programs with analog 0x09 and 0x0c raised. */
	tlsr_spi_flash_trim_raise();
#endif
	err = revert_finish(j.other, running, j.other_crc);
#if defined(CONFIG_TLSR_SPI_FLASH) && !defined(CONFIG_TLSR_SLOTS_SIM)
	if (err != 0) {
		tlsr_spi_flash_trim_restore();
	}
#endif
	return err == 0 ? 1 : err;
}

#ifndef CONFIG_TLSR_BOOT_GUARD
void tlsr_boot_guard_feed(void)
{
}

void tlsr_boot_guard_forget(void)
{
}
#endif

