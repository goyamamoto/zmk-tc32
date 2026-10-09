/*
 * Boot guard and slot revert on the native_sim flash simulator.
 *
 * Slot A (0x00000) holds "our" image and runs; slot B (0x20000) holds the
 * "stock" image with its flag word cleared, as the stock OTA leaves it.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include "tlsr_slots.h"

#define SLOT_A  TLSR_SLOT_A
#define SLOT_B  TLSR_SLOT_B
#define COUNTER CONFIG_TLSR_BOOT_GUARD_OFFSET
#define JOURNAL CONFIG_TLSR_SLOTS_JOURNAL_OFFSET
#define KNLT    0x544c4e4bU

static uint32_t running = SLOT_A;  /* what boot ROM register 0x63e says */
static uint32_t cpu_slot = SLOT_A; /* what the CPU really executes */
static int reboots;
static bool next_reset_wd;
static int wd_starts;
static int wd_feeds;
static int stop_step;
static int stop_chunk;   /* stop after this chunk of step 3's sector copy (1-64) */
static int cur_step;
static int chunks_seen;
static uint32_t sim_sclk = 16000000U;
static uint32_t prot_first, prot_last; /* a locked flash: writes and erases in here are dropped */
static bool drop_flag_write; /* the write of slot B's flag word is dropped */

bool tlsr_slots_sim_protected(uint32_t off, size_t len)
{
	if (drop_flag_write && off == SLOT_B + TLSR_SLOT_FLAG && len == 4U) {
		return true;
	}
	return prot_last != 0U && off <= prot_last && off + len - 1U >= prot_first;
}

uint32_t tlsr_slots_sim_running_slot(void)
{
	return running;
}

void tlsr_slots_sim_reboot(void)
{
	reboots++;
}

bool tlsr_slots_sim_reset_was_watchdog(void)
{
	bool wd = next_reset_wd;

	next_reset_wd = false;
	return wd;
}

void tlsr_slots_sim_watchdog(bool start)
{
	if (start) {
		wd_starts++;
	} else {
		wd_feeds++;
	}
}

static void write_image(uint32_t slot, uint32_t seed, size_t body);
static void clear_flag(uint32_t slot);
static int swap_b_at; /* at this revert step, slot B gets another valid image (it changed during the revert) */

int tlsr_slots_sim_step(int step)
{
	if (swap_b_at != 0 && step == swap_b_at) {
		swap_b_at = 0;
		write_image(SLOT_B, 7, 8192);
		clear_flag(SLOT_B);
	}
	if (step == TLSR_STEP_CHUNK) {
		chunks_seen += cur_step == 3 ? 1 : 0;
		return stop_chunk != 0 && cur_step == 3 && chunks_seen == stop_chunk;
	}
	cur_step = step;
	chunks_seen = 0;
	return step == stop_step;
}

int tlsr_slots_sim_cpu_view(uint32_t off, uint8_t *buf, size_t len)
{
	return flash_read(tlsr_slot_flash(), cpu_slot + off, buf, len);
}

/* A read error: while revert step fault_step runs, every read of fault_addr comes back with bit 0 of its first byte flipped. */
static int fault_step;
static uint32_t fault_addr;
/* Slot B's flag word as it was when step 3 read slot B past its first sector (the image check). */
static bool watch_flag;
static uint32_t flag_during_check;
/* While step 3 runs, a read of slot B's flag word comes back as "KNLT" whatever it holds. */
static bool fake_flag_set;
/* One read error, whatever runs: the next read that covers this address has bit 0 of that byte flipped. */
static uint32_t fault_once;

void tlsr_slots_sim_read_done(uint32_t off, void *buf, size_t len)
{
	if (fault_step != 0 && cur_step == fault_step && off == fault_addr && len > 0U) {
		((uint8_t *)buf)[0] ^= 1U;
	}
	if (fault_once != 0U && fault_once >= off && fault_once - off < len) {
		((uint8_t *)buf)[fault_once - off] ^= 1U;
		fault_once = 0U;
	}
	if (fake_flag_set && cur_step == 3 && off == SLOT_B + TLSR_SLOT_FLAG && len == 4U) {
		sys_put_le32(KNLT, buf);
	}
	if (watch_flag && cur_step == 3 && off >= SLOT_B + TLSR_SECTOR && off < SLOT_B + 2U * TLSR_SECTOR) {
		(void)flash_read(tlsr_slot_flash(), SLOT_B + TLSR_SLOT_FLAG, &flag_during_check,
				 sizeof(flag_during_check));
	}
}

/* The retained register of the planned-reboot mark: 0x0f at power-on (the setup), kept across reboots. */
static uint8_t retained = 0x0fU;
static bool retained_stuck; /* the register ignores writes */

uint8_t tlsr_slots_sim_retained_read(void)
{
	return retained;
}

void tlsr_slots_sim_retained_write(uint8_t value)
{
	if (!retained_stuck) {
		retained = value;
	}
}

uint32_t tlsr_slots_sim_sclk_hz(void)
{
	return sim_sclk;
}

static const struct device *flash(void)
{
	return tlsr_slot_flash();
}

/* A Telink image: "KNLT" at 8, size word at 0x18, CRC-32 at the end. */
static void write_image(uint32_t slot, uint32_t seed, size_t body)
{
	static uint8_t img[9000];
	uint32_t size = body + 4U;
	uint32_t x = seed;

	zassert_true(size <= sizeof(img) && body % 16U == 0U);
	for (size_t i = 0; i < body; i++) {
		x = x * 1103515245U + 12345U;
		img[i] = x >> 24;
	}
	sys_put_le32(KNLT, &img[8]);
	sys_put_le32(size, &img[0x18]);
	/* The image's CRC-32 has no final XOR: the complement of crc32_ieee(). */
	sys_put_le32(~crc32_ieee(img, body), &img[body]);
	zassert_ok(flash_erase(flash(), slot, 3U * TLSR_SECTOR));
	zassert_ok(flash_write(flash(), slot, img, size));
}

static uint32_t flag_word(uint32_t slot)
{
	uint32_t v;

	zassert_ok(flash_read(flash(), slot + TLSR_SLOT_FLAG, &v, sizeof(v)));
	return v;
}

/* Whether the boot ROM model (rom_boot_slot()) could start the slot: its byte 8. */
static bool bootable(uint32_t slot)
{
	uint8_t b;

	zassert_ok(flash_read(flash(), slot + TLSR_SLOT_FLAG, &b, 1));
	return b == TLSR_SLOT_FLAG_OK;
}

static void clear_flag(uint32_t slot)
{
	static const uint8_t zero[4];

	zassert_ok(flash_write(flash(), slot + TLSR_SLOT_FLAG, zero, sizeof(zero)));
}

static bool blank(uint32_t addr, size_t len)
{
	uint8_t buf[64];

	for (size_t off = 0; off < len; off += sizeof(buf)) {
		zassert_ok(flash_read(flash(), addr + off, buf, sizeof(buf)));
		for (size_t i = 0; i < sizeof(buf); i++) {
			if (buf[i] != 0xffU) {
				return false;
			}
		}
	}
	return true;
}

/* An image's tag: the CRC-32 word at its end. */
static uint32_t tag(uint32_t slot)
{
	uint32_t crc;

	zassert_ok(tlsr_slot_check(slot, &crc));
	return crc;
}

static uint32_t counter(void)
{
	uint8_t buf[16];
	uint32_t n = 0;

	zassert_ok(flash_read(flash(), COUNTER + 16U, buf, sizeof(buf)));
	while (n < sizeof(buf) && buf[n] == 0U) {
		n++;
	}
	return n;
}

/* Ours in A (running), the stock image in B with its flag cleared. */
static void setup_slots(void *fixture)
{
	ARG_UNUSED(fixture);
	running = SLOT_A;
	cpu_slot = SLOT_A;
	sim_sclk = 16000000U;
	retained = 0x0fU;
	retained_stuck = false;
	reboots = 0;
	next_reset_wd = false;
	stop_step = 0;
	stop_chunk = 0;
	cur_step = 0;
	prot_first = 0U;
	prot_last = 0U;
	fault_step = 0;
	fault_addr = 0U;
	watch_flag = false;
	flag_during_check = 0U;
	fake_flag_set = false;
	fault_once = 0U;
	drop_flag_write = false;
	swap_b_at = 0;
	tlsr_slots_sim_new_boot();
	tlsr_boot_guard_sim_new_boot();
	write_image(SLOT_A, 1, 5008);
	write_image(SLOT_B, 2, 8192);
	clear_flag(SLOT_B);
	zassert_ok(flash_erase(flash(), COUNTER, TLSR_SECTOR));
	zassert_ok(flash_erase(flash(), JOURNAL, 2U * TLSR_SECTOR));
}

static void boot(bool after_watchdog)
{
	cur_step = 0; /* a new boot: no revert step runs yet */
	next_reset_wd = after_watchdog;
	zassert_ok(tlsr_boot_guard_boot());
}

static void assert_reverted(void)
{
	zassert_equal(reboots, 1, "%d reboots", reboots);
	zassert_equal(flag_word(SLOT_B), KNLT, "slot B flag 0x%08x", flag_word(SLOT_B));
	zassert_equal(flag_word(SLOT_A), 0U, "slot A flag 0x%08x", flag_word(SLOT_A));
	zassert_ok(tlsr_slot_check(SLOT_B, NULL));
	zassert_true(blank(JOURNAL, 2U * TLSR_SECTOR), "journal left behind");
	/* The revert's reboot, direct or finished from the journal, is planned for slot B's image. */
	zassert_equal(retained, tlsr_planned_mark(tag(SLOT_B)),
		      "the revert's reboot is not marked for slot B's image: 0x%02x", retained);
}

static void assert_untouched(void)
{
	zassert_equal(reboots, 0, "%d reboots", reboots);
	zassert_equal(flag_word(SLOT_A), KNLT, "slot A flag 0x%08x", flag_word(SLOT_A));
	zassert_equal(flag_word(SLOT_B), 0U, "slot B flag 0x%08x", flag_word(SLOT_B));
}

ZTEST_SUITE(tlsr_slots, NULL, NULL, setup_slots, NULL, NULL);

ZTEST(tlsr_slots, test_image_check)
{
	uint32_t crc;

	zassert_ok(tlsr_slot_check(SLOT_A, &crc));
	zassert_ok(tlsr_slot_check(SLOT_B, NULL), "a cleared flag word must not matter");
	zassert_equal(tlsr_slot_check(0x40000U, NULL), -ENOENT, "blank flash");
	/* One flipped bit in the body. */
	static const uint8_t zero;

	zassert_ok(flash_write(flash(), SLOT_B + 0x1000U, &zero, 1));
	zassert_equal(tlsr_slot_check(SLOT_B, NULL), -ENOENT);
}

static bool confirmed_byte(void)
{
	uint8_t c;

	zassert_ok(flash_read(flash(), COUNTER + 4U, &c, 1));
	return c == 0x00U;
}

/*
 * Until the image is confirmed, every boot the firmware did not ask for
 * counts, whatever the reset cause; once it is confirmed, none does, watchdog
 * resets included, and the counter sector is not written.
 */
ZTEST(tlsr_slots, test_counts_only_before_the_confirm)
{
	static const uint8_t canary = 0x00U;
	uint8_t c;

	boot(false);
	boot(true);
	zassert_equal(counter(), 2U, "%u counted", counter());
	zassert_ok(tlsr_boot_guard_confirm());
	zassert_equal(counter(), 0U, "the confirm clears the count");
	/* A byte the guard never writes: an erase would put 0xff back. */
	zassert_ok(flash_write(flash(), COUNTER + 0x800U, &canary, 1));
	boot(true);
	boot(false);
	zassert_equal(counter(), 0U, "a confirmed image counted %u", counter());
	zassert_ok(flash_read(flash(), COUNTER + 0x800U, &c, 1));
	zassert_equal(c, 0x00U, "the counter sector was erased");
	zassert_true(confirmed_byte());
	assert_untouched();
	zassert_true(wd_starts > 0);
}

/* Until the host confirms the image, the automatic healthy mark clears nothing. */
ZTEST(tlsr_slots, test_unconfirmed_image_ignores_the_automatic_mark)
{
	boot(false);
	boot(false);
	tlsr_boot_guard_healthy_now();
	zassert_equal(counter(), 2U, "%u counted: the automatic mark cleared an unconfirmed image",
		      counter());
	zassert_false(confirmed_byte());
	boot(false);
	assert_reverted();
}

/* The confirmation stays with the image across boots, which it does not count. */
ZTEST(tlsr_slots, test_confirmation_persists)
{
	uint32_t hz;
	uint16_t capture;
	uint8_t resets;
	bool measured;
	bool confirmed;

	boot(false);
	zassert_ok(tlsr_boot_guard_confirm());
	zassert_true(confirmed_byte());
	boot(false);
	boot(true);
	tlsr_boot_guard_info(&hz, &capture, &resets, &measured, &confirmed);
	zassert_true(confirmed, "the confirmation did not survive the boots");
	zassert_equal(resets, 0U, "%u counted", resets);
	tlsr_boot_guard_healthy_now();
	zassert_equal(counter(), 0U, "%u counted", counter());
	zassert_true(confirmed_byte(), "the mark lost the confirmation");
	assert_untouched();
}

/*
 * Nothing to clear, nothing erased: once the image is confirmed and no boot is
 * counted, the automatic mark and a repeated confirm leave the counter sector
 * as it is (every erase is a window in which a power cut loses the
 * confirmation). A counted boot is still cleared.
 */
ZTEST(tlsr_slots, test_healthy_mark_with_nothing_to_clear_erases_nothing)
{
	static const uint8_t canary = 0x00U;
	uint8_t c;

	boot(false);
	zassert_ok(tlsr_boot_guard_confirm());
	zassert_equal(counter(), 0U, "%u counted", counter());
	/* A byte the guard never writes: an erase would put 0xff back. */
	zassert_ok(flash_write(flash(), COUNTER + 0x800U, &canary, 1));
	tlsr_boot_guard_healthy_now();
	tlsr_boot_guard_healthy_now();
	zassert_ok(tlsr_boot_guard_confirm());
	zassert_ok(flash_read(flash(), COUNTER + 0x800U, &c, 1));
	zassert_equal(c, 0x00U, "the counter sector was erased with nothing to clear");
	zassert_true(confirmed_byte());
	/* A count in the sector (as an older rule left it) is still cleared. */
	static const uint8_t zero = 0x00U;

	zassert_ok(flash_write(flash(), COUNTER + 16U, &zero, 1));
	zassert_equal(counter(), 1U, "%u counted", counter());
	tlsr_boot_guard_healthy_now();
	zassert_equal(counter(), 0U, "a counted boot was not cleared");
	zassert_true(confirmed_byte(), "the clearing lost the confirmation");
	assert_untouched();
}

/*
 * A revert that stops after it forgot the count (slot B will not take its
 * writes) leaves the boot running unconfirmed: a healthy mark later in it
 * writes neither the tag nor the confirmation back.
 */
ZTEST(tlsr_slots, test_stopped_revert_leaves_the_image_unconfirmed)
{
	uint32_t tag;
	uint8_t c;

	boot(false);
	zassert_ok(tlsr_boot_guard_confirm());
	zassert_true(confirmed_byte());
	prot_first = SLOT_B;
	prot_last = SLOT_B + TLSR_SECTOR - 1U;
	zassert_not_equal(tlsr_slot_revert(), 0, "the revert did not stop");
	zassert_equal(reboots, 0);
	tlsr_boot_guard_healthy_now();
	zassert_ok(flash_read(flash(), COUNTER, &tag, sizeof(tag)));
	zassert_ok(flash_read(flash(), COUNTER + 4U, &c, 1));
	zassert_equal(tag, 0xffffffffU, "the tag was written back: 0x%08x", tag);
	zassert_equal(c, 0xffU, "the confirmation was written back");
}

/* A counter tagged for another image is dropped with its confirmation. */
ZTEST(tlsr_slots, test_confirmation_of_another_image_dropped)
{
	static const uint32_t other_tag = 0x12345678U;
	static const uint8_t confirmed = 0x00U;

	zassert_ok(flash_write(flash(), COUNTER, &other_tag, sizeof(other_tag)));
	zassert_ok(flash_write(flash(), COUNTER + 4U, &confirmed, 1));
	boot(false);
	zassert_false(confirmed_byte(), "another image's confirmation was kept");
	zassert_equal(counter(), 1U, "%u counted", counter());
}

/* The early stage counts; the POST_KERNEL step then does not count again. */
ZTEST(tlsr_slots, test_early_stage_counts_once)
{
	next_reset_wd = false;
	tlsr_boot_guard_early();
	zassert_ok(tlsr_boot_guard_boot());
	zassert_equal(counter(), 1U, "%u counted", counter());
	next_reset_wd = true;
	tlsr_boot_guard_early();
	zassert_ok(tlsr_boot_guard_boot());
	zassert_equal(counter(), 2U, "%u counted", counter());
	assert_untouched();
}

/* Three early stages without a healthy boot go back, before any POST_KERNEL step. */
ZTEST(tlsr_slots, test_early_stage_reverts)
{
	next_reset_wd = true;
	tlsr_boot_guard_early();
	zassert_ok(tlsr_boot_guard_boot());
	tlsr_boot_guard_early();
	zassert_ok(tlsr_boot_guard_boot());
	zassert_equal(reboots, 0);
	tlsr_boot_guard_early();
	assert_reverted();
}

/* Power-ons in a row without a healthy boot in between go back as well. */
ZTEST(tlsr_slots, test_reverts_after_three_power_ons)
{
	boot(false);
	boot(false);
	zassert_equal(reboots, 0);
	boot(false);
	assert_reverted();
}

/* A confirmed image's power-ons and watchdog resets never add up to a revert. */
ZTEST(tlsr_slots, test_confirmed_image_never_reverts_by_count)
{
	boot(false);
	zassert_ok(tlsr_boot_guard_confirm());
	for (int i = 0; i < 6; i++) {
		boot(false);
		boot(true);
	}
	zassert_equal(counter(), 0U, "%u counted", counter());
	assert_untouched();
}

/*
 * A count already in the sector of a confirmed image (only a flash fault
 * could put one there now) still goes back at the limit: the count the early
 * stage reads is acted on as before, only no boot adds to it.
 */
ZTEST(tlsr_slots, test_confirmed_image_with_a_full_count_goes_back)
{
	static const uint8_t zero[3] = {0x00U, 0x00U, 0x00U};

	boot(false);
	zassert_ok(tlsr_boot_guard_confirm());
	zassert_ok(flash_write(flash(), COUNTER + 16U, zero, sizeof(zero)));
	boot(false);
	assert_reverted();
}

ZTEST(tlsr_slots, test_reverts_after_three_watchdog_resets)
{
	boot(true);
	boot(true);
	zassert_equal(reboots, 0);
	boot(true);
	assert_reverted();
}

ZTEST(tlsr_slots, test_no_revert_without_a_valid_image)
{
	static const uint8_t zero;

	zassert_ok(flash_write(flash(), SLOT_B + 0x1800U, &zero, 1));
	boot(true);
	boot(true);
	boot(true);
	boot(true);
	assert_untouched();

	zassert_ok(flash_erase(flash(), SLOT_B, 3U * TLSR_SECTOR));
	zassert_equal(tlsr_slot_revert(), -ENOENT);
	zassert_equal(reboots, 0);
	zassert_equal(flag_word(SLOT_A), KNLT);
}

ZTEST(tlsr_slots, test_manual_revert_from_slot_b)
{
	/* The same with the roles swapped: running from B, going back to A. */
	running = SLOT_B;
	cpu_slot = SLOT_B;
	write_image(SLOT_B, 3, 4096);
	write_image(SLOT_A, 4, 6000 / 16 * 16);
	clear_flag(SLOT_A);
	zassert_ok(tlsr_slot_revert());
	zassert_equal(reboots, 1);
	zassert_equal(flag_word(SLOT_A), KNLT);
	zassert_equal(flag_word(SLOT_B), 0U);
	zassert_equal(retained, tlsr_planned_mark(tag(SLOT_A)),
		      "the revert's reboot is not marked for slot A's image: 0x%02x", retained);
}

/* A reset before each step, then the next boot. */
ZTEST(tlsr_slots, test_reset_at_each_step)
{
	for (int step = 1; step <= 6; step++) {
		setup_slots(NULL);
		stop_step = step;
		zassert_equal(tlsr_slot_revert(), -EINTR, "step %d", step);
		stop_step = 0;
		boot(false);

		switch (step) {
		case 1:
		case 2:
			/* Nothing written that matters: as before. */
			assert_untouched();
			break;
		case 3:
		case 4:
			/* The journal is complete: the boot finishes the revert. */
			assert_reverted();
			break;
		case 5:
			/* Both bootable; the running image stays in charge. */
			zassert_equal(reboots, 0);
			zassert_equal(flag_word(SLOT_A), KNLT);
			zassert_equal(flag_word(SLOT_B), KNLT);
			break;
		case 6:
			/* Only the reboot was missing. */
			zassert_equal(flag_word(SLOT_A), 0U);
			zassert_equal(flag_word(SLOT_B), KNLT);
			break;
		}
		printk("reset before step %d: OK\n", step);
	}
}

/* A reset in the middle of rewriting slot B's first sector. */
ZTEST(tlsr_slots, test_reset_while_rewriting_the_sector)
{
	stop_step = 3;
	zassert_equal(tlsr_slot_revert(), -EINTR);
	stop_step = 0;
	/* The erase happened, the write did not. */
	zassert_ok(flash_erase(flash(), SLOT_B, TLSR_SECTOR));
	zassert_equal(tlsr_slot_check(SLOT_B, NULL), -ENOENT);
	boot(false);
	assert_reverted();
}

/* The boot ROM model: the first slot whose byte 8 is 0x4b, slot A first. */
static uint32_t rom_boot_slot(void)
{
	uint8_t b;

	zassert_ok(flash_read(flash(), SLOT_A + TLSR_SLOT_FLAG, &b, 1));
	if (b == TLSR_SLOT_FLAG_OK) {
		return SLOT_A;
	}
	zassert_ok(flash_read(flash(), SLOT_B + TLSR_SLOT_FLAG, &b, 1));
	zassert_equal(b, TLSR_SLOT_FLAG_OK, "no bootable slot");
	return SLOT_B;
}

/* Every slot the ROM could start must hold a complete image. */
static void assert_bootable_slots_complete(const char *what, int chunk)
{
	uint32_t slots[] = {SLOT_A, SLOT_B};

	for (int i = 0; i < 2; i++) {
		uint8_t b;

		zassert_ok(flash_read(flash(), slots[i] + TLSR_SLOT_FLAG, &b, 1));
		if (b == TLSR_SLOT_FLAG_OK) {
			zassert_ok(tlsr_slot_check(slots[i], NULL),
				   "%s, cut after chunk %d: slot 0x%05x bootable but incomplete", what,
				   chunk, slots[i]);
		}
	}
}

/*
 * A power cut after each 64-byte chunk of step 3, going back from B to A (the
 * first install's layout: stock in A) and from A to B. The slot being written
 * must not look bootable until its sector is complete, and the boot the ROM
 * then makes must finish the revert.
 */
ZTEST(tlsr_slots, test_cut_inside_the_sector_copy)
{
	for (int dir = 0; dir < 2; dir++) {
		uint32_t from = dir == 0 ? SLOT_B : SLOT_A;
		uint32_t to = dir == 0 ? SLOT_A : SLOT_B;
		const char *what = dir == 0 ? "B to A" : "A to B";

		for (int chunk = 1; chunk <= TLSR_SECTOR / 64; chunk++) {
			setup_slots(NULL);
			if (dir == 0) {
				write_image(SLOT_B, 3, 4096);
				write_image(SLOT_A, 4, 6000 / 16 * 16);
				clear_flag(SLOT_A);
			}
			running = from;
			cpu_slot = from;
			stop_chunk = chunk;
			zassert_equal(tlsr_slot_revert(), -EINTR, "%s chunk %d", what, chunk);
			stop_chunk = 0;
			assert_bootable_slots_complete(what, chunk);
			running = rom_boot_slot();
			cpu_slot = running;
			boot(false);
			zassert_equal(reboots, 1, "%s chunk %d: %d reboots", what, chunk, reboots);
			zassert_equal(flag_word(to), KNLT, "%s chunk %d", what, chunk);
			zassert_equal(flag_word(from), 0U, "%s chunk %d", what, chunk);
			zassert_ok(tlsr_slot_check(to, NULL));
			zassert_true(blank(JOURNAL, 2U * TLSR_SECTOR), "journal left behind");
		}
		printk("cut after each chunk of step 3, %s: OK\n", what);
	}
}

/* A journal from an earlier revert, after the images changed, is dropped. */
ZTEST(tlsr_slots, test_stale_journal)
{
	stop_step = 4;
	zassert_equal(tlsr_slot_revert(), -EINTR);
	stop_step = 0;
	write_image(SLOT_A, 5, 4096); /* a new build in A */
	boot(false);
	zassert_equal(reboots, 0);
	zassert_equal(flag_word(SLOT_A), KNLT);
	zassert_true(blank(JOURNAL, 2U * TLSR_SECTOR), "stale journal kept");
}

/*
 * A read that goes wrong while slot B's first sector is copied into the
 * journal: the copy does not give back slot B's CRC, so the revert stops
 * before writing the journal record, with both slots as they were.
 */
ZTEST(tlsr_slots, test_bad_read_into_the_journal_stops_the_revert)
{
	fault_step = 1;
	fault_addr = SLOT_B + 0x40U;
	/* The broken copy fails the image's own CRC (check_image()). */
	zassert_equal(tlsr_slot_revert(), -ENOENT);
	fault_step = 0;
	assert_untouched();
	zassert_true(blank(JOURNAL, TLSR_SECTOR), "journal record written");
	boot(false);
	assert_untouched();
	/* The same revert without the read error goes through. */
	zassert_ok(tlsr_slot_revert());
	assert_reverted();
}

/*
 * A read that goes wrong while the journal copy is written back into slot B:
 * slot B does not check, so it never gets its flag (the copy leaves it 0xff;
 * the failed check also clears it), this slot keeps its own, and the journal
 * stays. The next boot copies the sector again and finishes.
 */
ZTEST(tlsr_slots, test_bad_read_while_rewriting_slot_b_keeps_this_slot)
{
	fault_step = 3;
	fault_addr = JOURNAL + TLSR_SECTOR + 0x40U;
	/* The broken copy fails the image's own CRC (check_image()). */
	zassert_equal(tlsr_slot_revert(), -ENOENT);
	fault_step = 0;
	zassert_equal(reboots, 0, "%d reboots", reboots);
	zassert_equal(flag_word(SLOT_A), KNLT, "slot A flag 0x%08x", flag_word(SLOT_A));
	zassert_false(bootable(SLOT_B), "slot B bootable");
	zassert_false(blank(JOURNAL, TLSR_SECTOR), "journal record dropped");
	boot(false);
	assert_reverted();
}

/*
 * Slot B gets its flag only once its rewritten image checks: while the check
 * reads slot B past its first sector, the flag is not set, so a cut or a
 * failed check never leaves a misread slot B bootable.
 */
ZTEST(tlsr_slots, test_slot_b_flag_written_after_its_check)
{
	watch_flag = true;
	zassert_ok(tlsr_slot_revert());
	watch_flag = false;
	zassert_not_equal(flag_during_check, 0U, "the check never read slot B past its first sector");
	zassert_not_equal(flag_during_check, KNLT, "slot B flagged before its check");
	assert_reverted();
}

/*
 * A read that goes wrong again in the boot that finishes the revert: that boot
 * stops as the revert did (slot A bootable, slot B not, the journal kept), and
 * the next one finishes it.
 */
ZTEST(tlsr_slots, test_bad_read_while_finishing_at_the_next_boot)
{
	fault_step = 3;
	fault_addr = JOURNAL + TLSR_SECTOR + 0x40U;
	/* The broken copy fails the image's own CRC (check_image()). */
	zassert_equal(tlsr_slot_revert(), -ENOENT);
	boot(false);
	zassert_equal(reboots, 0, "%d reboots", reboots);
	zassert_equal(flag_word(SLOT_A), KNLT, "slot A flag 0x%08x", flag_word(SLOT_A));
	zassert_false(bootable(SLOT_B), "slot B bootable");
	zassert_false(blank(JOURNAL, TLSR_SECTOR), "journal record dropped");
	fault_step = 0;
	boot(false);
	assert_reverted();
}

/*
 * Slot B holds another image (a valid one) by the time its first sector is
 * copied: the copy checks, but not with the CRC the revert started from, so
 * the revert stops before the journal record.
 */
ZTEST(tlsr_slots, test_other_image_changed_during_the_revert_stops_it)
{
	swap_b_at = 1;
	zassert_equal(tlsr_slot_revert(), -EIO);
	assert_untouched();
	zassert_true(blank(JOURNAL, TLSR_SECTOR), "journal record written");
}

/*
 * Slot B already rewritten, verified and flagged (a cut after step 3), then the
 * boot that finishes the revert misreads slot B while checking it: the flag is
 * cleared, so only this slot is bootable, and the journal stays; the boot after
 * copies the sector again and finishes.
 */
ZTEST(tlsr_slots, test_failed_check_of_a_flagged_slot_b_clears_its_flag)
{
	stop_step = 4;
	zassert_equal(tlsr_slot_revert(), -EINTR);
	stop_step = 0;
	zassert_equal(flag_word(SLOT_B), KNLT, "slot B not flagged after step 3");
	fault_step = 3;
	fault_addr = SLOT_B + TLSR_SECTOR + 0x40U;
	boot(false);
	fault_step = 0;
	zassert_equal(reboots, 0, "%d reboots", reboots);
	zassert_equal(flag_word(SLOT_A), KNLT, "slot A flag 0x%08x", flag_word(SLOT_A));
	zassert_false(bootable(SLOT_B), "a slot B that did not check stays bootable");
	zassert_false(blank(JOURNAL, TLSR_SECTOR), "journal record dropped");
	boot(false);
	assert_reverted();
}

/*
 * A read of slot B's flag that comes back as "KNLT" when it is not set does not
 * keep the revert from writing it: the flag is written, then read back.
 */
ZTEST(tlsr_slots, test_flag_written_even_if_it_reads_as_set)
{
	fake_flag_set = true;
	zassert_ok(tlsr_slot_revert());
	fake_flag_set = false;
	assert_reverted();
}

/*
 * Slot B's flag write does not take: the read-back stops the revert before
 * this slot gives up its flag, with the journal kept; the next boot finishes.
 */
ZTEST(tlsr_slots, test_flag_write_that_does_not_take_keeps_this_slot)
{
	drop_flag_write = true;
	zassert_equal(tlsr_slot_revert(), -EIO);
	drop_flag_write = false;
	zassert_equal(reboots, 0, "%d reboots", reboots);
	zassert_true(bootable(SLOT_A), "slot A not bootable");
	zassert_false(bootable(SLOT_B), "slot B bootable");
	zassert_false(blank(JOURNAL, TLSR_SECTOR), "journal record dropped");
	boot(false);
	assert_reverted();
}

/*
 * A reset inside step 3's sector copy leaves the journal copy as the only
 * intact first sector of slot B. One read that goes wrong while the next boot
 * compares the record with the slots (in the record, the journal copy, slot
 * B's image or this one's) does not drop it: that boot finishes the revert.
 */
ZTEST(tlsr_slots, test_one_bad_read_after_a_cut_in_the_copy_keeps_the_journal)
{
	const uint32_t addrs[] = {
		JOURNAL + 8U,                  /* the record's CRC of slot B's image */
		JOURNAL + TLSR_SECTOR + 0x40U, /* the journal copy */
		SLOT_B + TLSR_SECTOR + 0x40U,  /* slot B's image past its first sector */
		SLOT_A + 0x400U,               /* this slot's image, past what the running-slot check reads */
	};

	for (size_t i = 0; i < ARRAY_SIZE(addrs); i++) {
		setup_slots(NULL);
		stop_chunk = 10;
		zassert_equal(tlsr_slot_revert(), -EINTR);
		stop_chunk = 0;
		zassert_not_ok(tlsr_slot_check(SLOT_B, NULL), "slot B whole after the cut");
		fault_once = addrs[i];
		boot(false);
		zassert_equal(fault_once, 0U, "0x%05x: the bad read did not happen", addrs[i]);
		assert_reverted();
	}
}

ZTEST(tlsr_slots, test_watchdog_is_fed)
{
	int feeds = wd_feeds;

	k_msleep(3 * CONFIG_TLSR_BOOT_GUARD_WATCHDOG_MS / 4 + 100);
	zassert_true(wd_feeds >= feeds + 2, "%d feeds", wd_feeds - feeds);
}

/* After an automatic revert, the same build installed again must not revert at once. */
ZTEST(tlsr_slots, test_reinstall_after_revert)
{
	boot(true);
	boot(true);
	boot(true);
	assert_reverted();
	zassert_equal(counter(), 0U, "the counter survived the revert");

	/* The stock image (slot B) installs the same build into slot A again. */
	write_image(SLOT_A, 1, 5008);
	clear_flag(SLOT_B);
	reboots = 0;
	boot(false);
	zassert_equal(reboots, 0, "went back again after a reinstall");
	zassert_equal(flag_word(SLOT_A), KNLT);
}

/* Resets counted for another image are dropped. */
ZTEST(tlsr_slots, test_counter_of_another_image)
{
	static const uint32_t other_tag = 0x12345678U;
	static const uint8_t zeros[2];

	zassert_ok(flash_write(flash(), COUNTER, &other_tag, sizeof(other_tag)));
	zassert_ok(flash_write(flash(), COUNTER + 16U, zeros, sizeof(zeros)));
	boot(true);
	zassert_equal(counter(), 1U, "%u resets counted", counter());
	zassert_equal(reboots, 0);
}

/* A reboot this firmware asked for is not counted, whatever the reset flag says. */
ZTEST(tlsr_slots, test_planned_reboot_not_counted)
{
	tlsr_reboot(tag(SLOT_A));
	zassert_equal(reboots, 1);
	zassert_equal(retained, tlsr_planned_mark(tag(SLOT_A)), "mark 0x%02x", retained);
	boot(true);
	zassert_equal(counter(), 0U, "a planned reboot was counted");
	zassert_equal(retained, 0x0fU, "the mark was left after the boot: 0x%02x", retained);
	zassert_equal(tlsr_boot_mark(), TLSR_BOOT_MARK_PLANNED);
	boot(true);
	zassert_equal(counter(), 1U, "the next watchdog reset was not counted");
	zassert_equal(tlsr_boot_mark(), TLSR_BOOT_MARK_NONE);
}

/*
 * A mark left for another image (a reboot into the stock, which never clears
 * it, then an install of this image with no power cycle between) does not
 * make this image's boot a planned one.
 */
ZTEST(tlsr_slots, test_planned_reboot_for_another_image_is_counted)
{
	tlsr_reboot(tag(SLOT_B));
	zassert_equal(reboots, 1);
	boot(false);
	zassert_equal(counter(), 1U, "a boot planned for another image was not counted");
	zassert_equal(retained, 0x0fU, "the mark was left after the boot: 0x%02x", retained);
	zassert_equal(tlsr_boot_mark(), TLSR_BOOT_MARK_OTHER, "the version reply's bit 4 would be clear");
}

/*
 * A mark register that does not take the clear: the boot is taken
 * as unplanned, so a mark that stays cannot make every later reset look
 * planned (not counted, the chord not read) until a power cycle.
 */
ZTEST(tlsr_slots, test_planned_mark_that_does_not_clear_is_not_taken)
{
	retained = tlsr_planned_mark(tag(SLOT_A));
	retained_stuck = true;
	boot(false);
	zassert_equal(counter(), 1U, "a boot whose mark did not clear was not counted");
	zassert_true(tlsr_boot_mark_stuck());
	zassert_equal(tlsr_boot_mark(), TLSR_BOOT_MARK_NONE);
	boot(false);
	zassert_equal(counter(), 2U, "the mark that stayed made the next boot a planned one");
	zassert_true(tlsr_boot_mark_stuck());
	retained_stuck = false;
	tlsr_reboot(tag(SLOT_A));
	boot(false);
	zassert_equal(counter(), 2U, "a planned boot after the register works again was counted");
	zassert_false(tlsr_boot_mark_stuck());
}

/* The mark is never the register's power-on value (0x0f; 0 in the emulator's model). */
ZTEST(tlsr_slots, test_planned_mark_is_never_the_power_on_value)
{
	static const uint32_t tags[] = {0U, 0x0000000fU, 0x0f000000U, 0x0f0f0f0fU, 0xffffffffU,
					0x12345678U};

	for (size_t i = 0; i < ARRAY_SIZE(tags); i++) {
		uint8_t m = tlsr_planned_mark(tags[i]);

		zassert_true(m != 0x0fU && m != 0U, "tag 0x%08x: mark 0x%02x", tags[i], m);
	}
	zassert_equal(tlsr_planned_mark(0x12345678U), 0x12U ^ 0x34U ^ 0x56U ^ 0x78U);
}

/* If register 0x63e disagrees with the code actually running, nothing is written. */
ZTEST(tlsr_slots, test_wrong_running_slot_refused)
{
	uint8_t before[64];
	uint8_t after[64];

	/* The register says B; the CPU runs A. B's flag word is cleared. */
	running = SLOT_B;
	zassert_ok(flash_read(flash(), SLOT_A, before, sizeof(before)));
	zassert_equal(tlsr_slot_revert(), -EIO);
	zassert_ok(flash_read(flash(), SLOT_A, after, sizeof(after)));
	zassert_mem_equal(before, after, sizeof(before), "slot A changed");
	zassert_equal(flag_word(SLOT_B), 0U);
	zassert_true(blank(JOURNAL, 2U * TLSR_SECTOR));

	/* Both slots bootable, register says B, CPU runs A: the code check catches it. */
	write_image(SLOT_B, 7, 4096);
	zassert_equal(tlsr_slot_revert(), -EIO);
	zassert_equal(flag_word(SLOT_A), KNLT);
	zassert_equal(flag_word(SLOT_B), KNLT);
	zassert_equal(reboots, 0);
}

/* The CRC over the other image is long on hardware: the watchdog is fed during it. */
ZTEST(tlsr_slots, test_revert_feeds_watchdog)
{
	boot(false); /* starts the watchdog */
	int feeds = wd_feeds;

	zassert_ok(tlsr_slot_revert());
	zassert_true(wd_feeds >= feeds + 3, "%d feeds during the revert", wd_feeds - feeds);
}

/* The watchdog period stays near 4 s whatever system clock is measured. */
ZTEST(tlsr_slots, test_watchdog_capture_follows_measured_clock)
{
	static const uint32_t clocks[] = {16000000U, 24000000U, 32000000U, 48000000U};

	for (size_t i = 0; i < ARRAY_SIZE(clocks); i++) {
		uint32_t hz;
		uint16_t capture;
		uint8_t resets;
		bool measured;
		bool confirmed;

		sim_sclk = clocks[i];
		boot(false);
		tlsr_boot_guard_info(&hz, &capture, &resets, &measured, &confirmed);
		uint32_t period_ms = (uint32_t)(((uint64_t)capture << 18) / (hz / 1000U));

		zassert_true(measured && hz == clocks[i], "%u Hz measured as %u", clocks[i], hz);
		zassert_true(period_ms > 3950U && period_ms <= 4000U, "%u Hz: capture %u, %u ms",
			     hz, capture, period_ms);
		printk("%u Hz: capture %u, period %u ms\n", hz, capture, period_ms);
	}
}

/* A measurement outside 4-64 MHz is not trusted: the configured clock is used. */
ZTEST(tlsr_slots, test_bad_clock_measurement_falls_back)
{
	static const uint32_t bad[] = {0U, 1000000U, 200000000U};

	for (size_t i = 0; i < ARRAY_SIZE(bad); i++) {
		uint32_t hz;
		uint16_t capture;
		uint8_t resets;
		bool measured;
		bool confirmed;

		sim_sclk = bad[i];
		boot(false);
		tlsr_boot_guard_info(&hz, &capture, &resets, &measured, &confirmed);
		zassert_false(measured, "%u Hz accepted", bad[i]);
		zassert_equal(hz, CONFIG_TLSR_BOOT_GUARD_SCLK_HZ);
		zassert_equal(capture, (uint16_t)(((uint64_t)CONFIG_TLSR_BOOT_GUARD_WATCHDOG_MS *
						   (CONFIG_TLSR_BOOT_GUARD_SCLK_HZ / 1000U)) >> 18),
			      "capture %u", capture);
	}
}

/* The reported reset count is the count after this boot. */
ZTEST(tlsr_slots, test_info_reports_resets)
{
	uint32_t hz;
	uint16_t capture;
	uint8_t resets;
	bool measured;
	bool confirmed;

	boot(true);
	boot(true);
	tlsr_boot_guard_info(&hz, &capture, &resets, &measured, &confirmed);
	zassert_equal(resets, 2U, "%u resets reported", resets);
}


/* A write that does not take (a locked or failing flash) is an error, and the count cannot be kept. */
ZTEST(tlsr_slots, test_write_that_does_not_take_is_an_error)
{
	static const uint8_t zero;
	uint8_t v;

	prot_first = COUNTER;
	prot_last = COUNTER + TLSR_SECTOR - 1U;
	zassert_false(tlsr_slot_write_failed());
	zassert_equal(tlsr_slot_write(COUNTER + 16U, &zero, 1), -EIO);
	zassert_true(tlsr_slot_write_failed());
	zassert_ok(flash_read(flash(), COUNTER + 16U, &v, 1));
	zassert_equal(v, 0xffU, "the dropped write left 0x%02x", v);
	/* The counter sector written, then locked: its erase does not take either. */
	prot_last = 0U;
	zassert_ok(tlsr_slot_write(COUNTER + 16U, &zero, 1));
	prot_last = COUNTER + TLSR_SECTOR - 1U;
	zassert_equal(tlsr_slot_erase(COUNTER, TLSR_SECTOR), -EIO);
}

/* The guard cannot count (its sector is locked): it goes back at once. */
ZTEST(tlsr_slots, test_count_that_cannot_be_kept_reverts_now)
{
	uint16_t st_boot, st_now;
	uint8_t flags;
	uint32_t mid;

	prot_first = COUNTER;
	prot_last = COUNTER + TLSR_SECTOR - 1U;
	boot(false);
	assert_reverted();
	tlsr_boot_guard_flash_info(&st_boot, &st_now, &flags, &mid);
	zassert_true((flags & BIT(2)) != 0U, "flags 0x%02x", flags);
}

/* Both slots locked (the stock's 0x18 without the unlock): the revert's writes do not take, nothing boots
 * from a half-written slot, and the revert finishes once the flash is writable again. */
ZTEST(tlsr_slots, test_locked_slots_stop_the_revert)
{
	prot_first = SLOT_A;
	prot_last = SLOT_B + 0x1ffffU;
	boot(false);
	boot(false);
	boot(false);
	zassert_true(tlsr_slot_write_failed());
	assert_untouched();
	zassert_false(blank(JOURNAL, TLSR_SECTOR), "the journal record should be there");
	/* Unlocked (the next boot after a stock OTA, say): the journal finishes the revert. */
	prot_last = 0U;
	boot(false);
	assert_reverted();
}

/*
 * A planned boot whose counter sector holds another image's tag and will not
 * erase: the guard cannot keep its count, but does not revert,
 * or two guard images would revert to each other at every boot.
 */
ZTEST(tlsr_slots, test_planned_boot_with_a_locked_counter_stays)
{
	static const uint32_t foreign = 0x12345678U;

	zassert_ok(flash_write(flash(), COUNTER, &foreign, sizeof(foreign)));
	prot_first = COUNTER;
	prot_last = COUNTER + TLSR_SECTOR - 1U;
	retained = tlsr_planned_mark(tag(SLOT_A));
	boot(false);
	assert_untouched();
	zassert_true(tlsr_slot_write_failed(), "the erase of the foreign counter was refused");
	/* The same with an unplanned boot reverts at once. */
	tlsr_slots_sim_new_boot();
	boot(false);
	assert_reverted();
}

/* A revert that stops before its journal is written keeps the count (and the confirmation). */
ZTEST(tlsr_slots, test_revert_stopped_before_the_journal_keeps_the_count)
{
	uint8_t buf[4];

	stop_step = 1;
	boot(false);
	boot(false);
	boot(false);
	assert_untouched();
	zassert_true(blank(JOURNAL, TLSR_SECTOR), "no journal record");
	zassert_ok(flash_read(flash(), COUNTER + 16U, buf, sizeof(buf)));
	zassert_equal(buf[0] | buf[1] << 8 | buf[2] << 16, 0U, "three boots still counted: %02x %02x %02x", buf[0], buf[1], buf[2]);
	zassert_equal(buf[3], 0xffU);
}
