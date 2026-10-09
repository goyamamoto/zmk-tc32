/*
 * The settings backend (tlsr_settings_log.c) on the native_sim flash
 * simulator: the log is in the storage partition's fifth to eighth sectors,
 * two areas of two sectors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#define SECTOR 4096U
#define AREA   (2U * SECTOR)
#define BASE   (DT_REG_ADDR(DT_NODELABEL(storage_partition)) + 4U * SECTOR)
#define MAGIC  0x32474c53U

void tlsr_settings_log_scan(void);

/* What the handler of "t" was given by the last load. */
#define SEEN_MAX 400
static struct seen {
	char name[24];
	uint8_t value[64];
	size_t len;
} seen[SEEN_MAX];
static int seen_n;

static int t_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	struct seen *s;

	zassert_true(seen_n < SEEN_MAX);
	s = &seen[seen_n++];
	snprintf(s->name, sizeof(s->name), "%s", name);
	zassert_true(len <= sizeof(s->value));
	zassert_equal(read_cb(cb_arg, s->value, sizeof(s->value)), (ssize_t)len);
	s->len = len;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(t, "t", NULL, t_set, NULL, NULL);

static const struct device *flash(void)
{
	return DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
}

static void load(void)
{
	seen_n = 0;
	zassert_ok(settings_load());
}

/* The value the last load gave for t/<name>, or NULL; fails on a name given twice. */
static const struct seen *got(const char *name)
{
	const struct seen *found = NULL;

	for (int i = 0; i < seen_n; i++) {
		if (strcmp(seen[i].name, name) == 0) {
			zassert_is_null(found, "%s given twice", name);
			found = &seen[i];
		}
	}
	return found;
}

static void expect(const char *name, const char *value)
{
	const struct seen *s = got(name);

	zassert_not_null(s, "%s not given", name);
	zassert_equal(s->len, strlen(value), "%s: %u octets", name, (unsigned int)s->len);
	zassert_mem_equal(s->value, value, s->len, "%s", name);
}

static void save(const char *name, const char *value)
{
	char full[32];

	snprintf(full, sizeof(full), "t/%s", name);
	zassert_ok(settings_save_one(full, value, strlen(value)));
}

static uint32_t word(uint32_t off)
{
	uint32_t w;

	zassert_ok(flash_read(flash(), BASE + off, &w, sizeof(w)));
	return w;
}

/* The area in use by its headers: the higher sequence number; -1: none. */
static int area_in_use(void)
{
	const bool a = word(0U) == MAGIC, b = word(AREA) == MAGIC;

	if (a && b) {
		return word(AREA + 4U) > word(4U) ? 1 : 0;
	}
	return a ? 0 : b ? 1 : -1;
}

/* The first offset of area a from which it is erased to its end. */
static uint32_t used(int a)
{
	static uint8_t b[AREA];
	uint32_t n = AREA;

	zassert_ok(flash_read(flash(), BASE + a * AREA, b, AREA));
	while (n > 0U && b[n - 1U] == 0xffU) {
		n--;
	}
	return n;
}

/* A live record as the backend writes it, put at off of area a by the test itself. */
static uint32_t craft(int a, uint32_t off, const char *name, const char *value)
{
	uint8_t b[64] = {0x5e, (uint8_t)strlen(name), (uint8_t)strlen(value), 0, 0, 0, 0, 0};
	const size_t n = strlen(name) + strlen(value);
	uint16_t crc = crc16_ccitt(0xffff, (const uint8_t *)name, strlen(name));

	b[6] = (uint8_t)crc;
	b[7] = (uint8_t)(crc >> 8);

	memcpy(b + 8, name, strlen(name));
	memcpy(b + 8 + strlen(name), value, strlen(value));
	crc = crc16_ccitt(0xffff, b + 8, n);
	b[4] = (uint8_t)crc;
	b[5] = (uint8_t)(crc >> 8);
	zassert_ok(flash_write(flash(), BASE + a * AREA + off, b, 8 + n));
	return off + ROUND_UP(8 + n, 4);
}

static void header(int a, uint32_t seq)
{
	const uint32_t h[2] = {MAGIC, seq};

	zassert_ok(flash_write(flash(), BASE + a * AREA, h, sizeof(h)));
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(flash_erase(flash(), BASE, 2U * AREA));
	zassert_ok(settings_subsys_init());
	tlsr_settings_log_scan();
}

ZTEST_SUITE(tlsr_settings_log, NULL, NULL, before, NULL, NULL);

ZTEST(tlsr_settings_log, test_empty_log_gives_nothing_and_writes_nothing)
{
	load();
	zassert_equal(seen_n, 0);
	zassert_equal(used(0), 0U);
	zassert_equal(used(1), 0U);
}

ZTEST(tlsr_settings_log, test_saved_values_come_back_after_a_boot)
{
	save("a", "one");
	save("b", "two");
	load();
	zassert_equal(seen_n, 2);
	expect("a", "one");
	expect("b", "two");
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 2);
	expect("a", "one");
	expect("b", "two");
	zassert_equal(area_in_use(), 0);
}

ZTEST(tlsr_settings_log, test_newest_value_wins_and_a_delete_removes)
{
	save("a", "one");
	save("b", "two");
	save("a", "three");
	zassert_ok(settings_delete("t/b"));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "three");
	save("b", "back");
	load();
	zassert_equal(seen_n, 2);
	expect("b", "back");
}

ZTEST(tlsr_settings_log, test_unchanged_value_is_not_written_again)
{
	uint32_t n;

	save("a", "one");
	n = used(0);
	save("a", "one");
	zassert_equal(used(0), n);
	zassert_ok(settings_delete("t/none"));
	zassert_equal(used(0), n);
	save("a", "onf");
	zassert_true(used(0) > n);
}

ZTEST(tlsr_settings_log, test_full_area_moves_the_live_records_to_the_other)
{
	char name[8], value[40];
	int moves = 0, in_use = 0;

	/* 20 names written over and over: each area fills several times. */
	for (int i = 0; i < 1500; i++) {
		snprintf(name, sizeof(name), "k%d", i % 20);
		snprintf(value, sizeof(value), "value %d of the key %d ........", i, i % 20);
		save(name, value);
		if (area_in_use() != in_use) {
			in_use = area_in_use();
			moves++;
		}
	}
	zassert_true(moves >= 4, "%d moves", moves);
	for (int boot = 0; boot < 2; boot++) {
		load();
		zassert_equal(seen_n, 20);
		for (int k = 0; k < 20; k++) {
			snprintf(name, sizeof(name), "k%d", k);
			snprintf(value, sizeof(value), "value %d of the key %d ........", 1480 + k, k);
			expect(name, value);
		}
		tlsr_settings_log_scan();
	}
}

ZTEST(tlsr_settings_log, test_record_cut_short_ends_the_log_and_the_next_save_moves)
{
	const uint8_t torn[] = {0x5e, 0x03, 0x04, 0x00}; /* half a record header */
	uint32_t n;

	save("a", "one");
	save("b", "two");
	n = used(0);
	zassert_ok(flash_write(flash(), BASE + n, torn, sizeof(torn)));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 2);
	expect("a", "one");
	save("c", "three");
	zassert_equal(area_in_use(), 1);
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 3);
	expect("a", "one");
	expect("b", "two");
	expect("c", "three");
}

ZTEST(tlsr_settings_log, test_record_with_a_changed_octet_is_not_used)
{
	const uint8_t zero = 0U;
	uint32_t n;

	save("a", "one");
	n = used(0);
	save("b", "two");
	/* the last octet of b's value */
	zassert_ok(flash_write(flash(), BASE + n + 8U + 3U + 2U, &zero, 1U));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "one");
}

ZTEST(tlsr_settings_log, test_move_cut_before_its_header_keeps_the_old_area)
{
	static uint8_t copy[256];

	save("a", "one");
	save("b", "two");
	/* The other area as a cut move leaves it: records, no header. */
	zassert_ok(flash_read(flash(), BASE + 8U, copy, sizeof(copy)));
	zassert_ok(flash_write(flash(), BASE + AREA + 8U, copy, used(0) - 8U));
	tlsr_settings_log_scan();
	zassert_equal(area_in_use(), 0);
	load();
	zassert_equal(seen_n, 2);
	expect("b", "two");
	/* The next move erases what the cut one left. */
	for (int i = 0; area_in_use() == 0; i++) {
		char value[40];

		zassert_true(i < 1000);
		snprintf(value, sizeof(value), "value %d ......................", i);
		save("a", value);
	}
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 2);
	expect("b", "two");
}

ZTEST(tlsr_settings_log, test_header_of_an_older_area_is_not_chosen)
{
	char value[40];

	save("a", "old");
	for (int i = 0; area_in_use() == 0; i++) {
		zassert_true(i < 1000);
		snprintf(value, sizeof(value), "value %d ......................", i);
		save("b", value);
	}
	/* Area 0 still holds its header and records. */
	zassert_equal(word(0U), MAGIC);
	save("a", "new");
	tlsr_settings_log_scan();
	load();
	expect("a", "new");
}

ZTEST(tlsr_settings_log, test_more_live_data_than_an_area_is_refused_and_nothing_lost)
{
	char name[32], value[64];
	int kept = 0, rc = 0;

	memset(value, 'x', sizeof(value) - 1U);
	value[sizeof(value) - 1U] = '\0';
	for (int i = 0; i < 200 && rc == 0; i++) {
		snprintf(name, sizeof(name), "t/n%d", i);
		rc = settings_save_one(name, value, strlen(value));
		if (rc == 0) {
			kept++;
		}
	}
	zassert_equal(rc, -ENOSPC, "rc %d", rc);
	zassert_true(kept > 80, "%d kept", kept);
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, kept);
	/* A delete makes room. */
	zassert_ok(settings_delete("t/n0"));
	zassert_ok(settings_delete("t/n1"));
	save("z", "fits");
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, kept - 1);
	expect("z", "fits");
}

ZTEST(tlsr_settings_log, test_names_and_values_out_of_range_are_refused)
{
	static char value[SETTINGS_MAX_VAL_LEN + 1];
	const char *long_name = "t/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"; /* 44 octets */

	zassert_equal(settings_save_one(long_name, "v", 1U), -EINVAL);
	zassert_not_equal(settings_save_one("t/big", value, sizeof(value)), 0);
	zassert_equal(used(0), 0U);
}

ZTEST(tlsr_settings_log, test_older_records_of_a_name_are_cleared)
{
	uint32_t first;
	uint8_t magic;

	save("a", "one");
	first = 8U;
	save("a", "two");
	zassert_ok(flash_read(flash(), BASE + first, &magic, 1U));
	zassert_equal(magic, 0x00, "the older record's magic is 0x%02x", magic);
	load();
	zassert_equal(seen_n, 1);
	expect("a", "two");
}

ZTEST(tlsr_settings_log, test_two_live_records_of_a_name_give_the_newer_and_the_next_save_clears_both)
{
	uint32_t off;

	/* As a cut between the append and the clearing leaves it. */
	header(0, 1U);
	off = craft(0, 8U, "t/a", "old");
	off = craft(0, off, "t/b", "two");
	(void)craft(0, off, "t/a", "new");
	tlsr_settings_log_scan();
	seen_n = 0;
	zassert_ok(settings_load());
	zassert_equal(seen_n, 3);
	zassert_mem_equal(seen[0].value, "old", 3);
	zassert_mem_equal(seen[2].value, "new", 3);
	zassert_equal(strcmp(seen[2].name, "a"), 0);
	save("a", "three");
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 2);
	expect("a", "three");
	expect("b", "two");
}

ZTEST(tlsr_settings_log, test_magic_cleared_in_part_is_a_cleared_record)
{
	const uint8_t part = 0x4e; /* some of 0x5e's bits cleared: a cut while clearing */
	uint32_t off;

	header(0, 1U);
	off = craft(0, 8U, "t/a", "old");
	off = craft(0, off, "t/a", "new");
	(void)craft(0, off, "t/b", "two");
	zassert_ok(flash_write(flash(), BASE + 8U, &part, 1U));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 2);
	expect("a", "new");
	expect("b", "two");
	save("c", "more");
	zassert_equal(area_in_use(), 0, "the log went on in its area");
}

ZTEST(tlsr_settings_log, test_sequence_numbers_compare_as_serial_numbers)
{
	header(0, UINT32_MAX);
	(void)craft(0, 8U, "t/a", "old");
	header(1, 0U);
	(void)craft(1, 8U, "t/a", "new");
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "new");
}

ZTEST(tlsr_settings_log, test_target_without_its_magic_is_not_chosen)
{
	const uint32_t torn[2] = {0U, 0xffffff05U}; /* magic zeroed, the old sequence number half erased */

	save("a", "one");
	zassert_ok(flash_write(flash(), BASE + AREA, torn, sizeof(torn)));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "one");
}

ZTEST(tlsr_settings_log, test_full_area_of_cleared_records_loads_and_moves)
{
	char value[40];

	/* One name saved until just before a move: the area is cleared records and one live. */
	for (int i = 0; i < 180; i++) {
		snprintf(value, sizeof(value), "value %03d ....................", i);
		save("a", value);
	}
	zassert_equal(area_in_use(), 0);
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "value 179 ....................");
	for (int i = 180; area_in_use() == 0; i++) {
		zassert_true(i < 400);
		snprintf(value, sizeof(value), "value %03d ....................", i);
		save("a", value);
	}
	zassert_true(used(1) < 100U, "%u octets after the move", used(1));
}

ZTEST(tlsr_settings_log, test_log_of_another_format_is_not_used)
{
	const uint32_t old[2] = {0x31474c53U, 7U};

	zassert_ok(flash_write(flash(), BASE, old, sizeof(old)));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 0);
	save("a", "one");
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "one");
}

ZTEST(tlsr_settings_log, test_record_whose_name_crc_reads_wrong_ends_the_log)
{
	const uint8_t zero = 0U;
	uint32_t off;

	header(0, 1U);
	off = craft(0, 8U, "t/a", "one");
	(void)craft(0, off, "t/b", "two");
	/* The low octet of t/b's name CRC: the searches would never find the record. */
	zassert_ok(flash_write(flash(), BASE + off + 6U, &zero, 1U));
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 1);
	expect("a", "one");
	save("b", "new");
	zassert_equal(area_in_use(), 1, "the save moved the log");
	tlsr_settings_log_scan();
	load();
	zassert_equal(seen_n, 2);
	expect("b", "new");
}

