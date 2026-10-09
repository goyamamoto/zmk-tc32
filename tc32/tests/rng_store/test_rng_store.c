/*
 * Host test of src/tlsr_rng_store.c (the seed record log) with zephyr-tc32's
 * tc32_rng (subsys/tc32_rng/tc32_rng.c) as the keyboards build it, plus
 * TC32_RNG_TEST for its status and re-init (a boot of the generator).
 *
 * The flash is a model of the two sectors: a program clears bits, an erase
 * sets them. A power cut is a longjmp out of the flash call at the n-th
 * octet programmed (or erase) of a boot: the octet being programmed gets
 * none, all or a random part of the bits it clears; an erase cut leaves each
 * octet erased, untouched or with random bits set. The registers are a
 * model too: a system timer that moves by a few uneven ticks at each access,
 * the 32 kHz count from it, a TRNG giving random words.
 *
 * Checked:
 * - The first boot (blank flash) is not ready; tc32_rng_collect() makes it
 *   ready; every later boot is ready at init, crediting the record the boot
 *   before stored last, and stores the next.
 * - A power cut at every octet of a store (in the middle of a sector, at
 *   the switch to the other sector in both directions, the first store on a
 *   blank flash, the store when the generator becomes ready), each with
 *   several patterns of the octet or the erase: the boot after it credits
 *   the record the cut boot loaded, or the record the cut boot was storing
 *   whole, or none; never a damaged record and never an older one. The log
 *   works on: the boot after that stores, the next one is ready again.
 * - Every single bit flipped in the last record or its commit mark: none
 *   is credited.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/random/tc32_rng.h>

#define PART  0x6b000U
#define PSIZE 0x2000U
#define SECT  4096U
#define SLOT  32U
#define REC   TC32_RNG_SEED_SIZE

#define NOT_READY 0
#define READY     1

int store_seed_load(uint8_t record[REC]);
int store_seed_store(const uint8_t record[REC]);
void store_reset(void);

const struct device test_flash_dev;
static int failures;

static void check(bool ok, const char *what)
{
	if (!ok) {
		failures++;
		printf("FAIL %s\n", what);
	}
}

/* ---------------------------------------------------------------- randomness of the models */

static uint64_t rs = 0x2545f4914f6cdd1dULL;

static uint64_t rnd(void)
{
	uint64_t z = (rs += 0x9e3779b97f4a7c15ULL);

	z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
	z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
	return z ^ (z >> 31);
}

/* ---------------------------------------------------------------- registers */

static uint64_t ticks; /* the 16 MHz system timer */
static uint8_t regs8[0x10000];

uint32_t test_reg_read(uint32_t a, int size)
{
	ticks += 3U + (rnd() & 3U);
	switch (a) {
	case 0x00800740U:
		return (uint32_t)ticks & ~7U;
	case 0x00800750U:
		return (uint32_t)(ticks * 32768U / 16000000U);
	case 0x00800754U:
		return 0x1234U;
	case 0x00804408U:
		return 1U; /* a TRNG word is ready */
	case 0x0080440cU:
		return (uint32_t)rnd();
	default:
		break;
	}
	if (size == 1 && (a & 0xff0000U) == 0x800000U) {
		return regs8[a & 0xffffU];
	}
	return 0U;
}

void test_reg_write(uint32_t a, uint32_t v, int size)
{
	if (size == 1 && (a & 0xff0000U) == 0x800000U) {
		regs8[a & 0xffffU] = (uint8_t)v;
	}
}

/* ---------------------------------------------------------------- the flash and the cut */

static uint8_t fl[PSIZE];
static long budget = -1; /* octets programmed (an erase counts one) before the cut; -1: none */
static int pattern;      /* the cut octet: 0 none of its bits, 1 all, 2 and up random */
static jmp_buf power;
static long events;      /* octets programmed and erases in this boot */
static int uid_reads;

static uint8_t *fp(off_t off, size_t len)
{
	if (off < (off_t)PART || (size_t)(off - PART) + len > PSIZE) {
		printf("FAIL flash access 0x%lx+%zu outside the partition\n", (long)off, len);
		exit(2);
	}
	return &fl[off - PART];
}

int flash_read(const struct device *dev, off_t off, void *data, size_t len)
{
	(void)dev;
	memcpy(data, fp(off, len), len);
	return 0;
}

int flash_write(const struct device *dev, off_t off, const void *data, size_t len)
{
	uint8_t *b = fp(off, len);

	(void)dev;
	for (size_t i = 0; i < len; i++) {
		uint8_t v = ((const uint8_t *)data)[i];

		if (budget == 0) {
			uint8_t clear = (uint8_t)(b[i] & ~v);
			uint8_t part = pattern == 0 ? 0U : (pattern == 1 ? clear : (uint8_t)(clear & rnd()));

			b[i] &= (uint8_t)~part;
			longjmp(power, 1);
		}
		if (budget > 0) {
			budget--;
		}
		events++;
		b[i] &= v;
	}
	return 0;
}

int flash_erase(const struct device *dev, off_t off, size_t size)
{
	uint8_t *b = fp(off, size);

	(void)dev;
	if (budget == 0) {
		for (size_t i = 0; i < size; i++) {
			uint64_t r = rnd();

			if ((r & 3U) == 0U) {
				b[i] = 0xffU;
			} else if ((r & 3U) != 1U) {
				b[i] |= (uint8_t)(r >> 8);
			}
		}
		longjmp(power, 1);
	}
	if (budget > 0) {
		budget--;
	}
	events++;
	memset(b, 0xff, size);
	return 0;
}

void test_flash_io_read(unsigned char cmd, unsigned long addr, unsigned char with_addr, unsigned char zeros,
			unsigned char *buf, unsigned long len)
{
	check(cmd == 0x4bU && addr == 0U && with_addr == 0U && zeros == 4U && len == 16U,
	      "the device ID is read with 0x4b, 4 dummy octets, 16 octets");
	for (unsigned long i = 0; i < len; i++) {
		buf[i] = (unsigned char)(0xa0U + i);
	}
	uid_reads++;
}

/* ---------------------------------------------------------------- the hooks, watched */

static uint8_t loaded[REC];
static int load_rc;
static uint8_t stored[4][REC];
static int nstored;

int tc32_rng_seed_load(uint8_t record[REC])
{
	load_rc = store_seed_load(record);
	if (load_rc == 0) {
		memcpy(loaded, record, REC);
	}
	return load_rc;
}

int tc32_rng_seed_store(const uint8_t record[REC])
{
	memcpy(stored[nstored < 4 ? nstored++ : 3], record, REC);
	return store_seed_store(record);
}

/* Every record whose store returned 0, in order. */
static uint8_t hist[4096][REC];
static int nhist;

static int hist_find(const uint8_t r[REC])
{
	for (int i = nhist - 1; i >= 0; i--) {
		if (memcmp(hist[i], r, REC) == 0) {
			return i;
		}
	}
	return -1;
}

static int state(void)
{
	struct tc32_rng_status st;

	tc32_rng_get_status(&st);
	return st.state;
}

/* A boot of the generator (its init), then a collection if it is not ready (the BLE thread's). The state at init. */
static int boot(bool collect)
{
	int s;

	store_reset();
	nstored = 0;
	load_rc = -999;
	events = 0;
	ticks += 16000000U + (rnd() & 0xffffU);
	s = tc32_rng_reinit(0);
	if (collect && s != READY) {
		check(tc32_rng_collect() == 0, "the collection makes the generator ready");
	}
	return s;
}

/* A boot without a cut, its stores counted into the history. */
static int boot_ok(bool collect)
{
	int before = nhist;
	int s = boot(collect);

	for (int i = 0; i < nstored; i++) {
		memcpy(hist[nhist++], stored[i], REC);
	}
	(void)before;
	return s;
}

static long n_old, n_new, n_none, n_runs;

/*
 * From the flash snap, one boot cut at event j with pattern pat, then two boots without a cut. False when the
 * first boot ended before event j.
 */
static bool cut_run(const uint8_t *snap, int snap_hist, long j, int pat, bool collect)
{
	uint8_t old[REC];
	bool had_old;
	uint8_t storing[4][REC];
	int nstoring;
	int s;
	char what[160];

	memcpy(fl, snap, PSIZE);
	nhist = snap_hist;
	budget = j;
	pattern = pat;
	if (setjmp(power) == 0) {
		(void)boot(collect);
		budget = -1;
		return false; /* the boot ran through: no cut at j */
	}
	budget = -1;
	had_old = load_rc == 0;
	memcpy(old, loaded, REC);
	nstoring = nstored;
	memcpy(storing, stored, sizeof(storing));
	n_runs++;

	/* The boot after the cut. */
	s = boot(false);
	snprintf(what, sizeof(what), "cut at event %ld pattern %d (%s)", j, pat, collect ? "with collection" : "init");
	if (s == READY) {
		bool is_old = had_old && memcmp(loaded, old, REC) == 0;
		bool is_new = false;

		for (int i = 0; i < nstoring; i++) {
			is_new |= memcmp(loaded, storing[i], REC) == 0 && storing[i][15] == 1U;
		}
		check(is_old || is_new, what);
		if (is_old) {
			int h = hist_find(loaded);

			check(h < 0 || h == nhist - 1, "the record credited after a cut is no older than the cut boot's");
			n_old++;
		} else {
			n_new++;
		}
	} else {
		n_none++;
	}
	/* The log works on: the next boot (after a collection if needed) stores, and the one after is ready. */
	(void)boot(true);
	snprintf(what, sizeof(what), "after the cut at event %ld pattern %d: a later boot is ready", j, pat);
	check(boot(false) == READY, what);
	return true;
}

/* Every cut position of the boot from snap, with every pattern. */
static void cut_all(const char *name, int snap_hist, bool collect)
{
	static uint8_t snap[PSIZE];
	long before_old = n_old, before_new = n_new, before_none = n_none, before_runs = n_runs;

	memcpy(snap, fl, PSIZE);
	for (long j = 0;; j++) {
		bool any = false;

		for (int pat = 0; pat < 6; pat++) {
			any |= cut_run(snap, snap_hist, j, pat, collect);
		}
		if (!any) {
			printf("ok   %s: %ld cut positions x 6 patterns: %ld runs; after the cut %ld credit the "
			       "cut boot's loaded record, %ld the new one whole, %ld none\n",
			       name, j, n_runs - before_runs, n_old - before_old, n_new - before_new,
			       n_none - before_none);
			break;
		}
	}
	memcpy(fl, snap, PSIZE);
	nhist = snap_hist;
}

static uint32_t sector_seq(int s)
{
	uint32_t h[2];

	memcpy(h, &fl[s * SECT], sizeof(h));
	return h[0] == 0x44454553U ? h[1] : 0U;
}

/* The slot of the last record: the last slot not erased in the sector in use. */
static uint8_t *last_slot(void)
{
	int s = sector_seq(1) != 0U && (sector_seq(0) == 0U || (int32_t)(sector_seq(1) - sector_seq(0)) > 0) ? 1 : 0;

	for (int n = SECT / SLOT - 1; n >= 1; n--) {
		uint8_t *b = &fl[s * SECT + n * SLOT];

		for (int i = 0; i < (int)SLOT; i++) {
			if (b[i] != 0xffU) {
				return b;
			}
		}
	}
	return NULL;
}

int main(void)
{
	static uint8_t snap[PSIZE];
	uint32_t seq0;
	int s;

	memset(fl, 0xff, sizeof(fl));

	/* The first boot: not ready, ready after the collection; then ready at every boot. */
	s = boot_ok(true);
	check(s == NOT_READY && load_rc != 0, "blank flash: no record, not ready at init");
	check(state() == READY && nstored == 2 && stored[1][15] == 1U,
	      "blank flash: ready after the collection, its record stored");
	check(uid_reads > 0, "the device ID read");
	for (int i = 0; i < 5; i++) {
		int h = nhist;

		s = boot_ok(false);
		check(s == READY && load_rc == 0 && memcmp(loaded, hist[h - 1], REC) == 0,
		      "a boot credits the record the boot before stored last");
		check(nstored == 1 && nhist == h + 1, "a ready boot stores one record");
	}
	printf("ok   first boot not ready, ready after the collection; 5 boots ready at init, each crediting the "
	       "last record stored\n");

	/* Cuts in the middle of a sector. */
	cut_all("store in the middle of sector 0", nhist, false);

	/* Up to the end of sector 0 (127 records), cuts at the switch to sector 1. */
	while (sector_seq(1) == 0U && last_slot() != &fl[SECT - SLOT]) {
		check(boot_ok(false) == READY, "boots up to a full sector are ready");
	}
	check(last_slot() == &fl[SECT - SLOT], "sector 0 full");
	cut_all("store switching from sector 0 to sector 1", nhist, false);
	seq0 = sector_seq(0);
	check(boot_ok(false) == READY && sector_seq(1) == seq0 + 1U,
	      "the switch: sector 1 in use with the next sequence number");
	check(fl[0] == 0U && fl[1] == 0U && fl[2] == 0U && fl[3] == 0U, "the switch: sector 0's magic cleared");

	/* Up to the end of sector 1, cuts at the switch back to sector 0. */
	while (last_slot() != &fl[2U * SECT - SLOT]) {
		check(boot_ok(false) == READY, "boots up to a full sector are ready");
	}
	cut_all("store switching from sector 1 back to sector 0", nhist, false);
	seq0 = sector_seq(1);
	check(boot_ok(false) == READY && sector_seq(0) == seq0 + 1U && sector_seq(1) == 0U,
	      "the switch back: sector 0 in use with the next sequence number, sector 1's magic cleared");

	/* The first stores on a blank flash, and the store when the generator becomes ready. */
	memcpy(snap, fl, PSIZE);
	memset(fl, 0xff, sizeof(fl));
	nhist = 0;
	cut_all("first boot on a blank flash: the init store and the store at readiness", 0, true);

	/* A record that was not credited (made before ready), then the cut of the store at readiness. */
	memset(fl, 0xff, sizeof(fl));
	nhist = 0;
	(void)boot_ok(false); /* not ready: its record is not one to credit */
	cut_all("boot after an uncredited record: the store at readiness", nhist, true);

	/* Every bit of the last record and of its commit mark flipped: nothing credited. */
	memcpy(fl, snap, PSIZE);
	{
		uint8_t *last = last_slot();
		long bad = 0;
		static uint8_t ref[PSIZE];

		memcpy(ref, fl, PSIZE);
		for (int bit = 0; bit < 8 * (REC + 1); bit++) {
			int at = bit / 8 < REC ? bit / 8 : (int)SLOT - 1;

			memcpy(fl, ref, PSIZE);
			last = last_slot();
			last[at] ^= (uint8_t)(1U << (bit % 8));
			if (boot(false) == READY) {
				bad++;
			}
		}
		check(bad == 0, "a flipped bit in the last record or its mark: not credited");
		printf("ok   %d single-bit flips of the last record and its commit mark: %ld credited\n",
		       8 * (REC + 1), bad);
	}

	printf("%d failure(s)\n", failures);
	return failures != 0;
}
