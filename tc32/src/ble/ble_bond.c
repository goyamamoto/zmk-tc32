/*
 * The own BLE stack's bonds and profiles: one bond per profile
 * (TLSR_BLE_PROFILES), what a pairing with bonding leaves (ble_smp.c) and
 * what the link looks up for a central's LL_ENC_REQ on the active profile
 * (ble_link.c); and the profile state: the active profile and each
 * profile's address generation (a cleared bond gives the profile a new
 * address, so a host does not take the new pairing for the old one).
 *
 * In flash they live in the board's "storage" partition (0x60000 on the
 * Cidoo boards, which the stock firmware never touches; its own bonds stay
 * at 0x74000-0x7afff), in the first two sectors:
 * - Each sector starts with a header (magic, sequence number); the one with
 *   the higher sequence number is in use.
 * - After the header, 64-octet records, appended: a bond record (profile,
 *   the bond, a CRC-16) or a state record (active profile, generations,
 *   on a board without a mode switch the link chosen by key, from octet 8
 *   each profile's database hash, CRC-16). The newest record of a kind wins; a record with every octet
 *   0xff ends the log (one only partly written is skipped, never written over).
 *   A state record from before the hashes holds zeros there: no hash kept.
 *   A new hash (a pairing, a host's confirmation of Service Changed) goes
 *   to the log when the link ends, or right before the mode switch's
 *   reboot, not inside the link: a flash write holds the radio, and a
 *   pairing writes its bond and its CCCDs already.
 * - A full sector is compacted into the other one: its header's magic
 *   programmed to 0, the sector erased, the bonds written, its header
 *   written last, then the old sector's magic programmed to 0. The magic
 *   goes before every erase, so a sector whose erase a power cut
 *   interrupted (bits set at random, a sequence number that may read
 *   higher) never passes for one in use; a cut before the new header
 *   leaves the old sector in use; and with the old header cleared after
 *   the new one, only one valid header exists except inside that moment,
 *   so a misread of one sequence number cannot make the older sector win.
 *   A log from before this (two valid headers) is read as before: the
 *   higher sequence number wins.
 * Every write goes through the flash driver, which holds the radio off
 * (ble_flash.h), and none leaves those two sectors.
 * - The flash test behind the host's confirm (tlsr_usb_ota.c,
 *   ble_bond_flash_test()) is this compaction with every step read back:
 *   the other sector erased and read back blank, each record written and
 *   read back equal, the header last, then the old sector erased and read
 *   back blank. It proves erase, program and read on the unit's own flash
 *   through the driver the ways back use, and the two sectors are the ones
 *   the log erases and writes anyway. It first reads both headers and the
 *   sector in use again and refuses (nothing erased) unless they give the
 *   sector the load chose and the bonds, state and records RAM holds, so a
 *   read that went wrong at the load can neither pick the sector to erase
 *   nor have its records rewritten; the old sector's magic is cleared only after the new header
 *   is in, so a power cut inside the test leaves one sector with a valid
 *   header, and the records in RAM are written again at the next save.
 *   The BLE thread's saves and the test take turns on a mutex (a save
 *   during the test waits for it, as the radio is held off for its erases
 *   anyway).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_TLSR_BLE)
#include "ble_crypto.h"
#endif
#include "ble_internal.h"
#include "tlsr_ble.h"

#define STORE_BASE   DT_REG_ADDR(DT_NODELABEL(storage_partition))
#define STORE_SECTOR 4096U
#define HDR_LEN      8U
#define REC_LEN      64U
#define RECS         ((STORE_SECTOR - HDR_LEN) / REC_LEN)
#define HDR_MAGIC    0x444e4f42U /* "BOND" */
#define REC_MAGIC    0xb0U /* a profile's bond */
#define STATE_MAGIC  0xb1U /* the profile state */
#define PROFILES     CONFIG_TLSR_BLE_PROFILES
#define SEEN_OFF     8U /* the state record's database hashes, 4 octets per profile */

BUILD_ASSERT(DT_REG_SIZE(DT_NODELABEL(storage_partition)) >= 2 * STORE_SECTOR,
	     "two sectors of the storage partition");
BUILD_ASSERT(STORE_BASE % STORE_SECTOR == 0, "the storage partition starts on a sector");
BUILD_ASSERT(2U + PROFILES + 1U <= SEEN_OFF && SEEN_OFF + 4U * PROFILES <= REC_LEN - 2U,
	     "the state record holds the database hashes after the profile state");

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define BOND_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define BOND_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

static const struct device *const flash_dev = DEVICE_DT_GET(BOND_FLASH_NODE);

static struct ble_bond bonds[PROFILES];
static struct ble_profile_state state;
static uint32_t db_seen[PROFILES]; /* the database each profile's host last saw (ble_att.c), 0: none kept */
static uint32_t db_seen_saved[PROFILES]; /* as the state record in use holds them */
static uint32_t sector;   /* in use */
static uint32_t seq;
static uint16_t next_rec; /* the first erased record of the sector in use */
static bool loaded;       /* ble_bond_load() ran (the BLE thread's, or the flash test's) */
static struct ble_profile_state state_given; /* the state as ble_profile_state_get() last handed it out */
static bool given;                            /* ble_profile_state_get() ran */
static K_MUTEX_DEFINE(store_mutex);

#ifdef CONFIG_TLSR_SLOTS_SIM
/* A test hook (tests/usb-sim): after each read the flash test makes, before it is compared. */
void ble_bond_sim_read_done(uint32_t addr, uint8_t *buf, size_t len);
#endif

/* A record: magic, slot, then the bond's fields, then a CRC-16 of the rest. */
static void pack(uint8_t r[REC_LEN], uint8_t slot, const struct ble_bond *b)
{
	memset(r, 0, REC_LEN);
	r[0] = REC_MAGIC;
	r[1] = slot;
	r[2] = b->valid;
	memcpy(&r[3], b->peer, 6);
	r[9] = b->peer_random;
	memcpy(&r[10], b->id_addr, 6);
	r[16] = b->id_random;
	memcpy(&r[17], b->irk, 16);
	memcpy(&r[33], b->ltk, 16);
	sys_put_le16(b->ediv, &r[49]);
	memcpy(&r[51], b->rand, 8);
	r[59] = b->cccd;
	r[60] = b->flags;
	sys_put_le16(crc16_ccitt(0xffff, r, REC_LEN - 2U), &r[REC_LEN - 2U]);
}

static void pack_state(uint8_t r[REC_LEN])
{
	memset(r, 0, REC_LEN);
	r[0] = STATE_MAGIC;
	r[1] = state.active;
	memcpy(&r[2], state.gen, PROFILES);
#if IS_ENABLED(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
	r[2 + PROFILES] = state.link;
#endif
	for (size_t i = 0; i < PROFILES; i++) {
		sys_put_le32(db_seen[i], &r[SEEN_OFF + 4U * i]);
	}
	memcpy(db_seen_saved, db_seen, sizeof(db_seen_saved));
	sys_put_le16(crc16_ccitt(0xffff, r, REC_LEN - 2U), &r[REC_LEN - 2U]);
}

static bool unpack(const uint8_t r[REC_LEN], uint8_t *slot, struct ble_bond *b)
{
	if (r[0] != REC_MAGIC || r[1] >= ARRAY_SIZE(bonds) ||
	    sys_get_le16(&r[REC_LEN - 2U]) != crc16_ccitt(0xffff, r, REC_LEN - 2U)) {
		return false;
	}
	*slot = r[1];
	b->valid = r[2] != 0U;
	memcpy(b->peer, &r[3], 6);
	b->peer_random = r[9];
	memcpy(b->id_addr, &r[10], 6);
	b->id_random = r[16];
	memcpy(b->irk, &r[17], 16);
	memcpy(b->ltk, &r[33], 16);
	b->ediv = sys_get_le16(&r[49]);
	memcpy(b->rand, &r[51], 8);
	b->cccd = r[59]; /* 0 in records from before it was kept */
	b->flags = r[60];
	if ((b->flags & BLE_BOND_SC) == 0U) {
		b->valid = false; /* a legacy pairing's bond: the host pairs again */
	}
	return true;
}

static uint32_t rec_addr(uint32_t sec, uint16_t n)
{
	return sec + HDR_LEN + n * REC_LEN;
}

/* Inside the two sectors (0x60000-0x61fff on the Cidoo boards). */
static bool in_store(uint32_t addr, size_t len)
{
	return addr >= STORE_BASE && len <= 2U * STORE_SECTOR &&
	       addr - STORE_BASE <= 2U * STORE_SECTOR - len;
}

static int store_write(uint32_t addr, const void *data, size_t len)
{
	return in_store(addr, len) ? flash_write(flash_dev, addr, data, len) : -EINVAL;
}

/* The sector's header invalidated (its magic programmed to 0, unless the word is blank), then erased. */
static int store_erase(uint32_t sec)
{
	static const uint8_t zero[4] = {0};
	uint8_t magic[4];

	if (!in_store(sec, STORE_SECTOR)) {
		return -EINVAL;
	}
	if (flash_read(flash_dev, sec, magic, sizeof(magic)) == 0 &&
	    sys_get_le32(magic) != 0xffffffffU && sys_get_le32(magic) != 0U) {
		int err = store_write(sec, zero, sizeof(zero));

		if (err == 0 && (flash_read(flash_dev, sec, magic, sizeof(magic)) != 0 ||
				 sys_get_le32(magic) != 0U)) {
			err = -EILSEQ; /* the magic did not take the clear: not erased over it */
		}
		if (err != 0) {
			return err;
		}
	}
	return flash_erase(flash_dev, sec, STORE_SECTOR);
}

/* The sector in use and its sequence number, from the two headers: the valid one with the higher number. */
static void choose(uint32_t *sec, uint32_t *n)
{
	uint8_t hdr[HDR_LEN];

	*sec = STORE_BASE;
	*n = 0;
	for (uint32_t s = STORE_BASE; s < STORE_BASE + 2U * STORE_SECTOR; s += STORE_SECTOR) {
		if (flash_read(flash_dev, s, hdr, HDR_LEN) == 0 && sys_get_le32(&hdr[0]) == HDR_MAGIC &&
		    sys_get_le32(&hdr[4]) != 0xffffffffU && sys_get_le32(&hdr[4]) >= *n) {
			*sec = s;
			*n = sys_get_le32(&hdr[4]);
		}
	}
}

static bool erased(const uint8_t r[REC_LEN])
{
	for (size_t i = 0; i < REC_LEN; i++) {
		if (r[i] != 0xffU) {
			return false;
		}
	}
	return true;
}

/* The bonds, compacted into the other sector; it becomes the one in use. */
static int compact(void)
{
	uint32_t other = sector == STORE_BASE ? STORE_BASE + STORE_SECTOR : STORE_BASE;
	uint8_t r[REC_LEN];
	uint8_t hdr[HDR_LEN];
	uint16_t n = 0;
	int err = store_erase(other);

	for (size_t i = 0; err == 0 && i < ARRAY_SIZE(bonds); i++) {
		if (bonds[i].valid) {
			pack(r, (uint8_t)i, &bonds[i]);
			err = store_write(rec_addr(other, n++), r, REC_LEN);
		}
	}
	if (err == 0) {
		pack_state(r);
		err = store_write(rec_addr(other, n++), r, REC_LEN);
	}
	if (err == 0) {
		sys_put_le32(HDR_MAGIC, &hdr[0]);
		sys_put_le32(seq + 1U, &hdr[4]);
		err = store_write(other, hdr, HDR_LEN);
	}
	if (err == 0) {
		static const uint8_t zero[4] = {0};
		uint32_t old = sector;

		sector = other;
		seq++;
		next_rec = n;
		/* only one valid header from here on: a misread of the old one cannot make it win */
		if (seq > 1U) {
			(void)store_write(old, zero, sizeof(zero));
		}
	}
	return err;
}

static void load(void);

/*
 * Before a change is made to RAM and saved, when the load saw no header:
 * both headers are read again, and a header a misread hid takes the log
 * back in, so that nothing is written over a valid log and the change goes
 * on top of what the log holds. The state's changes are applied as the
 * caller's difference (ble_profile_state_set()), so this reload cannot
 * write a stale state over the log's.
 */
static void take_in(void)
{
	uint32_t s, n;

	if (seq != 0U) {
		return;
	}
	choose(&s, &n);
	if (n != 0U) {
		load();
	}
}

/* A bond record (slot < PROFILES) or the state record (slot = PROFILES). */
static void save(uint8_t slot)
{
	uint8_t r[REC_LEN];

	if (next_rec >= RECS || seq == 0U) {
		(void)compact(); /* writes every bond and the state */
		return;
	}
	if (slot < PROFILES) {
		pack(r, slot, &bonds[slot]);
	} else {
		pack_state(r);
	}
	if (store_write(rec_addr(sector, next_rec), r, REC_LEN) == 0) {
		next_rec++;
	}
}

/* The records of a sector in use (sequence number n, 0: none): the newest of each kind; returns the first erased record. */
static uint16_t scan(uint32_t sec, uint32_t n, struct ble_bond *b, struct ble_profile_state *st, uint32_t *seen)
{
	uint8_t r[REC_LEN];
	uint16_t i = 0;

	memset(b, 0, PROFILES * sizeof(*b));
	memset(st, 0, sizeof(*st));
	memset(seen, 0, PROFILES * sizeof(*seen));
	if (n == 0U) {
		return 0; /* nothing stored yet: the first save writes a header */
	}
	for (; i < RECS; i++) {
		struct ble_bond one;
		uint8_t slot;

		if (flash_read(flash_dev, rec_addr(sec, i), r, REC_LEN) != 0 || erased(r)) {
			break;
		}
		if (unpack(r, &slot, &one)) {
			b[slot] = one;
		} else if (r[0] == STATE_MAGIC && r[1] < PROFILES &&
			   sys_get_le16(&r[REC_LEN - 2U]) == crc16_ccitt(0xffff, r, REC_LEN - 2U)) {
			st->active = r[1];
			memcpy(st->gen, &r[2], PROFILES);
#if IS_ENABLED(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
			st->link = r[2 + PROFILES] <= LINK_P24 ? r[2 + PROFILES] : LINK_USB;
#endif
			for (size_t k = 0; k < PROFILES; k++) {
				seen[k] = sys_get_le32(&r[SEEN_OFF + 4U * k]);
			}
		}
	}
	return i;
}

static void load(void)
{
	loaded = true;
	choose(&sector, &seq);
	next_rec = scan(sector, seq, bonds, &state, db_seen);
	memcpy(db_seen_saved, db_seen, sizeof(db_seen_saved));
}

/* The sector in use reads as RAM has it: the same bonds (as packed), state and first erased record. */
static bool flash_matches_ram(void)
{
	struct ble_bond b[PROFILES];
	struct ble_profile_state st;
	uint32_t seen[PROFILES];
	uint8_t x[REC_LEN], y[REC_LEN];

	if (scan(sector, seq, b, &st, seen) != next_rec || st.active != state.active ||
	    memcmp(st.gen, state.gen, sizeof(st.gen)) != 0 || memcmp(seen, db_seen_saved, sizeof(seen)) != 0) {
		return false;
	}
	for (size_t i = 0; i < PROFILES; i++) {
		if (b[i].valid != bonds[i].valid) {
			return false;
		}
		if (b[i].valid) {
			pack(x, (uint8_t)i, &b[i]);
			pack(y, (uint8_t)i, &bonds[i]);
			if (memcmp(x, y, REC_LEN) != 0) {
				return false;
			}
		}
	}
	return true;
}

void ble_bond_load(void)
{
	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	load();
	(void)k_mutex_unlock(&store_mutex);
}

void ble_bond_store(uint8_t profile, const struct ble_bond *bond, uint32_t db_hash)
{
	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	take_in();
	bonds[profile] = *bond;
	bonds[profile].valid = true;
	save(profile);
	db_seen[profile] = db_hash; /* to the state record by ble_bond_save_db_seen() */
	(void)k_mutex_unlock(&store_mutex);
}

uint32_t ble_bond_db_seen(uint8_t profile)
{
	return profile < PROFILES ? db_seen[profile] : 0U;
}

void ble_bond_set_db_seen(uint8_t profile, uint32_t hash)
{
	if (profile >= PROFILES) {
		return;
	}
	db_seen[profile] = hash; /* to the state record by ble_bond_save_db_seen() */
}

void ble_bond_save_db_seen(void)
{
	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	take_in();
	if (memcmp(db_seen, db_seen_saved, sizeof(db_seen)) != 0) {
		save(PROFILES);
	}
	(void)k_mutex_unlock(&store_mutex);
}

bool ble_bond_set_cccd(uint8_t profile, uint8_t bits)
{
	bool written = false;

	if (profile >= PROFILES) {
		return false;
	}
	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	take_in();
	if (bonds[profile].valid && bonds[profile].cccd != bits) {
		bonds[profile].cccd = bits;
		save(profile);
		written = true;
	}
	(void)k_mutex_unlock(&store_mutex);
	return written;
}

uint8_t ble_bond_cccd(uint8_t profile)
{
	return profile < PROFILES && bonds[profile].valid ? bonds[profile].cccd : 0U;
}

void ble_bond_clear(uint8_t profile)
{
	/* Saved even when RAM holds no bond: the load may have misread one the log still holds. */
	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	take_in();
	memset(&bonds[profile], 0, sizeof(bonds[profile]));
	save(profile);
	(void)k_mutex_unlock(&store_mutex);
}

bool ble_bond_valid(uint8_t profile)
{
	return bonds[profile].valid;
}

static bool all_zero(const uint8_t *p, size_t len)
{
	uint8_t acc = 0;

	for (size_t i = 0; i < len; i++) {
		acc |= p[i];
	}
	return acc == 0U;
}

#if IS_ENABLED(CONFIG_TLSR_BLE)
bool ble_bond_peer_known(uint8_t profile, const uint8_t addr[BLE_ADDR_LEN], bool random)
{
	const struct ble_bond *b = &bonds[profile];

	if (profile >= PROFILES || !b->valid) {
		return false;
	}
	if ((b->peer_random != 0U) == random && memcmp(b->peer, addr, BLE_ADDR_LEN) == 0) {
		return true;
	}
	if (!all_zero(b->id_addr, BLE_ADDR_LEN) && (b->id_random != 0U) == random &&
	    memcmp(b->id_addr, addr, BLE_ADDR_LEN) == 0) {
		return true;
	}
	if (random && (addr[5] & 0xc0U) == 0x40U && !all_zero(b->irk, sizeof(b->irk))) {
		/* a resolvable private address: hash (octets 0-2) = ah(IRK, prand (octets 3-5)) */
		uint8_t hash[3];

		ble_ah(b->irk, &addr[3], hash);
		return memcmp(hash, addr, sizeof(hash)) == 0;
	}
	return false;
}
#endif /* CONFIG_TLSR_BLE */

bool ble_bond_find_ltk(uint8_t profile, uint16_t ediv, const uint8_t rand[8], uint8_t ltk[16],
		       bool *authenticated)
{
	const struct ble_bond *b = &bonds[profile];

	if (b->valid && b->ediv == ediv && memcmp(b->rand, rand, 8) == 0) {
		memcpy(ltk, b->ltk, 16);
		*authenticated = (b->flags & BLE_BOND_AUTHENTICATED) != 0U;
		return true;
	}
	return false;
}

void ble_profile_state_get(struct ble_profile_state *st)
{
	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	*st = state;
	state_given = state;
	given = true;
	(void)k_mutex_unlock(&store_mutex);
}

/*
 * The caller's state saved: the active profile as the caller means it, each
 * generation as the difference from the state the caller was last handed
 * (ble_profile_state_get()), so a state built on a copy the load misread, or
 * one a bond's save has since taken back in from the log, never writes stale
 * generations over the log's.
 */
void ble_profile_state_set(const struct ble_profile_state *st)
{
	struct ble_profile_state was;

	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	take_in();
	was = state;
	if (!given) {
		state_given = state; /* no get before this set: the differences count from the log's */
	}
	state.active = st->active; /* a selection: as the caller means it */
#if IS_ENABLED(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
	state.link = st->link; /* the link chosen by key: as the caller means it */
#endif
	for (size_t i = 0; i < PROFILES; i++) {
		state.gen[i] = (uint8_t)(state.gen[i] + (uint8_t)(st->gen[i] - state_given.gen[i]));
	}
	state_given = state;
	if (memcmp(&was, &state, sizeof(state)) != 0) {
		save(PROFILES);
	}
	(void)k_mutex_unlock(&store_mutex);
}

/* The flash from addr for len octets reads back as data (NULL: all 0xff), a record's length at a time. */
static int store_read_back(uint32_t addr, const uint8_t *data, size_t len)
{
	uint8_t r[REC_LEN];

	for (size_t off = 0; off < len; off += REC_LEN) {
		size_t n = MIN(REC_LEN, len - off);
		int err = flash_read(flash_dev, addr + off, r, n);

		if (err != 0) {
			return err;
		}
#ifdef CONFIG_TLSR_SLOTS_SIM
		ble_bond_sim_read_done(addr + off, r, n);
#endif
		for (size_t i = 0; i < n; i++) {
			if (r[i] != (data != NULL ? data[off + i] : 0xffU)) {
				return -EILSEQ;
			}
		}
	}
	return 0;
}

static int store_write_back(uint32_t addr, const uint8_t *data, size_t len)
{
	int err = store_write(addr, data, len);

	return err != 0 ? err : store_read_back(addr, data, len);
}

int ble_bond_flash_test(void)
{
	uint8_t r[REC_LEN];
	uint8_t hdr[HDR_LEN];
	uint32_t old, other;
	uint16_t n = 0;
	int err = 0;

	(void)k_mutex_lock(&store_mutex, K_FOREVER);
	if (!loaded) {
		load();
	}
	old = sector;
	other = old == STORE_BASE ? STORE_BASE + STORE_SECTOR : STORE_BASE;
	/* 1. both headers read again: the same choice as the load's, else nothing is erased */
	{
		uint32_t now_sec, now_seq;

		choose(&now_sec, &now_seq);
		if (now_sec != old || now_seq != seq || !flash_matches_ram()) {
			err = -EILSEQ;
		}
	}
	/* 2. the other sector erased (its magic cleared first) and read back blank */
	if (err == 0) {
		err = store_erase(other);
	}
	if (err == 0) {
		err = store_read_back(other, NULL, STORE_SECTOR);
	}
	/* 3. every bond and the state written there, each read back */
	for (size_t i = 0; err == 0 && i < ARRAY_SIZE(bonds); i++) {
		if (bonds[i].valid) {
			pack(r, (uint8_t)i, &bonds[i]);
			err = store_write_back(rec_addr(other, n++), r, REC_LEN);
		}
	}
	if (err == 0) {
		pack_state(r);
		err = store_write_back(rec_addr(other, n++), r, REC_LEN);
	}
	/* 4. the header last: that sector is now the one in use */
	if (err == 0) {
		static const uint8_t zero[4] = {0};

		sys_put_le32(HDR_MAGIC, &hdr[0]);
		sys_put_le32(seq + 1U, &hdr[4]);
		err = store_write_back(other, hdr, HDR_LEN);
		if (err != 0) {
			/* what the flash may hold of it must not pass for a header */
			(void)store_write(other, zero, sizeof(zero));
		}
	}
	if (err == 0) {
		sector = other;
		seq++;
		next_rec = n;
		/* 5. the old sector erased (its magic cleared first) and read back blank */
		err = store_erase(old);
	}
	if (err == 0) {
		err = store_read_back(old, NULL, STORE_SECTOR);
	}
	(void)k_mutex_unlock(&store_mutex);
	return err;
}
