/*
 * Settings backend: a log of records in the storage partition's last four
 * sectors, in two areas of two sectors (0x64000 and 0x66000 on the Cidoo
 * boards).
 *
 * An area starts with a header (magic, sequence number) and holds records one
 * after another, each a header (magic, name length, value length, CRC-16 of
 * name and value, CRC-16 of the name), the name and the value, padded to four
 * octets. A name has
 * one live record. A save appends the new record and then clears the magic of
 * the name's older ones (a program that only clears bits), so that a load and
 * a move pass over the log once. A delete clears the name's records. A cut
 * between the two steps leaves two live records of a name: a load gives both
 * in order, the newer last, and the next save of the name clears them.
 *
 * The area in use is the one with a valid header and the newer sequence
 * number (compared as serial numbers). When the area has no room, the live
 * records and the new one are written to the other area: its magic is zeroed,
 * it is erased, the records are written, then the sequence number and last
 * the magic. A power cut before that leaves the old area in use. A record
 * that does not check ends the log; the next save then moves it. Nothing is
 * written until the first save.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#define SECTOR     4096U
#define AREA_SIZE  (2U * SECTOR)
#define AREA_BASE  (DT_REG_ADDR(DT_NODELABEL(storage_partition)) + 4U * SECTOR)
#define AREA_MAGIC 0x32474c53U /* "SLG2" */
#define REC_MAGIC  0x5eU
#define NAME_MAX   40U
#define NO_AREA    2U

BUILD_ASSERT(DT_REG_SIZE(DT_NODELABEL(storage_partition)) >= 4U * SECTOR + 2U * AREA_SIZE,
	     "the storage partition's fifth to eighth sectors");

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define LOG_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define LOG_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

struct area_hdr {
	uint32_t magic;
	uint32_t seq;
};

struct rec_hdr {
	uint8_t magic;
	uint8_t name_len;
	uint16_t val_len;
	uint16_t crc; /* CRC-16/CCITT of the name, then the value */
	uint16_t name_crc; /* CRC-16/CCITT of the name: a search reads the name only when it agrees */
};
BUILD_ASSERT(sizeof(struct rec_hdr) == 8U, "8-octet record headers");

static const struct device *const flash = DEVICE_DT_GET(LOG_FLASH_NODE);
static uint8_t area = NO_AREA; /* the area in use */
static uint32_t seq;           /* its sequence number */
static uint32_t end;           /* the end of its checked records */
static bool full;              /* no record is appended after end: what follows does not check */

static uint32_t rec_size(const struct rec_hdr *h)
{
	return ROUND_UP(sizeof(*h) + h->name_len + h->val_len, 4U);
}

static uint32_t addr(uint8_t a, uint32_t off)
{
	__ASSERT(a < 2U, "no area");
	return AREA_BASE + a * AREA_SIZE + off;
}

static uint16_t crc_flash(uint16_t crc, uint32_t at, uint32_t len)
{
	uint8_t b[32];

	while (len > 0U) {
		const uint32_t n = MIN(len, sizeof(b));

		if (flash_read(flash, at, b, n) != 0) {
			return (uint16_t)~crc;
		}
		crc = crc16_ccitt(crc, b, n);
		at += n;
		len -= n;
	}
	return crc;
}

enum { REC_BAD = -1, REC_END = 0, REC_LIVE = 1, REC_DEAD = 2 };

/*
 * The record at off of area a, *h filled: REC_LIVE, REC_DEAD (its magic
 * cleared, wholly or in part), REC_END (erased from there) or REC_BAD. With
 * check, a live record's CRC is computed too.
 */
static int rec_at(uint8_t a, uint32_t off, struct rec_hdr *h, bool check)
{
	bool live;

	if (off + sizeof(*h) > AREA_SIZE || flash_read(flash, addr(a, off), h, sizeof(*h)) != 0) {
		return REC_BAD;
	}
	if (h->magic == 0xffU && h->name_len == 0xffU && h->val_len == 0xffffU &&
	    h->crc == 0xffffU && h->name_crc == 0xffffU) {
		return REC_END;
	}
	live = h->magic == REC_MAGIC;
	if ((h->magic & (uint8_t)~REC_MAGIC) != 0U || h->name_len == 0U || h->name_len > NAME_MAX ||
	    h->val_len > SETTINGS_MAX_VAL_LEN || off + rec_size(h) > AREA_SIZE) {
		return REC_BAD;
	}
	if (live && check) {
		/* The name's CRC, which the searches go by, and then the record's over the value. */
		const uint16_t crc = crc_flash(0xffffU, addr(a, off + sizeof(*h)), h->name_len);

		if (crc != h->name_crc ||
		    crc_flash(crc, addr(a, off + sizeof(*h) + h->name_len), h->val_len) != h->crc) {
			return REC_BAD;
		}
	}
	return live ? REC_LIVE : REC_DEAD;
}

/* Finds the area in use and the end of its log, as at boot. */
void tlsr_settings_log_scan(void);

void tlsr_settings_log_scan(void)
{
	struct area_hdr ah[2];
	bool valid[2];
	struct rec_hdr h;
	int r;

	for (uint8_t a = 0U; a < 2U; a++) {
		valid[a] = flash_read(flash, addr(a, 0U), &ah[a], sizeof(ah[a])) == 0 &&
			   ah[a].magic == AREA_MAGIC;
	}
	if (valid[0] && valid[1]) {
		area = (int32_t)(ah[1].seq - ah[0].seq) > 0 ? 1U : 0U;
	} else if (valid[0] || valid[1]) {
		area = valid[1] ? 1U : 0U;
	} else {
		area = NO_AREA;
		seq = 0U;
		return;
	}
	seq = ah[area].seq;
	end = sizeof(struct area_hdr);
	while ((r = rec_at(area, end, &h, true)) > REC_END) {
		end += rec_size(&h);
	}
	full = r < 0;
}

static uint16_t name_crc(const char *name, uint8_t name_len)
{
	return crc16_ccitt(0xffffU, (const uint8_t *)name, name_len);
}

/* Whether the record h at off of the area in use is named name, whose CRC is crc. */
static bool named(uint32_t off, const struct rec_hdr *h, const char *name, uint8_t name_len,
		  uint16_t crc)
{
	char n[NAME_MAX];

	return h->name_crc == crc && h->name_len == name_len &&
	       flash_read(flash, addr(area, off + sizeof(*h)), n, name_len) == 0 &&
	       memcmp(n, name, name_len) == 0;
}

struct read_arg {
	uint32_t at;
	uint16_t len;
};

static ssize_t read_value(void *cb_arg, void *data, size_t len)
{
	const struct read_arg *ra = cb_arg;
	const size_t n = MIN(len, ra->len);

	return flash_read(flash, ra->at, data, n) != 0 ? -EIO : (ssize_t)n;
}

static int log_load(struct settings_store *cs, const struct settings_load_arg *arg)
{
	struct rec_hdr h;
	char name[NAME_MAX + 1U];
	int r;

	ARG_UNUSED(cs);
	if (area == NO_AREA) {
		return 0;
	}
	for (uint32_t off = sizeof(struct area_hdr);
	     off < end && (r = rec_at(area, off, &h, false)) > REC_END; off += rec_size(&h)) {
		struct read_arg ra = {
			.at = addr(area, off + sizeof(h) + h.name_len),
			.len = h.val_len,
		};

		if (r != REC_LIVE || h.val_len == 0U ||
		    flash_read(flash, addr(area, off + sizeof(h)), name, h.name_len) != 0) {
			continue;
		}
		name[h.name_len] = '\0';
		(void)settings_call_set_handler(name, h.val_len, read_value, &ra, arg);
	}
	return 0;
}

/* One record at off of area a; the next free offset, or 0 when the write failed. */
static uint32_t put(uint8_t a, uint32_t off, const char *name, uint8_t name_len,
		    const uint8_t *value, uint16_t val_len)
{
	const struct rec_hdr h = {
		.magic = REC_MAGIC,
		.name_len = name_len,
		.val_len = val_len,
		.crc = crc16_ccitt(name_crc(name, name_len), value, val_len),
		.name_crc = name_crc(name, name_len),
	};

	if (flash_write(flash, addr(a, off), &h, sizeof(h)) != 0 ||
	    flash_write(flash, addr(a, off + sizeof(h)), name, name_len) != 0 ||
	    (val_len > 0U &&
	     flash_write(flash, addr(a, off + sizeof(h) + name_len), value, val_len) != 0)) {
		return 0U;
	}
	return off + rec_size(&h);
}

/* The record h at from of the area in use, to off of area a; the next free offset, or 0. */
static uint32_t copy(uint8_t a, uint32_t off, uint32_t from, const struct rec_hdr *h)
{
	uint8_t b[32];
	uint32_t left = rec_size(h);

	while (left > 0U) {
		const uint32_t n = MIN(left, sizeof(b));

		if (flash_read(flash, addr(area, from), b, n) != 0 ||
		    flash_write(flash, addr(a, off), b, n) != 0) {
			return 0U;
		}
		from += n;
		off += n;
		left -= n;
	}
	return off;
}

/* The live records but name's, and then the new one, into the other area; it becomes the one in use. */
static int move(const char *name, uint8_t name_len, const uint8_t *value, uint16_t val_len)
{
	static const uint32_t zero;
	const uint8_t to = area == 0U ? 1U : 0U;
	const struct area_hdr ah = {.magic = AREA_MAGIC, .seq = seq + 1U};
	const uint16_t crc = name_crc(name, name_len);
	uint32_t at = sizeof(struct area_hdr);
	struct rec_hdr h;
	int r;

	/*
	 * No magic in the target while it is erased: an erase cut short could
	 * otherwise leave its old header with a changed sequence number.
	 */
	if (flash_write(flash, addr(to, 0U), &zero, sizeof(zero)) != 0 ||
	    flash_erase(flash, addr(to, 0U), AREA_SIZE) != 0) {
		return -EIO;
	}
	for (uint32_t off = sizeof(struct area_hdr);
	     area != NO_AREA && off < end && (r = rec_at(area, off, &h, false)) > REC_END;
	     off += rec_size(&h)) {
		if (r != REC_LIVE || h.val_len == 0U || named(off, &h, name, name_len, crc)) {
			continue;
		}
		if (at + rec_size(&h) > AREA_SIZE) {
			return -ENOSPC;
		}
		at = copy(to, at, off, &h);
		if (at == 0U) {
			return -EIO;
		}
	}
	if (val_len > 0U) {
		if (at + ROUND_UP(sizeof(h) + name_len + val_len, 4U) > AREA_SIZE) {
			return -ENOSPC;
		}
		at = put(to, at, name, name_len, value, val_len);
		if (at == 0U) {
			return -EIO;
		}
	}
	/* The sequence number, then the magic: the header counts only when whole. */
	if (flash_write(flash, addr(to, offsetof(struct area_hdr, seq)), &ah.seq, sizeof(ah.seq)) != 0 ||
	    flash_write(flash, addr(to, 0U), &ah.magic, sizeof(ah.magic)) != 0) {
		return -EIO;
	}
	area = to;
	seq = ah.seq;
	end = at;
	full = false;
	return 0;
}

/*
 * The offset of name's newest live record in the area in use, with its header
 * in *found, or 0; *count the number of its live records.
 */
static uint32_t newest(const char *name, uint8_t name_len, uint16_t crc, struct rec_hdr *found,
		       uint32_t *count)
{
	struct rec_hdr h;
	uint32_t at = 0U;
	int r;

	*count = 0U;
	for (uint32_t off = sizeof(struct area_hdr);
	     area != NO_AREA && off < end && (r = rec_at(area, off, &h, false)) > REC_END;
	     off += rec_size(&h)) {
		if (r == REC_LIVE && named(off, &h, name, name_len, crc)) {
			at = off;
			*found = h;
			(*count)++;
		}
	}
	return at;
}

static void clear_at(uint32_t off)
{
	static const uint8_t dead;

	(void)flash_write(flash, addr(area, off), &dead, sizeof(dead));
}

/* Clears the magic of name's live records below limit. */
static void clear_older(const char *name, uint8_t name_len, uint16_t crc, uint32_t limit)
{
	struct rec_hdr h;
	int r;

	for (uint32_t off = sizeof(struct area_hdr);
	     area != NO_AREA && off < limit && (r = rec_at(area, off, &h, false)) > REC_END;
	     off += rec_size(&h)) {
		if (r == REC_LIVE && named(off, &h, name, name_len, crc)) {
			clear_at(off);
		}
	}
}

/* Whether the record h at off holds value. */
static bool holds(uint32_t off, const struct rec_hdr *h, const uint8_t *value, uint16_t val_len)
{
	uint8_t b[32];

	if (h->val_len != val_len) {
		return false;
	}
	for (uint16_t done = 0U; done < val_len;) {
		const uint16_t n = MIN(val_len - done, (uint16_t)sizeof(b));

		if (flash_read(flash, addr(area, off + sizeof(*h) + h->name_len + done), b, n) != 0 ||
		    memcmp(b, value + done, n) != 0) {
			return false;
		}
		done += n;
	}
	return true;
}

/* One pass over the log's headers finds the name's records; a second only after a cut left two. */
static int save(const char *name, uint8_t name_len, const uint8_t *value, uint16_t val_len)
{
	const uint16_t crc = name_crc(name, name_len);
	struct rec_hdr h;
	uint32_t count, at, next;
	const uint32_t old = newest(name, name_len, crc, &h, &count);

	if (val_len == 0U) {
		if (count == 1U) {
			clear_at(old);
		} else if (count > 1U) {
			clear_older(name, name_len, crc, end);
		}
		return 0;
	}
	if (old != 0U && holds(old, &h, value, val_len)) {
		return 0;
	}
	if (area == NO_AREA || full ||
	    end + ROUND_UP(sizeof(h) + name_len + val_len, 4U) > AREA_SIZE) {
		return move(name, name_len, value, val_len);
	}
	at = end;
	next = put(area, at, name, name_len, value, val_len);
	if (next == 0U) {
		full = true;
		return -EIO;
	}
	end = next;
	if (count == 1U) {
		clear_at(old);
	} else if (count > 1U) {
		clear_older(name, name_len, crc, at);
	}
	return 0;
}

static int log_save(struct settings_store *cs, const char *name, const char *value, size_t val_len)
{
	const size_t name_len = strlen(name);
	int err;

	ARG_UNUSED(cs);
	if (name_len == 0U || name_len > NAME_MAX || val_len > SETTINGS_MAX_VAL_LEN) {
		return -EINVAL;
	}
	err = save(name, name_len, (const uint8_t *)value, value == NULL ? 0U : val_len);
	/* A run of saves (a whole keymap, a reset of all settings) lets threads of this priority in. */
	k_yield();
	return err;
}

static const struct settings_store_itf log_itf = {
	.csi_load = log_load,
	.csi_save = log_save,
};

static struct settings_store log_store = {.cs_itf = &log_itf};

int settings_backend_init(void)
{
	if (!device_is_ready(flash)) {
		return -ENODEV;
	}
	tlsr_settings_log_scan();
	settings_src_register(&log_store);
	settings_dst_register(&log_store);
	return 0;
}
