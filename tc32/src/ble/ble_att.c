/*
 * The own BLE stack's ATT server and GATT database (Core Vol 3 Parts F and
 * G), in the BLE thread. ATT_MTU stays 23. The database is HID over GATT
 * for ZMK's reports, with ZMK's report map and IDs:
 *
 *   1-7   GAP: Device Name (the active profile's, ble_device_name()),
 *         Appearance keyboard, Peripheral Preferred Connection Parameters
 *         (the first connection parameter request's values)
 *   8     no attribute
 *   9-11  Device Information: PnP ID (USB IDs of the ZMK image); with
 *         TLSR_BLE_DIS_MANUFACTURER also a Manufacturer Name String,
 *         which moves every later handle by two (DIS_EXTRA; the numbers here
 *         are those without it)
 *   12-15 Battery: level, notify
 *   16-35 HID: Protocol Mode; keyboard input (ID 1) and consumer input
 *         (ID 2), each with CCCD and Report Reference; keyboard LED output
 *         (ID 1); Report Map; HID Information; HID Control Point. Every
 *         value and CCCD needs a secure link to be read or written (HOGP
 *         1.0 6.1: security mode 1, level 2, or level 3 in the Passkey
 *         build); the Report References stay readable, as the declarations
 *         do, so a client can sort the reports before it pairs
 *   36-40 HID, continued: Boot Keyboard Input Report (with CCCD) and Boot
 *         Keyboard Output Report, under the same security
 *   41-47 no attribute: room for the services above to grow
 *   48-51 GATT: Service Changed, indicate, with its CCCD
 *
 * Protocol Mode (HIDS 1.0 2.2): Report Protocol at every connection; a
 * client that writes Boot Protocol (0) gets the keyboard report as the boot
 * keyboard input report (the same 8 octets: modifiers, reserved, six keys)
 * on 0x2A22, with that characteristic's CCCD, and no consumer reports; its
 * LED byte goes from the Boot Keyboard Output Report to the same indicators
 * as the report-mode output report's. The two input characteristics read
 * the same last keyboard report, the two output ones the same LED byte.
 *
 * Handles 1-35 are those of images whose GATT service had no
 * characteristic (handle 8): a host that bonded with one of those and
 * keeps its handles finds the same ones. The GATT service sits at handles
 * of its own above the rest (ATT handles need not be contiguous, Core Vol
 * 3 Part F 3.2.2): adding it moved none of those, and Service Changed
 * stays at the handle a bonded host saw it at while later images add up
 * to seven attributes to the services above (GATT_HANDLE); a host looks
 * for the indication at the handle it discovered.
 *
 * Every 16-bit UUID; no attribute longer than MTU - 1 except the report
 * map, read with Read Blob. GAP, Device Information, Battery and GATT are
 * open. An attribute that needs a secure link, read or written on a link
 * that is not encrypted, is refused with Insufficient Encryption when a
 * bond for the client exists, else Insufficient Authentication (Core Vol 3
 * Part C 10.3.1), which makes a host pair; on an encrypted link whose key
 * Passkey Entry did not authenticate (the Passkey build), with Insufficient
 * Authentication. Notifications go out only on an
 * encrypted link with the CCCD set. A bonded client's CCCDs are kept with
 * its bond and set again when it reconnects with that bond's key (Core
 * Vol 3 Part G 3.3.3.3): a host that relies on that writes none after
 * reconnecting.
 * - When they go to the bond: each change a bonded client makes, at once
 *   for the link's first CCCD_SAVES_AT_ONCE records (a host's writes after
 *   pairing, so a power cut soon after loses none), then at most one record
 *   per CCCD_SAVE_GAP_MS, and what is left when the link ends: a host that
 *   toggles them cannot make a flash record per write, with a sector erase
 *   every 59.
 * - CCCDs written on this link before it was encrypted with the bond's key
 *   stand, and go to the bond at the encryption; only a link with no CCCD
 *   written gets the bond's.
 *
 * Service Changed (Core Vol 3 Part G 2.5.2, 7.1): a hash of the database
 * (every attribute's handle, type, permissions and length, the values that
 * never change, the report map among them; ble_att_db_hash()) is kept per
 * profile with the bond as the database its host last saw: set at the
 * pairing, whose discovery saw this one, and written to the bond log when
 * the link ends or the mode switch reboots the keyboard (ble.c). When a
 * bonded host reconnects with its key
 * and the database differs from the one it saw, the keyboard indicates
 * Service Changed over 0x0001-0xffff if the host turned its indications on,
 * and keeps the new hash once the host confirms; a link that ends before
 * the confirmation leaves the old hash, so the next reconnection indicates
 * again. A host that never turned the indications on cannot be told; its
 * hash is updated at once.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stddef.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zmk/hid.h>

#include "ble_internal.h"

#define ATT_CID 0x0004U
#define ATT_MTU 23U

#define CCCD_SAVES_AT_ONCE 4U
#define CCCD_SAVE_GAP_MS   10000U

#define ATT_ERROR_RSP           0x01U
#define ATT_MTU_REQ             0x02U
#define ATT_MTU_RSP             0x03U
#define ATT_FIND_INFO_REQ       0x04U
#define ATT_FIND_INFO_RSP       0x05U
#define ATT_FIND_TYPE_REQ       0x06U
#define ATT_FIND_TYPE_RSP       0x07U
#define ATT_READ_TYPE_REQ       0x08U
#define ATT_READ_TYPE_RSP       0x09U
#define ATT_READ_REQ            0x0aU
#define ATT_READ_RSP            0x0bU
#define ATT_READ_BLOB_REQ       0x0cU
#define ATT_READ_BLOB_RSP       0x0dU
#define ATT_READ_GROUP_REQ      0x10U
#define ATT_READ_GROUP_RSP      0x11U
#define ATT_WRITE_REQ           0x12U
#define ATT_WRITE_RSP           0x13U
#define ATT_NOTIFY              0x1bU
#define ATT_INDICATE            0x1dU
#define ATT_CONFIRM             0x1eU
#define ATT_WRITE_CMD           0x52U

#define ATT_ERR_INVALID_HANDLE   0x01U
#define ATT_ERR_READ_NOT_PERM    0x02U
#define ATT_ERR_WRITE_NOT_PERM   0x03U
#define ATT_ERR_NOT_SUPPORTED    0x06U
#define ATT_ERR_INVALID_OFFSET   0x07U
#define ATT_ERR_NOT_FOUND        0x0aU
#define ATT_ERR_ATTR_NOT_LONG    0x0bU
#define ATT_ERR_INVALID_LEN      0x0dU
#define ATT_ERR_INSUFFICIENT_ENC 0x0fU
#define ATT_ERR_INSUFFICIENT_AUTHEN 0x05U
#define ATT_ERR_UNSUPPORTED_GROUP 0x10U

#define UUID_PRIMARY   0x2800U
#define UUID_CHAR      0x2803U
#define UUID_NONE      0x0000U /* a handle with no attribute */
#define UUID_CCCD      0x2902U
#define UUID_REPORT_REF 0x2908U

#define PROP_READ      BIT(1)
#define PROP_WRITE_NR  BIT(2)
#define PROP_WRITE     BIT(3)
#define PROP_NOTIFY    BIT(4)
#define PROP_INDICATE  BIT(5)

/* Access: read, write; reading and writing need a secure link (link_secure()). */
#define A_R   BIT(0)
#define A_W   BIT(1)
#define A_E   BIT(2)

struct attr {
	uint16_t type;
	uint8_t access;
	uint8_t len;      /* the value's length (the report map's is its size) */
	const void *value; /* flash, or RAM for the values that change */
};

static const uint16_t svc_gap = 0x1800U, svc_gatt = 0x1801U, svc_dis = 0x180aU, svc_bas = 0x180fU,
		      svc_hid = 0x1812U;
static const uint8_t props_r = PROP_READ, props_rn = PROP_READ | PROP_NOTIFY,
		     props_rwn = PROP_READ | PROP_WRITE_NR,
		     props_out = PROP_READ | PROP_WRITE | PROP_WRITE_NR, props_wn = PROP_WRITE_NR,
		     props_ind = PROP_INDICATE;
static const uint8_t appearance[2] = {0xc1, 0x03}; /* keyboard */
/* the first connection parameter request's values (ble_link_request_conn_params()) */
static const uint8_t ppcp[8] = {
	CONFIG_TLSR_BLE_CONN_INTERVAL & 0xff, CONFIG_TLSR_BLE_CONN_INTERVAL >> 8,
	CONFIG_TLSR_BLE_CONN_INTERVAL & 0xff, CONFIG_TLSR_BLE_CONN_INTERVAL >> 8,
	CONFIG_TLSR_BLE_CONN_LATENCY & 0xff, CONFIG_TLSR_BLE_CONN_LATENCY >> 8,
	CONFIG_TLSR_BLE_CONN_TIMEOUT & 0xff, CONFIG_TLSR_BLE_CONN_TIMEOUT >> 8,
};
/* USB-IF vendor ID source, the image's USB IDs, version 1.00 */
static const uint8_t pnp_id[7] = {
	0x02, CONFIG_USB_DEVICE_VID & 0xff, CONFIG_USB_DEVICE_VID >> 8,
	CONFIG_USB_DEVICE_PID & 0xff, CONFIG_USB_DEVICE_PID >> 8, 0x00, 0x01,
};
#if IS_ENABLED(CONFIG_TLSR_BLE_DIS_MANUFACTURER_ON)
static const char manufacturer[] = CONFIG_TLSR_BLE_DIS_MANUFACTURER;
#define DIS_EXTRA 2U /* the handles the Manufacturer Name String adds */
#else
#define DIS_EXTRA 0U
#endif
static const uint8_t hid_info[4] = {0x11, 0x01, 0x00, 0x01}; /* HID 1.11, remote wake */
static const uint8_t ref_kbd_in[2] = {ZMK_HID_REPORT_ID_KEYBOARD, 1};
static const uint8_t ref_consumer_in[2] = {ZMK_HID_REPORT_ID_CONSUMER, 1};
static const uint8_t ref_leds_out[2] = {ZMK_HID_REPORT_ID_LEDS, 2};

#define PROTOCOL_BOOT 0U

/* The GATT service's first handle, which no change to the other services moves. */
#define GATT_HANDLE 48U

static struct {
	uint8_t battery;
	uint8_t protocol_mode;
	uint8_t kbd_in[8];
	uint8_t consumer_in[sizeof(struct zmk_hid_consumer_report_body)];
	uint8_t leds_out;
	uint8_t control_point;
	uint8_t cccd_battery[2];
	uint8_t cccd_kbd[2];
	uint8_t cccd_consumer[2];
	uint8_t cccd_sc[2];
	uint8_t cccd_boot_in[2];
} v = {.battery = 100, .protocol_mode = 1};

BUILD_ASSERT(sizeof(v.consumer_in) <= ATT_MTU - 3, "a consumer report fits one notification");
BUILD_ASSERT(IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_HKRO) && CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE == 6,
	     "the keyboard report body is the boot keyboard input report");
BUILD_ASSERT(sizeof(zmk_hid_report_desc) <= 0xff, "the report map's length fits the table");

static const struct attr db[] = {
	/* 1 */ {UUID_PRIMARY, A_R, 2, &svc_gap},
	/* 2 */ {UUID_CHAR, A_R, 1, &props_r},
	/* 3 */ {0x2a00, A_R, 0, NULL}, /* the profile's name, from value_of() */
	/* 4 */ {UUID_CHAR, A_R, 1, &props_r},
	/* 5 */ {0x2a01, A_R, 2, appearance},
	/* 6 */ {UUID_CHAR, A_R, 1, &props_r},
	/* 7 */ {0x2a04, A_R, 8, ppcp},
	/* 8 */ {UUID_NONE, 0, 0, NULL},
	/* 9 */ {UUID_PRIMARY, A_R, 2, &svc_dis},
	/* 10 */ {UUID_CHAR, A_R, 1, &props_r},
	/* 11 */ {0x2a50, A_R, 7, pnp_id},
#if IS_ENABLED(CONFIG_TLSR_BLE_DIS_MANUFACTURER_ON)
	{UUID_CHAR, A_R, 1, &props_r},
	{0x2a29, A_R, sizeof(manufacturer) - 1U, manufacturer},
#endif
	/* 12 */ {UUID_PRIMARY, A_R, 2, &svc_bas},
	/* 13 */ {UUID_CHAR, A_R, 1, &props_rn},
	/* 14 */ {0x2a19, A_R, 1, &v.battery},
	/* 15 */ {UUID_CCCD, A_R | A_W, 2, v.cccd_battery},
	/* 16 */ {UUID_PRIMARY, A_R, 2, &svc_hid},
	/* 17 */ {UUID_CHAR, A_R, 1, &props_rwn},
	/* 18 */ {0x2a4e, A_R | A_W | A_E, 1, &v.protocol_mode},
	/* 19 */ {UUID_CHAR, A_R, 1, &props_rn},
	/* 20 */ {0x2a4d, A_R | A_E, sizeof(v.kbd_in), v.kbd_in},
	/* 21 */ {UUID_CCCD, A_R | A_W | A_E, 2, v.cccd_kbd},
	/* 22 */ {UUID_REPORT_REF, A_R, 2, ref_kbd_in},
	/* 23 */ {UUID_CHAR, A_R, 1, &props_rn},
	/* 24 */ {0x2a4d, A_R | A_E, sizeof(v.consumer_in), v.consumer_in},
	/* 25 */ {UUID_CCCD, A_R | A_W | A_E, 2, v.cccd_consumer},
	/* 26 */ {UUID_REPORT_REF, A_R, 2, ref_consumer_in},
	/* 27 */ {UUID_CHAR, A_R, 1, &props_out},
	/* 28 */ {0x2a4d, A_R | A_W | A_E, 1, &v.leds_out},
	/* 29 */ {UUID_REPORT_REF, A_R, 2, ref_leds_out},
	/* 30 */ {UUID_CHAR, A_R, 1, &props_r},
	/* 31 */ {0x2a4b, A_R | A_E, sizeof(zmk_hid_report_desc), zmk_hid_report_desc},
	/* 32 */ {UUID_CHAR, A_R, 1, &props_r},
	/* 33 */ {0x2a4a, A_R | A_E, 4, hid_info},
	/* 34 */ {UUID_CHAR, A_R, 1, &props_wn},
	/* 35 */ {0x2a4c, A_W | A_E, 1, &v.control_point},
	/* 36 */ {UUID_CHAR, A_R, 1, &props_rn},
	/* 37 */ {0x2a22, A_R | A_E, sizeof(v.kbd_in), v.kbd_in}, /* the keyboard report's octets */
	/* 38 */ {UUID_CCCD, A_R | A_W | A_E, 2, v.cccd_boot_in},
	/* 39 */ {UUID_CHAR, A_R, 1, &props_out},
	/* 40 */ {0x2a32, A_R | A_W | A_E, 1, &v.leds_out}, /* the LED byte, either way */
	/* 41-47: none (zeros, UUID_NONE) */
	/* 48 */ [GATT_HANDLE - 1U] = {UUID_PRIMARY, A_R, 2, &svc_gatt},
	/* 49 */ {UUID_CHAR, A_R, 1, &props_ind},
	/* 50 */ {0x2a05, 0, 4, NULL}, /* indicated only */
	/* 51 */ {UUID_CCCD, A_R | A_W, 2, v.cccd_sc},
};

#define HANDLE_NAME       3U
#define HANDLE_BATTERY    (14U + DIS_EXTRA)
#define HANDLE_KBD_IN     (20U + DIS_EXTRA)
#define HANDLE_CONSUMER_IN (24U + DIS_EXTRA)
#define HANDLE_LEDS_OUT   (28U + DIS_EXTRA)
#define HANDLE_BOOT_IN    (37U + DIS_EXTRA)
#define HANDLE_BOOT_OUT   (40U + DIS_EXTRA)
#define HANDLE_SC         (GATT_HANDLE + 2U)
#define HANDLES           ARRAY_SIZE(db)

BUILD_ASSERT(HANDLE_BOOT_OUT < GATT_HANDLE, "the other services end below the GATT service");

static const struct attr *attr_at(uint16_t h)
{
	return (h >= 1U && h <= HANDLES && db[h - 1U].type != UUID_NONE) ? &db[h - 1U] : NULL;
}

/* A characteristic declaration's value: properties, value handle, UUID. */
static uint8_t char_decl(uint16_t h, uint8_t out[5])
{
	out[0] = *(const uint8_t *)db[h - 1U].value;
	sys_put_le16(h + 1U, &out[1]);
	sys_put_le16(db[h].type, &out[3]);
	return 5;
}

/* The attribute's value, or its characteristic declaration's. */
static const uint8_t *value_of(uint16_t h, uint8_t *len, uint8_t scratch[5])
{
	const struct attr *a = attr_at(h);

	if (a->type == UUID_CHAR) {
		*len = char_decl(h, scratch);
		return scratch;
	}
	if (h == HANDLE_NAME) {
		static char name[BLE_NAME_MAX];

		*len = ble_device_name(ble_profile_active(), name);
		return (const uint8_t *)name;
	}
	*len = a->value != NULL ? a->len : 0U;
	return a->value;
}

/* The last attribute of the service declared at h. */
static uint16_t group_end(uint16_t h)
{
	uint16_t n = h + 1U;

	while (n <= HANDLES && db[n - 1U].type != UUID_PRIMARY) {
		n++;
	}
	while (db[n - 2U].type == UUID_NONE) {
		n--;
	}
	return n - 1U;
}

static void send(const uint8_t *pdu, uint8_t len)
{
	(void)ble_l2cap_send(ATT_CID, pdu, len);
}

static void error(uint8_t req, uint16_t h, uint8_t err)
{
	uint8_t pdu[5] = {ATT_ERROR_RSP, req};

	sys_put_le16(h, &pdu[2]);
	pdu[4] = err;
	send(pdu, sizeof(pdu));
}

/* Service Changed to the bonded client of this link (the header's rules). */
enum { SC_IDLE, SC_DUE, SC_SENT };

/* The link's CCCD records to the bond (the header's rules), and its Service Changed. */
static struct {
	bool written;     /* a CCCD written on this link */
	bool pending;     /* a bonded client's change not yet in the bond */
	uint8_t profile;  /* the bond it goes to */
	uint8_t saves;    /* records written on this link */
	uint8_t sc;       /* SC_IDLE, SC_DUE (to send), SC_SENT (waiting for the confirmation) */
	uint32_t saved_at;
} cs;

static void cccd_save(void)
{
	cs.pending = false;
	if (ble_bond_set_cccd(cs.profile, ble_att_cccd_bits())) {
		cs.saves++;
		cs.saved_at = k_uptime_get_32();
	}
}

/* A bonded client's CCCDs changed. */
static void cccd_changed(void)
{
	cs.pending = true;
	cs.profile = ble_profile_active();
	if (cs.saves < CCCD_SAVES_AT_ONCE) {
		cccd_save();
	}
}

void ble_att_cccd_poll(void)
{
	if (cs.pending && (uint32_t)(k_uptime_get_32() - cs.saved_at) >= CCCD_SAVE_GAP_MS) {
		cccd_save();
	}
	if (cs.sc == SC_DUE && ble_link_bonded()) {
		/* the whole handle range: the client discovers it all again */
		uint8_t pdu[7] = {ATT_INDICATE, HANDLE_SC, 0, 0x01, 0x00, 0xff, 0xff};

		if (ble_l2cap_send(ATT_CID, pdu, sizeof(pdu)) == 0) {
			cs.sc = SC_SENT;
		}
	}
}

uint32_t ble_att_db_hash(void)
{
	static uint32_t hash;

	if (hash == 0U) {
		uint32_t c = 0U;

		/* each handle's type, permissions and length in turn (a handle with no attribute: zeros) */
		for (const struct attr *a = db; a < &db[HANDLES]; a++) {
			const uint8_t *val = a->value;

			c = crc32_ieee_update(c, (const uint8_t *)a, offsetof(struct attr, value));
			/* the values that never change; those in RAM vary while the keyboard runs */
			if (val != NULL && !(val >= (const uint8_t *)&v && val < (const uint8_t *)(&v + 1))) {
				c = crc32_ieee_update(c, val, a->len);
			}
		}
		hash = c != 0U ? c : 1U; /* 0 is a profile's "no hash kept" */
	}
	return hash;
}

void ble_att_reset(void)
{
	if (cs.pending) {
		cccd_save(); /* the link ends: what its client left */
	}
	ble_bond_save_db_seen();
	memset(&cs, 0, sizeof(cs));
	memset(v.cccd_sc, 0, sizeof(v.cccd_sc));
	memset(v.cccd_boot_in, 0, sizeof(v.cccd_boot_in));
	memset(v.cccd_battery, 0, sizeof(v.cccd_battery));
	memset(v.cccd_kbd, 0, sizeof(v.cccd_kbd));
	memset(v.cccd_consumer, 0, sizeof(v.cccd_consumer));
	v.protocol_mode = 1;
}

static bool range_ok(uint8_t req, uint16_t start, uint16_t end)
{
	if (start == 0U || start > end) {
		error(req, start, ATT_ERR_INVALID_HANDLE);
		return false;
	}
	return true;
}

static void find_info(const uint8_t *p, uint8_t len)
{
	uint8_t rsp[ATT_MTU] = {ATT_FIND_INFO_RSP, 0x01};
	uint8_t n = 2;
	uint16_t start, end;

	if (len != 5U) {
		error(p[0], 0, ATT_ERR_INVALID_LEN);
		return;
	}
	start = sys_get_le16(&p[1]);
	end = sys_get_le16(&p[3]);
	if (!range_ok(p[0], start, end)) {
		return;
	}
	for (uint16_t h = start; h <= MIN(end, HANDLES) && n + 4U <= ATT_MTU; h++) {
		if (db[h - 1U].type == UUID_NONE) {
			continue;
		}
		sys_put_le16(h, &rsp[n]);
		sys_put_le16(db[h - 1U].type, &rsp[n + 2U]);
		n += 4U;
	}
	if (n == 2U) {
		error(p[0], start, ATT_ERR_NOT_FOUND);
	} else {
		send(rsp, n);
	}
}

static void find_by_type(const uint8_t *p, uint8_t len)
{
	uint8_t rsp[ATT_MTU] = {ATT_FIND_TYPE_RSP};
	uint8_t n = 1, vlen, scratch[5];
	uint16_t start, end;

	if (len < 7U) {
		error(p[0], 0, ATT_ERR_INVALID_LEN);
		return;
	}
	start = sys_get_le16(&p[1]);
	end = sys_get_le16(&p[3]);
	if (!range_ok(p[0], start, end)) {
		return;
	}
	for (uint16_t h = start; h <= MIN(end, HANDLES) && n + 4U <= ATT_MTU; h++) {
		const uint8_t *val;

		if (db[h - 1U].type != sys_get_le16(&p[5]) || db[h - 1U].type == UUID_NONE) {
			continue;
		}
		val = value_of(h, &vlen, scratch);
		if (vlen == len - 7U && memcmp(val, &p[7], vlen) == 0) {
			sys_put_le16(h, &rsp[n]);
			sys_put_le16(db[h - 1U].type == UUID_PRIMARY ? group_end(h) : h, &rsp[n + 2U]);
			n += 4U;
		}
	}
	if (n == 1U) {
		error(p[0], start, ATT_ERR_NOT_FOUND);
	} else {
		send(rsp, n);
	}
}

/* Encrypted, and with Passkey Entry pairing (TLSR_BLE_SC_PASSKEY) with a key that pairing authenticated. */
static bool link_secure(void)
{
	return ble_link_encrypted() &&
	       (!IS_ENABLED(CONFIG_TLSR_BLE_SC_PASSKEY) || ble_link_authenticated());
}

/*
 * Why the attribute cannot be read (need A_R) or written (A_W) now, or 0.
 * One that needs a secure link, on a link that is not (Core Vol 3 Part C
 * 10.3.1): Insufficient Encryption when the link is not encrypted and the
 * keyboard holds a key for this client (the profile's bond is its), so
 * that it encrypts with that key; else Insufficient Authentication: no key
 * for the client, so that it pairs, or an encrypted link whose key Passkey
 * Entry did not authenticate.
 */
static uint8_t access_error(const struct attr *a, uint8_t need)
{
	BUILD_ASSERT(ATT_ERR_READ_NOT_PERM + (A_R >> 1) == ATT_ERR_READ_NOT_PERM &&
		     ATT_ERR_READ_NOT_PERM + (A_W >> 1) == ATT_ERR_WRITE_NOT_PERM);

	if ((a->access & need) == 0U) {
		return ATT_ERR_READ_NOT_PERM + (need >> 1);
	}
	if ((a->access & A_E) != 0U && !link_secure()) {
		return !ble_link_encrypted() && ble_link_peer_bonded() ? ATT_ERR_INSUFFICIENT_ENC
								     : ATT_ERR_INSUFFICIENT_AUTHEN;
	}
	return 0U;
}

static bool readable(uint8_t req, uint16_t h)
{
	uint8_t err = access_error(attr_at(h), A_R);

	if (err != 0U) {
		error(req, h, err);
		return false;
	}
	return true;
}

static void read_by_type(const uint8_t *p, uint8_t len)
{
	uint8_t rsp[ATT_MTU] = {ATT_READ_TYPE_RSP, 0};
	uint8_t n = 2, vlen, scratch[5];
	uint16_t start, end;

	if (len != 7U) {
		error(p[0], 0, len == 21U ? ATT_ERR_NOT_FOUND : ATT_ERR_INVALID_LEN);
		return;
	}
	start = sys_get_le16(&p[1]);
	end = sys_get_le16(&p[3]);
	if (!range_ok(p[0], start, end)) {
		return;
	}
	for (uint16_t h = start; h <= MIN(end, HANDLES); h++) {
		const uint8_t *val;
		uint8_t take;

		if (db[h - 1U].type != sys_get_le16(&p[5]) || db[h - 1U].type == UUID_NONE) {
			continue;
		}
		if (rsp[1] == 0U && !readable(p[0], h)) {
			return;
		}
		val = value_of(h, &vlen, scratch);
		take = MIN(vlen, ATT_MTU - 4U);
		if (rsp[1] == 0U) {
			rsp[1] = take + 2U;
		} else if (rsp[1] != take + 2U || n + rsp[1] > ATT_MTU || access_error(&db[h - 1U], A_R) != 0U) {
			break;
		}
		sys_put_le16(h, &rsp[n]);
		memcpy(&rsp[n + 2U], val, take);
		n += rsp[1];
		if (n + rsp[1] > ATT_MTU) {
			break;
		}
	}
	if (n == 2U) {
		error(p[0], start, ATT_ERR_NOT_FOUND);
	} else {
		send(rsp, n);
	}
}

static void read_by_group(const uint8_t *p, uint8_t len)
{
	uint8_t rsp[ATT_MTU] = {ATT_READ_GROUP_RSP, 6};
	uint8_t n = 2;
	uint16_t start, end;

	if (len != 7U) {
		error(p[0], 0, len == 21U ? ATT_ERR_UNSUPPORTED_GROUP : ATT_ERR_INVALID_LEN);
		return;
	}
	start = sys_get_le16(&p[1]);
	end = sys_get_le16(&p[3]);
	if (!range_ok(p[0], start, end)) {
		return;
	}
	if (sys_get_le16(&p[5]) != UUID_PRIMARY) {
		error(p[0], start, ATT_ERR_UNSUPPORTED_GROUP);
		return;
	}
	for (uint16_t h = start; h <= MIN(end, HANDLES) && n + 6U <= ATT_MTU; h++) {
		if (db[h - 1U].type != UUID_PRIMARY) {
			continue;
		}
		sys_put_le16(h, &rsp[n]);
		sys_put_le16(group_end(h), &rsp[n + 2U]);
		memcpy(&rsp[n + 4U], db[h - 1U].value, 2);
		n += 6U;
	}
	if (n == 2U) {
		error(p[0], start, ATT_ERR_NOT_FOUND);
	} else {
		send(rsp, n);
	}
}

static void read(const uint8_t *p, uint8_t len, bool blob)
{
	uint8_t rsp[ATT_MTU] = {blob ? ATT_READ_BLOB_RSP : ATT_READ_RSP};
	uint8_t vlen, scratch[5];
	uint16_t h, off = 0;
	const uint8_t *val;

	if (len != (blob ? 5U : 3U)) {
		error(p[0], 0, ATT_ERR_INVALID_LEN);
		return;
	}
	h = sys_get_le16(&p[1]);
	if (attr_at(h) == NULL) {
		error(p[0], h, ATT_ERR_INVALID_HANDLE);
		return;
	}
	if (!readable(p[0], h)) {
		return;
	}
	val = value_of(h, &vlen, scratch);
	if (blob) {
		off = sys_get_le16(&p[3]);
		if (off > vlen) {
			error(p[0], h, ATT_ERR_INVALID_OFFSET);
			return;
		}
	}
	uint8_t take = MIN((uint8_t)(vlen - off), ATT_MTU - 1U);

	memcpy(&rsp[1], val + off, take);
	send(rsp, take + 1U);
}

static void write(const uint8_t *p, uint8_t len, bool cmd)
{
	uint16_t h;
	const struct attr *a;
	uint8_t vlen = len - 3U;

	if (len < 3U) {
		if (!cmd) {
			error(p[0], 0, ATT_ERR_INVALID_LEN);
		}
		return;
	}
	h = sys_get_le16(&p[1]);
	a = attr_at(h);
	if (a == NULL || access_error(a, A_W) != 0U) {
		if (!cmd) {
			error(p[0], h, a == NULL ? ATT_ERR_INVALID_HANDLE : access_error(a, A_W));
		}
		return;
	}
	if (vlen != a->len) {
		if (!cmd) {
			error(p[0], h, ATT_ERR_INVALID_LEN);
		}
		return;
	}
	memcpy((void *)a->value, &p[3], vlen);
	if (h == HANDLE_LEDS_OUT || h == HANDLE_BOOT_OUT) {
		ble_hid_leds(v.leds_out);
	}
	if (a->type == UUID_CCCD) {
		cs.written = true;
		if (ble_link_bonded()) {
			/* a bonded client's CCCDs are kept for its next connection */
			cccd_changed();
		}
	}
	if (!cmd) {
		uint8_t rsp[1] = {ATT_WRITE_RSP};

		send(rsp, sizeof(rsp));
	}
}

void ble_att_rx(const uint8_t *p, uint8_t len)
{
	if (len == 0U) {
		return;
	}
	switch (p[0]) {
	case ATT_MTU_REQ: {
		uint8_t rsp[3] = {ATT_MTU_RSP, ATT_MTU, 0};

		send(rsp, sizeof(rsp));
		break;
	}
	case ATT_FIND_INFO_REQ:
		find_info(p, len);
		break;
	case ATT_FIND_TYPE_REQ:
		find_by_type(p, len);
		break;
	case ATT_READ_TYPE_REQ:
		read_by_type(p, len);
		break;
	case ATT_READ_REQ:
		read(p, len, false);
		break;
	case ATT_READ_BLOB_REQ:
		read(p, len, true);
		break;
	case ATT_READ_GROUP_REQ:
		read_by_group(p, len);
		break;
	case ATT_WRITE_REQ:
		write(p, len, false);
		break;
	case ATT_WRITE_CMD:
		write(p, len, true);
		break;
	case ATT_CONFIRM:
		if (cs.sc == SC_SENT) {
			/* the client has the new database: it is the one its bond saw */
			cs.sc = SC_IDLE;
			ble_bond_set_db_seen(ble_profile_active(), ble_att_db_hash());
		}
		break;
	default:
		/* commands (bit 6) and the server's own opcodes get no answer */
		if ((p[0] & 0x40U) == 0U && p[0] != ATT_NOTIFY && p[0] != ATT_INDICATE) {
			error(p[0], 0, ATT_ERR_NOT_SUPPORTED);
		}
		break;
	}
}

static int notify(uint16_t h, const uint8_t cccd[2], uint8_t *store, const uint8_t *data,
		  uint8_t len)
{
	uint8_t pdu[ATT_MTU] = {ATT_NOTIFY};

	memcpy(store, data, len);
	if (!link_secure() || (cccd[0] & 0x01U) == 0U) {
		return -EAGAIN;
	}
	sys_put_le16(h, &pdu[1]);
	memcpy(&pdu[3], data, len);
	return ble_l2cap_send(ATT_CID, pdu, len + 3U);
}

int ble_att_notify_keyboard(const uint8_t report[8])
{
	if (v.protocol_mode == PROTOCOL_BOOT) {
		return notify(HANDLE_BOOT_IN, v.cccd_boot_in, v.kbd_in, report, sizeof(v.kbd_in));
	}
	return notify(HANDLE_KBD_IN, v.cccd_kbd, v.kbd_in, report, sizeof(v.kbd_in));
}

int ble_att_notify_consumer(const uint8_t *report, uint8_t len)
{
	if (v.protocol_mode == PROTOCOL_BOOT) {
		return 0; /* no consumer reports in boot protocol */
	}
	return notify(HANDLE_CONSUMER_IN, v.cccd_consumer, v.consumer_in, report,
		      MIN(len, (uint8_t)sizeof(v.consumer_in)));
}

uint8_t ble_att_cccd_bits(void)
{
	return ((v.cccd_battery[0] & 0x01U) ? BLE_CCCD_BATTERY : 0U) |
	       ((v.cccd_kbd[0] & 0x01U) ? BLE_CCCD_KEYBOARD : 0U) |
	       ((v.cccd_consumer[0] & 0x01U) ? BLE_CCCD_CONSUMER : 0U) |
	       ((v.cccd_sc[0] & 0x02U) ? BLE_CCCD_SERVICE_CHANGED : 0U) |
	       ((v.cccd_boot_in[0] & 0x01U) ? BLE_CCCD_BOOT_KEYBOARD : 0U);
}

void ble_att_bond_encrypted(uint8_t bits)
{
	uint8_t profile = ble_profile_active();

	if (cs.written) {
		cccd_changed(); /* the client set them on this link: those go to the bond */
	} else {
		v.cccd_battery[0] = (bits & BLE_CCCD_BATTERY) ? 0x01U : 0x00U;
		v.cccd_kbd[0] = (bits & BLE_CCCD_KEYBOARD) ? 0x01U : 0x00U;
		v.cccd_consumer[0] = (bits & BLE_CCCD_CONSUMER) ? 0x01U : 0x00U;
		v.cccd_sc[0] = (bits & BLE_CCCD_SERVICE_CHANGED) ? 0x02U : 0x00U;
		v.cccd_boot_in[0] = (bits & BLE_CCCD_BOOT_KEYBOARD) ? 0x01U : 0x00U;
		v.cccd_battery[1] = 0U;
		v.cccd_kbd[1] = 0U;
		v.cccd_consumer[1] = 0U;
		v.cccd_sc[1] = 0U;
		v.cccd_boot_in[1] = 0U;
	}
	if (ble_bond_db_seen(profile) != ble_att_db_hash()) {
		if ((v.cccd_sc[0] & 0x02U) != 0U) {
			cs.sc = SC_DUE; /* sent by ble_att_cccd_poll() */
		} else {
			ble_bond_set_db_seen(profile, ble_att_db_hash());
		}
	}
}

bool ble_att_ready(void)
{
	return link_secure() &&
	       ((v.protocol_mode == PROTOCOL_BOOT ? v.cccd_boot_in[0] : v.cccd_kbd[0]) & 0x01U) != 0U;
}

/* The BLE thread: the battery level to read, notified when it changes. */
void ble_att_battery_level(uint8_t level)
{
	static bool pending;

	if (level == v.battery && !pending) {
		return;
	}
	uint8_t value = level;

	pending = notify(HANDLE_BATTERY, v.cccd_battery, &v.battery, &value, 1) == -ENOBUFS;
}
