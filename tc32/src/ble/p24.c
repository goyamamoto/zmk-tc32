/*
 * The Cidoo keyboards' proprietary 2.4G link to their USB dongle, run from the
 * BLE thread in the 2.4G position of the mode switch (CONFIG_TLSR_P24):
 *
 * - Radio: the private 2M mode (zephyr-tc32's tlsr_radio.c), on four channels, 2405,
 *   2422, 2440 and 2460 MHz. One exchange: this side sends,
 *   then listens 400 us (450 us after a control packet) on the same channel
 *   and access code; no valid reply within 1.2 ms of the start is a failure.
 *   The channel moves on after the third failure in a row and after every
 *   failure from then on.
 * - Pairing (state 1): when no dongle ID is kept, or after Fn + KP4 held 3 s
 *   (&p24_pair). Beacons on the pairing access code at the per-unit beacon
 *   level until a pairing reply gives the dongle ID; 60 s without one ends in
 *   deep sleep.
 * - Linked (state 3) and reconnecting (state 2): report records on the
 *   dongle's access code, each sent until an ACK or a command answers it;
 *   more than 600 failures in a row fall back to state 2, where any answer
 *   brings the link back. A heartbeat goes when nothing else is queued:
 *   every 10 ms, every 26 ms once no key has come for 2 s. A consumer record
 *   holds one 16-bit usage: of ZMK's consumer report the first goes. The ACK
 *   carries the host's keyboard LEDs, handed to ble_hid_leds() (used only
 *   with CONFIG_ZMK_HID_INDICATORS). Commands: 0xb3 stops the packets and
 *   sleeps 0.6 s later, 0xb4 is stored
 *   and answered with 0xb5; VIA requests (0xb0) are not answered.
 * - Power: in state 3, 300 s without a key stops all traffic and suspends
 *   the chip until a key; 30 min without a key sleeps. State 2 sleeps after
 *   20 s without a key. A key wakes from deep sleep (a boot); that key is
 *   not sent (ZMK's reports go to the link only once the dongle answers),
 *   while the key that ends the low-power state is. The times are
 *   CONFIG_TLSR_P24_IDLE_S, _SLEEP_S, _PAIR_S and _RECONNECT_S. While a USB
 *   host has the keyboard over the cable (ble_usb_host(): ZMK's HID ready
 *   and power in) there is neither deep sleep nor low-power state: a linked
 *   keyboard starts its times again, and one in state 1 or 2 stops sending
 *   where it would sleep on the battery, until a key or the knob; once the
 *   cable is pulled (no power in), the deep sleep comes. With
 *   TLSR_USB_WIRED_POSITION_ONLY no USB host has it in this position, and
 *   the times run as on the battery.
 * - A reply's time stamp (0x450) and RSSI (the octet before its status
 *   byte) go to tc32_rng_add_event() and tc32_rng_add_sample().
 * - Kept in flash: the dongle ID and the 0xb4 bytes, in the storage
 *   partition's fourth sector (a log of 16-octet records). The keyboard ID
 *   is the per-unit bytes 0-2 (read only).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_internal.h"
#include "key_matrix_sleep.h"
#include <tlsr_radio.h>
#include "tlsr_slots.h"
#include <tlsr8278_suspend.h>

#define PAIR_CODE       0x4a5b6c55U
#define AC_SEARCH       0x71U
#define AC_LINKED       0x77U
#define WINDOW_US       400U
#define WINDOW_CTRL_US  450U
#define EXCHANGE_US     1200U
#define FAILS_TO_STATE2 600U
#define SETTLE_FAST       2U
#define SETTLE_MARK       1U

/* Times in kernel ticks (ticks_now()), every one a whole number of ticks. */
#define PAIR_TIMEOUT      BLE_MS_TICKS(CONFIG_TLSR_P24_PAIR_S * 1000U)
#define STATE2_IDLE       BLE_MS_TICKS(CONFIG_TLSR_P24_RECONNECT_S * 1000U)
#define LOW_POWER         BLE_MS_TICKS(CONFIG_TLSR_P24_IDLE_S * 1000U)
#define LINKED_IDLE       BLE_MS_TICKS(CONFIG_TLSR_P24_SLEEP_S * 1000U)
#define SLEEP_CMD         BLE_MS_TICKS(600U)
#define HEARTBEAT_FAST    BLE_MS_TICKS(10U)
#define HEARTBEAT_SLOW    BLE_MS_TICKS(26U)
#define HEARTBEAT_SLOW_AFTER BLE_MS_TICKS(2000U)
#define SETTLE_NORMAL     BLE_MS_TICKS(2000U)
#define LATER_MAX         BLE_MS_TICKS(1000U) /* since(): a time this much after now counts as now */

BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC % 1000 == 0, "a millisecond is a whole number of kernel ticks");
BUILD_ASSERT((uint64_t)CONFIG_TLSR_P24_PAIR_S * CONFIG_SYS_CLOCK_TICKS_PER_SEC < BIT64(31) &&
	     (uint64_t)CONFIG_TLSR_P24_RECONNECT_S * CONFIG_SYS_CLOCK_TICKS_PER_SEC < BIT64(31) &&
	     (uint64_t)CONFIG_TLSR_P24_IDLE_S * CONFIG_SYS_CLOCK_TICKS_PER_SEC < BIT64(31) &&
	     (uint64_t)CONFIG_TLSR_P24_SLEEP_S * CONFIG_SYS_CLOCK_TICKS_PER_SEC < BIT64(31),
	     "the 2.4G link's times fit in 31 bits of kernel ticks");

/*
 * The kernel ticks that cover a wait of us microseconds, rounded up as
 * K_USEC() rounds, for us up to EXCHANGE_US, by a multiply and a shift: with
 * M = ceil(2^S / U) and E = M * U - 2^S, (x * M) >> S equals x / U for every
 * x with x * E < 2^S (U the microseconds per tick, x = us + U - 1).
 */
#define US_PER_TICK (1000000U / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define CEIL_SHIFT  19U
#define CEIL_MUL    ((BIT(CEIL_SHIFT) + US_PER_TICK - 1U) / US_PER_TICK)

BUILD_ASSERT(1000000 % CONFIG_SYS_CLOCK_TICKS_PER_SEC == 0, "a tick is a whole number of microseconds");
BUILD_ASSERT((uint64_t)(EXCHANGE_US + US_PER_TICK - 1U) * (CEIL_MUL * US_PER_TICK - BIT(CEIL_SHIFT)) <
	     BIT(CEIL_SHIFT), "the multiply and shift divide exactly up to EXCHANGE_US");
BUILD_ASSERT((uint64_t)(EXCHANGE_US + US_PER_TICK - 1U) * CEIL_MUL <= UINT32_MAX,
	     "the multiply fits in 32 bits up to EXCHANGE_US");

static uint32_t us_to_ticks_ceil(uint32_t us)
{
	return ((us + US_PER_TICK - 1U) * CEIL_MUL) >> CEIL_SHIFT;
}

#define ST_PAIRING      1U
#define ST_RECONNECTING 2U
#define ST_LINKED       3U

#define TAG_HEARTBEAT 0x00U
#define TAG_MOUSE     0x23U
#define TAG_KEYBOARD  0x24U
#define TAG_CONSUMER  0x25U
#define TAG_SYSTEM    0x26U
#define TAG_NKRO      0x27U

#define PAYLOAD_MAX 49U
#define ENTRY_MAX   18U
#define QUEUE_LEN   16U

#define STORE_SECTOR 4096U
#define STORE_BASE   (DT_REG_ADDR(DT_NODELABEL(storage_partition)) + 3U * STORE_SECTOR)
#define REC_LEN      16U
#define REC_MAGIC    0xd4U

BUILD_ASSERT(DT_REG_SIZE(DT_NODELABEL(storage_partition)) >= 4U * STORE_SECTOR,
	     "the storage partition's fourth sector");

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define P24_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define P24_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

static const struct device *const flash_dev = DEVICE_DT_GET(P24_FLASH_NODE);

/* The channels: 2400 + these MHz; the link uses every fourth (chan moves by 4). */
static const int8_t chan_off[16] = {5, 9, 13, 17, 22, 26, 30, 35, 40, 45, 50, 55, 60, 65, 70, 76};

/* DMA buffers: a u32 DMA length, the header (the payload length), the payload. */
static uint8_t tx_buf[4 + 1 + PAYLOAD_MAX] __aligned(4);
static uint8_t rx_buf[64] __aligned(4);

static struct {
	uint8_t state;
	uint8_t chan;         /* index into chan_off, moved by 4 */
	uint8_t seq;
	uint8_t id[3];        /* the keyboard ID, per-unit bytes 0-2 */
	uint8_t level_beacon; /* per-unit byte 10 */
	uint8_t level_data;   /* per-unit byte 9 */
	uint32_t dongle;      /* the dongle ID, as its pairing reply gives it (LE) */
	uint32_t image_crc;   /* the running image's CRC-32 trailer, for the beacon */
	uint16_t fails;       /* failures in a row */
	uint8_t settle;       /* SETTLE_FAST, SETTLE_MARK or 0 (exchange()) */
	uint32_t settle_at;   /* when the settle was last marked (ticks_now(), as all times here) */
	uint8_t b4[3];        /* the dongle's 0xb4 bytes */
	bool b5_pending;      /* the answer to a 0xb4 goes with the next packet */
	uint32_t sleep_at;    /* a 0xb3 asked for deep sleep at this ticks_now() */
	bool sleep_asked;
	uint32_t state_since;
	uint32_t last_hb;
	bool low;             /* the low-power state: no traffic */
	bool paused;          /* state 1 or 2 with a USB host on the cable, past its time: no traffic */
} p;

static volatile uint32_t last_key; /* ticks_now() */
static volatile bool key_seen; /* p24_activity(): a key or the knob */

/* What the link has done, for the emulator's checks (read by symbol). */
struct cidoo_p24_stats {
	uint32_t exchanges;
	uint32_t answered;
	uint32_t failures;
	uint32_t low_enters;
	uint32_t low_exits;
	uint32_t tx_while_low; /* exchanges started in the low-power state: must stay 0 */
	uint32_t pairings;
	uint32_t state;
	uint32_t stores;       /* records written to flash */
};
struct cidoo_p24_stats cidoo_p24_stats;

const uint8_t *p24_stats(size_t *len)
{
	*len = sizeof(cidoo_p24_stats);
	return (const uint8_t *)&cidoo_p24_stats;
}

/* The record queue: entries [tag, len, data...], the head in flight until answered. */
static uint8_t queue[QUEUE_LEN][ENTRY_MAX];
static uint8_t q_rd, q_wr;

/* The exchange, between the thread and the RF interrupt. */
static volatile bool p24_on;
static volatile bool x_done;
static volatile bool x_reply;
static volatile uint16_t x_window;
static uint8_t reply[64];
static uint8_t reply_len;

#ifdef CONFIG_TLSR_P24_DIAG
#include "tlsr_analog.h"

#define REJ_SHORT BIT(0) /* a pairing reply under 20 octets */
#define REJ_TYPE  BIT(1) /* its octet 0 not 0x03 */
#define REJ_MARK  BIT(2) /* its octet 1 not 0x57 */
#define REJ_KIND  BIT(3) /* its octet 3 not 0x80 */
#define REJ_ECHO  BIT(4) /* its octets 16..19 not the beacon's 8..11 */
#define REJ_ID    BIT(5) /* a dongle ID of 0 or 0xffffffff */

/* What p24_diag_read() gives, little-endian, in this order. */
static struct __packed {
	uint32_t exchanges;   /* packets started */
	uint32_t tx_irq;      /* TX interrupts taken while the exchange ran */
	uint32_t rx_irq;      /* RX interrupts */
	uint32_t rx_good;     /* frames that passed the length and CRC checks */
	uint32_t rx_crc;      /* frames with the status byte's bit 0 set, or too long for the buffer */
	uint32_t rx_len;      /* frames whose DMA length is not the header's + 11 */
	uint32_t rx_late;     /* RX interrupts after the exchange had ended */
	uint32_t to_irq;      /* interrupts with status bit 10 or bit 2 (a receive window ended empty) */
	uint32_t pair_ok;     /* pairing replies taken */
	uint32_t pair_reject; /* frames received while pairing and not taken as the reply */
	uint32_t answered;    /* linked packets answered */
	uint32_t failures;
	uint32_t stores;
	uint16_t tx_us_min;   /* the TX interrupt's time after the packet's start tick, us */
	uint16_t tx_us_max;
	uint16_t rx_us_min;   /* the RX interrupt's */
	uint16_t rx_us_max;
	uint8_t state;
	uint8_t chan;
	uint8_t id[3];
	uint8_t level_beacon;
	uint8_t level_data;
	uint8_t unit13;       /* per-unit byte 13 */
	uint8_t ana_8a;       /* analog 0x8a now */
	uint8_t reject_why;   /* REJ_ bits of the last frame not taken while pairing */
	uint8_t unit9;        /* per-unit bytes 9 and 10 as the flash holds them */
	uint8_t unit10;
	uint32_t dongle;
	uint32_t image_crc;
	uint8_t last_rx[40];  /* the RX buffer's start at the last RX interrupt */
	uint8_t last_rej[24]; /* the payload's start of the last frame not taken while pairing */
	uint32_t tries[6];    /* linked packets answered at the 1st, 2nd, 3rd, 4th, 5th-8th, a later try */
	uint32_t sent_fast;   /* packets started with the fast settle, and those answered */
	uint32_t got_fast;
	uint32_t sent_normal; /* with the normal settle */
	uint32_t got_normal;
	uint32_t sent_chan[4]; /* packets started per channel (2405, 2422, 2440, 2460 MHz), and those answered */
	uint32_t got_chan[4];
	uint32_t gap[5];      /* a packet's start after the start of the one before it that failed:
			       * under 1.5 ms, 2.5 ms, 4 ms, 8 ms, longer */
} diag = {.tx_us_min = 0xffff, .rx_us_min = 0xffff};

static volatile uint32_t x_start;
static uint32_t diag_prev_start;
static bool diag_prev_failed;

/* A packet starts (interrupts locked or not yet enabled for it). */
static void diag_sent(uint32_t start, bool fast, uint8_t chan)
{
	if (fast) {
		diag.sent_fast++;
	} else {
		diag.sent_normal++;
	}
	diag.sent_chan[(chan >> 2) & 3U]++;
	if (diag_prev_failed) {
		uint32_t us = (start - diag_prev_start) / TLSR_RADIO_TICKS_PER_US;

		diag.gap[us < 1500U ? 0 : us < 2500U ? 1 : us < 4000U ? 2 : us < 8000U ? 3 : 4]++;
	}
	diag_prev_start = start;
}

/* The exchange ended: a frame that passed the radio's checks came, or none. */
static void diag_ended(bool got, bool fast, uint8_t chan)
{
	diag_prev_failed = !got;
	if (got) {
		if (fast) {
			diag.got_fast++;
		} else {
			diag.got_normal++;
		}
		diag.got_chan[(chan >> 2) & 3U]++;
	}
}

static void diag_time(uint32_t tick, uint16_t *lo, uint16_t *hi)
{
	uint32_t us = (tick - x_start) / TLSR_RADIO_TICKS_PER_US;
	uint16_t v = (uint16_t)MIN(us, 0xfffeU);

	if (v < *lo) {
		*lo = v;
	}
	if (v > *hi) {
		*hi = v;
	}
}

size_t p24_diag_read(size_t at, uint8_t *out, size_t n)
{
	unsigned int key = irq_lock();

	diag.exchanges = cidoo_p24_stats.exchanges;
	diag.pair_ok = cidoo_p24_stats.pairings;
	diag.answered = cidoo_p24_stats.answered;
	diag.failures = cidoo_p24_stats.failures;
	diag.stores = cidoo_p24_stats.stores;
	diag.state = p.state;
	diag.chan = p.chan;
	memcpy(diag.id, p.id, 3);
	diag.level_beacon = p.level_beacon;
	diag.level_data = p.level_data;
	diag.ana_8a = tlsr_analog_read(0x8aU);
	diag.dongle = p.dongle;
	diag.image_crc = p.image_crc;
	if (at >= sizeof(diag)) {
		n = 0;
	} else {
		n = MIN(n, sizeof(diag) - at);
		memcpy(out, (const uint8_t *)&diag + at, n);
	}
	irq_unlock(key);
	return n;
}
#endif /* CONFIG_TLSR_P24_DIAG */

/* CRC-16/MODBUS, the records' and commands' CRC */
static uint16_t crc16(const uint8_t *d, size_t n)
{
	uint16_t crc = 0xffff;

	for (size_t i = 0; i < n; i++) {
		crc ^= d[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xa001U) : (uint16_t)(crc >> 1);
		}
	}
	return crc;
}

/* Kernel ticks now: the low 32 bits of the uptime in ticks. */
static uint32_t ticks_now(void)
{
	return (uint32_t)k_uptime_ticks();
}

/* Ticks from t to now; 0 when t is later by up to LATER_MAX (another thread set it after now was read). */
static uint32_t since(uint32_t now, uint32_t t)
{
	uint32_t d = now - t;

	return d > UINT32_MAX - LATER_MAX ? 0U : d;
}

/* ---------------------------------------------------------------- flash */

static uint32_t store_next; /* the first free record */
static struct {
	uint32_t dongle;
	uint8_t b4[3];
} stored; /* what the last good record holds */

static bool rec_free(const uint8_t r[REC_LEN])
{
	for (size_t i = 0; i < REC_LEN; i++) {
		if (r[i] != 0xffU) {
			return false;
		}
	}
	return true;
}

static void store_load(void)
{
	uint8_t r[REC_LEN];

	p.dongle = 0;
	memset(p.b4, 0, sizeof(p.b4));
	for (store_next = 0; store_next < STORE_SECTOR; store_next += REC_LEN) {
		if (flash_read(flash_dev, STORE_BASE + store_next, r, REC_LEN) != 0 || rec_free(r)) {
			break;
		}
		if (r[0] == REC_MAGIC && sys_get_le16(&r[REC_LEN - 2U]) == crc16(r, REC_LEN - 2U)) {
			p.dongle = sys_get_le32(&r[1]);
			memcpy(p.b4, &r[5], sizeof(p.b4));
		}
	}
	stored.dongle = p.dongle;
	memcpy(stored.b4, p.b4, sizeof(p.b4));
}

/* Appends a record, unless the last good one holds the same; each write is
 * read back, and a bad record (which store_load skips) is followed by one
 * more try. A record left all 0xff is never followed by another, as
 * store_load stops at the first free one. */
static void store_save(void)
{
	uint8_t r[REC_LEN], back[REC_LEN];

	if (p.dongle == stored.dongle && memcmp(p.b4, stored.b4, sizeof(p.b4)) == 0) {
		return;
	}
	memset(r, 0, sizeof(r));
	r[0] = REC_MAGIC;
	sys_put_le32(p.dongle, &r[1]);
	memcpy(&r[5], p.b4, sizeof(p.b4));
	sys_put_le16(crc16(r, REC_LEN - 2U), &r[REC_LEN - 2U]);
	for (int tries = 0; tries < 2; tries++) {
		if (store_next + REC_LEN > STORE_SECTOR) {
			if (flash_erase(flash_dev, STORE_BASE, STORE_SECTOR) != 0) {
				return;
			}
			store_next = 0;
		}
		(void)flash_write(flash_dev, STORE_BASE + store_next, r, REC_LEN);
		if (flash_read(flash_dev, STORE_BASE + store_next, back, REC_LEN) != 0 ||
		    rec_free(back)) {
			return; /* nothing written: the slot stays the next */
		}
		store_next += REC_LEN;
		cidoo_p24_stats.stores++;
		if (memcmp(back, r, REC_LEN) == 0) {
			stored.dongle = p.dongle;
			memcpy(stored.b4, p.b4, sizeof(p.b4));
			return;
		}
	}
}

static bool dongle_known(void)
{
	return p.dongle != 0U && p.dongle != 0xffffffffU;
}

/* ---------------------------------------------------------------- radio */

/* The RF interrupt while the 2.4G link runs (ble_conn.c hands it over): a
 * received frame ends the exchange; it counts as a reply when its status
 * byte's bit 0 is clear and its length matches its header. */
void p24_rf_isr(void)
{
	uint16_t st = tlsr_radio_irq_status();
#ifdef CONFIG_TLSR_P24_DIAG
	uint32_t entry = tlsr_radio_now();
#endif

	if ((st & TLSR_RF_IRQ_RX) != 0U) {
		tlsr_radio_p24_ack_irq(TLSR_RF_IRQ_RX);
		uint8_t len = rx_buf[0];
#ifdef CONFIG_TLSR_P24_DIAG
		diag.rx_irq++;
		diag_time(entry, &diag.rx_us_min, &diag.rx_us_max);
		memcpy(diag.last_rx, rx_buf, sizeof(diag.last_rx));
		if (x_done) {
			diag.rx_late++;
		} else if (len != (rx_buf[4] & 0x3fU) + 11U) {
			diag.rx_len++;
		} else if (len + 3U >= sizeof(rx_buf) || (rx_buf[len + 3U] & 1U) != 0U) {
			diag.rx_crc++;
		} else {
			diag.rx_good++;
		}
#endif

		/* any frame ends the exchange; one that fails the checks is a failure */
		if (!x_done) {
			if (len >= 11U && len + 3U < sizeof(rx_buf) &&
			    (rx_buf[len + 3U] & 1U) == 0U && len == (rx_buf[4] & 0x3fU) + 11U) {
				reply_len = rx_buf[4] & 0x3fU;
				memcpy(reply, &rx_buf[5], reply_len);
				x_reply = true;
				tc32_rng_add_event(tlsr_radio_rx_timestamp(), TC32_RNG_SRC_RADIO_RX);
				tc32_rng_add_sample(rx_buf[len + 2U], TC32_RNG_SRC_RADIO_RSSI);
			}
			x_done = true;
			ble_conn_kick();
		}
		rx_buf[0] = 1;
	}
	if ((st & TLSR_RF_IRQ_TX) != 0U) {
		tlsr_radio_p24_ack_irq(TLSR_RF_IRQ_TX);
		if (!x_done) {
#ifndef CONFIG_TLSR_P24_AUTO_RX
			tlsr_radio_p24_listen(x_window, tlsr_radio_now());
#endif
#ifdef CONFIG_TLSR_P24_DIAG
			diag.tx_irq++;
			diag_time(entry, &diag.tx_us_min, &diag.tx_us_max);
#endif
		}
	}
	if ((st & TLSR_RF_IRQ_RX_TIMEOUT) != 0U) {
		tlsr_radio_p24_ack_irq(TLSR_RF_IRQ_RX_TIMEOUT);
	}
#ifdef CONFIG_TLSR_P24_DIAG
	if ((st & (TLSR_RF_IRQ_RX_TIMEOUT | 0x0004U)) != 0U) {
		diag.to_irq++; /* status bit 10, or bit 2: the window after a send-then-receive ended */
	}
#endif
	st = tlsr_radio_irq_status() & (uint16_t)~(TLSR_RF_IRQ_RX | TLSR_RF_IRQ_TX);
	if (st != 0U) {
		tlsr_radio_clear_irq(st);
	}
}

/* One exchange: the packet in tx_buf, then the reply window. True on a reply. */
static bool exchange(uint8_t len, uint8_t first, uint32_t code, uint8_t level, uint16_t window)
{
	uint32_t start;
	uint32_t now = ticks_now();

	sys_put_le32((uint32_t)len + 1U, tx_buf);
	tx_buf[4] = len;
	ble_radio_take();
	x_done = false;
	x_reply = false;
	x_window = window;
	tlsr_radio_p24_access(code, first);
	tlsr_radio_set_power_level(level);
	tlsr_radio_off();
	if (since(now, p.settle_at) >= SETTLE_NORMAL) {
		p.settle = SETTLE_MARK; /* every 2 s: at least two packets with the normal settle */
		p.settle_at = now;
	}
	tlsr_radio_p24_settle(p.settle == SETTLE_FAST);
	if (p.chan > 15U) {
		p.chan = 0;
	}
	tlsr_radio_p24_channel((uint16_t)(2400 + chan_off[p.chan]));
#ifdef CONFIG_TLSR_P24_AUTO_RX
	/* the start tick read and the command written with nothing in between */
	unsigned int key = irq_lock();
#endif
	start = tlsr_radio_now();
#ifdef CONFIG_TLSR_P24_DIAG
	x_start = start;
	diag_sent(start, p.settle == SETTLE_FAST, p.chan);
#endif
	cidoo_p24_stats.exchanges++;
	if (p.low) {
		cidoo_p24_stats.tx_while_low++;
	}
#ifdef CONFIG_TLSR_P24_AUTO_RX
	tlsr_radio_p24_send_listen(tx_buf, start, window);
	irq_unlock(key);
#else
	tlsr_radio_p24_send(tx_buf, start);
#endif
	while (!x_done) {
		int32_t left = (int32_t)(EXCHANGE_US - (tlsr_radio_now() - start) / TLSR_RADIO_TICKS_PER_US);

		if (left <= 0) {
			break;
		}
		(void)ble_conn_wait_ticks(us_to_ticks_ceil((uint32_t)left));
	}
#ifdef CONFIG_TLSR_P24_AUTO_RX
	/* a frame received whose interrupt has not run yet is taken here */
	key = irq_lock();
	if (!x_done && (tlsr_radio_irq_status() & TLSR_RF_IRQ_RX) != 0U) {
		p24_rf_isr();
	}
	irq_unlock(key);
#endif
	x_done = true;
	tlsr_radio_stop();
	tlsr_radio_off();
	ble_radio_give();
#ifdef CONFIG_TLSR_P24_DIAG
	diag_ended(x_reply, p.settle == SETTLE_FAST, p.chan);
#endif
	if (x_reply) {
		/* a failure leaves the flag as it is */
		p.settle = p.settle == SETTLE_MARK ? 0U : SETTLE_FAST;
	}
	return x_reply;
}

static void failed(void)
{
	cidoo_p24_stats.failures++;
	if (++p.fails >= 3U) {
		p.chan = (uint8_t)((p.chan + 4U) & 15U);
	}
	if (p.state == ST_LINKED && p.fails > FAILS_TO_STATE2) {
		p.state = ST_RECONNECTING;
		p.state_since = ticks_now();
	}
}

/* ---------------------------------------------------------------- queue */

static bool q_empty(void)
{
	return q_rd == q_wr;
}

static uint8_t q_next(uint8_t i)
{
	return (uint8_t)((i + 1U) % QUEUE_LEN);
}

/* The queue full (no dongle answering): the oldest entry that a later one of
 * its kind replaces goes, the head (in flight) excepted. Every entry holds the
 * whole state of its kind, so the last state of each kind stays: a release is
 * never the one lost. With 14 entries behind the head and six kinds, one
 * always goes. */
static void q_drop_replaced(void)
{
	for (uint8_t i = q_next(q_rd); i != q_wr; i = q_next(i)) {
		for (uint8_t j = q_next(i); j != q_wr; j = q_next(j)) {
			if (queue[j][0] != queue[i][0]) {
				continue;
			}
			for (uint8_t k = i; q_next(k) != q_wr; k = q_next(k)) {
				memcpy(queue[k], queue[q_next(k)], ENTRY_MAX);
			}
			q_wr = (uint8_t)((q_wr + QUEUE_LEN - 1U) % QUEUE_LEN);
			return;
		}
	}
}

static void q_put(uint8_t tag, const uint8_t *data, uint8_t len)
{
	if (len + 2U > ENTRY_MAX) {
		return;
	}
	if (q_next(q_wr) == q_rd) {
		q_drop_replaced();
		if (q_next(q_wr) == q_rd) {
			return;
		}
	}
	memset(queue[q_wr], 0, ENTRY_MAX);
	queue[q_wr][0] = tag;
	queue[q_wr][1] = len;
	memcpy(&queue[q_wr][2], data, len);
	q_wr = q_next(q_wr);
}

/* Every report kind all-zero, queued at boot and after pairing, so the host holds no key. */
static void q_release_all(void)
{
	static const uint8_t zero[15];

	q_put(TAG_KEYBOARD, zero, 8);
	q_put(TAG_NKRO, zero, 15);
	q_put(TAG_CONSUMER, zero, 2);
	q_put(TAG_SYSTEM, zero, 2);
	q_put(TAG_MOUSE, zero, 7);
}

/* ZMK's reports, from ble_zmk.c's queue. */
static void q_take_zmk(void)
{
	uint8_t kind, len, data[16];

	while (ble_zmk_take(&kind, data, &len)) {
		if (kind == BLE_ZMK_REPORT_KEYBOARD) {
			q_put(TAG_KEYBOARD, data, 8);
		} else {
			q_put(TAG_CONSUMER, data, 2); /* the first usage, as one 16-bit field */
		}
	}
}

/* ---------------------------------------------------------------- packets */

static uint8_t build_report(const uint8_t *e)
{
	uint8_t *q = &tx_buf[5];
	uint8_t n = (uint8_t)(e[1] + 2U);
	uint8_t r[24];
	uint8_t rl, copies;

	memset(q, 0, PAYLOAD_MAX);
	q[0] = 0x06;
	memset(r, 0, sizeof(r));
	r[1] = p.seq;
	r[2] = p.id[0];
	r[3] = p.id[1];
	if (n <= 10U) {
		r[0] = 0x01;
		memcpy(&r[4], e, n);
		sys_put_le16(crc16(r, 14), &r[14]);
		rl = 16;
		copies = 3;
	} else {
		r[0] = 0x0d;
		memcpy(&r[4], e, n);
		sys_put_le16(crc16(r, 22), &r[22]);
		rl = 24;
		copies = 2;
	}
	for (uint8_t i = 0; i < copies; i++) {
		memcpy(&q[1 + i * rl], r, rl);
	}
	return PAYLOAD_MAX;
}

static uint8_t build_control_b5(void)
{
	uint8_t *q = &tx_buf[5];

	memset(q, 0, PAYLOAD_MAX);
	q[0] = 0x06;
	q[1] = 0x0c;
	q[2] = p.seq;
	q[3] = p.id[0];
	q[4] = p.id[1];
	q[5] = 0xb5;
	q[6] = 0x04;
	memcpy(&q[7], p.b4, 3);
	q[10] = (uint8_t)(p.b4[0] + p.b4[1] + p.b4[2]);
	return PAYLOAD_MAX;
}

static uint8_t build_beacon(void)
{
	uint8_t *q = &tx_buf[5];

	memset(q, 0, 24);
	q[0] = 0x02;
	q[1] = 0x57;
	q[2] = 0x10;
	q[3] = 0x02;
	sys_put_le32(p.image_crc, &q[4]);
	q[8] = p.id[0];
	q[9] = p.id[1];
	q[10] = p.id[2];
	q[11] = 0x02;
	return 24;
}

/* An ACK: the first slot (of those the frame holds) with 57 82 and a good CRC; its LED byte. */
static bool parse_ack(uint8_t *leds)
{
	if (reply_len < 6U || reply[0] != 0x07U) {
		return false;
	}
	for (uint8_t s = 0; s < 3 && 1U + 5U * s + 5U <= reply_len; s++) {
		const uint8_t *slot = &reply[1 + 5 * s];

		if (slot[0] == 0x57U && slot[1] == 0x82U && sys_get_le16(&slot[3]) == crc16(slot, 3)) {
			*leds = slot[2];
			return true;
		}
	}
	return false;
}

/* A command: its code, or 0. */
static uint8_t parse_command(void)
{
	if (reply_len < 44U || reply[0] != 0x07U || reply[1] != 0x57U || reply[3] != 0x8cU ||
	    sys_get_le16(&reply[42]) != crc16(&reply[1], 41)) {
		return 0;
	}
	return reply[4];
}

static void command(uint8_t cmd)
{
	switch (cmd) {
	case 0xb3:
		if (reply[6] == 1U && reply[7] == (uint8_t)(reply[5] + 1U) && reply[5] != 0xffU) {
			p.sleep_asked = true;
			p.sleep_at = ticks_now() + SLEEP_CMD;
		}
		break;
	case 0xb4:
	case 0xb6:
		if ((uint16_t)reply[6] + reply[7] + reply[8] != reply[9]) {
			break;
		}
		if (cmd == 0xb6 && !p.b5_pending) {
			break;
		}
		if (memcmp(p.b4, &reply[6], 3) != 0) {
			memcpy(p.b4, &reply[6], 3);
			store_save();
		}
		p.b5_pending = cmd == 0xb4;
		break;
	default:
		break; /* 0xb0 (VIA) is not answered; 0xb1, 0xb2, 0xb5 do nothing */
	}
}

/* An answer to a linked packet: ACK or command. */
static bool answered(bool report)
{
	uint8_t leds;

	if (parse_ack(&leds)) {
		if (report) {
			ble_hid_leds(leds); /* an ACK to a control packet carries none */
		}
	} else {
		uint8_t cmd = parse_command();

		if (cmd == 0U) {
			return false;
		}
		command(cmd);
	}
	if (report && !q_empty()) {
		q_rd = (uint8_t)((q_rd + 1U) % QUEUE_LEN); /* the entry is committed */
	}
	p.seq++;
#ifdef CONFIG_TLSR_P24_DIAG
	diag.tries[p.fails < 4U ? p.fails : p.fails < 8U ? 4U : 5U]++;
#endif
	p.fails = 0;
	cidoo_p24_stats.answered++;
	if (p.state != ST_LINKED) {
		p.state = ST_LINKED;
		p.state_since = ticks_now();
		ble_zmk_p24_ready(true);
	}
	return true;
}

/* ---------------------------------------------------------------- states */

static void pairing_step(void)
{
	uint8_t len = build_beacon();

	/* the reply must echo this keyboard's ID from the beacon */
	if (exchange(len, AC_SEARCH, PAIR_CODE, p.level_beacon, WINDOW_US) && reply_len >= 20U &&
	    reply[0] == 0x03U && reply[1] == 0x57U && reply[3] == 0x80U &&
	    memcmp(&reply[16], &tx_buf[5 + 8], 4) == 0) {
		uint32_t id = sys_get_le32(&reply[12]);

		if (id != 0U && id != 0xffffffffU) {
			p.dongle = id;
			cidoo_p24_stats.pairings++;
			if (reply[9] != 0xaaU) {
				store_save();
			}
			p.state = ST_LINKED;
			p.state_since = ticks_now();
			p.fails = 0;
			q_rd = q_wr = 0;
			q_release_all();
			ble_zmk_p24_ready(true);
			return;
		}
	}
#ifdef CONFIG_TLSR_P24_DIAG
	if (x_reply) {
		diag.pair_reject++;
		diag.reject_why = (reply_len < 20U ? REJ_SHORT : 0U) | (reply[0] != 0x03U ? REJ_TYPE : 0U) |
				  (reply[1] != 0x57U ? REJ_MARK : 0U) | (reply[3] != 0x80U ? REJ_KIND : 0U) |
				  (memcmp(&reply[16], &tx_buf[5 + 8], 4) != 0 ? REJ_ECHO : 0U) |
				  (sys_get_le32(&reply[12]) == 0U || sys_get_le32(&reply[12]) == 0xffffffffU
					   ? REJ_ID : 0U);
		memcpy(diag.last_rej, reply, sizeof(diag.last_rej));
	}
#endif
	failed();
}

static void linked_step(void)
{
	uint32_t now = ticks_now();
	uint8_t len;
	bool report = false;

	q_take_zmk();
	if (p.b5_pending) {
		len = build_control_b5();
		if (exchange(len, AC_LINKED, p.dongle, p.level_data, WINDOW_CTRL_US)) {
			p.b5_pending = false;
			(void)answered(false);
		} else {
			failed();
		}
		return;
	}
	if (q_empty()) {
		uint32_t every = since(now, last_key) >= HEARTBEAT_SLOW_AFTER ? HEARTBEAT_SLOW : HEARTBEAT_FAST;

		if (since(now, p.last_hb) < every) {
			(void)ble_conn_wait_ticks(every - since(now, p.last_hb));
			return;
		}
		static const uint8_t hb = 0;

		q_put(TAG_HEARTBEAT, &hb, 1);
		p.last_hb = now;
	}
	len = build_report(queue[q_rd]);
	report = true;
	if (!exchange(len, AC_LINKED, p.dongle, p.level_data, WINDOW_US) || !answered(report)) {
		failed();
	}
}

/* Before pairing: every report kind all-zero, 15 times each, 1 ms apart, unanswered. */
static void release_burst(void)
{
	static const uint8_t zero[15];
	static const struct {
		uint8_t tag, len;
	} kinds[] = {{TAG_NKRO, 15}, {TAG_KEYBOARD, 8}, {TAG_MOUSE, 7}, {TAG_SYSTEM, 2}, {TAG_CONSUMER, 2}};

	for (size_t k = 0; k < ARRAY_SIZE(kinds); k++) {
		uint8_t e[ENTRY_MAX] = {kinds[k].tag, kinds[k].len};

		memcpy(&e[2], zero, kinds[k].len);
		p.seq++;
		for (int i = 0; i < 15; i++) {
			(void)exchange(build_report(e), AC_LINKED, p.dongle, p.level_data, WINDOW_US);
			k_sleep(K_MSEC(1));
		}
	}
}

static volatile bool pair_request;

void p24_pair_request(void)
{
	pair_request = true;
	ble_conn_kick();
}

bool p24_running(void)
{
	return p24_on;
}

void p24_status(bool *linked, bool *pairing)
{
	*linked = p.state == ST_LINKED;
	*pairing = p.state == ST_PAIRING;
}

void p24_activity(void)
{
	last_key = ticks_now();
	key_seen = true;
	if (p.low || p.paused) {
		ble_conn_kick();
	}
}

/* ---------------------------------------------------------------- power */

static volatile bool pad_woke;

static bool quiet(void)
{
	return p.low;
}

static void resumed(uint32_t status)
{
	if ((status & TLSR8278_WAKEUP_STATUS_PAD) != 0U) {
		pad_woke = true;
		ble_conn_kick();
	}
}

static void low_power(bool on)
{
	if (on == p.low) {
		return;
	}
	p.low = on;
	if (on) {
		cidoo_p24_stats.low_enters++;
	} else {
		cidoo_p24_stats.low_exits++;
	}
	if (on) {
		key_matrix_sleep(true);
		tlsr8278_idle_suspend(TLSR8278_WAKEUP_PAD, quiet, resumed);
	} else {
		tlsr8278_idle_suspend(0U, NULL, NULL);
		key_matrix_sleep(false);
		/* the suspends reset the radio's registers */
		tlsr_radio_init_p24(rx_buf, sizeof(rx_buf));
		last_key = ticks_now();
	}
}

static void power_step(void)
{
	uint32_t now = ticks_now();

	if (pad_woke) {
		pad_woke = false;
		last_key = now;
	}
	uint32_t idle = since(now, last_key);

	if (key_seen) {
		key_seen = false;
		if (p.paused) {
			p.paused = false;
			p.state_since = now;
		}
	}
	if (ble_usb_host()) {
		/* a USB host on the cable (an update, say): no sleep and no suspend.
		 * Linked, the times start again; in state 1 or 2, sending stops where
		 * the deep sleep would come */
		low_power(false);
		p.sleep_asked = false;
		if (p.state == ST_LINKED) {
			p.state_since = now;
			last_key = now;
		} else if (p.state == ST_PAIRING ? since(now, p.state_since) >= PAIR_TIMEOUT
						 : idle >= STATE2_IDLE && since(now, p.state_since) >= STATE2_IDLE) {
			p.paused = true;
		}
		return;
	}
	if (p.paused) {
		/* stopped with a USB host on the cable: asleep once the cable is
		 * pulled; while power still comes in (the host enumerating again, a
		 * charger), stopped until a key */
		if (!ble_cable_power()) {
			ble_deep_sleep();
		}
		return;
	}
	if (p.sleep_asked && (int32_t)(now - p.sleep_at) >= 0) {
		ble_deep_sleep();
	}

	switch (p.state) {
	case ST_PAIRING:
		if (since(now, p.state_since) >= PAIR_TIMEOUT) {
			ble_deep_sleep();
		}
		break;
	case ST_RECONNECTING:
		if (idle >= STATE2_IDLE && since(now, p.state_since) >= STATE2_IDLE) {
			ble_deep_sleep();
		}
		break;
	default:
		if (idle >= LINKED_IDLE) {
			ble_deep_sleep();
		}
		low_power(idle >= LOW_POWER);
		break;
	}
}

/* ---------------------------------------------------------------- run */

static uint32_t image_crc(void)
{
	uint32_t slot = tlsr_slot_running();
	uint32_t size = 0, crc = 0;

	if (tlsr_slot_read(slot + TLSR_SLOT_SIZE_WORD, &size, sizeof(size)) == 0 && size >= 8U &&
	    size <= TLSR_SLOT_SIZE) {
		(void)tlsr_slot_read(slot + size - 4U, &crc, sizeof(crc));
	}
	return crc;
}

FUNC_NORETURN void p24_run(const uint8_t unit[16])
{
	memcpy(p.id, unit, 3);
	p.level_data = unit[9] == 0xffU ? CONFIG_TLSR_BLE_TX_POWER_DEFAULT : unit[9];
	p.level_beacon = unit[10] == 0xffU ? 0x8aU : unit[10];
#ifdef CONFIG_TLSR_P24_DIAG
	diag.unit9 = unit[9];
	diag.unit10 = unit[10];
	diag.unit13 = unit[13];
#endif
	p.image_crc = image_crc();
	store_load();
	last_key = ticks_now();
	p.state_since = last_key;
	p.settle_at = last_key;
	tlsr_radio_init_p24(rx_buf, sizeof(rx_buf));
	p24_on = true;
	irq_enable(TLSR_RADIO_IRQ);
	if (dongle_known()) {
		/* ZMK's reports only once the dongle answers (answered()): the key
		 * that woke the chip from deep sleep is not sent */
		p.state = ST_RECONNECTING;
		q_release_all();
	} else {
		p.state = ST_PAIRING;
	}
	for (;;) {
		if (pair_request) {
			pair_request = false;
			if (dongle_known()) {
				release_burst();
			}
			ble_zmk_p24_ready(false);
			p.state = ST_PAIRING;
			p.state_since = ticks_now();
			p.fails = 0;
		}
		ble_battery_poll();
		power_step();
		cidoo_p24_stats.state = p.state;
		if (p.low || p.paused) {
			(void)ble_conn_wait(1000);
			continue;
		}
		if (p.sleep_asked) {
			(void)ble_conn_wait(10); /* no packet from a 0xb3 to the deep sleep */
			continue;
		}
		if (p.state == ST_PAIRING) {
			pairing_step();
		} else {
			linked_step();
		}
	}
}
