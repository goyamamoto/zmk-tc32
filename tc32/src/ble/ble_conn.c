/*
 * The peripheral's connection events (Core Vol 6 Part B 4.5). The radio
 * registers named here (0xf03, 0xf18, 0xf22, 0xf23, 0x448, 0x450) are not
 * documented; zephyr-tc32's tlsr_radio.c writes and reads them.
 *
 * - Each event is one command 0x82 at a start tick (tlsr_radio_start_conn_event()).
 *   The code does not send the replies itself: it queues its packets in the
 *   radio's TX FIFO (the empty packet at the TX DMA address for when there is
 *   none), writes the SN and NESN bits it keeps to 0xf03 bits 5:4 at the
 *   start, reads them back from 0xf22 bit 0 and 0xf23 bit 4 after each
 *   received packet, and takes the FIFO's read pointer as the packets the
 *   central has acknowledged.
 * - The event starts from a kernel timer interrupt 2 ms ahead
 *   (TIMER_EARLY_US; the timer comes late by the flash cache misses on its
 *   way), and the start tick written to 0xf18 gives the exact time. The system timer
 *   compare is not free for this: the kernel clock uses it. ZMK's system
 *   work queue is cooperative, so no thread could start an event on time.
 * - The RF interrupt takes each received packet: the next RX slot goes to the
 *   DMA at once, then the packet is checked (its DMA length and status bit,
 *   tlsr_radio.h), duplicates are dropped by their SN, and data PDUs are
 *   kept for the thread. The first packet gives the anchor: the u32 at 0x450
 *   minus 0x500 ticks. That time stamp and the packet's RSSI (the RX
 *   entry's octet before its status byte) go to tc32_rng_add_event() and
 *   tc32_rng_add_sample(), once per event. The event ends on the RF
 *   interrupt bits the code takes as its end (TLSR_RF_IRQ_EVENT_END), or
 *   from the timer when no packet has come by the latest time the central
 *   could start one. A packet the radio is still receiving then (0x448 bit
 *   5) is waited for, up to EXTEND_MAX_US.
 * - Timing: the receiver starts 500 us plus the window widening before the
 *   earliest time the central may send; the first event, and one after a
 *   connection update, listen over the whole transmit window. Channels by
 *   channel selection algorithm #1: the advertising PDU does not offer #2.
 * - Peripheral latency (Core Vol 6 Part B 4.5.1), only while the thread asks
 *   for it (ble_conn_latency(), ble_sleep.c's low-power state): after an
 *   event, with nothing queued or unacknowledged, nothing received left
 *   unread, no procedure with an instant pending and no transmit window, up
 *   to the link's latency of the events that follow are skipped (their
 *   channels still counted). Queuing a PDU or turning latency off moves the
 *   next event back to the first one still ahead, so nothing waits for the
 *   skipped ones.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_internal.h"
#include <tlsr_radio.h>

#define TICKS_US   TLSR_RADIO_TICKS_PER_US
#define UNIT_TICKS (1250U * TICKS_US) /* 1.25 ms */

#define RX_LEAD_US      500U /* receiver on before the earliest packet */
/*
 * The timer fires this much before the receiver starts. The kernel rounds a
 * timer up to its next 100 us tick, and the way from the system timer
 * interrupt to event_start() takes about 170 flash cache misses besides its
 * instructions, each as long as the flash takes to give a cache line: the
 * handler's lateness is counted in cidoo_ble_stats.timer_lat_max. An event
 * whose start is then less than START_MIN_TICKS ahead is skipped, unless a
 * transmit window is open, so the timer's lead has to cover that time. An
 * event has to end that much sooner (END_MARGIN_US), and a PDU has to be
 * queued that much sooner to go in it.
 */
#define TIMER_EARLY_US  2000U
/*
 * An event whose start is less ahead than event_start() takes to give the
 * radio its command, about 30 flash cache lines, is skipped: what the radio
 * does with a start tick that has passed is not known.
 */
#define START_MIN_TICKS (300U * TICKS_US)
/*
 * An event that starts its receiver at once, inside a transmit window, gives
 * the radio a start this far ahead: the time event_start() takes from there
 * to the radio's command, flash cache misses included.
 */
#define START_NOW_US    300U
#define RX_GUARD_US     500U /* after the latest packet start: a packet lasts 328 us at most */
#define END_MARGIN_US   (RX_LEAD_US + TIMER_EARLY_US + 200U)
/*
 * An event the timer would end while a packet is being received waits for
 * it, looking again every EXTEND_STEP_US, for at most EXTEND_MAX_US: a packet
 * (328 us), the 150 us before the reply and the reply (328 us). The next
 * event may then start too late and be skipped, which the central's
 * retransmission covers, as it would cover the cut packet.
 */
#define EXTEND_STEP_US  100U
#define EXTEND_MAX_US   800U
#define OWN_SCA_PPM     50U
#define EVENT_TIMEOUT_MARGIN_US   1250U  /* the radio's FSM timeout: interval - 1.25 ms, */
#define EVENT_TIMEOUT_MAX_US      25000U /* at most 25 ms */
#define STAMP_MAX_AGE   0x7cffU /* a time stamp older than this is not this packet's */

#define RX_SLOTS     8U
#define RX_SLOT_SIZE 64U
#define TX_SLOTS     BLE_TX_SLOTS
#define HW_FIFO      16U
#define HW_FIFO_FILL 6U /* the radio's FIFO is filled up to 7 packets */

struct tx_pkt {
	uint32_t dma_len;
	uint8_t hdr;
	uint8_t len;
	uint8_t data[BLE_PDU_MAX + 4U];
} __aligned(4);

static uint8_t rx_ring[RX_SLOTS][RX_SLOT_SIZE] __aligned(16);
static uint16_t rx_event[RX_SLOTS];
static volatile uint8_t rx_wr; /* interrupt: PDUs kept */
static volatile uint8_t rx_rd; /* thread: PDUs consumed */

static struct tx_pkt tx_ring[TX_SLOTS];
static volatile uint8_t tx_wr;    /* thread: PDUs queued */
static volatile uint8_t tx_acked; /* interrupt: PDUs the central acknowledged */
static uint8_t tx_push;           /* interrupt: PDUs given to the radio */
static bool hw_empty[HW_FIFO];    /* the radio FIFO entry is the empty packet */
static uint8_t ack_rptr;          /* acknowledgements counted up to this FIFO entry */

/* The packet the radio sends when its FIFO is empty, an empty PDU. The
 * radio sets NESN, SN and MD. SRAM, for the DMA. */
static uint8_t empty_pkt[8] __aligned(4) = {2, 0, 0, 0, LLID_CONTINUE, 0};

/* Upper bounds of the central's sleep clock accuracy classes, in ppm. */
static const uint16_t sca_ppm[8] = {500, 250, 150, 100, 75, 50, 30, 20};

enum conn_state {
	ST_IDLE,
	ST_WAIT,
	ST_EVENT,
};

static struct {
	uint8_t state;
	uint8_t reason;
	uint8_t terminate;
	bool established;
	uint8_t unestablished_events;
	/* parameters */
	uint32_t aa;
	uint32_t crc_init;
	uint32_t interval_ticks;
	uint32_t timeout_ticks;
	uint16_t interval;
	uint16_t latency;
	uint16_t sca_ppm;
	uint8_t chm[5];
	uint8_t used[BLE_CHANNELS];
	uint8_t n_used;
	uint8_t hop;
	uint8_t unmapped;
	uint16_t counter; /* this event's, or the next one's */
	/* timing, in system ticks */
	uint32_t anchor;      /* the next anchor, or the start of a transmit window */
	uint32_t window;      /* a transmit window's length, until a packet came in it */
	uint32_t last_anchor; /* the last anchor a packet showed */
	uint32_t last_rx;
	uint32_t start;
	uint32_t guard;
	uint32_t latest_end;
	/* this event */
	uint8_t rx_count;
	bool anchor_seen;
	uint32_t anchor_rx;
	bool extending;      /* waiting for a packet that was arriving at the end */
	uint32_t extend_end; /* the longest that wait lasts */
	/* the link's state in the radio between events */
	uint8_t sn_nesn;
	uint8_t hw_rptr;
	uint8_t last_rx_sn;
	/* procedures with an instant */
	bool upd_pending;
	uint8_t upd_win_size;
	uint16_t upd_win_offset;
	uint16_t upd_interval;
	uint16_t upd_latency;
	uint16_t upd_timeout;
	uint16_t upd_instant;
	bool chm_pending;
	uint8_t chm_new[5];
	uint16_t chm_instant;
	/* peripheral latency: the events skipped after the one below */
	uint16_t skipped;
	uint32_t base_anchor;
	uint16_t base_counter;
	uint8_t base_unmapped;
} c;

static volatile bool use_latency;

static struct k_timer ev_timer;
static uint32_t timer_due;  /* when the kernel runs ev_timer's handler (arm()) */
static bool in_timer;       /* in ev_timer's handler */
static volatile bool held; /* ble_conn_hold(): no event starts */
static K_SEM_DEFINE(ll_sem, 0, 1);

static void event_end(bool radio_used);
static void skip_events(void);

/*
 * The timer set for the time `at`. A relative timeout set in the timer's own
 * handler counts from the tick the handler was due at, where the kernel's
 * clock stands while its handlers run, not from now: the handler comes late
 * by the flash cache misses on its way, and a timeout counted from now would
 * expire that much early. timer_due is that tick's time as far as this file
 * knows it: in the handler the kernel expires a timeout of n ticks n ticks
 * after the handler's tick; outside it, n + 1 ticks after the tick under way,
 * which began up to a tick before now.
 */
static void arm(uint32_t at)
{
	uint32_t from = in_timer ? timer_due : tlsr_radio_now();
	int32_t us = (int32_t)(at - from) / (int32_t)TICKS_US;
	uint32_t ticks = us > 0 ? k_us_to_ticks_ceil32((uint32_t)us) : 0U;

	timer_due = from + k_ticks_to_us_floor32(ticks + (in_timer ? 0U : 1U)) * TICKS_US;
	k_timer_start(&ev_timer, K_TICKS(ticks), K_NO_WAIT);
}

static void set_channel_map(const uint8_t chm[5])
{
	memcpy(c.chm, chm, sizeof(c.chm));
	c.n_used = 0;
	for (uint8_t ch = 0; ch < BLE_CHANNELS; ch++) {
		if ((chm[ch >> 3] & BIT(ch & 7U)) != 0U) {
			c.used[c.n_used++] = ch;
		}
	}
}

/* Channel selection algorithm #1 (Core Vol 6 Part B 4.5.8.2) */
static uint8_t next_channel(void)
{
	c.unmapped = (uint8_t)((c.unmapped + c.hop) % BLE_CHANNELS);
	if ((c.chm[c.unmapped >> 3] & BIT(c.unmapped & 7U)) != 0U) {
		return c.unmapped;
	}
	return c.used[c.unmapped % c.n_used];
}

/* Window widening (Core Vol 6 Part B 4.5.7): the drift of both sleep clocks
 * since the last anchor seen, plus 16 us. */
static uint32_t widening(uint32_t anchor)
{
	uint32_t ms = (anchor - c.last_anchor) / (1000U * TICKS_US) + 1U;
	uint32_t us = ms * c.sca_ppm / 1000U + 16U;

	return MIN(us, c.interval * 1250U / 2U - 150U) * TICKS_US;
}

static void schedule(void)
{
	uint32_t wid = widening(c.anchor + c.window);

	c.start = c.anchor - wid - RX_LEAD_US * TICKS_US;
	c.guard = c.anchor + c.window + wid + RX_GUARD_US * TICKS_US;
	c.latest_end = c.anchor + c.window + c.interval_ticks - END_MARGIN_US * TICKS_US;
	c.state = ST_WAIT;
	arm(c.start - TIMER_EARLY_US * TICKS_US);
}

static void end_connection(uint8_t reason)
{
	k_timer_stop(&ev_timer);
	tlsr_radio_stop();
	tlsr_radio_set_irq_mask(0);
	tlsr_radio_clear_irq(TLSR_RF_IRQ_ALL);
	tlsr_radio_set_rx_dma(rx_ring[0]);
	/* The packets still queued for the central go: advertising follows. */
	(void)tlsr_radio_tx_fifo_clear();
	c.reason = reason;
	c.state = ST_IDLE;
	cidoo_ble_stats.conn_ended++;
	cidoo_ble_stats.last_reason = reason;
	k_sem_give(&ll_sem);
}

/* The radio's FIFO read pointer as it was after the last packet, then
 * queued PDUs. When the FIFO is empty, the empty packet goes first: it
 * stands for the one the radio sent last, which the central's next packet
 * acknowledges. */
static void push_tx(void)
{
	uint8_t n = tlsr_radio_tx_fifo_restore(c.hw_rptr);
	uint8_t wptr = (uint8_t)((c.hw_rptr + n) & (HW_FIFO - 1U));

	if (tx_push == tx_wr) {
		return;
	}
	if (n == 0U) {
		tlsr_radio_tx_fifo_push(empty_pkt);
		hw_empty[wptr] = true;
		wptr = (uint8_t)((wptr + 1U) & (HW_FIFO - 1U));
		n = 1U;
	}
	while (n <= HW_FIFO_FILL && tx_push != tx_wr) {
		tlsr_radio_tx_fifo_push(&tx_ring[tx_push % TX_SLOTS]);
		hw_empty[wptr] = false;
		wptr = (uint8_t)((wptr + 1U) & (HW_FIFO - 1U));
		tx_push++;
		n++;
	}
}

static void tx_acks(void)
{
	while (ack_rptr != c.hw_rptr) {
		if (!hw_empty[ack_rptr]) {
			tx_acked++;
		}
		ack_rptr = (uint8_t)((ack_rptr + 1U) & (HW_FIFO - 1U));
	}
}

/* After an event: up to the link's latency of the next events skipped, if nothing needs them. */
static void skip_events(void)
{
	c.skipped = 0U;
	c.base_anchor = c.anchor;
	c.base_counter = c.counter;
	c.base_unmapped = c.unmapped;
	/* Nothing skipped while a PDU the thread has not read yet may carry an instant. */
	if (!use_latency || !c.established || c.window != 0U || c.upd_pending || c.chm_pending ||
	    tx_push != tx_wr || tx_acked != tx_wr || rx_wr != rx_rd) {
		return;
	}
	while (c.skipped < c.latency) {
		(void)next_channel(); /* the skipped event's channel */
		c.anchor += c.interval_ticks;
		c.counter++;
		c.skipped++;
	}
	cidoo_ble_stats.conn_skipped += c.skipped;
}

/* Interrupts locked: the next event is the first one still ahead of those skipped. */
static void unskip(void)
{
	uint32_t now = tlsr_radio_now();
	uint16_t j;

	if (c.state != ST_WAIT || c.skipped == 0U) {
		return;
	}
	for (j = 0U; j < c.skipped; j++) {
		uint32_t at = c.base_anchor + j * c.interval_ticks;
		uint32_t start = at - widening(at) - (RX_LEAD_US + TIMER_EARLY_US) * TICKS_US;

		if ((int32_t)(start - now) > (int32_t)(2U * START_MIN_TICKS)) {
			break;
		}
	}
	if (j == c.skipped) {
		return; /* the one already set is the first ahead */
	}
	k_timer_stop(&ev_timer);
	c.anchor = c.base_anchor + j * c.interval_ticks;
	c.counter = (uint16_t)(c.base_counter + j);
	c.unmapped = c.base_unmapped;
	for (uint16_t i = 0U; i < j; i++) {
		(void)next_channel();
	}
	cidoo_ble_stats.conn_skipped -= c.skipped - j;
	c.skipped = 0U;
	schedule();
}

bool ble_conn_between_events(void)
{
	return c.state != ST_EVENT;
}

void ble_conn_latency(bool on)
{
	unsigned int key = irq_lock();

	use_latency = on;
	if (!on) {
		unskip();
	}
	irq_unlock(key);
}

static void event_start(void)
{
	uint8_t chn;

	if (c.chm_pending && c.counter == c.chm_instant) {
		set_channel_map(c.chm_new);
		c.chm_pending = false;
	}
	chn = next_channel();
	{
		/* how long after the tick it was due at the timer's handler got here */
		int32_t lat = (int32_t)(tlsr_radio_now() - timer_due);

		if (lat > (int32_t)cidoo_ble_stats.timer_lat_max) {
			cidoo_ble_stats.timer_lat_max = (uint32_t)lat;
		}
	}
	uint32_t now = tlsr_radio_now();
	int32_t ahead = (int32_t)(c.start - now);

	if (c.window != 0U && ahead < (int32_t)(START_NOW_US * TICKS_US) &&
	    (int32_t)(c.anchor + c.window - now) > (int32_t)((RX_LEAD_US + START_NOW_US) * TICKS_US)) {
		/* Too late for the receiver's lead, but the central may send
		 * anywhere in the transmit window: the receiver starts START_NOW_US
		 * from now, when the lines below have run, on what is left of it. */
		c.start = now + START_NOW_US * TICKS_US;
		ahead = (int32_t)(START_NOW_US * TICKS_US);
	}
	if (held || ahead < (int32_t)START_MIN_TICKS || (uint8_t)(rx_wr - rx_rd) > RX_SLOTS - 3U) {
		/* Held for a flash write, too late (interrupts were off), or no
		 * room for what the central could send: it sends again in a later
		 * event. */
		if (held) {
			cidoo_ble_stats.conn_held++;
		} else {
			cidoo_ble_stats.conn_late++;
		}
		event_end(false);
		return;
	}
	c.rx_count = 0;
	c.anchor_seen = false;
	c.extending = false;
	tlsr_radio_conn_event_prepare(chn, c.aa, c.crc_init);
	tlsr_radio_set_sn_nesn(c.sn_nesn);
	push_tx();
	tlsr_radio_conn_timing(c.established,
			       MIN(c.interval * 1250U - EVENT_TIMEOUT_MARGIN_US, EVENT_TIMEOUT_MAX_US));
	tlsr_radio_start_conn_event(empty_pkt, c.start);
	c.state = ST_EVENT;
	arm(c.guard);
}

static void event_end(bool radio_used)
{
	if (radio_used) {
		tlsr_radio_stop();
		tlsr_radio_clear_irq(TLSR_RF_IRQ_ALL);
		if (c.rx_count == 0U) {
			c.sn_nesn = tlsr_radio_sn_nesn();
			c.hw_rptr = tlsr_radio_tx_fifo_rptr();
		}
	}
	tx_acks();
	cidoo_ble_stats.conn_events++;
	if (c.rx_count == 0U) {
		cidoo_ble_stats.conn_missed++;
	}
	if (c.anchor_seen) {
		c.anchor = c.anchor_rx;
		c.last_anchor = c.anchor_rx;
		c.window = 0;
	}
	c.rx_count = 0;
	c.anchor_seen = false;
	c.anchor += c.interval_ticks;
	c.counter++;

	uint32_t now = tlsr_radio_now();

	if (c.upd_pending && c.counter == c.upd_instant) {
		/* Core Vol 6 Part B 5.1.1: the transmit window starts WinOffset
		 * after the anchor the old interval gives the instant. */
		c.anchor += c.upd_win_offset * UNIT_TICKS;
		c.window = c.upd_win_size * UNIT_TICKS;
		c.interval = c.upd_interval;
		c.latency = c.upd_latency;
		c.interval_ticks = c.upd_interval * UNIT_TICKS;
		c.timeout_ticks = c.upd_timeout * 10000U * TICKS_US;
		c.last_rx = now;
		c.upd_pending = false;
	}
	if (!c.established) {
		if (++c.unestablished_events >= 6U) {
			c.terminate = BLE_ERR_CONN_NOT_ESTABLISHED;
		}
	} else if (now - c.last_rx > c.timeout_ticks) {
		c.terminate = BLE_ERR_CONN_TIMEOUT;
	}
	if (c.terminate != 0U) {
		end_connection(c.terminate);
		return;
	}
	skip_events();
	schedule();
}

static void rx_packet(void)
{
	uint8_t slot = rx_wr % RX_SLOTS;
	uint8_t *e = rx_ring[slot];
	bool room = (uint8_t)(rx_wr - rx_rd) < RX_SLOTS - 1U;
	bool keep = false;

	/* The next packet can follow 150 us after the reply: the DMA moves on
	 * first. */
	if (room) {
		tlsr_radio_set_rx_dma(rx_ring[(rx_wr + 1U) % RX_SLOTS]);
	}
	tlsr_radio_clear_irq(TLSR_RF_IRQ_RX);

	uint32_t now = tlsr_radio_now();
	uint8_t len = e[TLSR_RADIO_DMA_HDR + 1];

	if (c.state != ST_EVENT) {
		/* after the event was given up */
	} else if (len <= BLE_PDU_MAX + 4U && sys_get_le32(e) == len + TLSR_RADIO_RX_EXTRA &&
		   (e[TLSR_RADIO_RX_STATUS(len)] & BIT(0)) == 0U) {
		c.sn_nesn = tlsr_radio_sn_nesn();
		c.hw_rptr = tlsr_radio_tx_fifo_rptr();
		tx_acks(); /* what this packet acknowledged, before the thread reads it */
		if (!c.anchor_seen) {
			uint32_t ts = tlsr_radio_rx_timestamp() & ~7U;

			uint32_t at = ts - TLSR_RADIO_TIMESTAMP_OFFSET;

			/* Outside a transmit window the event's first packet starts
			 * within the window widening w of the anchor expected, and one
			 * after a reply no sooner than 456 us after the first (80 us of
			 * packet, 150 us less 2, the reply's 80 us, 150 us less 2). A
			 * packet read late, when the time stamp is the next one's
			 * already, must not move the anchor: while w is under 128 us a
			 * time stamp more than w + 200 us after the anchor expected is
			 * not the first packet's. With a wider w the two ranges overlap
			 * and every time stamp is taken, as in a transmit window. The
			 * guard time is w and RX_GUARD_US after the anchor. */
			int32_t w = (int32_t)(c.guard - c.anchor) - (int32_t)(RX_GUARD_US * TICKS_US);
			bool first = c.window != 0U || w >= (int32_t)(128U * TICKS_US) ||
				     (int32_t)(at - c.anchor) <= w + (int32_t)(200U * TICKS_US);

			if (now - ts <= STAMP_MAX_AGE && first) {
				tc32_rng_add_event(ts, TC32_RNG_SRC_RADIO_RX);
				tc32_rng_add_sample(e[TLSR_RADIO_RX_STATUS(len) - 1U], TC32_RNG_SRC_RADIO_RSSI);
				c.anchor_rx = at;
				c.anchor_seen = true;
				c.latest_end = c.anchor_rx + c.interval_ticks - END_MARGIN_US * TICKS_US;
			} else if (!first) {
				cidoo_ble_stats.conn_anchor_late++;
			}
		}
		c.rx_count++;
		c.last_rx = now;
		c.established = true;
		cidoo_ble_stats.conn_rx++;

		uint8_t sn = (e[TLSR_RADIO_DMA_HDR] >> 3) & 1U;

		if (sn == c.last_rx_sn) {
			cidoo_ble_stats.conn_rx_dup++;
		} else {
			c.last_rx_sn = sn;
			keep = len != 0U && room;
		}
	} else {
		cidoo_ble_stats.conn_rx_bad++;
	}

	if (keep) {
		rx_event[slot] = c.counter;
		rx_wr++;
		k_sem_give(&ll_sem);
		if ((uint8_t)(rx_wr - rx_rd) >= RX_SLOTS - 2U) {
			/* No room for more in this event. */
			event_end(true);
		}
	} else if (room) {
		tlsr_radio_set_rx_dma(e);
	}
}

static void rf_isr(const void *arg)
{
	ARG_UNUSED(arg);
#if IS_ENABLED(CONFIG_TLSR_P24)
	if (p24_running()) {
		p24_rf_isr();
		return;
	}
#endif
	uint16_t st = tlsr_radio_irq_status();

	if ((st & TLSR_RF_IRQ_RX) != 0U) {
		rx_packet();
	}
	if ((st & TLSR_RF_IRQ_TX) != 0U) {
		tlsr_radio_clear_irq(TLSR_RF_IRQ_TX);
	}
	if ((st & TLSR_RF_IRQ_EVENT_END) != 0U) {
		tlsr_radio_clear_irq(TLSR_RF_IRQ_EVENT_END);
		if (c.state == ST_EVENT) {
			event_end(true);
		}
	}
	st &= (uint16_t)~(TLSR_RF_IRQ_RX | TLSR_RF_IRQ_TX | TLSR_RF_IRQ_EVENT_END);
	if (st != 0U) {
		tlsr_radio_clear_irq(st);
	}
}

/* A packet still arriving when the timer would end the event: wait for it
 * (EXTEND_STEP_US, EXTEND_MAX_US). */
static bool extend_event(uint32_t now)
{
	if (!tlsr_radio_receiving()) {
		return false;
	}
	if (!c.extending) {
		c.extending = true;
		c.extend_end = now + EXTEND_MAX_US * TICKS_US;
	} else if ((int32_t)(now - c.extend_end) >= 0) {
		return false;
	}
	cidoo_ble_stats.conn_extended++;
	arm(now + EXTEND_STEP_US * TICKS_US);
	return true;
}

static void ev_timer_run(void)
{
	if (c.state == ST_WAIT) {
		event_start();
	} else if (c.state == ST_EVENT) {
		/* A packet whose RF interrupt has not run yet is taken first: an
		 * event ended before it is read would lose it, and the radio has
		 * acknowledged it if its CRC was right. */
		if ((tlsr_radio_irq_status() & TLSR_RF_IRQ_RX) != 0U) {
			cidoo_ble_stats.conn_rx_timer++;
			rx_packet();
			if (c.state != ST_EVENT) {
				return;
			}
		}

		uint32_t now = tlsr_radio_now();

		if (c.rx_count == 0U || (int32_t)(now - c.latest_end) >= 0) {
			if (!extend_event(now)) {
				event_end(true);
			}
		} else {
			arm(c.latest_end);
		}
	}
}

static void ev_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	in_timer = true; /* arm() counts from this handler's tick */
	ev_timer_run();
	in_timer = false;
}

void ble_conn_init(void)
{
	k_timer_init(&ev_timer, ev_timer_fn, NULL);
	tlsr_radio_set_rx_buffer(rx_ring[0], RX_SLOT_SIZE);
	IRQ_CONNECT(TLSR_RADIO_IRQ, 0, rf_isr, NULL, 0);
	irq_enable(TLSR_RADIO_IRQ);
}

uint8_t *ble_conn_rx_buf(void)
{
	return rx_ring[0];
}

void ble_conn_start(const struct ble_conn_req *req)
{
	unsigned int key = irq_lock();

	memset(&c, 0, sizeof(c));
	c.aa = req->aa;
	c.crc_init = req->crc_init;
	c.interval = req->interval;
	c.latency = req->latency;
	c.interval_ticks = req->interval * UNIT_TICKS;
	c.timeout_ticks = req->timeout * 10000U * TICKS_US;
	c.sca_ppm = sca_ppm[req->sca & 7U] + OWN_SCA_PPM;
	set_channel_map(req->chm);
	c.hop = req->hop;
	/* The first transmit window: 1.25 ms + WinOffset after the CONNECT_IND
	 * (Core Vol 6 Part B 4.5.3). */
	c.anchor = req->t_end + (1U + req->win_offset) * UNIT_TICKS;
	c.window = req->win_size * UNIT_TICKS;
	c.last_anchor = req->t_end;
	c.last_rx = req->t_end;
	/* 0x10 for 0xf03 in the first event (bit 4, the last SN, 1; bit 5, NESN,
	 * 0): as if a packet with SN 1 were in flight, so the central's first
	 * NESN 0 acknowledges it and the first reply has SN 0; SN 0 is expected
	 * from the central. */
	c.sn_nesn = 0x10;
	c.last_rx_sn = 1;
	rx_wr = 0;
	rx_rd = 0;
	tx_wr = 0;
	tx_acked = 0;
	tx_push = 0;
	/* The FIFO's entries count from where its read pointer is after the
	 * FIFO was emptied, which need not be 0: from 0, the entries of the
	 * connection before would be sent again. */
	c.hw_rptr = tlsr_radio_conn_setup();
	ack_rptr = c.hw_rptr;
	tlsr_radio_set_rx_dma(rx_ring[0]);
	cidoo_ble_stats.connections++;
	schedule();
	if ((int32_t)(c.start - TIMER_EARLY_US * TICKS_US - tlsr_radio_now()) <= 0) {
		/* The first transmit window opens sooner than the timer's lead:
		 * the event starts from here, not by the timer's way. */
		k_timer_stop(&ev_timer);
		event_start();
	}
	irq_unlock(key);
}

bool ble_conn_active(void)
{
	return c.state != ST_IDLE;
}

uint8_t ble_conn_reason(void)
{
	return c.reason;
}

int ble_conn_wait(int32_t timeout_ms)
{
	return k_sem_take(&ll_sem, K_MSEC(timeout_ms));
}

uint8_t ble_conn_tx_mark(void)
{
	return tx_wr;
}

bool ble_conn_tx_acked(uint8_t mark)
{
	return (int8_t)(tx_acked - mark) >= 0;
}

uint32_t ble_conn_timeout_ms(void)
{
	return c.timeout_ticks / (1000U * TICKS_US);
}

int ble_conn_wait_ticks(uint32_t ticks)
{
	return k_sem_take(&ll_sem, K_TICKS(ticks));
}

void ble_conn_kick(void)
{
	k_sem_give(&ll_sem);
}

/* An event on air (its BRX armed or running) ends by its own timer or radio interrupt. */
#define HOLD_WAIT_US  100U
#define HOLD_WAIT_MAX 500U /* 50 ms: longer than any event, short of a deadlock */

void ble_conn_hold(void)
{
	held = true;
	for (uint32_t i = 0; i < HOLD_WAIT_MAX && c.state == ST_EVENT; i++) {
		k_busy_wait(HOLD_WAIT_US);
	}
	if (c.state == ST_EVENT) {
		cidoo_ble_stats.hold_timeouts++;
	}
}

void ble_conn_release(void)
{
	held = false;
}

const uint8_t *ble_conn_rx_peek(uint8_t *llid, uint8_t *len, uint16_t *event)
{
	if (rx_rd == rx_wr) {
		return NULL;
	}

	const uint8_t *e = rx_ring[rx_rd % RX_SLOTS];

	*llid = e[TLSR_RADIO_DMA_HDR] & 0x03U;
	*len = e[TLSR_RADIO_DMA_HDR + 1];
	*event = rx_event[rx_rd % RX_SLOTS];
	return &e[TLSR_RADIO_DMA_HDR + 2];
}

void ble_conn_rx_done(void)
{
	rx_rd++;
}

int ble_conn_tx(uint8_t llid, const uint8_t *data, uint8_t len)
{
	if (len > BLE_PDU_MAX + 4U) { /* an encrypted PDU carries its MIC */
		return -EMSGSIZE;
	}
	if ((uint8_t)(tx_wr - tx_acked) >= TX_SLOTS) {
		return -ENOBUFS;
	}

	struct tx_pkt *p = &tx_ring[tx_wr % TX_SLOTS];

	p->dma_len = len + 2U;
	p->hdr = llid;
	p->len = len;
	memcpy(p->data, data, len);
	compiler_barrier();

	unsigned int key = irq_lock();

	tx_wr++;
	unskip(); /* it goes at the next event, not after those skipped */
	irq_unlock(key);
	return 0;
}

uint8_t ble_conn_tx_room(void)
{
	return (uint8_t)(TX_SLOTS - (uint8_t)(tx_wr - tx_acked));
}

uint16_t ble_conn_event_counter(void)
{
	return c.counter;
}

/*
 * An instant must lie after the event the PDU came in (rx_event), and not
 * before the event whose timing is already set (c.counter). When the PDU
 * came in the event just before the instant and is read after that event
 * ended, the event set is the instant itself: while it has not started
 * (ST_WAIT), ble_conn_update() sets its timing again and event_start() takes
 * a new channel map at its start. Events are never skipped for latency
 * while a received PDU is unread (skip_events()).
 */
static bool instant_passed(uint16_t instant, uint16_t rx_event)
{
	uint16_t after_rx = (uint16_t)(instant - rx_event);
	uint16_t ahead = (uint16_t)(instant - c.counter);

	return after_rx == 0U || after_rx >= 0x8000U || ahead >= 0x8000U ||
	       (ahead == 0U && c.state != ST_WAIT);
}

/* Core Vol 6 Part B 2.4.2.1 and 5.1.1, the limits CONNECT_IND's parameters are held to (ble_adv.c). */
static bool update_valid(uint8_t win_size, uint16_t win_offset, uint16_t interval,
			 uint16_t latency, uint16_t timeout)
{
	return interval >= 6U && interval <= 3200U && win_size >= 1U && win_size <= 8U &&
	       win_size < interval && win_offset <= interval && timeout >= 10U && timeout <= 3200U &&
	       latency <= 499U && 4U * (uint32_t)timeout > (1U + latency) * (uint32_t)interval;
}

static bool channel_map_valid(const uint8_t chm[5])
{
	uint8_t used = 0;

	for (size_t i = 0; i < 5; i++) {
		for (uint8_t b = chm[i]; b != 0U; b &= (uint8_t)(b - 1U)) {
			used++;
		}
	}
	return (chm[4] & 0xe0U) == 0U && used >= 2U;
}

void ble_conn_update(uint8_t win_size, uint16_t win_offset, uint16_t interval, uint16_t latency,
		     uint16_t timeout, uint16_t instant, uint16_t rx_event)
{
	unsigned int key = irq_lock();

	if (c.state == ST_IDLE) {
		/* over already */
	} else if (!update_valid(win_size, win_offset, interval, latency, timeout)) {
		ble_conn_terminate(BLE_ERR_INVALID_LL_PARAMS);
	} else if (instant_passed(instant, rx_event)) {
		ble_conn_terminate(BLE_ERR_INSTANT_PASSED);
	} else if (c.counter == instant) {
		/*
		 * The instant's event was set with the old timing and has not
		 * started: it is set again from the new parameters, as
		 * event_end() sets an instant's event.
		 */
		k_timer_stop(&ev_timer);
		c.anchor += win_offset * UNIT_TICKS;
		c.window = win_size * UNIT_TICKS;
		c.interval = interval;
		c.latency = latency;
		c.interval_ticks = interval * UNIT_TICKS;
		c.timeout_ticks = timeout * 10000U * TICKS_US;
		c.last_rx = tlsr_radio_now();
		schedule();
	} else {
		c.upd_win_size = win_size;
		c.upd_win_offset = win_offset;
		c.upd_interval = interval;
		c.upd_latency = latency;
		c.upd_timeout = timeout;
		c.upd_instant = instant;
		c.upd_pending = true;
	}
	irq_unlock(key);
}

void ble_conn_channel_map(const uint8_t chm[5], uint16_t instant, uint16_t rx_event)
{
	unsigned int key = irq_lock();

	if (c.state == ST_IDLE) {
		/* over already */
	} else if (!channel_map_valid(chm)) {
		ble_conn_terminate(BLE_ERR_INVALID_LL_PARAMS);
	} else if (instant_passed(instant, rx_event)) {
		ble_conn_terminate(BLE_ERR_INSTANT_PASSED);
	} else {
		memcpy(c.chm_new, chm, sizeof(c.chm_new));
		c.chm_instant = instant;
		c.chm_pending = true;
	}
	irq_unlock(key);
}

void ble_conn_terminate(uint8_t reason)
{
	unsigned int key = irq_lock();

	if (c.state == ST_WAIT) {
		end_connection(reason);
	} else if (c.state == ST_EVENT) {
		c.terminate = reason;
	}
	irq_unlock(key);
}
