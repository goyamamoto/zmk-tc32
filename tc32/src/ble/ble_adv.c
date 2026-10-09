/*
 * Advertising: legacy ADV_IND on channels 37, 38 and 39 with this
 * firmware's address and data, and the CONNECT_IND that opens a connection.
 *
 * - Address: random static, ID[0..3], the profile's address generation,
 *   0xc0 | (profile + 1): a top byte of this firmware's own, so that none
 *   of its three profiles' addresses is one a host may already know for the
 *   keyboard; a new generation (a cleared bond) is a new address.
 * - Flags: LE General Discoverable while the profile has no bond, so a host
 *   lists it for pairing; with a bond, not discoverable (its host reconnects
 *   by the address), and only that host's CONNECT_IND is taken: from the
 *   address it paired from, its identity address, or a resolvable private
 *   address its IRK resolves (ble_bond_peer_known()). Any other initiator
 *   is ignored. Scan requests are answered from any scanner in both states
 *   (the Core's advertising filter policy 0x02 with a bond, 0x00 without:
 *   Vol 4 Part E 7.8.5).
 * - Name: the profile's (ble_device_name(), "V21 ZMK 2" for the V21's second
 *   profile), so the profiles, and the addresses a host remembers of each,
 *   show under names that tell them apart.
 * - One event: the access address and CRC init of advertising, then per
 *   channel stop, channel, clear the RF status, STX2RX 100 ticks ahead, wait
 *   until TLSR_RF_IRQ_TX reads 1, and a receive window up to 11,520 ticks
 *   (720 us) after the command. The chip's interrupts are off from
 *   ADV_QUIET_LEAD before the earliest time an answer can start (T_IFS after
 *   the ADV_IND's last bit, the ADV_IND starting at its start tick at the
 *   earliest) to the window's end, so that an interrupt cannot make the code
 *   see a scan request late. When a packet starts arriving in the window,
 *   a wait of up to 0x18ff ticks (400 us) until TLSR_RF_IRQ_RX reads 1. A
 *   CONNECT_IND for this address with parameters the Core allows ends the
 *   event; the system tick at its end is the reference of the connection's
 *   first transmit window.
 * - Scan response: a packet that starts as a SCAN_REQ (its header's type)
 *   gets one, with the chip's interrupts still off until the request has
 *   been checked (a CONNECT_IND turns them on again at once): when its RX
 *   status rises,
 *   tlsr_radio_tx_on_rx() starts a single TX of the SCAN_RSP from RAM with a
 *   start tick SCAN_RSP_AHEAD after the tick read with the status, which
 *   puts the response's first bit T_IFS (150 us) after the request's last
 *   bit; then the request is checked (length, CRC, RxAdd, AdvA this
 *   address), and one that is not for this device stops the TX before its
 *   packet goes out. The radio is stopped SCAN_RSP_HOLD after the start
 *   tick, when the response has been sent whatever the TX takes to start
 *   (up to 160 us), and the event goes on to the next channel. The SCAN_RSP
 *   is AdvA and no data: the ADV_IND carries the flags, the appearance, the
 *   16-bit UUIDs and the name already, and a scanner takes the two
 *   together, so a response with nothing more is the shortest on air.
 * - The event runs with the scheduler locked, so that ZMK's cooperative
 *   threads cannot delay the polling; interrupts stay on but for the receive
 *   windows and a scan response's wait and check.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_internal.h"
#include <tlsr_radio.h>

#define ADV_START_AHEAD 100U    /* ticks between the STX2RX command and its start */
#define ADV_WINDOW      11520U  /* ticks from the command to the end of the receive window */
#define ADV_TX_TIMEOUT  32000U  /* 2 ms: TLSR_RF_IRQ_TX never came */
#define ADV_RX_END      0x18ffU /* ticks to wait for the end of a packet that has started */
#define ADV_PDU_MAX     37U

#define PDU_ADV_IND     0x00U
#define PDU_SCAN_REQ    0x03U
#define PDU_SCAN_RSP    0x04U
#define PDU_CONNECT_IND 0x05U
#define PDU_TXADD       BIT(6)
#define PDU_RXADD       BIT(7)
#define CONNECT_IND_LEN 34U
#define SCAN_REQ_LEN    12U
#define ADV_AD_FIXED    13U     /* the ADV_IND's flags, appearance and UUIDs */
/* The ADV_IND's length, its name complete or shortened to what fits. */
#define ADV_LEN         MIN(ADV_PDU_MAX, BLE_ADDR_LEN + ADV_AD_FIXED + 2U + BLE_NAME_MAX - 1U)
#define ADV_QUIET_LEAD  50U     /* us before the earliest answer that the interrupts go off */
/* Ticks from the STX2RX command to that point: its start tick, the ADV_IND (preamble, access address, header,
 * payload, CRC at 1 Mbit/s), T_IFS.
 */
#define ADV_QUIET       (ADV_START_AHEAD + ((1U + 4U + 2U + ADV_LEN + 3U) * 8U + 150U - ADV_QUIET_LEAD) * \
			 TLSR_RADIO_TICKS_PER_US)

#define SCAN_RSP_AHEAD  48U     /* ticks from the tick read with the SCAN_REQ's RX status to the TX's start */
/* From the TX's start tick until the radio is stopped: its first bit at most 160 us after the start tick, then
 * the SCAN_RSP's preamble, access address, header, AdvA and CRC at 1 Mbit/s.
 */
#define SCAN_RSP_HOLD   ((160U + (1U + 4U + 2U + BLE_ADDR_LEN + 3U) * 8U) * TLSR_RADIO_TICKS_PER_US)

BUILD_ASSERT(CONFIG_TLSR_BLE_PROFILES <= 9, "a profile's number is one digit");

static uint8_t adv_pkt[TLSR_RADIO_DMA_HDR + 2 + ADV_PDU_MAX] __aligned(4);
static uint8_t adv_addr[BLE_ADDR_LEN];
static uint8_t adv_profile;
static bool adv_bonded_only; /* not discoverable: the profile's bonded host may connect, no one else */
/* The SCAN_RSP: DMA length, header (TxAdd), length, AdvA. */
static uint8_t scan_rsp_pkt[TLSR_RADIO_DMA_HDR + 2 + BLE_ADDR_LEN] __aligned(4);

uint8_t ble_device_name(uint8_t profile, char out[BLE_NAME_MAX])
{
	uint8_t len = sizeof(CONFIG_ZMK_KEYBOARD_NAME) - 1U;

	memcpy(out, CONFIG_ZMK_KEYBOARD_NAME, len);
	out[len++] = ' ';
	out[len++] = (char)('1' + profile);
	out[len] = '\0';
	return len;
}

void ble_adv_init(const uint8_t id[4], uint8_t profile, uint8_t gen, bool discoverable)
{
	const uint8_t ad[ADV_AD_FIXED] = {
		0x02, 0x01, discoverable ? 0x06 : 0x04, /* flags: (LE General Discoverable,) no BR/EDR */
		0x03, 0x19, 0xc1, 0x03,       /* appearance: keyboard */
		0x05, 0x03, 0x12, 0x18, 0x0f, 0x18, /* 16-bit UUIDs: HID, Battery */
	};
	uint8_t *pdu = &adv_pkt[TLSR_RADIO_DMA_HDR];
	uint8_t *pl = &pdu[2];
	char name[BLE_NAME_MAX];
	size_t n = 0;

	memcpy(adv_addr, id, 4);
	adv_addr[4] = gen;
	adv_addr[5] = 0xc0U | (uint8_t)(profile + 1U);
	adv_profile = profile;
	adv_bonded_only = !discoverable;
	memcpy(&pl[n], adv_addr, BLE_ADDR_LEN);
	n += BLE_ADDR_LEN;
	memcpy(&pl[n], ad, sizeof(ad));
	n += sizeof(ad);

	size_t len = ble_device_name(profile, name);
	size_t room = ADV_PDU_MAX - n - 2;
	bool complete = len <= room;

	if (!complete) {
		len = room;
	}
	pl[n++] = (uint8_t)(len + 1);
	pl[n++] = complete ? 0x09 : 0x08;
	memcpy(&pl[n], name, len);
	n += len;

	pdu[0] = PDU_ADV_IND | PDU_TXADD;
	pdu[1] = (uint8_t)n;
	sys_put_le32(n + 2, adv_pkt);
	sys_put_le32(2 + BLE_ADDR_LEN, scan_rsp_pkt);
	scan_rsp_pkt[TLSR_RADIO_DMA_HDR] = PDU_SCAN_RSP | PDU_TXADD;
	scan_rsp_pkt[TLSR_RADIO_DMA_HDR + 1] = BLE_ADDR_LEN;
	memcpy(&scan_rsp_pkt[TLSR_RADIO_DMA_HDR + 2], adv_addr, BLE_ADDR_LEN);
}

void ble_adv_address(uint8_t addr[BLE_ADDR_LEN])
{
	memcpy(addr, adv_addr, BLE_ADDR_LEN);
}

static uint8_t popcount8(uint8_t v)
{
	uint8_t n = 0;

	for (; v != 0U; v &= (uint8_t)(v - 1U)) {
		n++;
	}
	return n;
}

/* A CONNECT_IND addressed to this device, from the bonded host while the
 * profile is not discoverable, with parameters the Core allows (Vol 6 Part B
 * 2.3.3.1 and 4.5). Anything else is ignored, and the central fails to
 * establish the connection. */
static bool connect_ind(const uint8_t *e, uint32_t t_end, struct ble_conn_req *req)
{
	uint8_t hdr = e[TLSR_RADIO_DMA_HDR];
	uint8_t len = e[TLSR_RADIO_DMA_HDR + 1];
	const uint8_t *p = &e[TLSR_RADIO_DMA_HDR + 2];

	if (len != CONNECT_IND_LEN || sys_get_le32(e) != len + TLSR_RADIO_RX_EXTRA ||
	    (e[TLSR_RADIO_RX_STATUS(len)] & BIT(0)) != 0U) {
		return false;
	}
	if ((hdr & 0x0fU) != PDU_CONNECT_IND || (hdr & PDU_RXADD) == 0U ||
	    memcmp(&p[6], adv_addr, BLE_ADDR_LEN) != 0) {
		return false;
	}
	if (adv_bonded_only && !ble_bond_peer_known(adv_profile, p, (hdr & PDU_TXADD) != 0U)) {
		cidoo_ble_stats.adv_refused++;
		return false;
	}

	const uint8_t *ll = &p[12];

	memcpy(req->peer, p, BLE_ADDR_LEN);
	req->peer_random = (hdr & PDU_TXADD) != 0U;
	req->aa = sys_get_le32(&ll[0]);
	req->crc_init = sys_get_le24(&ll[4]);
	req->win_size = ll[7];
	req->win_offset = sys_get_le16(&ll[8]);
	req->interval = sys_get_le16(&ll[10]);
	req->latency = sys_get_le16(&ll[12]);
	req->timeout = sys_get_le16(&ll[14]);
	memcpy(req->chm, &ll[16], sizeof(req->chm));
	req->hop = ll[21] & 0x1fU;
	req->sca = ll[21] >> 5;
	req->t_end = t_end;

	uint8_t used = 0;

	for (size_t i = 0; i < sizeof(req->chm); i++) {
		used += popcount8(req->chm[i]);
	}

	return req->interval >= 6U && req->interval <= 3200U && req->win_size >= 1U &&
	       req->win_size <= 8U && req->win_size < req->interval &&
	       req->win_offset <= req->interval && req->timeout >= 10U && req->timeout <= 3200U &&
	       req->latency <= 499U &&
	       4U * (uint32_t)req->timeout > (1U + req->latency) * (uint32_t)req->interval &&
	       req->hop >= 5U && req->hop <= 16U && (req->chm[4] & 0xe0U) == 0U && used >= 2U &&
	       req->aa != TLSR_BLE_ADV_ACCESS_ADDRESS;
}

/* A SCAN_REQ addressed to this device: its DMA length (the low octet), CRC, type with RxAdd, and AdvA. */
static bool scan_req_for_us(const uint8_t *e)
{
	return e[0] == SCAN_REQ_LEN + TLSR_RADIO_RX_EXTRA && e[TLSR_RADIO_DMA_HDR + 1] == SCAN_REQ_LEN &&
	       (e[TLSR_RADIO_RX_STATUS(SCAN_REQ_LEN)] & BIT(0)) == 0U &&
	       (e[TLSR_RADIO_DMA_HDR] & (0x0fU | PDU_RXADD)) == (PDU_SCAN_REQ | PDU_RXADD) &&
	       memcmp(&e[TLSR_RADIO_DMA_HDR + 2 + BLE_ADDR_LEN], adv_addr, BLE_ADDR_LEN) == 0;
}

/*
 * The packet arriving since t1 starts as a SCAN_REQ: the SCAN_RSP when its RX
 * status rises, then the request checked, and the TX stopped at once when the
 * request is not for this device; else the wait until the response has been
 * sent. The chip's interrupts, off at the call, are turned on again (irq_en)
 * once the request has been checked. The radio is stopped and its status
 * cleared by the caller (the next channel, or the event's end).
 */
static void scan_response(const uint8_t *rx, uint32_t t1, uint8_t irq_en)
{
	uint32_t tick;
	bool sent = tlsr_radio_tx_on_rx(scan_rsp_pkt, SCAN_RSP_AHEAD, t1, ADV_RX_END, &tick);

	if (sent && !scan_req_for_us(rx)) {
		tlsr_radio_stop();
		sent = false;
	}
	tlsr_radio_irq_restore(irq_en);
	if (sent) {
		cidoo_ble_stats.scan_rsps++;
		while ((int32_t)(tlsr_radio_now() - tick) < (int32_t)SCAN_RSP_HOLD) {
		}
	}
}

/*
 * One advertising event on the three channels. True when a central's
 * CONNECT_IND was taken: req holds it and the connection's events have been
 * started (ble_conn_start()).
 */
bool ble_adv_event(struct ble_conn_req *req)
{
	static const uint8_t channels[] = {37, 38, 39};
	uint8_t *rx = ble_conn_rx_buf();
	volatile uint32_t *rx_hdr = (volatile uint32_t *)&rx[TLSR_RADIO_DMA_HDR];
	bool found = false;

	k_sched_lock();
	tlsr_radio_off();
	tlsr_radio_set_access_address(TLSR_BLE_ADV_ACCESS_ADDRESS);
	tlsr_radio_set_crc_init(TLSR_BLE_ADV_CRC_INIT);
	tlsr_radio_adv_timing();

	for (size_t i = 0; i < ARRAY_SIZE(channels) && !found; i++) {
		tlsr_radio_stop();
		tlsr_radio_set_ble_channel(channels[i]);
		tlsr_radio_clear_irq(TLSR_RF_IRQ_RX | TLSR_RF_IRQ_TX);
		*rx_hdr = 0;
		tlsr_radio_start_tx_then_rx(adv_pkt, tlsr_radio_now() + ADV_START_AHEAD);

		uint32_t t0 = tlsr_radio_now();

		while ((tlsr_radio_irq_status() & TLSR_RF_IRQ_TX) == 0U) {
			if (tlsr_radio_now() - t0 > ADV_TX_TIMEOUT) {
				cidoo_ble_stats.adv_tx_timeouts++;
				break;
			}
		}
		tlsr_radio_clear_irq(TLSR_RF_IRQ_TX);
		while (*rx_hdr == 0U && tlsr_radio_now() - t0 < ADV_QUIET) {
		}

		uint8_t irq_en = tlsr_radio_irq_off();

		while (*rx_hdr == 0U && tlsr_radio_now() - t0 < ADV_WINDOW) {
		}
		if (*rx_hdr == 0U) {
			tlsr_radio_irq_restore(irq_en);
			continue;
		}
		/* A packet is arriving: a scan request or a connection request. */
		cidoo_ble_stats.adv_rx++;

		uint32_t t1 = tlsr_radio_now();

		if ((rx[TLSR_RADIO_DMA_HDR] & 0x0fU) == PDU_SCAN_REQ) {
			scan_response(rx, t1, irq_en);
			continue;
		}
		tlsr_radio_irq_restore(irq_en);

		while ((tlsr_radio_irq_status() & TLSR_RF_IRQ_RX) == 0U &&
		       tlsr_radio_now() - t1 <= ADV_RX_END) {
		}

		uint32_t t_end = tlsr_radio_now();

		if ((tlsr_radio_irq_status() & TLSR_RF_IRQ_RX) != 0U) {
			tlsr_radio_clear_irq(TLSR_RF_IRQ_RX);
			found = connect_ind(rx, t_end, req);
		}
	}
	tlsr_radio_stop();
	tlsr_radio_clear_irq(TLSR_RF_IRQ_ALL);
	if (found) {
		/* The connection's events start here, before the scheduler lets
		 * other threads run: the first transmit window opens 1.25 ms after
		 * the CONNECT_IND. */
		ble_conn_start(req);
	}
	k_sched_unlock();
	cidoo_ble_stats.adv_events++;
	return found;
}
