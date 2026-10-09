/*
 * The link's security and L2CAP for the own BLE stack, in the BLE thread.
 *
 * - Encryption (Core Vol 6 Part B 5.1.3.1): on LL_ENC_REQ the peripheral
 *   answers LL_ENC_RSP with its SKDs and IVs (tc32_rng_get(); when it gives
 *   none, LL_REJECT_IND with 0x1f, unspecified error, and no LL_ENC_RSP),
 *   takes the LTK (the key of the pairing in progress for EDIV 0 and
 *   Rand 0 while one is under way, else a bond's), sends
 *   LL_START_ENC_REQ in the clear and decrypts from then on; the central's
 *   encrypted LL_START_ENC_RSP turns encryption on for sending, and the
 *   peripheral's own LL_START_ENC_RSP is its first encrypted PDU. No LTK:
 *   LL_REJECT_IND with 0x06 (PIN or key missing). A MIC failure ends the
 *   connection (0x3d). From its LL_START_ENC_REQ until its own
 *   LL_START_ENC_RSP the peripheral sends no data PDU: ble_link_tx() refuses
 *   one with -ENOBUFS, and its sender tries again later or drops it.
 * - Encryption pause (5.1.3.2), on an encrypted link: on LL_PAUSE_ENC_REQ
 *   the peripheral queues LL_PAUSE_ENC_RSP, encrypted, behind the PDUs
 *   already queued, and receives in the clear once the central has
 *   acknowledged it (ble_conn.c counts the acknowledgement at the packet
 *   that carries it); the central's LL_PAUSE_ENC_RSP, in the clear, turns
 *   encryption off for sending. An
 *   encryption start as above follows, with the key of the pairing in
 *   progress or a bond's; without a key the peripheral ends the link with
 *   LL_TERMINATE_IND 0x06 (PIN or key missing). From LL_PAUSE_ENC_REQ until
 *   the central's LL_START_ENC_RSP no data PDU is sent (-ENOBUFS, as above),
 *   and a PDU from the central other than an empty one, LL_PAUSE_ENC_RSP,
 *   LL_ENC_REQ, LL_START_ENC_RSP or LL_TERMINATE_IND ends the connection
 *   (0x3d). Reports wait for the end of the pause; the link stays encrypted
 *   for the ATT server.
 * - Room for answers: a report (an ATT notification), the Service Changed
 *   indication or the connection parameter request takes a slot of the
 *   connection's TX ring (BLE_TX_SLOTS PDUs the central has not
 *   acknowledged) only while more than BLE_TX_ANSWER_ROOM are free, else
 *   -ENOBUFS and its sender gives it again; the slots kept free are for LL control PDUs, ATT responses, SMP
 *   PDUs and signalling answers, so reports have 8 slots and answers 8 of
 *   their own. A central that keeps the procedures' rules has at most seven
 *   answers waiting at once: two ATT responses and two signalling answers
 *   (an answer, and the next request's when that request comes in the
 *   packet that acknowledges it: ble_conn.c counts an acknowledgement at
 *   that packet, before the thread reads it), and the three LL PDUs of an
 *   encryption start or the SMP PDUs of a pairing step (the pairing and the
 *   encryption start do not overlap); the Security Request, on a link that
 *   is not encrypted, makes eight. An answer goes in the next connection
 *   events, behind the reports queued before it, within the procedures'
 *   response timeouts (Core Vol 6 Part B 5.2: 40 s; ATT: 30 s).
 * - Every non-empty PDU goes through here: ble_link_tx() encrypts it when
 *   sending is encrypted, ble_link_rx() decrypts it when receiving is; the
 *   packet counters count those PDUs (ble_conn.c retransmits the same
 *   octets).
 * - The central's public values go to tc32_rng_add_sample() (no credit): its
 *   address, the connection's access address and CRC init at the start,
 *   its SKDm and IVm at each LL_ENC_REQ.
 * - L2CAP: basic frames: 0x0004 ATT (MTU 23, one LL PDU), 0x0005 LE
 *   signaling (requests answered with Command Reject; the result of the
 *   answer to this side's Connection Parameter Update Request kept for
 *   ble.c), 0x0006 SMP, whose
 *   Pairing Public Key PDU (65 octets) comes and goes in three LL PDUs:
 *   a frame longer than one PDU is put together from its continuation
 *   PDUs, for SMP only and up to that length; sending one queues all its
 *   PDUs at once or none.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_crypto.h"
#include "ble_internal.h"

#define LL_TERMINATE_IND    0x02U
#define LL_ENC_REQ          0x03U
#define LL_ENC_RSP          0x04U
#define LL_START_ENC_REQ    0x05U
#define LL_START_ENC_RSP    0x06U
#define LL_PAUSE_ENC_RSP    0x0bU
#define LL_REJECT_IND       0x0dU
/* The PDUs the central may send during an encryption pause (Core Vol 6 Part B 5.1.3.2). */
#define PAUSE_PDUS (BIT(LL_TERMINATE_IND) | BIT(LL_ENC_REQ) | BIT(LL_START_ENC_RSP) | BIT(LL_PAUSE_ENC_RSP))
#define BLE_ERR_KEY_MISSING 0x06U
#define BLE_ERR_MIC_FAILURE 0x3dU
#define BLE_ERR_UNSPECIFIED 0x1fU

#define L2CAP_HDR       4U
#define L2CAP_FRAME_MAX 65U /* the Pairing Public Key PDU */
#define L2CAP_CID_ATT   0x0004U
#define L2CAP_CID_SIG   0x0005U
#define L2CAP_CID_SMP   0x0006U
#define SIG_CMD_REJECT  0x01U
#define SIG_CONN_PARAM_REQ 0x12U
#define SIG_CONN_PARAM_RSP 0x13U
#define ATT_NOTIFY         0x1bU
#define ATT_INDICATE       0x1dU

/* A frame longer than one LL PDU being put together (SMP only). got == 0: none. */
static struct {
	uint16_t len;
	uint8_t got;
	uint8_t buf[L2CAP_FRAME_MAX];
} frag;

static struct {
	bool rx_enc, tx_enc;
	bool encrypted;       /* both ways, until the connection ends */
	bool authenticated;   /* the key came from a pairing with MITM protection */
	bool have_pairing_key, pairing_key_authenticated;
	bool bonded;          /* a bond's LTK encrypts the link, or a bond was made on it */
	bool enc_req;         /* the central has sent LL_ENC_REQ on this link */
	bool paused;          /* from LL_PAUSE_ENC_REQ until the central's LL_START_ENC_RSP */
	bool pause_wait;      /* this side's LL_PAUSE_ENC_RSP is queued: once acknowledged, receiving is in the clear */
	uint8_t pause_mark;   /* the TX ring after that LL_PAUSE_ENC_RSP (ble_conn_tx_mark()) */
	uint8_t end_reason;   /* the link is to end with LL_TERMINATE_IND and this reason (ble.c) */
	uint8_t sk[16];
	uint8_t iv[8];
	uint8_t stk[16];
	uint64_t rx_ctr, tx_ctr;
	uint8_t peer[BLE_ADDR_LEN];
	uint8_t peer_random;
	uint8_t own[BLE_ADDR_LEN];
	/* this side's last Connection Parameter Update Request: its identifier (0: none) and the central's
	 * result (-1: no answer yet, 0 accepted, 1 rejected) */
	uint8_t cpu_ident;
	int8_t cpu_result;
} l;

void ble_link_start(const struct ble_conn_req *req)
{
	memset(&l, 0, sizeof(l));
	memcpy(l.peer, req->peer, BLE_ADDR_LEN);
	l.peer_random = req->peer_random;
	tc32_rng_add_sample(sys_get_le32(&req->peer[2]) ^ req->peer[0] ^ ((uint32_t)req->peer[1] << 8),
			    TC32_RNG_SRC_PEER_ADDRESS);
	tc32_rng_add_sample(req->aa ^ (req->crc_init << 8), TC32_RNG_SRC_PEER_RANDOM);
	ble_adv_address(l.own);
	frag.got = 0U;
	l.cpu_result = -1;
	ble_smp_reset();
	ble_att_reset();
}

void ble_link_end(void)
{
	memset(&l, 0, sizeof(l));
	ble_smp_reset();
	ble_att_reset();
}

void ble_link_addresses(uint8_t peer[BLE_ADDR_LEN], uint8_t *peer_random, uint8_t own[BLE_ADDR_LEN])
{
	memcpy(peer, l.peer, BLE_ADDR_LEN);
	*peer_random = l.peer_random;
	memcpy(own, l.own, BLE_ADDR_LEN);
}

bool ble_link_encrypted(void)
{
	return l.encrypted;
}

bool ble_link_bonded(void)
{
	return l.encrypted && l.bonded;
}

void ble_link_set_bonded(void)
{
	l.bonded = true;
}

void ble_link_set_pairing_key(const uint8_t ltk[16], bool authenticated)
{
	memcpy(l.stk, ltk, sizeof(l.stk));
	l.have_pairing_key = true;
	l.pairing_key_authenticated = authenticated;
}

bool ble_link_peer_bonded(void)
{
	return ble_bond_peer_known(ble_profile_active(), l.peer, l.peer_random != 0U);
}

bool ble_link_authenticated(void)
{
	return l.encrypted && l.authenticated;
}

bool ble_link_enc_requested(void)
{
	return l.enc_req;
}

uint8_t ble_link_end_reason(void)
{
	return l.end_reason;
}

int ble_link_tx(uint8_t llid, const uint8_t *data, uint8_t len)
{
	uint8_t buf[BLE_PDU_MAX + 4U];
	int err;

	if (len > BLE_PDU_MAX) {
		return -EMSGSIZE;
	}
	if (llid != LLID_CONTROL && (l.paused || (l.rx_enc && !l.tx_enc))) {
		return -ENOBUFS; /* an encryption start or pause is under way */
	}
	if (!l.tx_enc || len == 0U) {
		return ble_conn_tx(llid, data, len);
	}
	memcpy(buf, data, len);
	ble_ccm_encrypt(l.sk, l.tx_ctr, false, l.iv, llid, buf, len);
	err = ble_conn_tx(llid, buf, len + 4U);
	if (err == 0) {
		l.tx_ctr++;
	}
	return err;
}

bool ble_link_rx(uint8_t *pdu, uint8_t hdr0, uint8_t *len)
{
	if (l.pause_wait && ble_conn_tx_acked(l.pause_mark)) {
		l.pause_wait = false;
		l.rx_enc = false; /* this side's LL_PAUSE_ENC_RSP acknowledged: the central sends in the clear */
	}
	if (l.rx_enc && *len != 0U) {
		if (!ble_ccm_decrypt(l.sk, l.rx_ctr, true, l.iv, hdr0, pdu, *len)) {
			cidoo_ble_stats.mic_failures++;
			ble_conn_terminate(BLE_ERR_MIC_FAILURE);
			return false;
		}
		l.rx_ctr++;
		*len -= 4U;
	}
	if (l.paused && *len != 0U &&
	    ((hdr0 & 0x03U) != LLID_CONTROL || pdu[0] > LL_PAUSE_ENC_RSP || (BIT(pdu[0]) & PAUSE_PDUS) == 0U)) {
		ble_conn_terminate(BLE_ERR_MIC_FAILURE); /* not a PDU of the pause */
		return false;
	}
	return true;
}

/* LL_ENC_REQ: Rand (8), EDIV (2), SKDm (8), IVm (4) after the opcode. */
void ble_link_enc_req(const uint8_t *p, uint8_t len)
{
	uint8_t rsp[13] = {LL_ENC_RSP};
	uint8_t rej[2] = {LL_REJECT_IND, BLE_ERR_KEY_MISSING};
	uint8_t ltk[16];
	bool found;

	if (len != 23U) {
		return;
	}
	l.enc_req = true;
	tc32_rng_add_sample(sys_get_le32(&p[11]) ^ sys_get_le32(&p[19]), TC32_RNG_SRC_PEER_RANDOM); /* SKDm, IVm */
	if (tc32_rng_get(&rsp[1], 12) != 0) { /* SKDs, IVs */
		rej[1] = BLE_ERR_UNSPECIFIED;
		(void)ble_link_tx(LLID_CONTROL, rej, sizeof(rej));
		return;
	}
	memcpy(&l.iv[0], &p[19], 4);
	memcpy(&l.iv[4], &rsp[9], 4);
	(void)ble_link_tx(LLID_CONTROL, rsp, sizeof(rsp));

	if (sys_get_le16(&p[9]) == 0U && sys_get_le64(&p[1]) == 0U && l.have_pairing_key) {
		memcpy(ltk, l.stk, sizeof(ltk));
		l.authenticated = l.pairing_key_authenticated;
		found = true;
	} else {
		found = ble_bond_find_ltk(ble_profile_active(), sys_get_le16(&p[9]), &p[1], ltk,
					  &l.authenticated);
		l.bonded = found;
	}
	if (!found) {
		if (l.paused) {
			l.end_reason = BLE_ERR_KEY_MISSING; /* after a pause: the link ends */
			return;
		}
		(void)ble_link_tx(LLID_CONTROL, rej, sizeof(rej));
		return;
	}
	ble_session_key(ltk, &p[11], &rsp[1], l.sk);
	memset(ltk, 0, sizeof(ltk));

	uint8_t start[1] = {LL_START_ENC_REQ};

	(void)ble_link_tx(LLID_CONTROL, start, sizeof(start));
	l.rx_enc = true;
	l.rx_ctr = 0;
}

/* The central's LL_START_ENC_RSP, received encrypted. */
void ble_link_start_enc_rsp(void)
{
	uint8_t rsp[1] = {LL_START_ENC_RSP};

	if (!l.rx_enc || l.tx_enc) {
		return;
	}
	l.tx_enc = true;
	l.tx_ctr = 0;
	l.paused = false;
	(void)ble_link_tx(LLID_CONTROL, rsp, sizeof(rsp));
	l.encrypted = true;
	cidoo_ble_stats.encryptions++;
	if (l.bonded && !l.have_pairing_key) {
		/* a reconnect with a stored key: the client's CCCDs as it left them,
		 * unless it wrote some already on this link */
		ble_att_bond_encrypted(ble_bond_cccd(ble_profile_active()));
	}
	ble_smp_encrypted(l.have_pairing_key);
}

/* The central's LL_PAUSE_ENC_REQ; false when the link is not encrypted (the caller answers LL_UNKNOWN_RSP). */
bool ble_link_pause_enc_req(void)
{
	uint8_t rsp[1] = {LL_PAUSE_ENC_RSP};

	if (!l.tx_enc || !l.rx_enc) {
		return false;
	}
	l.paused = true;
	(void)ble_link_tx(LLID_CONTROL, rsp, sizeof(rsp));
	l.pause_mark = ble_conn_tx_mark();
	l.pause_wait = true;
	return true;
}

/* The central's LL_PAUSE_ENC_RSP, received in the clear. */
void ble_link_pause_enc_rsp(void)
{
	if (l.paused && !l.rx_enc) {
		l.tx_enc = false;
	}
}

static int l2cap_send(uint16_t cid, const uint8_t *data, uint8_t len, bool later)
{
	uint8_t buf[L2CAP_HDR + L2CAP_FRAME_MAX];
	uint8_t total = len + L2CAP_HDR;
	int err = 0;

	if (len > L2CAP_FRAME_MAX) {
		return -EMSGSIZE;
	}
	if ((later || (cid == L2CAP_CID_ATT && (data[0] == ATT_NOTIFY || data[0] == ATT_INDICATE))) &&
	    ble_conn_tx_room() <= BLE_TX_ANSWER_ROOM) {
		return -ENOBUFS; /* a report or a request of this side: the slots left are for answers */
	}
	/* every PDU of the frame, or none: the ring's room checked first, the encryption state by the first PDU */
	if (total > BLE_PDU_MAX && ble_conn_tx_room() < (total + BLE_PDU_MAX - 1U) / BLE_PDU_MAX) {
		return -ENOBUFS;
	}
	sys_put_le16(len, &buf[0]);
	sys_put_le16(cid, &buf[2]);
	memcpy(&buf[L2CAP_HDR], data, len);
	for (uint8_t off = 0; off < total && err == 0; off += BLE_PDU_MAX) {
		err = ble_link_tx(off == 0U ? LLID_START : LLID_CONTINUE, &buf[off],
				  MIN((uint8_t)(total - off), BLE_PDU_MAX));
	}
	return err;
}

int ble_l2cap_send(uint16_t cid, const uint8_t *data, uint8_t len)
{
	return l2cap_send(cid, data, len, false);
}

/*
 * An L2CAP Connection Parameter Update Request (Core Vol 3 Part A 4.20),
 * about 1 s after the connection (ble.c): by default interval 6 to 6
 * (7.5 ms), latency 44, timeout 300 (3 s), identifier 1. The central
 * answers with a response (signaling_rx() keeps its result) and applies
 * what it accepts with LL_CONNECTION_UPDATE_IND (ble_ll_ctrl.c). After a
 * rejection ble.c may ask once more with the fallback set (identifier 2),
 * by default 15 ms to 15 ms, latency 22, 6 s, inside the limits Apple
 * hosts accept. -ENOBUFS (only the TX ring's slots for answers free, or an
 * encryption start or pause under way) leaves it to a later call.
 */
int ble_link_request_conn_params(bool fallback)
{
	/* interval min = max, latency, timeout: the first request's, the fallback's */
	static const uint8_t sets[][6] = {
		{CONFIG_TLSR_BLE_CONN_INTERVAL & 0xff, CONFIG_TLSR_BLE_CONN_INTERVAL >> 8,
		 CONFIG_TLSR_BLE_CONN_LATENCY & 0xff, CONFIG_TLSR_BLE_CONN_LATENCY >> 8,
		 CONFIG_TLSR_BLE_CONN_TIMEOUT & 0xff, CONFIG_TLSR_BLE_CONN_TIMEOUT >> 8},
#if IS_ENABLED(CONFIG_TLSR_BLE_CONN_FALLBACK)
		{CONFIG_TLSR_BLE_CONN_FALLBACK_INTERVAL & 0xff, CONFIG_TLSR_BLE_CONN_FALLBACK_INTERVAL >> 8,
		 CONFIG_TLSR_BLE_CONN_FALLBACK_LATENCY & 0xff, CONFIG_TLSR_BLE_CONN_FALLBACK_LATENCY >> 8,
		 CONFIG_TLSR_BLE_CONN_FALLBACK_TIMEOUT & 0xff, CONFIG_TLSR_BLE_CONN_FALLBACK_TIMEOUT >> 8},
#endif
	};
	const uint8_t *set = sets[fallback && ARRAY_SIZE(sets) > 1U ? 1U : 0U];
	uint8_t req[12] = {SIG_CONN_PARAM_REQ, fallback ? 2U : 1U, 8, 0};
	int err;

	memcpy(&req[4], set, 2);
	memcpy(&req[6], set, 6);
	err = l2cap_send(L2CAP_CID_SIG, req, sizeof(req), true);
	if (err == 0) {
		l.cpu_ident = req[1];
		l.cpu_result = -1;
	}
	return err;
}

int ble_link_conn_params_result(void)
{
	return l.cpu_result;
}

static void signaling_rx(const uint8_t *p, uint8_t len)
{
	if (len < 4U) {
		return;
	}
	/*
	 * Nothing to answer: Command Reject, the responses (disconnection,
	 * connection parameter update, the credit based connections) and the
	 * flow control credit indication (Core Vol 3 Part A 4). The response
	 * to this side's request: its result (0x0000 accepted, 0x0001
	 * rejected, 4.21).
	 */
	switch (p[0]) {
	case SIG_CONN_PARAM_RSP:
		if (len >= 6U && p[1] == l.cpu_ident && l.cpu_ident != 0U) {
			l.cpu_result = sys_get_le16(&p[4]) == 0U ? 0 : 1;
		}
		return;
	case SIG_CMD_REJECT:
	case 0x07U:
	case 0x15U:
	case 0x16U:
	case 0x18U:
	case 0x1aU:
		return;
	default:
		break;
	}
	uint8_t rej[6] = {SIG_CMD_REJECT, p[1], 2, 0, 0, 0}; /* reason 0: not understood */

	(void)ble_l2cap_send(L2CAP_CID_SIG, rej, sizeof(rej));
}

void ble_link_rx_data(const uint8_t *p, uint8_t llid, uint8_t len)
{
	uint16_t cid = L2CAP_CID_SMP;
	const uint8_t *data = frag.buf;
	uint16_t l2len = frag.len;

	cidoo_ble_stats.data_rx++;
	if (llid == LLID_START) {
		frag.got = 0U; /* a start PDU ends any frame left unfinished */
		if (len < L2CAP_HDR) {
			goto drop;
		}
		l2len = sys_get_le16(&p[0]);
		cid = sys_get_le16(&p[2]);
		data = &p[L2CAP_HDR];
		len -= L2CAP_HDR;
		if (l2len != len) {
			if (cid != L2CAP_CID_SMP || l2len < len || l2len > L2CAP_FRAME_MAX) {
				goto drop;
			}
			frag.len = l2len;
			frag.got = len;
			memcpy(frag.buf, data, len);
			return;
		}
	} else if (llid == LLID_CONTINUE && frag.got != 0U) {
		if ((uint16_t)frag.got + len > frag.len) {
			frag.got = 0U;
			goto drop;
		}
		memcpy(&frag.buf[frag.got], p, len);
		frag.got += len;
		if (frag.got != frag.len) {
			return;
		}
		frag.got = 0U;
	} else {
		goto drop;
	}
	switch (cid) {
	case L2CAP_CID_ATT:
		ble_att_rx(data, (uint8_t)l2len);
		return;
	case L2CAP_CID_SMP:
		ble_smp_rx(data, (uint8_t)l2len);
		return;
	case L2CAP_CID_SIG:
		signaling_rx(data, (uint8_t)l2len);
		return;
	default:
		break;
	}
drop:
	cidoo_ble_stats.l2cap_dropped++;
}
