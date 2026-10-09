/*
 * LL control PDUs from the central (Core Vol 6 Part B 2.4.2, 5.1), answered
 * in the BLE thread. The peripheral starts no procedure of its own yet.
 *
 * - Version exchange: Core 4.2 (VersNr 8), the version from which a host
 *   pairs by LE Secure Connections (Android sends a Pairing Request without
 *   SC to a peer of a lower one), with the feature set this link layer
 *   has; company ID 0xFFFF (none assigned), CONFIG_TLSR_BLE_LL_SUBVERSION.
 * - Feature exchange: LE Encryption (no ping, data length extension or
 *   other PHYs).
 * - Encryption start and pause: LL_ENC_REQ, LL_PAUSE_ENC_REQ and the
 *   central's LL_START_ENC_RSP and LL_PAUSE_ENC_RSP go to ble_link.c;
 *   LL_PAUSE_ENC_REQ on a link that is not encrypted gets LL_UNKNOWN_RSP.
 * - Connection update and channel map: applied at their instant by
 *   ble_conn.c; an instant already passed loses the connection.
 * - LL_TERMINATE_IND ends the connection once the radio has acknowledged it.
 * - Procedures this link layer does not have: LL_UNKNOWN_RSP.
 * - An answer goes into the TX ring's slots kept for answers (ble_link.c); one
 *   the ring refuses is not sent again, and the version exchange stays open
 *   for the central's next LL_VERSION_IND.
 * - cidoo_ble_stats: the control PDUs received and answered (ll_ctrl_rx,
 *   ll_ctrl_tx), the opcodes received and answered (bit masks), the last
 *   opcode received.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "ble_internal.h"

#define LL_CONNECTION_UPDATE_IND 0x00U
#define LL_CHANNEL_MAP_IND       0x01U
#define LL_TERMINATE_IND         0x02U
#define LL_ENC_REQ               0x03U
#define LL_START_ENC_RSP         0x06U
#define LL_UNKNOWN_RSP           0x07U
#define LL_FEATURE_REQ           0x08U
#define LL_FEATURE_RSP           0x09U
#define LL_PAUSE_ENC_REQ         0x0aU
#define LL_PAUSE_ENC_RSP         0x0bU
#define LL_VERSION_IND           0x0cU
#define LL_REJECT_IND            0x0dU
#define LL_REJECT_EXT_IND        0x11U
#define LL_PING_REQ              0x12U
#define LL_PING_RSP              0x13U

#define LL_VERSION_4_2 0x08U
#define LL_COMPANY_ID  0xffffU

static const uint8_t features[8] = {0x01}; /* LE Encryption */
static bool version_sent;

static int send(const uint8_t *pdu, uint8_t len)
{
	int err = ble_link_tx(LLID_CONTROL, pdu, len);

	if (err == 0) {
		cidoo_ble_stats.ll_ctrl_tx++;
	}
	return err;
}

void ble_ll_ctrl_reset(void)
{
	version_sent = false;
}

void ble_ll_ctrl_rx(const uint8_t *p, uint8_t len, uint16_t event)
{
	uint32_t sent = cidoo_ble_stats.ll_ctrl_tx;

	cidoo_ble_stats.ll_ctrl_rx++;
	if (len == 0U) {
		return;
	}
	cidoo_ble_stats.ll_ctrl_last = p[0];
	cidoo_ble_stats.ll_ctrl_seen |= BIT(p[0] & 0x1fU);

	switch (p[0]) {
	case LL_CONNECTION_UPDATE_IND:
		if (len == 12U) {
			ble_conn_update(p[1], sys_get_le16(&p[2]), sys_get_le16(&p[4]),
					sys_get_le16(&p[6]), sys_get_le16(&p[8]), sys_get_le16(&p[10]),
					event);
		}
		break;
	case LL_CHANNEL_MAP_IND:
		if (len == 8U) {
			ble_conn_channel_map(&p[1], sys_get_le16(&p[6]), event);
		}
		break;
	case LL_TERMINATE_IND:
		ble_conn_terminate(len >= 2U ? p[1] : BLE_ERR_REMOTE_TERMINATED);
		break;
	case LL_FEATURE_REQ: {
		uint8_t rsp[9] = {LL_FEATURE_RSP};

		for (size_t i = 0; i < sizeof(features); i++) {
			rsp[1 + i] = features[i];
		}
		/* Octet 0 holds the features both sides have (Core Vol 6 Part B
		 * 2.4.2.10). */
		if (len >= 2U) {
			rsp[1] &= p[1];
		}
		send(rsp, sizeof(rsp));
		break;
	}
	case LL_VERSION_IND:
		if (!version_sent) {
			uint8_t ind[6] = {LL_VERSION_IND, LL_VERSION_4_2};

			sys_put_le16(LL_COMPANY_ID, &ind[2]);
			sys_put_le16(CONFIG_TLSR_BLE_LL_SUBVERSION, &ind[4]);
			version_sent = send(ind, sizeof(ind)) == 0;
		}
		break;
	case LL_PING_REQ: {
		uint8_t rsp[1] = {LL_PING_RSP};

		send(rsp, sizeof(rsp));
		break;
	}
	case LL_ENC_REQ:
		ble_link_enc_req(p, len);
		break;
	case LL_START_ENC_RSP:
		ble_link_start_enc_rsp();
		break;
	case LL_PAUSE_ENC_RSP:
		ble_link_pause_enc_rsp();
		break;
	case LL_UNKNOWN_RSP:
	case LL_REJECT_IND:
	case LL_REJECT_EXT_IND:
	case LL_PING_RSP:
		/* answers to procedures this side never starts */
		break;
	case LL_PAUSE_ENC_REQ:
		if (ble_link_pause_enc_req()) {
			break;
		}
		__fallthrough;
	default: {
		uint8_t rsp[2] = {LL_UNKNOWN_RSP, p[0]};

		send(rsp, sizeof(rsp));
		break;
	}
	}
	if (cidoo_ble_stats.ll_ctrl_tx != sent) {
		cidoo_ble_stats.ll_ctrl_answered |= BIT(p[0] & 0x1fU);
	}
}
