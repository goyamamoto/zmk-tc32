/*
 * The security functions of the own BLE stack on tc32_aes128_encrypt()
 * (e): the legacy pairing functions c1 and s1, the Secure Connections
 * functions f4, f5 and f6 on AES-CMAC (Core Vol 3 Part H 2.2), the
 * session key and the link layer's AES-CCM (Core Vol 6 Part E).
 *
 * Byte order: the functions whose values travel in PDUs (keys, random
 * numbers, SKD, public key coordinates, the DHKey) take and give them as
 * on air, least significant octet first. tc32_aes128_encrypt(), ble_aes_cmac()
 * and the session key use the standard (FIPS-197, RFC 4493) orientation,
 * most significant octet first. tc32-devtools' emulator/
 * aes128.py computes the same functions from the specification's sample
 * data.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TLSR_BLE_CRYPTO_H_
#define TLSR_BLE_CRYPTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Legacy confirm value c1 (on-air order): preq and pres are the 7-octet
 * SMP Pairing Request and Response as sent, ia/ra the initiator's and
 * responder's addresses as sent, iat/rat their address types. */
void ble_c1(const uint8_t k[16], const uint8_t r[16], const uint8_t preq[7], const uint8_t pres[7],
	    uint8_t iat, const uint8_t ia[6], uint8_t rat, const uint8_t ra[6], uint8_t out[16]);

/* Legacy key generation s1 (on-air order): STK from the responder's r1 and
 * the initiator's r2. */
void ble_s1(const uint8_t k[16], const uint8_t r1[16], const uint8_t r2[16], uint8_t out[16]);

/* The random address hash ah (Core Vol 3 Part H 2.2.2), on-air order: the
 * IRK, the resolvable address's prand (its octets 3-5) and its hash (octets
 * 0-2). */
void ble_ah(const uint8_t k[16], const uint8_t prand[3], uint8_t hash[3]);

/* AES-CMAC (RFC 4493) in the standard orientation: the key and the
 * message as written, the MAC most significant octet first. */
void ble_aes_cmac(const uint8_t k[16], const uint8_t *m, size_t len, uint8_t mac[16]);

/* f4 (Core Vol 3 Part H 2.2.6), on-air order: U and V the x coordinates as
 * the Pairing Public Key PDUs carry them, X the 16-octet random as the
 * Pairing Random PDU carries it, Z one octet (0, or 0x80 | a passkey bit);
 * the confirm value as the Pairing Confirm PDU carries it. */
void ble_f4(const uint8_t u[32], const uint8_t v[32], const uint8_t x[16], uint8_t z,
	    uint8_t out[16]);

/* f5 (2.2.7), on-air order: W the DHKey (an x coordinate), N1 and N2 the
 * initiator's and the responder's randoms, A1 and A2 their addresses (the
 * type, 0 public or 1 random, and the 6 octets as on air); MacKey and LTK
 * as the later PDUs and the LL_ENC_REQ use them. */
void ble_f5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16], uint8_t at1,
	    const uint8_t a1[6], uint8_t at2, const uint8_t a2[6], uint8_t mackey[16],
	    uint8_t ltk[16]);

/* f6 (2.2.8), on-air order: W the MacKey, N1 and N2 the randoms, R the
 * passkey (or 0), iocap the three octets AuthReq, OOB data flag and IO
 * capability of the side being checked (octets 3, 2 and 1 of its Pairing
 * Request or Response), A1 and A2 as for f5 with that side's address
 * first; the DHKey Check value as its PDU carries it. */
void ble_f6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16], const uint8_t r[16],
	    const uint8_t iocap[3], uint8_t at1, const uint8_t a1[6], uint8_t at2,
	    const uint8_t a2[6], uint8_t out[16]);

/* The session key SK = e(LTK, SKDs || SKDm) (Core Vol 6 Part E 1): LTK,
 * SKDm and SKDs on-air order, SK in standard orientation (the CCM key). */
void ble_session_key(const uint8_t ltk[16], const uint8_t skdm[8], const uint8_t skds[8],
		     uint8_t sk[16]);

/*
 * The link layer's AES-CCM for one data PDU: nonce = packet counter (39
 * bits, least significant octet first) with the direction bit (1: central
 * to peripheral) in bit 7 of octet 4, then IVm || IVs as on air; AAD = the
 * header's first octet & 0xe3; a 4-octet MIC.
 * ble_ccm_encrypt encrypts len octets in place and writes the MIC after
 * them; ble_ccm_decrypt decrypts len - 4 octets in place and checks the
 * MIC in the last 4.
 */
void ble_ccm_encrypt(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		     uint8_t hdr0, uint8_t *data, uint8_t len);
bool ble_ccm_decrypt(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		     uint8_t hdr0, uint8_t *data, uint8_t len);

#endif /* TLSR_BLE_CRYPTO_H_ */
