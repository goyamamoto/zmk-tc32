/*
 * The own BLE stack's security functions (ble_crypto.h) on
 * tc32_aes128_encrypt(), the image's one AES-128 (zephyr-tc32's
 * TC32_AES128: the TLSR8278's AES block, each block with interrupts off,
 * shared with tc32_rng).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/crypto/tc32_aes.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_crypto.h"

static void reverse(uint8_t *dst, const uint8_t *src, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		dst[i] = src[len - 1U - i];
	}
}

/* e with on-air (least significant first) key, input and output. */
static void e_lso(const uint8_t k[16], const uint8_t in[16], uint8_t out[16])
{
	uint8_t kb[16], ib[16], ob[16];

	reverse(kb, k, 16);
	reverse(ib, in, 16);
	tc32_aes128_encrypt(kb, ib, ob);
	reverse(out, ob, 16);
}

void ble_c1(const uint8_t k[16], const uint8_t r[16], const uint8_t preq[7], const uint8_t pres[7],
	    uint8_t iat, const uint8_t ia[6], uint8_t rat, const uint8_t ra[6], uint8_t out[16])
{
	/* p1 = pres || preq || rat' || iat', p2 = padding || ia || ra, least
	 * significant octet first */
	uint8_t p[16];

	p[0] = iat;
	p[1] = rat;
	memcpy(&p[2], preq, 7);
	memcpy(&p[9], pres, 7);
	for (int i = 0; i < 16; i++) {
		p[i] ^= r[i];
	}
	e_lso(k, p, p);
	for (int i = 0; i < 6; i++) {
		p[i] ^= ra[i];
		p[6 + i] ^= ia[i];
	}
	e_lso(k, p, out);
}

void ble_s1(const uint8_t k[16], const uint8_t r1[16], const uint8_t r2[16], uint8_t out[16])
{
	/* r' = r1' || r2', the low 64 bits of each */
	uint8_t p[16];

	memcpy(&p[0], r2, 8);
	memcpy(&p[8], r1, 8);
	e_lso(k, p, out);
}

void ble_ah(const uint8_t k[16], const uint8_t prand[3], uint8_t hash[3])
{
	/* r' = padding || r: the 24-bit r in the low octets, the rest 0 */
	uint8_t p[16] = {0};
	uint8_t out[16];

	memcpy(p, prand, 3);
	e_lso(k, p, out);
	memcpy(hash, out, 3);
}

/* The subkey step of RFC 4493 2.3: k = k << 1, the top bit folded in as 0x87. */
static void cmac_shift(uint8_t k[16])
{
	uint8_t carry = 0;

	for (int i = 15; i >= 0; i--) {
		uint8_t next = k[i] >> 7;

		k[i] = (uint8_t)(k[i] << 1) | carry;
		carry = next;
	}
	k[15] ^= carry ? 0x87U : 0U;
}

void ble_aes_cmac(const uint8_t k[16], const uint8_t *m, size_t len, uint8_t mac[16])
{
	uint8_t sub[16] = {0}, x[16] = {0}, last[16] = {0};
	size_t blocks = (len + 15U) / 16U;
	bool whole = len > 0U && len % 16U == 0U;
	size_t tail;

	if (blocks == 0U) {
		blocks = 1U;
	}
	tc32_aes128_encrypt(k, sub, sub); /* L */
	cmac_shift(sub);                  /* K1 */
	if (!whole) {
		cmac_shift(sub);          /* K2 */
	}
	tail = len - (blocks - 1U) * 16U;
	memcpy(last, &m[(blocks - 1U) * 16U], tail);
	if (!whole) {
		last[tail] = 0x80U;
	}
	for (size_t b = 0; b + 1U < blocks; b++) {
		for (int i = 0; i < 16; i++) {
			x[i] ^= m[b * 16U + (size_t)i];
		}
		tc32_aes128_encrypt(k, x, x);
	}
	for (int i = 0; i < 16; i++) {
		x[i] ^= last[i] ^ sub[i];
	}
	tc32_aes128_encrypt(k, x, mac);
}

/* AES-CMAC with an on-air key and MAC over a standard-orientation message. */
static void cmac_lso(const uint8_t k[16], const uint8_t *m, size_t len, uint8_t out[16])
{
	uint8_t kb[16], ob[16];

	reverse(kb, k, 16);
	ble_aes_cmac(kb, m, len, ob);
	reverse(out, ob, 16);
}

/* A1 or A2 of f5 and f6: the address type, then the address most significant octet first. */
static void addr7(uint8_t *dst, uint8_t type, const uint8_t a[6])
{
	dst[0] = type;
	reverse(&dst[1], a, 6);
}

void ble_f4(const uint8_t u[32], const uint8_t v[32], const uint8_t x[16], uint8_t z,
	    uint8_t out[16])
{
	uint8_t m[65];

	reverse(&m[0], u, 32);
	reverse(&m[32], v, 32);
	m[64] = z;
	cmac_lso(x, m, sizeof(m), out);
}

void ble_f5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16], uint8_t at1,
	    const uint8_t a1[6], uint8_t at2, const uint8_t a2[6], uint8_t mackey[16],
	    uint8_t ltk[16])
{
	static const uint8_t salt[16] = {0x6c, 0x88, 0x83, 0x91, 0xaa, 0xf5, 0xa5, 0x38,
					 0x60, 0x37, 0x0b, 0xdb, 0x5a, 0x60, 0x83, 0xbe};
	uint8_t t[16], wb[32], m[53], ob[16];

	reverse(wb, w, 32);
	ble_aes_cmac(salt, wb, sizeof(wb), t);
	/* Counter || keyID "btle" || N1 || N2 || A1 || A2 || Length 256 */
	m[0] = 0U;
	memcpy(&m[1], "btle", 4);
	reverse(&m[5], n1, 16);
	reverse(&m[21], n2, 16);
	addr7(&m[37], at1, a1);
	addr7(&m[44], at2, a2);
	m[51] = 0x01U;
	m[52] = 0x00U;
	ble_aes_cmac(t, m, sizeof(m), ob);
	reverse(mackey, ob, 16);
	m[0] = 1U;
	ble_aes_cmac(t, m, sizeof(m), ob);
	reverse(ltk, ob, 16);
	memset(t, 0, sizeof(t));
}

void ble_f6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16], const uint8_t r[16],
	    const uint8_t iocap[3], uint8_t at1, const uint8_t a1[6], uint8_t at2,
	    const uint8_t a2[6], uint8_t out[16])
{
	uint8_t m[65];

	reverse(&m[0], n1, 16);
	reverse(&m[16], n2, 16);
	reverse(&m[32], r, 16);
	memcpy(&m[48], iocap, 3);
	addr7(&m[51], at1, a1);
	addr7(&m[58], at2, a2);
	cmac_lso(w, m, sizeof(m), out);
}

void ble_session_key(const uint8_t ltk[16], const uint8_t skdm[8], const uint8_t skds[8],
		     uint8_t sk[16])
{
	uint8_t kb[16], skd[16];

	reverse(kb, ltk, 16);
	reverse(&skd[0], skds, 8);
	reverse(&skd[8], skdm, 8);
	tc32_aes128_encrypt(kb, skd, sk);
}

static void ccm_block(uint8_t b[16], uint8_t flags, uint64_t counter, bool from_central,
		      const uint8_t iv[8], uint16_t tail)
{
	b[0] = flags;
	for (int i = 0; i < 4; i++) {
		b[1 + i] = (uint8_t)(counter >> (8 * i));
	}
	b[5] = (uint8_t)((counter >> 32) & 0x7fU) | (from_central ? 0x80U : 0U);
	memcpy(&b[6], iv, 8);
	sys_put_be16(tail, &b[14]);
}

/* The CBC-MAC over B0, the AAD block and the payload; T in x. */
static void ccm_mac(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		    uint8_t hdr0, const uint8_t *data, uint8_t len, uint8_t x[16])
{
	uint8_t b[16];

	ccm_block(b, 0x49U, counter, from_central, iv, len);
	tc32_aes128_encrypt(sk, b, x);
	memset(b, 0, sizeof(b));
	b[1] = 1U; /* AAD length 1 */
	b[2] = hdr0 & 0xe3U;
	for (int i = 0; i < 16; i++) {
		x[i] ^= b[i];
	}
	tc32_aes128_encrypt(sk, x, x);
	for (uint8_t off = 0; off < len; off += 16U) {
		for (uint8_t i = 0; i < 16U && off + i < len; i++) {
			x[i] ^= data[off + i];
		}
		tc32_aes128_encrypt(sk, x, x);
	}
}

/* data ^= the CCM key stream from counter block 1 on. */
static void ccm_ctr(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		    uint8_t *data, uint8_t len)
{
	uint8_t a[16], s[16];

	for (uint8_t off = 0, n = 1; off < len; off += 16U, n++) {
		ccm_block(a, 0x01U, counter, from_central, iv, n);
		tc32_aes128_encrypt(sk, a, s);
		for (uint8_t i = 0; i < 16U && off + i < len; i++) {
			data[off + i] ^= s[i];
		}
	}
}

static void ccm_s0(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		   uint8_t s0[16])
{
	uint8_t a[16];

	ccm_block(a, 0x01U, counter, from_central, iv, 0);
	tc32_aes128_encrypt(sk, a, s0);
}

void ble_ccm_encrypt(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		     uint8_t hdr0, uint8_t *data, uint8_t len)
{
	uint8_t x[16], s0[16];

	ccm_mac(sk, counter, from_central, iv, hdr0, data, len, x);
	ccm_s0(sk, counter, from_central, iv, s0);
	ccm_ctr(sk, counter, from_central, iv, data, len);
	for (int i = 0; i < 4; i++) {
		data[len + i] = x[i] ^ s0[i];
	}
}

bool ble_ccm_decrypt(const uint8_t sk[16], uint64_t counter, bool from_central, const uint8_t iv[8],
		     uint8_t hdr0, uint8_t *data, uint8_t len)
{
	uint8_t x[16], s0[16];
	uint8_t n;
	uint8_t diff = 0;

	if (len < 4U) {
		return false;
	}
	n = len - 4U;
	ccm_ctr(sk, counter, from_central, iv, data, n);
	ccm_mac(sk, counter, from_central, iv, hdr0, data, n, x);
	ccm_s0(sk, counter, from_central, iv, s0);
	for (int i = 0; i < 4; i++) {
		diff |= (uint8_t)(data[n + i] ^ x[i] ^ s0[i]);
	}
	return diff == 0U;
}
