/*
 * A plain software AES-128 encryption for the host test, in place of the
 * image's tc32_aes128_encrypt() (zephyr-tc32's TC32_AES128).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdint.h>
#include <string.h>

static uint8_t sbox[256];

static uint8_t xt(uint8_t a)
{
	return (uint8_t)((a << 1) ^ ((a & 0x80) ? 0x1b : 0));
}

static void init(void)
{
	uint8_t p = 1, q = 1;

	if (sbox[0]) {
		return;
	}
	do {
		p = p ^ (uint8_t)(p << 1) ^ ((p & 0x80) ? 0x1b : 0);
		q ^= q << 1;
		q ^= q << 2;
		q ^= q << 4;
		q ^= (q & 0x80) ? 0x09 : 0;
		uint8_t x = q ^ (uint8_t)(q << 1 | q >> 7) ^ (uint8_t)(q << 2 | q >> 6) ^
			    (uint8_t)(q << 3 | q >> 5) ^ (uint8_t)(q << 4 | q >> 4);
		sbox[p] = x ^ 0x63;
	} while (p != 1);
	sbox[0] = 0x63;
}

void tc32_aes128_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	uint8_t rk[176], s[16], rcon = 1;

	init();
	memcpy(rk, key, 16);
	for (int i = 16; i < 176; i += 4) {
		uint8_t t[4];

		memcpy(t, &rk[i - 4], 4);
		if (i % 16 == 0) {
			uint8_t u = t[0];

			t[0] = sbox[t[1]] ^ rcon;
			t[1] = sbox[t[2]];
			t[2] = sbox[t[3]];
			t[3] = sbox[u];
			rcon = xt(rcon);
		}
		for (int j = 0; j < 4; j++) {
			rk[i + j] = rk[i - 16 + j] ^ t[j];
		}
	}
	for (int i = 0; i < 16; i++) {
		s[i] = in[i] ^ rk[i];
	}
	for (int round = 1; round <= 10; round++) {
		uint8_t t[16];

		for (int c = 0; c < 4; c++) {
			for (int r = 0; r < 4; r++) {
				t[4 * c + r] = sbox[s[4 * ((c + r) % 4) + r]];
			}
		}
		if (round < 10) {
			for (int c = 0; c < 4; c++) {
				uint8_t *a = &t[4 * c], b0 = a[0], b1 = a[1], b2 = a[2], b3 = a[3];
				uint8_t all = b0 ^ b1 ^ b2 ^ b3;

				a[0] ^= all ^ xt(b0 ^ b1);
				a[1] ^= all ^ xt(b1 ^ b2);
				a[2] ^= all ^ xt(b2 ^ b3);
				a[3] ^= all ^ xt(b3 ^ b0);
			}
		}
		for (int i = 0; i < 16; i++) {
			s[i] = t[i] ^ rk[16 * round + i];
		}
	}
	memcpy(out, s, 16);
}
