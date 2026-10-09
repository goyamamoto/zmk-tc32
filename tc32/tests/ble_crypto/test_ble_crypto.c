/*
 * Host test of src/ble/ble_crypto.c against the Bluetooth Core
 * specification's sample data (the same as tc32-devtools emulator/
 * aes128.py): AES-128 (FIPS-197 C.1), c1, s1 and ah (Vol 3 Part H 2.2.3/2.2.4/2.2.2),
 * the session key and one LL data PDU (Vol 6 Part C 1), AES-CMAC, f4, f5 and
 * f6 (Appendix D, and a pairing Apache NimBLE's tests recorded). The AES block is
 * replaced by a software AES; everything else is the file as built.
 * Run: sh run.sh
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/crypto/tc32_aes.h>

#include "ble_crypto.h"

static int failures;

static void check(int ok, const char *what)
{
	printf("%s %s\n", ok ? "ok  " : "FAIL", what);
	failures += !ok;
}

static void hex(const char *s, uint8_t *out, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		sscanf(s + 2 * i, "%2hhx", &out[i]);
	}
}

/* an integer's hex (most significant first) as on-air bytes (least first) */
static void hex_lso(const char *s, uint8_t *out, size_t n)
{
	uint8_t b[32];

	hex(s, b, n);
	for (size_t i = 0; i < n; i++) {
		out[i] = b[n - 1 - i];
	}
}

int main(void)
{
	uint8_t k[16], p[16], o[16], want[16];

	hex("000102030405060708090a0b0c0d0e0f", k, 16);
	hex("00112233445566778899aabbccddeeff", p, 16);
	tc32_aes128_encrypt(k, p, o);
	hex("69c4e0d86a7b0430d8cdb78070b4c55a", want, 16);
	check(memcmp(o, want, 16) == 0, "AES-128 FIPS-197 C.1");

	uint8_t zero[16] = {0}, r[16], preq[7], pres[7], ia[6], ra[6];

	hex_lso("5783D52156AD6F0E6388274EC6702EE0", r, 16);
	hex("01010000100707", preq, 7);
	hex("02030000080005", pres, 7);
	hex("a6a5a4a3a2a1", ia, 6);
	hex("b6b5b4b3b2b1", ra, 6);
	ble_c1(zero, r, preq, pres, 1, ia, 0, ra, o);
	hex_lso("1E1E3FEF878988EAD2A74DC5BEF13B86", want, 16);
	check(memcmp(o, want, 16) == 0, "c1 sample (Vol 3 Part H 2.2.3)");

	uint8_t r1[16], r2[16];

	hex_lso("000F0E0D0C0B0A091122334455667788", r1, 16);
	hex_lso("010203040506070899AABBCCDDEEFF00", r2, 16);
	ble_s1(zero, r1, r2, o);
	hex_lso("9A1FE1F0E8B0F49B5B4216AE796DA062", want, 16);
	check(memcmp(o, want, 16) == 0, "s1 sample (Vol 3 Part H 2.2.4)");

	/* Vol 3 Part H 2.2.2 (Appendix D): IRK ec0234a357c8ad05341010a60a397d9b, prand 0x708194,
	 * hash 0x0dfbaa; on air the address is hash then prand, least significant octet first */
	uint8_t irk[16], prand[3] = {0x94, 0x81, 0x70}, hash[3];

	hex_lso("EC0234A357C8AD05341010A60A397D9B", irk, 16);
	ble_ah(irk, prand, hash);
	check(hash[0] == 0xaa && hash[1] == 0xfb && hash[2] == 0x0d, "ah sample (Vol 3 Part H 2.2.2)");

	/* Vol 6 Part C 1: SKD = SKDs || SKDm = 0x0213243546576879 || 0xACBDCEDFE0F10213 */
	uint8_t ltk[16], skdm[8], skds[8], sk[16], iv[8];

	hex_lso("4C68384139F574D836BCF34E9DFB01BF", ltk, 16);
	hex_lso("ACBDCEDFE0F10213", skdm, 8);
	hex_lso("0213243546576879", skds, 8);
	ble_session_key(ltk, skdm, skds, sk);
	hex("99AD1B5226A37E3E058E3B8E27C2C666", want, 16);
	check(memcmp(sk, want, 16) == 0, "session key (Vol 6 Part C 1)");

	hex("24abdcbabebaafde", iv, 8); /* IVm || IVs as on air */
	uint8_t pdu[8] = {0x06};
	uint8_t onair[5];

	hex("9fcda7f448", onair, 5);
	ble_ccm_encrypt(sk, 0, true, iv, 0x0f, pdu, 1);
	check(memcmp(pdu, onair, 5) == 0, "CCM encrypt: LL_START_ENC_RSP, counter 0, central to peripheral");
	memcpy(pdu, onair, 5);
	check(ble_ccm_decrypt(sk, 0, true, iv, 0x0f, pdu, 5) && pdu[0] == 0x06, "CCM decrypt and MIC");
	memcpy(pdu, onair, 5);
	pdu[4] ^= 1;
	check(!ble_ccm_decrypt(sk, 0, true, iv, 0x0f, pdu, 5), "CCM: a MIC with one bit flipped fails");
	memcpy(pdu, onair, 5);
	check(!ble_ccm_decrypt(sk, 1, true, iv, 0x0f, pdu, 5), "CCM: the wrong packet counter fails");

	/* a 27-octet payload round trip (two CCM blocks) */
	uint8_t big[31], copy[27];

	for (int i = 0; i < 27; i++) {
		big[i] = copy[i] = (uint8_t)(i * 7 + 1);
	}
	ble_ccm_encrypt(sk, 0x123456789aULL, false, iv, 0x02, big, 27);
	check(memcmp(big, copy, 27) != 0 && ble_ccm_decrypt(sk, 0x123456789aULL, false, iv, 0x02, big, 31) &&
		      memcmp(big, copy, 27) == 0,
	      "CCM: 27 octets round trip, peripheral to central, counter over 32 bits");

	/* AES-CMAC, RFC 4493 (Core Vol 3 Part H Appendix D.1): K and M as written */
	uint8_t ck[16], cm[64];

	hex("2b7e151628aed2a6abf7158809cf4f3c", ck, 16);
	hex("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710",
	    cm, 64);
	ble_aes_cmac(ck, cm, 0, o);
	hex("bb1d6929e95937287fa37d129b756746", want, 16);
	check(memcmp(o, want, 16) == 0, "AES-CMAC of 0 octets (RFC 4493 example 1)");
	ble_aes_cmac(ck, cm, 16, o);
	hex("070a16b46b4d4144f79bdd9dd04a287c", want, 16);
	check(memcmp(o, want, 16) == 0, "AES-CMAC of 16 octets (example 2)");
	ble_aes_cmac(ck, cm, 40, o);
	hex("dfa66747de9ae63030ca32611497c827", want, 16);
	check(memcmp(o, want, 16) == 0, "AES-CMAC of 40 octets (example 3)");
	ble_aes_cmac(ck, cm, 64, o);
	hex("51f0bebf7e3b9d92fc49741779363cfe", want, 16);
	check(memcmp(o, want, 16) == 0, "AES-CMAC of 64 octets (example 4)");

	/* f4, f5, f6: Appendix D.2-D.4, on-air order (the values as Zephyr's smp.c self-tests hold them) */
	uint8_t u[32], v[32], x[16], n2[16], w[32], a1[6], a2[6], mackey[16], ltk2[16], rr[16];

	hex("e69d350e480103ccdbfdf4ac1191f4efb9a5f9e9a7832c5e2cbe97f2d203b020", u, 32);
	hex("fdc57ff449dd4f6bfb7c9df1c29acb592ae7d4eefbfc0a909abbf6323d8b1855", v, 32);
	hex("abae2b71ecb2ffff3e7377d15484cbd5", x, 16);
	ble_f4(u, v, x, 0, o);
	hex("2d8774a9bea1edf11cbda907f116c9f2", want, 16);
	check(memcmp(o, want, 16) == 0, "f4 (Appendix D.2)");

	hex("98a6bf73f3348d86f166f8b4136b79999b7d390aa610103405adc857a33402ec", w, 32);
	hex("cfc43dfff78365216e5fa725cce7e8a6", n2, 16);
	hex("cebf37371256", a1, 6);
	hex("c1cf2d7013a7", a2, 6);
	ble_f5(w, x, n2, 0, a1, 0, a2, mackey, ltk2);
	hex("206e63ce206a3ffd024a08a176f16529", want, 16);
	check(memcmp(mackey, want, 16) == 0, "f5 MacKey (Appendix D.3)");
	hex("380a7594b522059823cdd76911798669", want, 16);
	check(memcmp(ltk2, want, 16) == 0, "f5 LTK (Appendix D.3)");

	hex("c80f2d0cd242da0854bb53b43b34a312", rr, 16);
	uint8_t iocap[3] = {0x01, 0x01, 0x02}; /* AuthReq, OOB flag, IO capability */

	ble_f6(mackey, x, n2, rr, iocap, 0, a1, 0, a2, o);
	hex("618f95da090b6cd2c5e8d09c9873c4e3", want, 16);
	check(memcmp(o, want, 16) == 0, "f6 (Appendix D.4)");

	/* The same functions on a recorded Secure Connections Just Works pairing (Apache NimBLE's
	 * ble_sm_sc_peer_jw_iio3_rio3_b1_iat0_rat0_ik5_rk7): the responder's confirm, the LTK and
	 * both DHKey checks must come out as the recording has them. */
	uint8_t pkax[32], pkbx[32], na[16], nb[16], dh[32], ia2[6], ra2[6], zero16[16] = {0};

	hex("bcf2d8a5dba3956c99f9110d4d2ef0bdee9b69b6cd8874be40e8e5ccdc884453", pkax, 32);
	hex("728cd188d7be49b2c55c95b364e01232b6c9476337385b9c1e1b1a0609e23185", pkbx, 32);
	hex("a4345fb3af734364cd191b5b87583166", na, 16);
	hex("c091fbb377a2020bc6cd6c0451454539", nb, 16);
	hex("75f59efd2838b8ed9ed55ca11c43645e6472780a35beba5a37828f2a3aa86b5d", dh, 32);
	hex("ca61a06794e0", ia2, 6);
	hex("33221100450a", ra2, 6);
	ble_f4(pkbx, pkax, nb, 0, o);
	hex("82edd062913d967f13c50d022b5e4316", want, 16);
	check(memcmp(o, want, 16) == 0, "f4: the recorded responder's confirm");
	ble_f5(dh, na, nb, 0, ia2, 0, ra2, mackey, ltk2);
	hex("63598a14094b946effae5e538602a36c", want, 16);
	check(memcmp(ltk2, want, 16) == 0, "f5: the recorded LTK");
	uint8_t iocap_jw[3] = {0x09, 0x00, 0x03}; /* both sides: bonding + SC, no OOB, NoInputNoOutput */

	ble_f6(mackey, na, nb, zero16, iocap_jw, 0, ia2, 0, ra2, o);
	hex("82651d02ed891344041a147c329a1e7d", want, 16);
	check(memcmp(o, want, 16) == 0, "f6: the recorded initiator's DHKey check");
	ble_f6(mackey, nb, na, zero16, iocap_jw, 0, ra2, 0, ia2, o);
	hex("063c284ae5484b51654e145e2fddfa22", want, 16);
	check(memcmp(o, want, 16) == 0, "f6: the recorded responder's DHKey check");

	printf("%d failure(s)\n", failures);
	return failures != 0;
}
