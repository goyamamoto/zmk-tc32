/*
 * SMP for the own BLE stack (Core Vol 3 Part H), in the BLE thread: LE
 * Secure Connections pairing as the responder, with Passkey Entry
 * (TLSR_BLE_SC_PASSKEY: the keyboard types the passkey the host shows,
 * MITM protection; a central that cannot show one is refused with 0x03) or
 * Just Works (NoInputNoOutput; a central's MITM bit changes nothing, as
 * the specification's method table has it), bonding when the central asks
 * for it, no OOB, 7 to 16 octet keys. Legacy pairing is refused (Pairing
 * Failed 0x03, authentication requirements).
 *
 * - Public keys (2.3.5.6.1): the central's is checked to be on the curve
 *   (0x0b when not) and not this side's own; this side's key pair is new
 *   for every pairing, its private key from tc32_rng_get_key_material()
 *   (no key material: 0x08, unspecified reason). Every P-256 operation
 *   runs through tc32_p256.h, started and polled, so the thread goes on
 *   serving the link while the engine works; a pairing that ends while one
 *   runs (Pairing Failed sent or received, the timeout, the link's end)
 *   gives it up, so that the next pairing finds the engine free.
 * - Just Works (2.3.5.6.2): Nb from tc32_rng, Cb = f4(PKbx, PKax, Nb, 0),
 *   then the randoms. Passkey Entry (2.3.5.6.3): 20 rounds, each with a
 *   new Nb and ri = 0x80 | bit i of the passkey; the central's Cai is
 *   checked against its Nai (0x04 on a mismatch). The passkey is typed on
 *   the keyboard (ble_passkey.c) from the Pairing Response on; the central's
 *   first confirm waits until it is in.
 * - DHKey check (2.3.5.6.5): MacKey and LTK from f5 over the DHKey and the
 *   last round's randoms; Ea checked (0x0b), Eb sent; the LTK, masked to
 *   the negotiated key size, goes to the link for the LL_ENC_REQ with
 *   EDIV 0 and Rand 0 that follows.
 * - Keys (3.6.1): no LTK is distributed with Secure Connections, and this
 *   side distributes no key at all (it never uses a resolvable address, so
 *   an IRK of its own would tell the central nothing); once encrypted it
 *   takes the central's IRK and identity address when the central offered
 *   them. The pairing ends, and the bond is stored (ble_bond.c) with the
 *   LTK and whether it is authenticated, when every expected key is in.
 * - A pairing in which no SMP PDU arrives for 30 s is abandoned (3.4) and
 *   no SMP is answered until the link ends. A PDU the state does not expect
 *   ends the pairing with 0x08.
 * - Repeated attempts (2.3.6): after a pairing this side ended with Pairing
 *   Failed, or abandoned, a Pairing Request gets 0x09 until a wait has
 *   passed, whatever it asks for (it is not looked at; 0x09 is no new
 *   failure); the wait doubles with every such failure, from 2 s up to
 *   64 s, and a pairing that ends well clears it. It outlives the link.
 * - Security Request (3.6.7, 2.4.6): once per link, to a host of a profile
 *   with a bond (the only one that can connect to it, ble_adv.c) that has
 *   not sent LL_ENC_REQ SEC_REQ_DELAY_MS after the link started; AuthReq
 *   Bonding and SC, and MITM with Passkey Entry. Not while a pairing
 *   is under way, after the SMP timeout, or within the repeated-attempts
 *   wait (it waits for its end). A host that keeps a key for the keyboard
 *   encrypts with it; one that has none pairs. Sending it starts the 30 s
 *   timer (3.4); the central's LL_ENC_REQ or any SMP PDU from it stops
 *   that timer, a Pairing Request starting the pairing's own. When it runs
 *   out, no SMP is answered until the link ends.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/crypto/tc32_p256.h>
#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_crypto.h"
#include "ble_internal.h"

#define SMP_CID 0x0006U

#define SMP_PAIRING_REQ     0x01U
#define SMP_PAIRING_RSP     0x02U
#define SMP_PAIRING_CONFIRM 0x03U
#define SMP_PAIRING_RANDOM  0x04U
#define SMP_PAIRING_FAILED  0x05U
#define SMP_ENC_INFO        0x06U
#define SMP_CENTRAL_IDENT   0x07U
#define SMP_IDENT_INFO      0x08U
#define SMP_IDENT_ADDR_INFO 0x09U
#define SMP_SIGNING_INFO    0x0aU
#define SMP_SECURITY_REQ    0x0bU
#define SMP_PUBLIC_KEY      0x0cU
#define SMP_DHKEY_CHECK     0x0dU
#define SMP_KEYPRESS        0x0eU

#define SMP_ERR_PASSKEY_FAILED  0x01U
#define SMP_ERR_AUTH_REQ        0x03U
#define SMP_ERR_CONFIRM_FAILED  0x04U
#define SMP_ERR_ENC_KEY_SIZE    0x06U
#define SMP_ERR_CMD_UNSUPPORTED 0x07U
#define SMP_ERR_UNSPECIFIED     0x08U
#define SMP_ERR_REPEATED        0x09U
#define SMP_ERR_DHKEY_CHECK     0x0bU

#define IO_DISPLAY_ONLY       0x00U
#define IO_KEYBOARD_ONLY      0x02U
#define IO_NO_INPUT_NO_OUTPUT 0x03U
#define IO_KEYBOARD_DISPLAY   0x04U
#define AUTH_BONDING          0x01U
#define AUTH_MITM             0x04U
#define AUTH_SC               0x08U
#define KEY_ID                BIT(1)

/* The association model of this build: Passkey Entry (the keyboard types) or Just Works. */
#define PASSKEY IS_ENABLED(CONFIG_TLSR_BLE_SC_PASSKEY)

#define SMP_TIMEOUT_MS 30000U
#define SEC_REQ_DELAY_MS 1000U
#define ATTEMPT_WAIT_FIRST_MS 2000U
#define ATTEMPT_WAIT_MAX_MS   64000U
#define PASSKEY_ROUNDS 20U
#define PASSKEY_MAX    999999U

enum smp_state {
	SMP_IDLE,
	SMP_WAIT_PUBLIC_KEY,  /* the response sent */
	SMP_VERIFY,           /* the central's public key being checked on the curve */
	SMP_KEYGEN,           /* this side's key pair computing, or its public key not yet queued */
	SMP_WAIT_RANDOM,      /* Just Works: the confirm sent */
	SMP_PASSKEY_CONFIRM,  /* a round: the central's confirm, and the passkey, awaited */
	SMP_PASSKEY_RANDOM,   /* a round: this side's confirm sent */
	SMP_WAIT_DHKEY_CHECK,
	SMP_WAIT_ENC,
	SMP_WAIT_KEYS,
};

static struct {
	enum smp_state state;
	bool locked; /* after a timeout: nothing answered until the link ends */
	bool sec_req; /* the Security Request sent and not yet answered: the timer runs */
	uint32_t t_last;
	uint8_t preq[7], pres[7];
	uint8_t key_size;
	/* The central's and this side's public keys, x then y. Once the DHKey has started neither y is
	 * needed again: the central's y holds Ea, this side's holds the round's Ca. */
	uint8_t pka[64], pkb[64];
	/* The private key until the DHKey starts, then the DHKey. */
	uint8_t priv[32];
	bool have_pkb;
	uint8_t na[16], nb[16];
	bool have_ca;
	uint8_t round;
	uint32_t passkey;
	bool have_passkey;
	bool have_ea;
	enum { DH_NONE, DH_RUNNING, DH_DONE } dh;
	bool p256; /* a tc32_p256.h operation started and its result not yet taken */
	uint8_t init_keys; /* still expected from the central */
	struct ble_bond bond;
} s;

#define s_ea    (&s.pka[32])
#define s_ca    (&s.pkb[32])
#define s_dhkey s.priv

/* The Security Request of this link: due at sec_req_at while sec_req_due. */
static bool sec_req_due;
static uint32_t sec_req_at;

/* Repeated attempts: the wait after a failed pairing, kept across links. */
static struct {
	bool waiting;
	uint32_t wait_ms;
	uint32_t until;
} attempts;

/* The passkey being typed (ble_passkey.c, another thread): digits and the end, under irq_lock. */
static struct {
	volatile bool wanted;
	volatile bool done;
	volatile bool given_up;
	uint8_t digits[6];
	uint8_t n;
} pk;

static void send(const uint8_t *pdu, uint8_t len)
{
	(void)ble_l2cap_send(SMP_CID, pdu, len);
}

/* The P-256 operation under way, if any, given up: the engine is free for the next start. */
static void p256_cancel(void)
{
	if (s.p256) {
		(void)tc32_p256_cancel();
		s.p256 = false;
	}
}

static int p256_poll(uint8_t *qx, uint8_t *qy)
{
	int rc = tc32_p256_poll(qx, qy);

	if (rc != -EAGAIN) {
		s.p256 = false;
	}
	return rc;
}

static void clear(void)
{
	bool locked = s.locked;

	p256_cancel();
	memset(&s, 0, sizeof(s));
	s.locked = locked;
	memset(&pk, 0, sizeof(pk));
}

static void refuse(uint8_t reason)
{
	uint8_t pdu[2] = {SMP_PAIRING_FAILED, reason};

	send(pdu, sizeof(pdu));
	cidoo_ble_stats.smp_failures++;
}

/* A failed attempt: the wait before the next one starts, or doubles. */
static void attempt_failed(void)
{
	attempts.wait_ms = attempts.waiting ? MIN(attempts.wait_ms * 2U, ATTEMPT_WAIT_MAX_MS)
					   : ATTEMPT_WAIT_FIRST_MS;
	attempts.until = k_uptime_get_32() + attempts.wait_ms;
	attempts.waiting = true;
}

static void fail(uint8_t reason)
{
	refuse(reason);
	attempt_failed();
	clear();
}

/* The PDU the state waits for, of that length: else the pairing ends with 0x08. */
static bool expected(enum smp_state state, uint8_t len, uint8_t want)
{
	if (s.state == state && len == want) {
		return true;
	}
	fail(SMP_ERR_UNSPECIFIED);
	return false;
}

void ble_smp_reset(void)
{
	p256_cancel();
	memset(&s, 0, sizeof(s));
	memset(&pk, 0, sizeof(pk));
	sec_req_due = true;
	sec_req_at = k_uptime_get_32() + SEC_REQ_DELAY_MS;
}

/* The Security Request, when it is due (the header's rules). */
static void security_request(void)
{
	uint8_t pdu[2] = {SMP_SECURITY_REQ, AUTH_BONDING | AUTH_SC | (PASSKEY ? AUTH_MITM : 0U)};
	uint32_t now = k_uptime_get_32();

	if (!sec_req_due || s.state != SMP_IDLE || s.locked || (int32_t)(now - sec_req_at) < 0 ||
	    (attempts.waiting && (int32_t)(now - attempts.until) < 0)) {
		return;
	}
	if (ble_link_enc_requested() || !ble_bond_valid(ble_profile_active())) {
		sec_req_due = false;
	} else if (ble_l2cap_send(SMP_CID, pdu, sizeof(pdu)) == 0) {
		sec_req_due = false;
		s.sec_req = true;
		s.t_last = now;
	}
}

static void pairing_req(const uint8_t *p, uint8_t len)
{
	bool bonding;

	if (s.state != SMP_IDLE) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	if (attempts.waiting && (int32_t)(k_uptime_get_32() - attempts.until) < 0) {
		refuse(SMP_ERR_REPEATED);
		return;
	}
	if (len != 7U) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	if (p[4] < 7U || p[4] > 16U) {
		fail(SMP_ERR_ENC_KEY_SIZE);
		return;
	}
	if ((p[3] & AUTH_SC) == 0U) {
		fail(SMP_ERR_AUTH_REQ); /* legacy pairing */
		return;
	}
	/* Core 2.3.5.1 Table 2.8 with this side KeyboardOnly: a central that cannot show or take a
	 * passkey leaves Just Works, which this side's MITM requirement refuses. */
	if (PASSKEY && (p[1] == IO_NO_INPUT_NO_OUTPUT || p[1] > IO_KEYBOARD_DISPLAY)) {
		fail(SMP_ERR_AUTH_REQ);
		return;
	}
	bonding = (p[3] & 0x03U) == AUTH_BONDING;
	memcpy(s.preq, p, 7);
	s.pres[0] = SMP_PAIRING_RSP;
	s.pres[1] = PASSKEY ? IO_KEYBOARD_ONLY : IO_NO_INPUT_NO_OUTPUT;
	s.pres[2] = 0x00U; /* no OOB data */
	s.pres[3] = (bonding ? AUTH_BONDING : 0x00U) | AUTH_SC | (PASSKEY ? AUTH_MITM : 0x00U);
	s.pres[4] = 16U;
	s.pres[5] = bonding ? (p[5] & KEY_ID) : 0U; /* the central's IRK and identity address, no LTK */
	s.pres[6] = 0U;                              /* nothing from this side */
	s.key_size = p[4];
	send(s.pres, sizeof(s.pres));
	s.state = SMP_WAIT_PUBLIC_KEY;
	pk.wanted = PASSKEY; /* the host shows the passkey from here on; typing may start at once */
}

static bool new_private_key(void)
{
	for (int i = 0; i < 4; i++) {
		if (tc32_rng_get_key_material(s.priv, sizeof(s.priv)) != 0) {
			return false;
		}
		if (tc32_p256_scalar_ok(s.priv)) {
			return true;
		}
	}
	return false;
}

static void public_key(const uint8_t *p, uint8_t len)
{
	if (!expected(SMP_WAIT_PUBLIC_KEY, len, 65U)) {
		return;
	}
	memcpy(s.pka, &p[1], 64);
	tc32_rng_add_sample(sys_get_le32(&s.pka[0]), TC32_RNG_SRC_PEER_RANDOM);
	if (tc32_p256_verify_start(&s.pka[0], &s.pka[32]) != 0) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	s.p256 = true;
	s.state = SMP_VERIFY;
}

/* The central's key is on the curve: this side's key pair starts. */
static void verified(void)
{
	if (!new_private_key() || tc32_p256_mul_start(s.priv, NULL, NULL) != 0) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	s.p256 = true;
	s.state = SMP_KEYGEN;
}

static void confirm_round(void)
{
	uint8_t pdu[17] = {SMP_PAIRING_CONFIRM};
	uint8_t z = 0U;

	if (tc32_rng_get_key_material(s.nb, sizeof(s.nb)) != 0) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	if (PASSKEY) {
		z = 0x80U | (uint8_t)((s.passkey >> s.round) & 1U);
	}
	ble_f4(&s.pkb[0], &s.pka[0], s.nb, z, &pdu[1]);
	send(pdu, sizeof(pdu));
	s.state = PASSKEY ? SMP_PASSKEY_RANDOM : SMP_WAIT_RANDOM;
}

/* The key pair is in: the public key goes out, the DHKey starts, the model's first step follows. */
static void keygen_done(void)
{
	uint8_t pdu[65] = {SMP_PUBLIC_KEY};

	if (memcmp(&s.pka[0], &s.pkb[0], 32) == 0) {
		fail(SMP_ERR_UNSPECIFIED); /* the central echoed this side's key */
		return;
	}
	memcpy(&pdu[1], s.pkb, 64);
	if (ble_l2cap_send(SMP_CID, pdu, sizeof(pdu)) != 0) {
		return; /* the ring is full, or an encryption start is under way: the next pass */
	}
	if (tc32_p256_mul_start(s.priv, &s.pka[0], &s.pka[32]) != 0) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	s.p256 = true;
	s.dh = DH_RUNNING; /* the engine has the private key now: its place takes the DHKey */
	memset(s.priv, 0, sizeof(s.priv));
	if (PASSKEY) {
		s.round = 0U;
		s.have_ca = false;
		s.state = SMP_PASSKEY_CONFIRM;
	} else {
		confirm_round();
	}
}

static void pairing_confirm(const uint8_t *p, uint8_t len)
{
	if (!expected(s.have_ca || !PASSKEY ? SMP_IDLE : SMP_PASSKEY_CONFIRM, len, 17U)) {
		return;
	}
	memcpy(s_ca, &p[1], 16);
	s.have_ca = true;
	tc32_rng_add_sample(sys_get_le32(s_ca), TC32_RNG_SRC_PEER_RANDOM);
	if (s.have_passkey) {
		confirm_round();
	}
}

static void pairing_random(const uint8_t *p, uint8_t len)
{
	uint8_t pdu[17] = {SMP_PAIRING_RANDOM};

	if ((s.state != SMP_WAIT_RANDOM && s.state != SMP_PASSKEY_RANDOM) || len != 17U) {
		fail(SMP_ERR_UNSPECIFIED);
		return;
	}
	memcpy(s.na, &p[1], 16);
	tc32_rng_add_sample(sys_get_le32(s.na), TC32_RNG_SRC_PEER_RANDOM);
	if (s.state == SMP_PASSKEY_RANDOM) {
		uint8_t check[16];
		uint8_t z = 0x80U | (uint8_t)((s.passkey >> s.round) & 1U);

		ble_f4(&s.pka[0], &s.pkb[0], s.na, z, check);
		if (memcmp(check, s_ca, sizeof(check)) != 0) {
			fail(SMP_ERR_CONFIRM_FAILED);
			return;
		}
	}
	memcpy(&pdu[1], s.nb, 16);
	send(pdu, sizeof(pdu));
	if (s.state == SMP_PASSKEY_RANDOM && ++s.round < PASSKEY_ROUNDS) {
		s.have_ca = false;
		s.state = SMP_PASSKEY_CONFIRM;
		return;
	}
	s.state = SMP_WAIT_DHKEY_CHECK;
}

static void dhkey_check(const uint8_t *p, uint8_t len)
{
	if (!expected(s.have_ea ? SMP_IDLE : SMP_WAIT_DHKEY_CHECK, len, 17U)) {
		return;
	}
	memcpy(s_ea, &p[1], 16);
	s.have_ea = true;
}

/* Ea is in and the DHKey is computed: MacKey and LTK, the check, Eb, the key to the link. */
static void finish(void)
{
	uint8_t peer[BLE_ADDR_LEN], own[BLE_ADDR_LEN], peer_random;
	struct {
		uint8_t mackey[16], ltk[16], r[16], e[16];
	} w = {.r = {0}};
	uint8_t pdu[17] = {SMP_DHKEY_CHECK};
	uint8_t iocap_a[3] = {s.preq[3], s.preq[2], s.preq[1]};
	uint8_t iocap_b[3] = {s.pres[3], s.pres[2], s.pres[1]};

	ble_link_addresses(peer, &peer_random, own);
	ble_f5(s_dhkey, s.na, s.nb, peer_random, peer, 1U, own, w.mackey, w.ltk);
	if (PASSKEY) {
		sys_put_le32(s.passkey, w.r);
	}
	ble_f6(w.mackey, s.na, s.nb, w.r, iocap_a, peer_random, peer, 1U, own, w.e);
	if (memcmp(w.e, s_ea, sizeof(w.e)) == 0) {
		ble_f6(w.mackey, s.nb, s.na, w.r, iocap_b, 1U, own, peer_random, peer, &pdu[1]);
		send(pdu, sizeof(pdu));
		memset(&w.ltk[s.key_size], 0, 16U - s.key_size);
		ble_link_set_pairing_key(w.ltk, PASSKEY);
		memcpy(s.bond.peer, peer, BLE_ADDR_LEN);
		s.bond.peer_random = peer_random;
		memcpy(s.bond.ltk, w.ltk, 16);
		s.bond.ediv = 0U;
		memset(s.bond.rand, 0, sizeof(s.bond.rand));
		s.bond.flags = BLE_BOND_SC | (PASSKEY ? BLE_BOND_AUTHENTICATED : 0U);
		s.state = SMP_WAIT_ENC;
	}
	memset(&w, 0, sizeof(w));
	memset(s_dhkey, 0, 32);
	if (s.state != SMP_WAIT_ENC) {
		fail(SMP_ERR_DHKEY_CHECK);
	}
}

static void keys_done(void)
{
	if (s.state == SMP_WAIT_KEYS && s.init_keys == 0U) {
		if (s.pres[3] & AUTH_BONDING) {
			s.bond.cccd = ble_att_cccd_bits(); /* any the client set before the keys */
			/* the host discovered this database on the link it paired on */
			ble_bond_store(ble_profile_active(), &s.bond, ble_att_db_hash());
			ble_link_set_bonded();
		}
		cidoo_ble_stats.pairings++;
		attempts.waiting = false;
		clear();
	}
}

/* The link is encrypted with the pairing's key: the central's keys, if any, come now. */
void ble_smp_encrypted(bool with_pairing_key)
{
	if (!with_pairing_key || s.state != SMP_WAIT_ENC) {
		return;
	}
	s.init_keys = s.pres[5];
	s.state = SMP_WAIT_KEYS;
	keys_done();
}

static void passkey_poll(void)
{
	bool done, given_up;
	uint32_t value = 0;
	unsigned int key;

	if (!PASSKEY || !pk.wanted) {
		return;
	}
	key = irq_lock();
	done = pk.done;
	given_up = pk.given_up;
	if (done) {
		for (uint8_t i = 0; i < pk.n; i++) {
			value = value * 10U + pk.digits[i];
		}
	}
	irq_unlock(key);
	if (given_up) {
		fail(SMP_ERR_PASSKEY_FAILED);
		return;
	}
	if (!done) {
		return;
	}
	pk.wanted = false;
	s.passkey = value;
	s.have_passkey = true;
	if (s.state == SMP_PASSKEY_CONFIRM && s.have_ca) {
		confirm_round();
	}
}

/* The P-256 operations under way, and the steps that wait for them. */
static void step(void)
{
	int rc;

	if (s.state == SMP_VERIFY) {
		rc = p256_poll(NULL, NULL);
		if (rc == -EAGAIN) {
			return;
		}
		if (rc != 0) {
			fail(SMP_ERR_DHKEY_CHECK); /* not a point of the curve */
			return;
		}
		verified();
		return;
	}
	if (s.state == SMP_KEYGEN) {
		if (!s.have_pkb) {
			rc = p256_poll(&s.pkb[0], &s.pkb[32]);
			if (rc == -EAGAIN) {
				return;
			}
			if (rc != 0) {
				fail(SMP_ERR_UNSPECIFIED);
				return;
			}
			s.have_pkb = true;
		}
		keygen_done(); /* queues the public key, or tries again next pass */
		return;
	}
	if (s.dh == DH_RUNNING) {
		rc = p256_poll(s_dhkey, NULL);
		if (rc == -EAGAIN) {
			return;
		}
		if (rc != 0) {
			fail(SMP_ERR_UNSPECIFIED);
			return;
		}
		s.dh = DH_DONE;
	}
	if (s.state == SMP_WAIT_DHKEY_CHECK && s.have_ea && s.dh == DH_DONE) {
		finish();
	}
}

bool ble_smp_poll(void)
{
	if (ble_link_enc_requested()) {
		s.sec_req = false;
	}
	if ((s.state != SMP_IDLE || s.sec_req) && (uint32_t)(k_uptime_get_32() - s.t_last) > SMP_TIMEOUT_MS) {
		cidoo_ble_stats.smp_failures++;
		if (s.state != SMP_IDLE) {
			attempt_failed();
		}
		clear();
		s.locked = true;
		return false;
	}
	passkey_poll();
	step();
	security_request();
	return false;
}

bool ble_smp_passkey_wanted(void)
{
	return pk.wanted;
}

void ble_smp_passkey_key(uint8_t key)
{
	unsigned int lock = irq_lock();

	if (pk.wanted && !pk.done) {
		if (key < 10U) {
			if (pk.n < 6U) {
				pk.digits[pk.n++] = key;
			}
		} else if (key == BLE_PASSKEY_BACKSPACE) {
			pk.n -= pk.n > 0U ? 1U : 0U;
		} else if (key == BLE_PASSKEY_ENTER) {
			pk.done = true;
		} else if (key == BLE_PASSKEY_ESCAPE) {
			pk.given_up = true;
		}
	}
	irq_unlock(lock);
	ble_conn_kick();
}

void ble_smp_rx(const uint8_t *p, uint8_t len)
{
	if (len == 0U || s.locked) {
		return;
	}
	s.t_last = k_uptime_get_32();
	s.sec_req = false;
	switch (p[0]) {
	case SMP_PAIRING_REQ:
		pairing_req(p, len);
		break;
	case SMP_PUBLIC_KEY:
		public_key(p, len);
		break;
	case SMP_PAIRING_CONFIRM:
		pairing_confirm(p, len);
		break;
	case SMP_PAIRING_RANDOM:
		pairing_random(p, len);
		break;
	case SMP_DHKEY_CHECK:
		dhkey_check(p, len);
		break;
	case SMP_PAIRING_FAILED:
		clear();
		break;
	case SMP_KEYPRESS:
		break; /* the central's typing, nothing to do with it */
	case SMP_ENC_INFO:
	case SMP_CENTRAL_IDENT:
	case SMP_SIGNING_INFO:
		break; /* not asked for; a central that sends them anyway loses nothing */
	case SMP_IDENT_INFO:
		if (s.state == SMP_WAIT_KEYS && len == 17U) {
			memcpy(s.bond.irk, &p[1], 16);
		}
		break;
	case SMP_IDENT_ADDR_INFO:
		if (s.state == SMP_WAIT_KEYS && len == 8U) {
			s.bond.id_random = p[1];
			memcpy(s.bond.id_addr, &p[2], BLE_ADDR_LEN);
			s.init_keys &= (uint8_t)~KEY_ID;
			keys_done();
		}
		break;
	default:
		fail(SMP_ERR_CMD_UNSUPPORTED);
		break;
	}
	step();
}
