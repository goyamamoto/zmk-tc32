# BLE Secure Connections on the own stack: test plan

The own BLE stack (`tc32/src/ble`) moves from legacy Just Works pairing to LE Secure Connections
(P-256 ECDH on the TLSR8278's public key engine, AES-CMAC based f4/f5/f6, Passkey Entry with the
keyboard as KeyboardOnly). This plan names every test, where its inputs and its expected values
come from, and where it runs. Nothing here is a schedule; the "status" column says what exists.

Test data comes from three outside sources before anything of our own:

| source | what it gives | where it is |
|---|---|---|
| Bluetooth Core specification Vol 3 Part H Appendix D | the official vectors for AES-CMAC (RFC 4493), f4, f5, f6, g2, h6-h8; the debug key pair (2.3.5.6.1) | the specification; the same values, already in PDU byte order, in Zephyr's `subsys/bluetooth/host/smp.c` self-tests (`CONFIG_BT_SMP_SELFTEST`, `smp_aes_cmac_test()` to `smp_h8_test()`) |
| Apache NimBLE host tests | whole recorded pairings: both public keys, the responder's private key, both randoms, every confirm, both DHKey checks, the LTK; Just Works, Passkey Entry and Numeric Comparison, both roles, public and random addresses; also legacy pairings | `refs/mynewt-nimble/nimble/host/test/src/ble_sm_sc_test.c`, `ble_sm_lgcy_test.c`, `ble_sm_test_util.c` (the way the recordings are replayed) |
| Zephyr's Bluetooth host | the reference behaviour of a full SMP: `subsys/bluetooth/host/smp.c`; the simulated pairing tests `tests/bsim/bluetooth/host/security/*` (`passkey_entry`, `bond_overwrite_*`, `bond_per_connection`, `id_addr_update`, `level_enforced`) | the Zephyr tree (`ws/zephyr`) |

Google's Bumble is the independent peer for the connection tests (it pairs with Secure Connections and
understands a KeyboardOnly responder); macOS, iOS, Windows and Android hosts are the real peers.

## Byte order, settled once

Every function of ours takes and gives octets in the order the SMP PDUs carry them (little-endian,
as `ble_crypto.h` already does for c1/s1). The specification prints its vectors most significant
octet first. Each crypto test therefore feeds a function twice: the specification's vector reversed
into PDU order (must match), and a value a real pairing carried (NimBLE's recording, already in PDU
order; must match). A function that passes only one of the two has its octet order wrong somewhere.

## L0: P-256 (`tc32_p256.h`)

Two implementations behind one API (`zephyr-tc32/include/zephyr/crypto/tc32_p256.h`, Kconfig
`TC32_P256`): the software one, `TC32_P256_SOFT` (p256-m, `lib/crypto`), which needs nothing of the
chip and is the reference, and the TLSR8278's public key engine, `TC32_P256_PKE`
(`soc/telink/tlsr/tlsr825x/tlsr_pke.c`), which the Cidoo boards select (their `Kconfig.defconfig`)
for its size and speed; the engine's behaviour on the chip is still to be confirmed (H1), until then
the emulator's model of it stands in. Every test below runs against both; E6-E8 only against the
engine.

| id | test | inputs and expected values | runs | status |
|---|---|---|---|---|
| E1 | debug private key times G is the debug public key | Core 2.3.5.6.1 | ztest `tests/crypto/tc32_p256` on both emulators; hardware over `telink_ota.py pke-test` | exists |
| E2 | the recorded responder's private key times G is the public key it sent; times the initiator's public key it is the DHKey whose f5 output is the recorded LTK | NimBLE `ble_sm_sc_peer_jw_iio3_rio3_b1_iat0_rat0_ik5_rk7`; the chain re-derived by `tests/crypto/tc32_p256/nimble_sc_vectors.py` | as E1 | exists |
| E3 | dA times B equals dB times A | E1 and E2 keys | ztest | exists |
| E4 | a point off the curve, a point with y = p, and the recorded public keys: verify refuses the first two and accepts the rest | E1, E2 | ztest; hardware (off-curve) | exists |
| E5 | scalars 0, n - 1, n, n + 1, 2^256 - 1: 0 and n and above refused before the engine runs | P-256's n | ztest; hardware (0) | exists |
| E6 | the asynchronous path: a second start while busy is refused, Done is cleared after the result is taken | — | ztest | exists |
| E7 | after every operation the operand RAM is cleared and the engine's clock is off | — | ztest | exists |
| E8 | an operation that never finishes ends with -ETIMEDOUT after the configured time and the engine stopped | emulator register 0xffec | ztest | exists |
| E9 | the time of each operation: p256-m in the emulator (cycle count), the engine on the chip in us (the specification gives none) | ztest `test_09_time`; hardware `pke-test` record | emulator; hardware session | p256-m 708 ms in the emulator; the chip's engine 30.1-31.1 ms a multiplication, 0.15-0.22 ms a check (V21, 2026-10-08) |

## L1: the crypto functions (AES-CMAC, f4, f5, f6, g2)

Code `tc32/src/ble/ble_crypto.c`; harness `tc32/tests/ble_crypto/run.sh`, a host build with a software
AES (`soft_aes.c`) that already tests c1, s1, ah, the session key and CCM against Appendix D. The SC
functions join that file; each test is one call and one comparison.

| id | test | inputs and expected values | status |
|---|---|---|---|
| C1 | AES-CMAC: the four RFC 4493 lengths (0, 16, 40, 64 octets) | Appendix D / Zephyr `smp_aes_cmac_test()` | to write |
| C2 | f4(U, V, X, Z): the specification's vector; then Cb from the recording: f4(PKbx, PKax, Nb, 0) = `82edd062...` | Appendix D; NimBLE JW recording | to write |
| C3 | f5(W, N1, N2, A1, A2): MacKey and LTK from the specification; then the recorded LTK `63598a14...` from the recorded DHKey, randoms and addresses (A1 = 00 + initiator address, A2 = 00 + responder address, each 7 octets) | Appendix D; NimBLE | to write |
| C4 | f6(W, N1, N2, R, IOcap, A1, A2): the specification's vector; then both recorded DHKey checks Ea `82651d02...` and Eb `063c284a...` (R = 0 for Just Works; IOcap = AuthReq, OOB flag, IO capability of each side's pairing PDU) | Appendix D; NimBLE | to write |
| C5 | g2(U, V, X, Y) mod 10^6: the specification's vector | Appendix D | only with Numeric Comparison |
| C6 | Passkey Entry's per-round values: for the recorded passkey pairing, round i's confirm from Na_i, Nb_i and bit i of the passkey (ri = 0x80 or 0x81) | NimBLE `ble_sm_sc_peer_pk_iio0_rio2_b1_iat0_rat0_ik5_rk7` (initiator shows, the responder types) | to write |

All of L1 runs on the host in under a second (`tc32/tests/ble_crypto/run.sh`); C1-C4 also run on the
hardware AES through the emulator when `ble_crypto.c` is built into a ztest image (the AES block is
the chip's; the software AES of the host test stands in for it).

## L2: the SMP state machine (the keyboard as responder)

The stack under test runs in the whole-keyboard emulator (Go runner `v75-keyboard`);
the peer is the emulator's own scripted central (`sim/ble_host.py`, steps `bt-pair`, `bt-keys`,
`bt-transcript`), which sends exactly the PDUs a test names and checks exactly the PDUs it gets. For
the recorded pairings the stack needs a test hook (a Kconfig for test images only) that sets the
private key and the randoms it would otherwise draw, so its confirms, DHKey check and LTK must equal
the recording octet for octet. The outcome of every negative test is the Pairing Failed reason the
specification names (Vol 3 Part H 3.5.5) and that the link is not encrypted afterwards.

| id | test | peer behaviour | expected | status |
|---|---|---|---|---|
| S1 | Just Works, public addresses | the emulator's central (NoInputNoOutput; `bt-pair sc`); the NimBLE replay needs a test hook for the private key and randoms | confirm, DHKey check, LTK agree; encryption and HID follow | passes in the emulator (Just Works build) |
| S2 | Passkey Entry, the initiator displays, we type (KeyboardOnly) | the emulator's central (DisplayOnly, MITM; `bt-pair passkey`, `bt-passkey` types it on the keypad); NimBLE `peer_pk_iio0_rio2_b1_iat0_rat0_ik5_rk7` with the hook | 20 rounds agree; the LTK authenticated; encryption and HID follow | passes in the emulator (default build) |
| S3 | random addresses | NimBLE `peer_jw_..._iat2_rat2_ik7_rk7`, `peer_pk_..._iat2_rat2_ik7_rk3` | as S1, S2 with A1/A2 of type 1 | to write |
| S4 | wrong passkey typed | S2 with `bt-passkey wrong` | Pairing Failed 0x04 (Confirm Value Failed) at the first round whose bit differs; no later rounds | passes in the emulator |
| S5 | confirm mismatch from the peer: a round's Confirm not matching its Random | S2 with `bt-fault confirm` | Pairing Failed 0x04 at that round | passes in the emulator |
| S6 | DHKey Check mismatch | S1 with `bt-fault dhkey` | Pairing Failed 0x0b (DHKey Check Failed); no LTK stored | passes in the emulator |
| S7 | public key off the curve, or equal to our own | S1 with `bt-fault offcurve`; PKa = PKb | Pairing Failed 0x0b at once; an echoed key gets 0x08 | off the curve passes in the emulator; the echoed key to write |
| S8 | the peer disconnects while we wait for the passkey | S2, link dropped mid-round | pairing state cleared; next connection pairs from the start | to write |
| S9 | the peer stops answering: SMP timeout | S1 with `bt-fault stall` | the procedure ends (Core 3.4: 30 s), no further SMP on this link until it reconnects; a new connection pairs | passes in the emulator (the lock itself is not observed from outside) |
| S10 | legacy pairing asked for | Pairing Request without the SC bit (`bt-pair`) | Pairing Failed 0x03 (Authentication Requirements, as Zephyr's SC-only answers) | passes in the emulator |
| S11 | key sizes: 7 accepted and the LTK masked on both sides; below 7 or above 16 refused | `bt-fault keysize7`, `bt-fault keysize6` | pairs and types; Pairing Failed 0x06 (Encryption Key Size) | both pass in the emulator |
| S12 | repeated failures | `sc-neg-repeated`: a failure, a retry on a new link within the wait, a retry after it | Pairing Failed 0x09 within the wait (2 s, doubling to 64 s), success after it | passes in the emulator |
| S13 | the 65-octet Pairing Public Key PDU over 27-octet links: fragmented both ways, a fragment of another channel interleaved, a wrong total length | S1 with the central fragmenting as the test says | reassembled; a wrong length dropped and the pairing fails cleanly | the plain fragmentation passes with S1; interleaving and wrong lengths to write |
| S14 | a second pairing attempt on an already encrypted link | S1, then a new Pairing Request | handled as the specification says (a new pairing replaces the bond or is refused; one behaviour, tested) | to write |

What Zephyr's `smp.c` does in each of S4-S14 is the reference when the specification leaves a choice.

## L3: connection tests with an independent peer (Bumble)

Runner `v75-bumble` of the whole-keyboard emulator, Bumble as the central with a scripted pairing delegate; the
keyboard is the emulated one, so these run on every change. Each test ends with typed keys arriving
as HID reports over the encrypted link, or with the reason they must not.

| id | test | expected | status |
|---|---|---|---|
| I1 | SC Just Works (Bumble NoInputNoOutput, also DisplayOnly, against the Just Works build) | pairs, bonds, HID reports flow | passes (Bumble, emulated radio) |
| I2 | SC Passkey Entry: Bumble displays, the keyboard types (digits then Enter) | pairs with authenticated LTK (MITM), reports flow | passes (Bumble, emulated radio) |
| I3 | reconnection after the keyboard's power cycle | encryption with the stored LTK, no pairing, reports flow | passes with the emulator's central and with Bumble (`--power-cycle`), both builds |
| I4 | the host forgets the bond, the keyboard keeps it | the keyboard advertises without the discoverable flag but with its name, and only its bonded host may connect; a host that forgot it still lists it (macOS does), connects, and pairs again from the start; the new bond replaces the old | passes with the emulator's central (`sc-host-forgets`); on the V21 with macOS the keyboard's bond was cleared first (the profile key held 3 s) and the re-pairing worked; Bumble to run |
| I5 | the keyboard forgets the bond (the profile key held 3 s), the host keeps it | the profile gets a new address, the link ends, the host's reconnection finds nothing and nothing typed arrives; a new pairing works | passes with the emulator's central (`sc-keyboard-forgets`); Bumble to run |
| I6 | a legacy-only peer, or one that cannot show a passkey against the Passkey build | refused (0x03); nothing typed reaches it | passes (the emulator's central `bt-pair`; Bumble `--io none` with `--expect-failure`) |
| I7 | policy: HID input needs authenticated encryption (TLSR_BLE_SC_PASSKEY, the default) | a Just Works host is refused (0x03) by the default build; a bond from a Just Works build (the keyboard updated to the Passkey build) encrypts but gets no notification, and reads of the HID attributes answer Insufficient Authentication (ATT 0x05) | passes in the keyboard emulator (`sim/scenarios/sc-i7-unauth-bond.txt`: nothing typed arrives; the host reused its cached GATT, so the 0x05 answer itself is not in the log) |
| I8 | bonds for several hosts (the keyboard's host slots) | each reconnects with its own LTK; a slot's bond removed affects that host only | to write |
| I9 | a host that lists a keyboard to pair only with its scan response (Android's active scan; Bumble `--scan active` with duplicate filtering) | each SCAN_REQ for the keyboard's address gets a SCAN_RSP T_IFS after it, discoverable or not; the host lists the keyboard, pairs by Passkey Entry, keys arrive | passes with the emulator's central (`sc-android-listing-v21-ble`, `-v75-ble`, `-v65-ble`; both runners, flash cache miss cost 0 and 190) and Bumble (`--scan active`); an image that does not answer fails both. Android on hardware: H4 |

The scenarios above ran on the V21 model; `sc-passkey-v75-ble` and a Bumble Passkey Entry session (with the
power cycle) also pass on the V75 Pro model.

## Sizes

The studio images, as the boards build them (P-256 on the engine, the BLE thread's stack 1152):

| image | flash | RAM | slot / RAM |
|---|---|---|---|
| V21, before Secure Connections (legacy SMP) | 118,480 | 20,074 | |
| V21, Secure Connections, Passkey Entry | 120,088 | 20,362 | 512 KB / 22 KB |
| V21, Secure Connections, Just Works | 119,424 | 20,362 | |
| V75 Pro, Secure Connections, Passkey Entry | 129,948 | 23,676 | 131,072 / 24,320 |
| V65 V3, Secure Connections, Passkey Entry | 129,436 | 23,036 | 131,072 / 24,320 |

What Secure Connections costs over the legacy pairing: SMP about 1.4 KB, the link's fragmentation 0.3 KB,
AES-CMAC and f4/f5/f6 0.5 KB, the engine driver 1.0 KB (of which 192 octets the curve's constants). With
P-256 in software instead (`TC32_P256_SOFT`): 3.9 KB more flash and 512 more stack.

## L4: hardware

| id | test | how | status |
|---|---|---|---|
| H1 | the engine's known answers and times on the chip (E1, E2, E4, E5, E9) | measurement image 04177171, `telink_ota.py pke-test`, the V21 hardware session of 2026-10-09 (image 392118c8) | done on the V21: the debug key, the recorded key and the DHKey match, on and off the curve as expected; a scalar of 0 is never finished by the engine (now refused before it runs; the emulator's model hangs on 0 like the chip) |
| H2 | the L1 functions on the chip's AES block | a ztest image or the measurement image's command | to write |
| H3 | pairing and reconnection with Bumble on a real radio (I1-I5) | Bumble on a Mac or Linux dongle | not run (macOS stood in, H4) |
| H4 | real hosts: macOS, iOS, Windows, Android: pair with Passkey Entry, type, power-cycle and reconnect, forget on the host, forget on the keyboard | by hand, logged in a session folder | macOS done on the V21 with the image 392118c8 (2026-10-08): the Mac showed the passkey, the keypad typed it, keys arrived, reconnection after Bluetooth off and on without a passkey, the keyboard's bond cleared and re-paired; link counters pairings 2, smp_failures 0, mic_failures 0. After the Mac forgot the keyboard it still listed it (directed advertising to the Mac's own address), unlike the emulator's host. iOS, Windows, Android to run |
| H5 | SC-only against a host that offers legacy first | the host falls back or fails as its UI shows; nothing typed leaks | with H4 |

## Order

L0 is done but for H1 (the hardware session comes last; the software P-256 carries the stack until then). L1 next (C1-C4 and C6: the functions the SMP needs), then the L2CAP
fragmentation (S13) and the Just Works path (S1, S6, S7, S9, S10, S11, S13), then Passkey Entry
(S2, S4, S5, S8, S12) and the L3 Bumble tests, then the hardware. Numeric Comparison (C5) only if a
display ever exists. Correctness first: the straightforward implementation and these tests, then
size and speed (the engine, the AES block's use, code layout) judged by the same tests.
