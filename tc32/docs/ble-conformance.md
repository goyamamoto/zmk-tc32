# Bluetooth conformance of the own BLE stack

The Bluetooth requirements that apply to what the keyboards are over BLE, each with the code that meets it, the tests that check it and their result. A requirement that fails is a row of `known-gaps.md`; one that no test checks is listed there under "Untested requirements".

## What the keyboard is

A BLE HID keyboard in the peripheral role (`tc32/src/ble`, `CONFIG_TLSR_BLE`):

- a connectable and scannable undirected advertiser (legacy ADV_IND on channels 37, 38 and 39), its address random static, one per profile;
- the peripheral of one connection at a time, on the 1M PHY, channel selection algorithm #1, data PDUs of 27 octets (no data length extension), ATT_MTU 23;
- LE Secure Connections only: Passkey Entry with the keyboard typing the passkey (KeyboardOnly, the default `TLSR_BLE_SC_PASSKEY`) or Just Works (a build option); bonding, with the host's IRK and identity address taken and resolved;
- LL encryption with its pause, the L2CAP Connection Parameter Update Request, connection and channel map updates from the central;
- a GATT server: GAP (Device Name, Appearance, Peripheral Preferred Connection Parameters), Device Information (PnP ID), Battery (Battery Level, notified), HID (Protocol Mode, keyboard and consumer input reports, the keyboard LED output report, Report Map, HID Information, HID Control Point, Boot Keyboard Input and Output Reports; every value and the reports' CCCDs need a secure link), GATT (Service Changed, indicated to a bonded host whose database changed, at handles 48-51 that later images keep).

Not applicable, from the code: the central, broadcaster and observer roles; directed, scannable-only, non-connectable and extended advertising; periodic advertising and isochronous channels; the 2M and Coded PHYs and the PHY update procedure; data length extension; channel selection algorithm #2; the Connection Parameters Request procedure, Extended Reject and peripheral-initiated feature exchange (not in the feature set, answered LL_UNKNOWN_RSP); LE Ping (not in the feature set; LL_PING_REQ is answered); privacy of the keyboard's own address (it is never resolvable, so it distributes no IRK); legacy pairing, OOB and Numeric Comparison (no display); LE credit based channels; GATT client; ATT long writes, Read Multiple, signed writes; indications other than Service Changed; Database Hash and Client Supported Features (GATT caching, Part G 7.3: Service Changed alone tells every host kind in the host rules that the database changed).

## Sources

- Bluetooth Core Specification 5.4 (cited as Vol/Part/section); HID over GATT Profile 1.0 (the requirements cited were checked against 1.1 and 1.2), HID Service 1.0, Battery Service 1.1, Device Information Service 1.1.
- The Bluetooth SIG test suites LL.TS.p28, GAP.TS.p50, SM.TS.p30, L2CAP.TS.p42, GATT.TS.p30 (ATT is tested there), HOGP.TS.p13, HIDS.TS.p8, BAS.TS.p8 and DIS.TS.p6, as published on bluetooth.com; their test case IDs are given where one covers the row. Which cases apply depends on an ICS; the selection assumed here: LE only, peripheral, legacy advertising, static random address, LE Encryption, Secure Connections Only mode, no filter accept list, Service Changed (GATT ICS item 4/23: GATT/SR/GAS and GAI), no Database Hash, HIDS boot keyboard selected.

## Tests

| short | what | where |
|---|---|---|
| CONF | `zmk-ble-conformance CHECK`: the image from its start flash on the whole-keyboard model, the BT position, a scripted central | `sim/zmk_ble_conformance.py` (Go: `go/cmd/zmk-ble-conformance`) of the whole-keyboard emulator, which is not published; the suite's `conform` jobs |
| SCEN | a whole-keyboard scenario | the keyboard emulator's `sim/scenarios/NAME.txt` |
| SUITE | a suite job | the keyboard emulator's `suite/v75pro/KIND/NAME.txt`, `suite/v21/jobs.txt` |
| CONN | C-5's connection checks against the stock firmware | the keyboard emulator's `sim/zmk_ble_conn_checks.py` (Go: `go/cmd/zmk-ble-conn-checks`) |
| CRYPTO | the crypto functions on the host (Core Vol 3 Part H Appendix D, RFC 4493, a recorded pairing) | `tc32/tests/ble_crypto/run.sh` |
| P256 | P-256 on both implementations | zephyr-tc32 `tests/crypto/tc32_p256` |
| BUMBLE | Google's Bumble as the host | the keyboard emulator's suite `bumble` |

Status: **passes** (the tests named pass on the V75 Pro and V21 images with Passkey Entry); **fails: GAP** (a test fails; `GAP` is its row in `known-gaps.md`, and the test is listed as an expected failure); **not tested** (no test checks it; listed in `known-gaps.md`, "Untested requirements"); **JW build** (the scenarios named pair with Just Works and run on a Just Works build; on Passkey Entry images they are refused by design).

CONN's `basic`, `drop` and `chm` scenarios and C-5's advertising checks (`sim/zmk_ble_checks.py`) look at a fixed 200-300 ms after a boot from a fresh flash. On the current images they see fewer events in that time than they expect (6 advertising events where the stock has 12; 18 connection events where 25 are wanted; no LL answer, no channel map instant) and fail on those counts; the requirements they check are checked by CONF from the start flash.

## Link layer: advertising (Vol 6 Part B)

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| ADV-PDU | ADV_IND: PDU type 0, TxAdd 1 for a random AdvA, ChSel 0 (no CSA #2), 6-37 octets (2.3.1.1) | LL/DDI/ADV/BV-02-C | `ble_adv.c` `ble_adv_init()` | CONF ADV-PDU | passes |
| ADV-CHANNELS | every event on 37, 38 and 39 in that order (4.4.2.1) | LL/DDI/ADV/BV-02-C | `ble_adv.c` `ble_adv_event()` | CONF ADV-CHANNELS | passes |
| ADV-INTERVAL | T_advEvent = advInterval (20 ms or more) + advDelay (0-10 ms) (4.4.2.2) | LL/DDI/ADV/BV-02-C | `ble.c` `ble_thread()` | CONF ADV-INTERVAL | passes |
| ADV-ADDR | static random address: two top bits 1, the rest neither all 0 nor all 1 (1.3.2.1) | — | `ble_adv.c` `ble_adv_init()` | CONF ADV-ADDR | passes |
| ADV-DATA | AD structures well formed, 31 octets at most (Vol 3 Part C 11); Flags with LE General Discoverable and BR/EDR Not Supported while discoverable (Vol 3 Part C 9.2.4); HOGP 1.0 3.1.3-3.1.5: the HID UUID, the name and the appearance should be in the advertising | GAP/ADV/BV-01-C, BV-02-C, BV-03-C, BV-11-C; GAP/DISC/GENM/BV-04-C | `ble_adv.c` `ble_adv_init()` | CONF ADV-DATA | passes |
| ADV-LIMITED | HOGP 1.0 5.1.1: a HID device "should use" the Limited Discoverable mode | — | `ble_adv.c` (General Discoverable for `TLSR_BLE_PAIRING_TIMEOUT_S`, 60 s) | — | not followed (a "should"), deliberate: hosts list general discoverable advertisers in every discovery, and the 60 s window ends the advertising (`known-gaps.md`, scope limits) |
| ADV-SCAN-RSP | a SCAN_REQ for this AdvA from an allowed scanner is answered with SCAN_RSP on the same channel (4.4.2.3) | LL/DDI/ADV/BV-05-C, BV-07-C; LL/DDI/ADV/BI-01-C; LL/ENC/ADV/BI-01-C | `ble_adv.c` `ble_adv_event()`, zephyr-tc32 `tlsr_radio_tx_on_rx()` | SCEN `sc-android-listing-v21-ble`, `-v75-ble`, `-v65-ble` (an Android host lists the keyboard only with its scan response, T_IFS after the SCAN_REQ in TC32-devtools' radio model) | passes |
| ADV-CONNECT-IND | a CONNECT_IND for this AdvA with valid parameters starts the connection (4.4.2.3, 2.3.3.1) | LL/DDI/ADV/BV-06-C; LL/CON/ADV/BV-01-C | `ble_adv.c` `connect_ind()` | every CONF run | passes |
| ADV-CONNECT-IND-INVALID | a CONNECT_IND with invalid parameters is not taken | LL/DDI/ADV/BI-07-C (hop); LL/DDI/ADV/BI-02-C (CRC: not modelled) | `ble_adv.c` `connect_ind()` | CONF LL-CONNECT-IND-INVALID-HOP | passes (hop); a bad CRC not tested |
| ADV-NON-DISCOVERABLE | with a bond: no discoverable flag, and only the bonded host's CONNECT_IND (its address, identity address or a resolvable private address its IRK resolves) is taken (Vol 3 Part C 9.2.2, 9.3.4) | GAP/DISC/NONM/BV-02-C; GAP/CONN/UCON/BV-01-C | `ble_adv.c` `connect_ind()`, `ble_bond.c` `ble_bond_peer_known()` | CONF ADV-BONDED-FILTER, SM-RPA-RECONNECT; SCEN `sc-host-forgets` | passes (CONF); SCEN: JW build |

## Link layer: connection (Vol 6 Part B)

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| LL-ESTABLISH | the first packet anywhere in the transmit window after WinOffset; any hop 5-16; the access address of the CONNECT_IND; a connection not established after 6 events is lost (4.5.1, 4.5.2) | LL/CON/ADV/BV-01-C; LL/FRH/ADV/BV-01-C; LL/CON/ADV/BI-01-C | `ble_conn.c` `ble_conn_start()`, `event_start()`, `event_end()` | CONF LL-TRANSMIT-WINDOW | passes (the 6-event loss: not tested here) |
| LL-ACK | SN/NESN: a NAKed packet sent again, a repeated packet dropped (4.5.9) | LL/CON/PER/BV-15-C to BV-18-C, BV-21-C | `ble_conn.c` `rx_packet()`, `tx_acks()` | CONF ATT-RSP-RING-ROOM (the central NAKs, the answer comes after) | passes |
| LL-DATA | sending and receiving data PDUs (4.5) | LL/CON/PER/BV-04-C to BV-06-C | `ble_conn.c`, `ble_link.c` | every CONF ATT check; SCEN `sc-passkey-*` | passes |
| LL-SUPERVISION | the connection is lost after connSupervisionTimeout without a packet (4.5.2) | LL/CON/PER/BV-13-C | `ble_conn.c` `event_end()` | CONF LL-SUPERVISION-TIMEOUT; CONN `timeout` | passes |
| LL-CONN-UPDATE | the central's LL_CONNECTION_UPDATE_IND applied at its instant, the new transmit window after WinOffset (5.1.1) | LL/CON/PER/BV-10-C | `ble_conn.c` `ble_conn_update()`, `event_end()` | CONF LL-CONN-UPDATE; CONN `update`; SCEN `sc-update-lead1-v75-ble`, `sc-update-lead1-v21-ble`; SUITE `mine/update-lead1-bt` | passes |
| LL-CHM-UPDATE | LL_CHANNEL_MAP_IND applied at its instant (5.1.2) | LL/FRH/PER/BV-01-C | `ble_conn.c` `ble_conn_channel_map()` | CONF LL-CHM-UPDATE | passes |
| LL-INSTANT-PASSED | an update or channel map whose instant has passed: the connection is lost, 0x28 (5.5.1) | LL/CON/PER/BI-04-C | `ble_conn.c` `instant_passed()` | CONF LL-INSTANT-PASSED-UPDATE, LL-INSTANT-PASSED-CHM | passes |
| LL-TERMINATE-RX | the central's LL_TERMINATE_IND ends the connection once acknowledged (5.1.6) | LL/CON/PER/BV-12-C | `ble_ll_ctrl.c` `ble_ll_ctrl_rx()` | CONF LL-TERMINATE; CONN `terminate` | passes |
| LL-TERMINATE-TX | the keyboard's LL_TERMINATE_IND (a profile switch): the link ends when acknowledged or after the supervision timeout (5.1.6) | LL/CON/PER/BV-11-C, BI-02-C | `ble.c` `run_connection()` | SCEN `v21-ble-terminate-ack`, `v21-ble-profiles` (V21 suite, group b) | passes |
| LL-VERSION | LL_VERSION_IND answered once (5.1.5), with VersNr 8 (Core 4.2): Android asks a peer below 4.2 for legacy pairing only, which the keyboard refuses | LL/CON/PER/BV-19-C, BV-20-C, BI-15-C | `ble_ll_ctrl.c` | CONN `basic` (VersNr 8, company, subversion); SCEN `sc-android-listing-*-ble` (the Android host's Pairing Request with the SC bit) | passes (the IUT-requesting cases do not apply: the keyboard starts no procedure) |
| LL-FEATURES | LL_FEATURE_REQ answered with the features both have (5.1.4) | LL/CON/PER/BV-14-C | `ble_ll_ctrl.c` | CONN `basic` | passes |
| LL-UNKNOWN | a control PDU not supported answered LL_UNKNOWN_RSP with its opcode (2.4.2) | LL/PAC/PER/BV-01-C | `ble_ll_ctrl.c` `ble_ll_ctrl_rx()` | CONF LL-PHY-REQ, LL-CONN-PARAM-REQ, LL-LENGTH-REQ-ENC; CONN `procedures` | passes |
| LL-WRONG-ROLE | a control PDU only a peripheral sends, received from the central: LL_UNKNOWN_RSP | LL/PAC/PER/BV-02-C | `ble_ll_ctrl.c` | CONF LL-CTRL-WRONG-ROLE | passes |
| LL-INVALID-LENGTH | a control PDU of a wrong length: the procedure goes on, or LL_UNKNOWN_RSP / LL_REJECT_IND (2.4.2) | LL/PAC/PER/BI-01-C | `ble_link.c` `ble_link_enc_req()`, `ble_ll_ctrl.c` | CONF LL-CTRL-INVALID-LENGTH | fails: BLE-LL-INVALID-LENGTH |
| LL-INVALID-LLID | a data PDU with the reserved LLID is ignored | LL/CON/PER/BI-17-C | `ble_link.c` `ble_link_rx_data()` | CONF LL-INVALID-LLID | passes |
| LL-ENC-START | the encryption start: LL_ENC_RSP, LL_START_ENC_REQ, the encrypted LL_START_ENC_RSP both ways; data encrypted from then on (5.1.3.1) | LL/SEC/PER/BV-01-C, BV-05-C | `ble_link.c` `ble_link_enc_req()`, `ble_link_start_enc_rsp()` | CONF SM-PASSKEY-PAIR, SM-RPA-RECONNECT; SCEN `sc-passkey-*` | passes |
| LL-ENC-NO-KEY | no LTK: LL_REJECT_IND 0x06 (5.1.3.1) | LL/SEC/PER/BV-04-C | `ble_link.c` `ble_link_enc_req()` | CONN `procedures` | passes |
| LL-ENC-PAUSE | the encryption pause: LL_PAUSE_ENC_RSP, then a new encryption start; no data PDU during it (4.6.1, 5.1.3.2) | LL/SEC/PER/BV-02-C, BV-03-C | `ble_link.c` `ble_link_pause_enc_req()`, `ble_link_pause_enc_rsp()`, `ble_link_rx()` | CONF LL-ENC-PAUSE (the central pauses and encrypts again with the pairing's key; a key typed then is notified), SM-PAIR-ON-ENCRYPTED | passes (a PDU the pause does not allow, and no key after it: not tested) |
| LL-ENC-TIMEOUT | the procedure response timeout: 40 s, then the link ends with 0x22 (5.2) | LL/SEC/PER/BI-01-C | — | CONF LL-PROC-TIMEOUT | fails: BLE-LL-RESPONSE-TIMEOUT |
| LL-MIC | a MIC failure ends the connection, 0x3d (5.1.3.1) | LL/SEC/PER/BI-03-C | `ble_link.c` `ble_link_rx()` | CONF LL-MIC-FAILURE | passes (a corrupted header, BI-04-C: not tested) |
| LL-ENC-UNEXPECTED | a PDU not allowed during the encryption start | LL/SEC/PER/BI-05-C, BI-07-C | `ble_link.c` | — | not tested |
| LL-NO-DLE | no data length extension: LL_LENGTH_REQ answered LL_UNKNOWN_RSP, 27-octet PDUs | — | `ble_ll_ctrl.c` | CONF LL-LENGTH-REQ-ENC; CONN `procedures` | passes |
| LL-PHY | 1M only: LL_PHY_REQ answered LL_UNKNOWN_RSP | — | `ble_ll_ctrl.c` | CONF LL-PHY-REQ | passes |
| LL-L2CAP-FRAG-TX | an L2CAP frame longer than a PDU sent as start and continuation PDUs (the 65-octet Pairing Public Key) | LL/DFL/PER/BV-01-C | `ble_link.c` `ble_l2cap_send()` | every pairing (CONF SM-PASSKEY-PAIR) | passes |
| LL-L2CAP-FRAG-RX | an L2CAP frame (or its header) in several PDUs put together (Vol 3 Part A 7.2) | LL/DFL/PER/BV-02-C | `ble_link.c` `ble_link_rx_data()` | CONF L2CAP-REASSEMBLY-ATT (SMP: every pairing) | fails: BLE-L2CAP-REASSEMBLY (ATT and signalling) |
| LL-LATENCY | peripheral latency: events skipped only with nothing to send, back at once when a PDU is queued (4.5.1) | — | `ble_conn.c` `skip_events()`, `unskip()` | SUITE `scen/v75-ble-lowpower`; SCEN `v21-ble-lowpower` (V21 suite, group b) | passes |
| LL-TIMING | the anchor follows the central, the receiver opens before the earliest packet, intervals 7.5 ms to 4 s | LL/TIM/PER/BV-01-C to BV-05-C | `ble_conn.c` `event_start()` | CONN `basic` (the receiver's lead), CONF LL-TRANSMIT-WINDOW, LL-CONN-UPDATE (30 ms); the suite's `idle-drift` jobs (a central 300 ppm off) | passes for 15 ms and 30 ms; the 4 s interval not tested |

## L2CAP (Vol 3 Part A)

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| L2-PARAM-REQ | the Connection Parameter Update Request: valid values, a non-zero identifier, sent only as peripheral, the values GAP's Peripheral Preferred Connection Parameters give (4.20; Vol 3 Part C 9.3.9.2, 12.3) | L2CAP/LE/CPU/BV-01-C; GAP/CONN/CPUP/BV-01-C | `ble_link.c` `ble_link_request_conn_params()`, `ble_att.c` `ppcp` | CONF L2CAP-PARAM-REQ; SCEN `ble-pair-at-param-request` | passes (CONF) |
| L2-PARAM-REFUSED | the central's rejection: the link goes on with its parameters, no new request within 30 s, TGAP(conn_param_timeout) (Vol 3 Part C 9.3.9.2) | GAP/CONN/CPUP/BV-02-C, BV-03-C | `ble_link.c` `signaling_rx()`, `ble.c` `run_connection()` | CONF L2CAP-PARAM-REFUSED, L2CAP-PARAM-FALLBACK | passes |
| L2-PARAM-FALLBACK | after a rejection, one more request 30 s later with values inside Apple's limits (Accessory Design Guidelines R31 58.6: min a multiple of 15 ms, latency 30 or less, timeout 6-18 s and above 3 x max x (latency + 1)), then none: an Apple host takes it instead of keeping its own parameters | — | `ble.c` `run_connection()`, `ble_link.c`, `TLSR_BLE_CONN_FALLBACK_*` | CONF L2CAP-PARAM-FALLBACK; SCEN `sc-apple-params-v21-ble`, `-v75-ble`, `-v65-ble` (host kind `macos`: 15 ms, latency 22, 6 s applied) | passes |
| L2-CMD-REJECT | a signalling command not understood: Command Reject, the same identifier, reason 0 (4.1) | L2CAP/LE/REJ/BI-02-C | `ble_link.c` `signaling_rx()` | CONF L2CAP-CMD-REJECT | passes |
| L2-CBFC | an LE Credit Based Connection Request with no such channel: refused | L2CAP/COS/CED/BI-19-C, BI-21-C, BI-23-C | `ble_link.c` `signaling_rx()` | CONF L2CAP-LE-CREDIT-REQ (Command Reject) | passes |
| L2-UNKNOWN-CID | a frame on a CID with no channel is dropped | L2CAP/LE/CID/BI-01-C | `ble_link.c` `ble_link_rx_data()` | CONF L2CAP-UNKNOWN-CID | passes |
| L2-INVALID-LENGTH | a signalling command whose length is wrong | L2CAP/COS/CED/BI-11-C | `ble_link.c` `signaling_rx()` | — | not tested |
| L2-RECOMBINE | recombination of any L2CAP PDU (7.2) | LL/DFL/PER/BV-02-C | `ble_link.c` | CONF L2CAP-REASSEMBLY-ATT | fails: BLE-L2CAP-REASSEMBLY |
| L2-RESPONSE-UNDER-LOAD | every request answered (Vol 3 Part F 3.3.2), whatever the reports waiting | — | `ble_link.c` `l2cap_send()`: reports take the TX ring only while more than `BLE_TX_ANSWER_ROOM` of its `BLE_TX_SLOTS` are free | CONF ATT-RSP-RING-FULL | passes |

## Security Manager (Vol 3 Part H) and security (Vol 3 Part C 10)

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| SM-PASSKEY | Secure Connections Passkey Entry as responder, the keyboard typing (2.3.5.6.3) | SM/PER/SCPK/BV-02-C | `ble_smp.c`, `ble_passkey.c` | CONF SM-PASSKEY-PAIR; SCEN `sc-passkey-v21-ble`, `sc-passkey-v75-ble`; BUMBLE | passes |
| SM-PASSKEY-WRONG | a confirm that does not check: Pairing Failed 0x04 (2.3.5.6.3) | SM/PER/SCPK/BI-03-C | `ble_smp.c` `pairing_random()` | SCEN `sc-neg-wrong-passkey`, `sc-neg-confirm` | passes |
| SM-PASSKEY-PEER-FAIL | the initiator's Pairing Failed during the passkey: pairing ends, typing goes back to the host | SM/PER/SCPK/BI-04-C | `ble_smp.c` `ble_smp_rx()` | — (`sc-link-lost-pke-*` end the link instead) | not tested |
| SM-JW | Just Works as responder (2.3.5.6.2), the Just Works build | SM/PER/SCJW/BV-02-C, BI-02-C | `ble_smp.c` | SCEN `sc-jw-v21-ble`, `sc-jw-reconnect-v21-ble` | JW build |
| SM-AUTHREQ-RFU | reserved AuthReq bits ignored | SM/PER/SCJW/BV-03-C, SCPK/BV-03-C | `ble_smp.c` `pairing_req()` | — | not tested |
| SM-SC-ONLY | legacy pairing refused, 0x03 (Secure Connections Only mode, Vol 3 Part C 10.2.4) | GAP/SEC/SEM/BV-23-C, BV-24-C | `ble_smp.c` `pairing_req()` | CONF SM-LEGACY-REFUSED; BUMBLE `session-legacy` | passes |
| SM-KEYSIZE | 7-16 octet keys, the LTK masked; below 7: 0x06 (2.3.4) | SM/PER/EKS/BV-02-C, BI-02-C | `ble_smp.c` `pairing_req()` | SCEN `sc-neg-keysize6` (passes), `sc-pos-keysize7` (JW build) | passes (below 7) |
| SM-PUBKEY | a public key off the curve or equal to the keyboard's: Pairing Failed (2.3.5.6.1) | SM/PER/KDU/BI-01-C | `ble_smp.c` `public_key()`, `keygen_done()` | SCEN `sc-neg-offcurve`; P256 E4 | JW build (P256: run in zephyr-tc32) |
| SM-DHKEY | a DHKey check that does not check: 0x0b (2.3.5.6.5) | — | `ble_smp.c` `dhkey_check()` | SCEN `sc-neg-dhkey` | JW build |
| SM-TIMEOUT | 30 s without an SMP PDU ends the pairing, nothing more on the link (3.4) | SM/PER/PROT/BV-02-C | `ble_smp.c` `ble_smp_poll()` | SCEN `sc-neg-stall` | JW build |
| SM-PAIRING-END | a pairing that ends (Pairing Failed, the SMP timeout, the link lost) while a P-256 operation of it runs gives the operation up, so the next pairing finds the engine free (3.4) | — | `ble_smp.c` `clear()`, `ble_smp_reset()`; zephyr-tc32 `tc32_p256_cancel()` | SCEN `sc-link-lost-pke-v21-ble`, `-v75-ble`, `sc-neg-repeated-passkey-v21-ble`; P256 `test_10_cancel` | passes |
| SM-REPEATED | repeated attempts refused with 0x09 for a growing wait (2.3.6) | — | `ble_smp.c` `attempt_failed()` | CONF SM-REPEATED-WAIT, SM-REPEATED-AFTER-WAIT; SCEN `sc-neg-repeated` (JW build: the doubling) | passes |
| SM-CRYPTO | AES-CMAC, f4, f5, f6, the session key, CCM (2.2, Appendix D; Vol 6 Part C 1) | — | `ble_crypto.c` | CRYPTO (Appendix D and a recorded pairing) | passes |
| SM-P256 | P-256 key pairs and DHKey | — | zephyr-tc32 `tc32_p256` | P256 E1-E8 | run in zephyr-tc32 |
| SM-KEYS | the host's IRK and identity address taken; the keyboard distributes none (3.6.1) | — | `ble_smp.c` `keys_done()` | CONF SM-RPA-RECONNECT; SUITE `mine/pair-lost-id-bond-bt` | passes (CONF) |
| SM-RPA | a bonded host back from a new resolvable private address is recognised and encrypts with its bond (Vol 3 Part C 10.8; Part H 2.2.2) | — | `ble_bond.c` `ble_bond_peer_known()`, `ble_bond_find_ltk()` | CONF SM-RPA-RECONNECT, ADV-BONDED-FILTER | passes |
| SM-BOND | bonding as responder; the bond used at reconnection | GAP/BOND/BON/BV-03-C | `ble_bond.c` | CONF SM-RPA-RECONNECT; SCEN `sc-update-lead1-*` (reconnection) | passes |
| SM-NO-BONDING | a central that does not ask for bonding: no bond stored | GAP/BOND/NBON/BV-03-C | `ble_smp.c` `pairing_req()` | — | not tested |
| SM-SECURITY-REQUEST | the peripheral asks for security: a server that reconnects to send a notification needing security initiates or requests encryption first (Vol 3 Part C 10.3.1.1); HOGP 1.0 6.1 recommends the Security Request | SM/PER/PIS/BV-01-C, BV-02-C; GAP/BOND/BON/BV-01-C | `ble_smp.c` `security_request()`: to a bonded profile's host without LL_ENC_REQ 1 s after the link started | CONF SM-SECURITY-REQUEST | passes |
| SM-SECURITY-REQUEST-TIMER | the Security Request, queued for transmission, resets the 30 s timer; a host that neither encrypts nor sends SMP within it gets no further SMP on that link; a Pairing Request within it starts the pairing's own timer (3.4) | — | `ble_smp.c` `security_request()`, `ble_smp_poll()`, `ble_smp_rx()` | CONF SM-SECURITY-REQUEST-TIMER, SM-SECURITY-REQUEST-TIMEOUT, SM-SECURITY-REQUEST-NEXT-LINK | passes |
| SM-PAIR-ON-ENCRYPTED | a new pairing on an encrypted link, then the new key | — | `ble_smp.c`, `ble_link.c` | CONF SM-PAIR-ON-ENCRYPTED (a second Passkey Entry pairing, its key taken by a pause, the bond's key at the next connection) | passes |
| GAP-SEC-LEVEL | HID characteristics need security mode 1 level 2 or 3; on a link that is not encrypted, Insufficient Authentication (0x05) when the server holds no key for the client, Insufficient Encryption (0x0f) when it does; on an encrypted link whose key lacks the needed authentication, Insufficient Authentication (HOGP 1.0 6.1; Vol 3 Part C 10.3.1) | GAP/SEC/AUT/BV-11-C, BV-14-C, BV-23-C; GAP/SEC/SEM/BV-21-C, BV-22-C, BV-37-C to BV-40-C | `ble_att.c` `access_error()`, `notify()`; `ble_link.c` `ble_link_peer_bonded()` | CONF HIDS-MAP-NEEDS-ENC, HIDS-READ-NEEDS-ENC, HIDS-WRITE-NEEDS-ENC (no bond: 0x05), HIDS-MAP-BONDED-UNENCRYPTED (the bonded host before encrypting: 0x0f); air parity (the Report Map's answer before pairing equals the stock's, 0x05) | passes |
| GAP-AUTHENTICATED | with Passkey Entry, HID input needs an authenticated key: a Just Works bond gets no notifications | — | `ble_att.c` `link_secure()` | the SC test plan's I7 scenario (the keyboard emulator's `sim/scenarios/sc-i7-unauth-bond.txt`) | not run here |

## ATT and GATT (Vol 3 Parts F and G)

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| ATT-MTU | Exchange MTU answered (3.4.2) | GATT/SR/GAC/BV-01-C | `ble_att.c` `ble_att_rx()` | CONF ATT-MTU | passes |
| GATT-PRIMARY | Discover All Primary Services; Read By Group Type ends with Attribute Not Found (Part G 4.4.1) | GATT/SR/GAD/BV-01-C | `ble_att.c` `read_by_group()` | CONF GATT-PRIMARY | passes |
| GATT-BY-UUID | Discover Primary Service by UUID (4.4.2) | GATT/SR/GAD/BV-02-C | `ble_att.c` `find_by_type()` | CONF GATT-FIND-BY-TYPE | passes |
| GATT-INCLUDED | Find Included Services: none (4.5.1) | GATT/SR/GAD/BV-03-C | `ble_att.c` `read_by_type()` | CONF GATT-INCLUDED | passes |
| GATT-CHARS | Discover All Characteristics, by UUID (4.6) | GATT/SR/GAD/BV-04-C, BV-05-C | `ble_att.c` `read_by_type()`, `char_decl()` | CONF GATT-CHARACTERISTICS | passes |
| GATT-DESCS | Discover All Characteristic Descriptors (4.7.1) | GATT/SR/GAD/BV-06-C | `ble_att.c` `find_info()` | CONF GATT-DESCRIPTORS | passes |
| GATT-READ | read a value, by UUID, long (Read Blob), a descriptor (4.8) | GATT/SR/GAR/BV-01-C, BV-03-C, BV-04-C, BV-06-C | `ble_att.c` `read()`, `read_by_type()` | CONF GAP-NAME, GATT-READ-BY-UUID, HIDS-MAP-CONTENT, HIDS-REPORTS | passes |
| GATT-READ-ERRORS | read not permitted, invalid handle, invalid offset, insufficient encryption or authentication (3.4.4) | GATT/SR/GAR/BI-01-C, BI-02-C, BI-04-C, BI-07-C, BI-08-C, BI-13-C, BI-14-C, BI-16-C | `ble_att.c` `readable()`, `read()` | CONF ATT-READ-NOT-PERMITTED, ATT-INVALID-HANDLE, ATT-BLOB-OFFSET, HIDS-MAP-NEEDS-ENC | passes (insufficient authentication on a Just Works key: not run here) |
| GATT-KEY-SIZE | insufficient encryption key size | GATT/SR/GAR/BI-05-C, BI-11-C, BI-17-C, GAW/BI-06-C | — | — | does not apply: no attribute asks for a key size |
| GATT-WRITE | Write Request, Write Command, descriptor write (4.9) | GATT/SR/GAW/BV-01-C, BV-03-C, BV-08-C | `ble_att.c` `write()` | CONF HIDS-NOTIFY (CCCD), HIDS-WRITE-NEEDS-ENC (the write itself) | passes |
| GATT-WRITE-ERRORS | invalid handle, write not permitted, value too long, insufficient authentication (3.4.5) | GATT/SR/GAW/BI-02-C, BI-03-C, BI-05-C, BI-32-C, BI-39-C | `ble_att.c` `write()` | CONF ATT-WRITE-INVALID-HANDLE, ATT-WRITE-NOT-PERMITTED, ATT-WRITE-TOO-LONG, HIDS-WRITE-NEEDS-ENC | passes |
| ATT-UNSUPPORTED | a request not supported: Request Not Supported, handle 0; a command not supported: nothing (3.4.1.1, 3.3) | GATT/SR/UNS/BI-01-C, BI-02-C | `ble_att.c` `ble_att_rx()` | CONF ATT-NOT-SUPPORTED, ATT-WRITE-CMD-SILENT | passes |
| ATT-INVALID-PDU | a malformed request: Invalid PDU (Table 3.4) | — | `ble_att.c` | CONF ATT-INVALID-PDU | fails: BLE-ATT-INVALID-PDU |
| ATT-GROUP-TYPE | Read By Group Type for 0x2801 with none: Attribute Not Found (3.4.4.9) | — | `ble_att.c` `read_by_group()` | CONF ATT-GROUP-SECONDARY | fails: BLE-ATT-GROUP-TYPE |
| GATT-NOTIFY | notifications with the CCCD set, bonded and not (4.10) | GATT/SR/GAN/BV-01-C, BV-03-C | `ble_att.c` `notify()` | CONF HIDS-NOTIFY; SCEN `sc-passkey-*` | passes |
| GATT-CCCD-BOND | a bonded client's CCCDs kept for its next connection (Part G 3.3.3.3) | — | `ble_att.c` `ble_att_bond_encrypted()`, `ble_bond.c` | CONF GATT-CCCD-BOND | passes |
| GATT-SERVICE-CHANGED | Service Changed exists unless the services can never change (Part G 7.1); indicated to a bonded client that configured it when the database differs from the one it saw, at its reconnection, until it confirms (Part G 2.5.2); its CCCD kept per bond | GATT/SR/GAS/BV-01-C | `ble_att.c` `db[]`, `ble_att_db_hash()`, `ble_att_bond_encrypted()`, `ble_att_cccd_poll()`; `ble_bond.c` (the hash per profile in the state record) | CONF GATT-SERVICE-CHANGED; SCEN `sc-gatt-update-v21-ble`, `-v75-ble`, `-v65-ble` (host kind `android`: an update that moves the handles, the host discovers again and the keys arrive) | passes |
| ATT-TIMEOUT | every request answered within the 30 s transaction timeout (3.3.3) | — | `ble_att.c`, `ble_link.c` | CONF ATT-RSP-RING-FULL | passes |

## GAP characteristics (Vol 3 Part C 12)

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| GAP-NAME | Device Name readable, the advertised name (12.1) | GAP/GAT/BV-16-C; GAP/IDLE/NAMP/BV-02-C | `ble_att.c` `value_of()`, `ble_adv.c` `ble_device_name()` | CONF GAP-NAME | passes |
| GAP-APPEARANCE | Appearance 0x03C1 (12.2) | GAP/GAT/BV-17-C | `ble_att.c` `db[]` | CONF GAP-APPEARANCE | passes |
| GAP-PPCP | Peripheral Preferred Connection Parameters, 8 octets in range (12.3) | GAP/GAT/BV-04-C | `ble_att.c` `ppcp` | CONF GAP-PPCP, L2CAP-PARAM-REQ (equal to the request) | passes |

## HID over GATT, HID Service, Battery Service, Device Information

| id | requirement | test cases | code | tests | status |
|---|---|---|---|---|---|
| HOGP-SERVICES | one HID Service, a Battery Service, a Device Information Service with PnP ID (HOGP 1.0 3) | HOGP/HD/SGGIT/SER/BV-01-C, BV-03-C, BV-04-C; DIS/SR/SGGIT/SER/BV-01-C | `ble_att.c` `db[]` | CONF GATT-PRIMARY | passes |
| HOGP-BOND | the HID device bonds (HOGP 1.0 6.1) | — | `ble_smp.c` | CONF SM-PASSKEY-PAIR, SM-RPA-RECONNECT | passes |
| HOGP-CONN-PARAMS | any valid connection parameters of the host accepted; the L2CAP request used (HOGP 1.0 5.1.2) | — | `ble_conn.c`, `ble_link.c` | CONF LL-CONN-UPDATE, LL-TRANSMIT-WINDOW, L2CAP-PARAM-REQ | passes |
| HIDS-MAP | Report Map: read (long), needs encryption, equal to the report descriptor | HIDS/SR/SGGIT/CHA/BV-01-C | `ble_att.c` `db[]` | CONF HIDS-MAP-CONTENT, HIDS-MAP-NEEDS-ENC | passes |
| HIDS-INFO | HID Information: 4 octets | HIDS/SR/SGGIT/CHA/BV-03-C | `ble_att.c` `hid_info` | CONF HIDS-INFO | passes |
| HIDS-REPORTS | input reports Read and Notify with a CCCD and a Report Reference; the output report Read, Write, Write Without Response, no CCCD; non-zero report IDs (HIDS 1.0 2.x) | HIDS/HD/DEC/BV-02-C, DES/BV-01-C, DES/BV-02-C, DR/BV-01-C, DR/BV-02-C, DW/BV-01-C, CR/BV-02-C, LCR/BV-02-C, LCR/BV-03-C, CW/BV-02-C, CW/BV-07-C | `ble_att.c` `db[]` | CONF HIDS-REPORTS, HIDS-NOTIFY | passes |
| HIDS-OUTPUT-REPORT | the keyboard LED output report: its Report Reference (ID 1, output) names an output report of the Report Map (`ZMK_HID_INDICATORS`, on in the boards' defconfigs), and a write reaches ZMK's HID indicators for the profile in use | — | `ble_att.c` `db[]`, `write()`; `ble_hid.c` `ble_hid_leds()` | SCEN `host-leds-v75`, `host-leds-v65`, `host-leds-v21` (the Caps Lock or Num Lock LED follows the host over BLE, also on a second profile) | passes |
| HIDS-NOTIFY | input report notifications (HIDS 1.0) | HIDS/HD/CN/BV-01-C | `ble_att.c` `ble_att_notify_keyboard()` | CONF HIDS-NOTIFY | passes |
| HIDS-MULTI-INPUT | keyboard and consumer input reports notified each on its own handle | HIDS/HD/SP/BV-01-C | `ble_att.c` `ble_att_notify_keyboard()`, `ble_att_notify_consumer()` | SUITE `mine/knob-low-power-bt`, SCEN `knob-low-power-v21-ble` (the knob's VOLU before the idle time); CONF HIDS-NOTIFY | passes |
| HIDS-PROTOCOL-MODE | Protocol Mode: Read and Write Without Response, Report Protocol after the connection | HIDS/SR/SGGIT/CHA/BV-04-C; HIDS/HD/CW/BV-09-C | `ble_att.c` `ble_att_reset()` | CONF HIDS-PROTOCOL-MODE | passes |
| HIDS-BOOT | a keyboard's Boot Keyboard Input and Output Reports (HIDS 1.0 Table 2.1 C.2), and boot protocol when written: the boot keyboard input report on its own characteristic, the LED byte from the boot output report, Report Protocol again at each connection | HIDS/HD/CW/BV-08-C | `ble_att.c` `db[]`, `ble_att_notify_keyboard()`, `write()` | CONF HIDS-BOOT-REPORTS; SCEN `sc-boot-protocol-v21-ble`, `-v75-ble`, `-v65-ble` | passes |
| HIDS-CONTROL-POINT | HID Control Point: Write Without Response only; Suspend and Exit Suspend written | HIDS/SR/SGGIT/CHA/BV-02-C; HIDS/HD/CW/BV-10-C, BV-11-C | `ble_att.c` `db[]` | CONF ATT-READ-NOT-PERMITTED (its properties) | passes (the writes themselves: not tested) |
| HIDS-SECURITY | HID characteristics need an encrypted link (HOGP 1.0 6.1): every HID value and the input reports' CCCDs, with Passkey Entry an authenticated key; the Report References readable | GAP/SEC/AUT/BV-23-C | `ble_att.c` `db[]` (`A_E`), `access_error()` | CONF HIDS-MAP-NEEDS-ENC, HIDS-READ-NEEDS-ENC, HIDS-WRITE-NEEDS-ENC; HIDS-INFO and HIDS-PROTOCOL-MODE read after the pairing | passes |
| BAS-LEVEL | Battery Level: Read and Notify, 0-100, a CCCD | BAS/SR/SGGIT/SER/BV-01-C, CHA/BV-13-C | `ble_att.c` `db[]`, `ble_battery.c` | CONF BAS-LEVEL, GATT-READ-BY-UUID | passes |
| BAS-NOTIFY | a change while connected is notified (BAS 1.1 3.1.1) | BAS/SR/CN/BV-01-C, CON/BV-01-C | `ble_att.c` `ble_att_battery_level()` | SUITE `mine/bas-reconnect-bt`, SCEN `bas-reconnect-v21-ble` (their first check: 49 % notified) | passes |
| BAS-RECONNECT | a change while the bonded host was away is notified at its return (BAS 1.1 3.1.1) | BAS/SR/CN/BV-21-C | `ble_att.c` | SUITE `mine/bas-reconnect-bt`, SCEN `bas-reconnect-v21-ble` | fails: BLE-BAS-RECONNECT |
| DIS-PNP | PnP ID: 7 octets, vendor ID source 1 or 2 (DIS 1.1 3.9; HOGP 1.0 3.3.2) | DIS/SR/SGGIT/CHA/BV-09-C | `ble_att.c` `pnp_id` | CONF DIS-PNP-ID | passes |

## Summary

| status | rows |
|---|---|
| passes (for what the row's tests cover) | 87 |
| fails (a row of `known-gaps.md`) | 7 |
| tested on the Just Works build only | 4 |
| not tested, or not run here | 7 |
| not followed (a "should"), does not apply | 2 |

CONF gives 72 passes and 5 known gaps on both boards' Passkey Entry images built from this tree, the Python and the Go port printing the same lines.
