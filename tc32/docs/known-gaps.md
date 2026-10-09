# Known gaps

Every place where the code or its documents say that something is not done, done in part, assumed, or left out, sorted into three classes:

- **(a)** a gap that can make the product misbehave with a real host or user: interoperability, a requirement of a specification, safety, lost input or data;
- **(b)** a deliberate scope limit, documented with its reason;
- **(c)** wording only (a stale or loose phrase); these are counted, not listed.

An (a) gap's test fails while the gap is open; it lives where tests of its kind live and is registered as an expected failure, so the suites stay green while the gap is counted (where no test is written yet, the row says what it would be and why it is not there):

| where | list | what a listed test prints |
|---|---|---|
| the whole-keyboard emulator (not published): `sim/zmk_ble_conformance.py` and its Go port `go/cmd/zmk-ble-conformance`; the suite `suite/v75pro` (`report.py`) | `known-gaps.txt` at the top of that emulator | `XFAIL CHECK (GAP)`; the suite report lists the job as `KNOWN GAP` |
| TC32-devtools: `checks/ble_model_gaps_check.py` and `go/cmd/ble-model-gaps` (run by `checks/run_checks.sh`) | `checks/known-gaps.txt` | `XFAIL CHECK (GAP)` |

A listed test that passes prints `FIXED` (the suite report: `GAP FIXED`) and fails its run until its line is removed from the list and its row here is closed. A gap is closed by deleting its row.

Columns: **where** is the file and function; **reference** the requirement it misses (Bluetooth Core Specification 5.4 as "Vol/Part/section", the profile and service specifications, the Bluetooth SIG test case that covers it); **test** the test that fails while the gap is open. The requirements the BLE stack does meet, and those no test checks yet, are in `ble-conformance.md`.

## BLE stack (`tc32/src/ble`)

| id | where | what goes wrong | reference | test | status |
|---|---|---|---|---|---|
| BLE-L2CAP-REASSEMBLY | `ble_link.c` `ble_link_rx_data()`: a frame longer than its first LL PDU is put together for SMP only; a start PDU shorter than the L2CAP header is dropped | An ATT or signalling frame that the central's controller splits over several LL PDUs is dropped; the request gets no answer (ATT: 30 s timeout, then no ATT on that link). Controllers split a frame when their buffers are smaller than the PDU; a 23-octet ATT PDU usually goes in one. | Vol 3 Part A 7.2 ("Fragmentation and Recombination may be applied to any L2CAP PDUs"); LL/DFL/PER/BV-02-C | `zmk-ble-conformance` L2CAP-REASSEMBLY-ATT | open |
| BLE-BONDS-BEFORE-SERVICE-CHANGED | `ble_att.c` `db[]`, `ble_bond.c`: a host bonded with an image that had no Service Changed never turned its indications on | Such a host keeps the database it discovered then (Android, Windows; AND-GATT-05/11, WIN-GATT-04 in the host rules): handles 1-35 are unchanged, so the keys arrive, but it never learns of the Report Map's LED output report, the boot keyboard reports or the Service Changed characteristic itself. The host's Caps Lock or Num Lock then never reaches the keyboard over BLE, and later changes to the database are not indicated to it, until the user removes the keyboard on the host and pairs again (or clears the profile, below). No firmware can reach a host that never enabled the indication. | Vol 3 Part G 2.5.2, 7.1 (indications only to a client that configured them) | none yet: a scenario that bonds a caching host with an image before Service Changed, takes the update and expects the LED output report to stay unwritten (it documents the behaviour; no fix exists) | open: re-pair once |
| BLE-BAS-RECONNECT | `ble_att.c` `ble_att_battery_level()`, `notify()`: the level is stored while no link is up; nothing records what the client last saw | A battery level that changes while the bonded host is away (the charge completes) is not notified when it comes back; the host shows the old level until the next change. | BAS 1.1 3.1.1 ("Upon reconnection with a bonded GATT Client, if the Battery Level characteristic is configured for notifications and the value ... has changed while the device was disconnected, then the Battery Level characteristic shall be notified"); BAS/SR/CN/BV-21-C | suite `mine/bas-reconnect-bt` (V75 Pro board model), `sim/scenarios/bas-reconnect-v21-ble.txt` (V21 board model) | open |
| BLE-LL-RESPONSE-TIMEOUT | `ble_link.c` `ble_link_enc_req()`, `ble_conn.c`: no procedure response timer | When the central never sends LL_START_ENC_RSP, the link stays with encryption half started and every data PDU refused (`-ENOBUFS`) until the central leaves; the Core ends it after 40 s with reason 0x22. Only a misbehaving central or a lost link reaches this. | Vol 6 Part B 5.2; LL/SEC/PER/BI-01-C | `zmk-ble-conformance` LL-PROC-TIMEOUT | open |
| BLE-LL-INVALID-LENGTH | `ble_link.c` `ble_link_enc_req()` (`if (len != 23U) return;`) | An LL_ENC_REQ of a wrong length gets no answer at all, so the central waits for its 40 s procedure timeout. (For LL_CONNECTION_UPDATE_IND and LL_CHANNEL_MAP_IND of a wrong length, which `ble_ll_ctrl_rx()` drops too, no answer is allowed.) Only a malformed PDU reaches this. | Vol 6 Part B 2.4.2 ("If it does not continue the procedure, it shall respond with an LL_UNKNOWN_RSP PDU or ... an LL_REJECT_IND"); LL/PAC/PER/BI-01-C | `zmk-ble-conformance` LL-CTRL-INVALID-LENGTH | open |
| BLE-ATT-GROUP-TYPE | `ble_att.c` `read_by_group()`: any group type other than 0x2800 gets Unsupported Group Type | ATT_READ_BY_GROUP_TYPE_REQ for 0x2801 (secondary service, a grouping attribute of GATT) is answered Unsupported Group Type instead of Attribute Not Found. No host is known to send it; Zephyr's server answers Attribute Not Found. | Vol 3 Part F 3.4.4.9; Part G 3.1 | `zmk-ble-conformance` ATT-GROUP-SECONDARY | open |
| BLE-ATT-INVALID-PDU | `ble_att.c` `find_info()`, `read_by_type()`, `read_by_group()`, `read()`, `write()`: a request of the wrong length is answered 0x0d | A malformed request gets Invalid Attribute Value Length (0x0d) where ATT defines Invalid PDU (0x04). Hosts do not send malformed requests. | Vol 3 Part F 3.4.1.1, Table 3.4 | `zmk-ble-conformance` ATT-INVALID-PDU | open |
| BLE-LOW-BATTERY | `ble_battery.c`: "Not done here yet: the low-battery indicator" | The keyboard shows nothing before it reaches 0 % and goes to deep sleep; the host still sees the level. | none | none yet: what the indicator shows is not specified | open |
| BLE-KNOB-LOW-POWER | `ble_sleep.c`: "The knob's pins are not wake pads: a turn while suspended is seen only if the chip is awake for an event at the time" | In the connected low-power state (`TLSR_BLE_IDLE_S`, 300 s without a key) a knob detent made while the chip is suspended is lost: no volume change reaches the host. | none | suite `mine/knob-low-power-bt` (V75 Pro board model), `sim/scenarios/knob-low-power-v21-ble.txt` (V21 board model): a detent before the idle time gives VOLU, one in the low-power state none | open |

## USB, the 2.4 GHz link, flash and settings (`tc32/src`, `tc32/drivers`, `app/`)

| id | where | what goes wrong | reference | test | status |
|---|---|---|---|---|---|
| USB-CLEAR-HALT-TOGGLE | `tc32/drivers/usb/device/usb_dc_b87.c`: feature requests are answered by the controller ("SET_FEATURE ... does not reach the stack"); the driver chooses the data PID itself (`ep_arm()`) | After a host's CLEAR_FEATURE(ENDPOINT_HALT) on the keyboard's IN endpoint the PID is not reset to DATA0, so the host can drop the next report as a repeat (a key press or release lost). Whether the controller resets the PID by itself is not known. | USB 2.0 9.4.5, 8.6.4 | none yet: needs the controller's behaviour on hardware, and a host model step that sends CLEAR_FEATURE | open, unverified |
| USB-GET-REPORT-UPDATE | `tc32/src/tlsr_usb_ota.c`: the update interface has `set_report` only | GET_REPORT on the update interface stalls. A tool that reads its input report with GET_REPORT gets an error. | HID 1.11 7.2.1 | none yet: `tc32/tests/usb-sim` (GET_REPORT(Input) on that interface must not stall) | open |
| USB-RESUME-LATE | `tc32/src/usb_suspend.c`: "A host resume that starts in those 20 ms can still meet a suspend; the chip then answers at the next timer wake, up to 1 s later" | A resume in that window is answered up to 1 s late: a host that sends its next request after the 10 ms resume recovery gets no answer for that long and may reset the device. | USB 2.0 7.1.7.7 | none yet: a scenario with a resume inside the 20 ms window and the first SETUP answered within 10 ms | open |
| USB-SUSPEND-CURRENT | `tc32/Kconfig` `TLSR_USB_SUSPEND_DELAY_MS` (8000): the chip suspends 8 s after the bus | Full current for 8 s after the host suspends the bus, where USB asks for suspend current within 10 ms. | USB 2.0 7.1.7.6, 7.2.3 | none | accepted: kept as configured |
| FLASH-ERASE-IRQ-OFF | `tc32/src/tlsr_spi_flash.c`: interrupts stay off for a whole sector erase (up to hundreds of ms) | During an erase (settings moves, the backlight save, the RNG seed log, the bond log, the boot counter) EP0 is not served and a key pressed and released within it is not seen (the V21's scan runs from a timer interrupt). | USB 2.0 9.2.6.4 | none yet: `tc32/tests/usb-sim` with a flash hook that holds the lock for 300 ms during a GET_DESCRIPTOR | open |
| SETTINGS-LOG-MISREAD | `tc32/src/tlsr_settings_log.c`: "A record that does not check ends the log; the next save then moves it" | One misread record at boot ends the log there; the next save moves only the records before it, so every later setting (Studio keymap edits, the kept layer, the backlight, the US-JIS mode) is lost. | none | none yet: `tc32/tests/settings-log` with record k read wrong once during the scan, then a save, then a scan: the records after k load | open |
| P24-VIA | `tc32/src/ble/p24.c` `command()`: VIA requests (0xb0) are not answered; 0xb1, 0xb2 and 0xb5 do nothing | A configuration tool used through the dongle gets no answer. What the stock firmware answers is not known. | the dongle protocol (not documented) | none yet: needs the stock's answer captured | open |
| OTA-TOOL-RESULT | `tc32/scripts/telink_ota.py`: "done" printed when the end command got no answer; `dev.write()` results not checked | An update whose end command is lost reports success; the keyboard keeps the image it had (no damage). | none | none yet: `tc32/tests/tool` with a fake device that drops the end report | open |

## Boards and keymaps (tc32-keyboards)

| id | where | what goes wrong | reference | test | status |
|---|---|---|---|---|---|
| BOARD-CHORD-V75 | `boards/cidoo/cidoo_v75pro/Kconfig.defconfig`: "The chord's active-low reading of these pins has not run on hardware" | One of the three ways back to the other image (the power-on chord) is not verified on the V75 Pro and V65 V3 boards; the boot guard and the go-back key remain. | none | a hardware check | open, hardware |
| BOARD-SLEEP-PA7 | `cidoo_v75pro/deep_sleep_pads.c`, `cidoo_v65v3/deep_sleep_pads.c`: "Whether a GPIO output keeps its level through the deep sleep is not documented ... has not been checked" | If PA7's drive is not held, PA0 wakes the chip at once in the BT position (a sleep and wake loop that drains the battery); if it is, about 0.33 mA flows for the whole sleep. | DS-TLSR8278 (silent) | a bench measurement; the emulator's `--deep-outputs release` runs the release case | open, hardware |
| BOARD-STACKS | `boards/cidoo/*/cidoo_*_studio.conf`: "stacks sized for the deepest use seen in the emulator"; `CONFIG_STACK_SENTINEL` is off | A path the scenarios do not take (a pairing that fails a certain way, a long Studio name) can overflow a stack; the TC32 has no stack protection, so it corrupts memory or hangs. | none | the suite runs every job with `--stacks` (a quarter of each stack unused on the paths it takes); the paths it does not take are untested | open |
| BOARD-REVISION | `INSTALL.md`: an unknown hardware revision or a keyboard that answers as the same product is not detected | The update proceeds on such a unit (dead keys, another flash part); the ways back remain. | none | none yet: `telink_ota.py` refusing an unknown JEDEC ID or signature (`tc32/tests/tool`) | open |

## zephyr-tc32

The port's own gaps are in zephyr-tc32's `TC32-gaps.md`, next to `TC32.md` and its list of what the port does not provide.

## The test models (TC32-devtools, the whole-keyboard emulator)

Where a model accepts what a chip or a host would not, a firmware defect of that kind passes every scenario.

| id | where | what it hides | test | status |
|---|---|---|---|---|
| MODEL-CRC-INIT | `ble_radio.py`: the CRC init register (0x424-0x426) is recorded and compared with nothing | A link layer that programs a wrong CRC init (byte order, the previous connection's): every packet fails its CRC on a chip. | `ble_model_gaps_check.py` MODEL-CRC-INIT | open |
| MODEL-AA-PER-CONNECTION | `emulator/ble_central.py`, the keyboard emulator's `sim/ble_host.py`: every connection uses access address 0x71764129 and CRC init 0x334455 | A link layer that keeps the last connection's access address or CRC init reconnects in the emulator and fails with every host. | `ble_model_gaps_check.py` MODEL-AA-PER-CONNECTION | open |
| MODEL-MIC-DEFAULT | `ble_central.py` (`mic_ends` off unless `bt-air`): a packet whose MIC fails is counted and taken | An encryption defect (a counter wrong after a retransmission) passes the scenarios that run without `bt-air`; every host ends the link. | none yet: an end-of-script check that fails on any MIC failure the host counted | open |
| MODEL-ONE-UPDATE | `ble_central.py` `l2cap()`: only the first accepted connection parameter request of a connection is applied | Later requests (after a low-power change, after pairing) are answered "accepted" and never applied, so a keyboard that takes "accepted" for "applied" passes. | none yet: a scenario with two requests on one link | open |
| MODEL-HOST-PUBLIC | `sim/ble_host.py`: the host always connects from its public address | Reconnection of a host that uses resolvable private addresses (the address changes, the IRK resolves it) is not run in the scenarios. | `zmk-ble-conformance` SM-RPA-RECONNECT runs it (passes); no scenario step | open in the runner |
| MODEL-CENTRAL-LL | `ble_central.py`: the scenarios' host never sends LL_PHY_REQ, LL_LENGTH_REQ, LL_PING_REQ, LL_PAUSE_ENC_REQ, a channel map update or a connection update of its own | Answers to procedures hosts start on their own are untested in whole-keyboard runs. | `zmk-ble-conformance` LL-PHY-REQ, LL-CONN-PARAM-REQ, LL-LENGTH-REQ-ENC, LL-ENC-PAUSE, SM-PAIR-ON-ENCRYPTED (`pause_encryption()`, and a pairing on an encrypted link), LL-INSTANT-PASSED-* send them | open in the runner |
| MODEL-USB-REQUESTS | the keyboard emulator's `sim/keyboard.py` `Host.enumerate()`: the USB host never sends SET_IDLE, string 0, the device qualifier, BOS, GET_STATUS, GET_REPORT, SET_PROTOCOL or CLEAR_FEATURE | ZMK's and the driver's answers to those requests are untested in whole-keyboard runs. | none yet: host profiles for `usb-host on` | open |
| MODEL-BATTERY | the keyboard emulator's `KEYBOARD.md`: the battery and charging are not modelled beyond the ADC code and the charger pins | Battery levels over time, charging curves. | the `adc` and `pad` steps cover the level's rules (`bas-reconnect` scenarios) | open |

## Untested requirements

Requirements of `ble-conformance.md` that no test checks on the Passkey Entry images; each is a test to write.

| row | what is not checked | where a test would go |
|---|---|---|
| LL-ENC-UNEXPECTED | an LL_VERSION_IND or an unencrypted data PDU during the encryption start | CONF, with a central that sends one there |
| L2-INVALID-LENGTH | a signalling command whose length field is wrong | CONF |
| SM-PASSKEY-PEER-FAIL | the host's Pairing Failed while the keyboard waits for the passkey: the pairing ends and the keys type again | CONF, or a scenario with a host fault that sends Pairing Failed |
| SM-AUTHREQ-RFU | reserved AuthReq bits in the Pairing Request ignored | CONF, with a Pairing Request built by the check |
| SM-P256 | the P-256 tests on the images' engine (they run in zephyr-tc32's twister, not here) | zephyr-tc32 `tests/crypto/tc32_p256` |
| SM-NO-BONDING | a pairing without bonding stores no bond | CONF, with a Pairing Request built by the check |
| GAP-AUTHENTICATED | a Just Works bond on a Passkey Entry image gets no notifications | a scenario from the SC test plan's I7 |
| SM-JW, SM-PUBKEY, SM-DHKEY, SM-TIMEOUT, the scenarios of ADV-NON-DISCOVERABLE and SM-REPEATED's doubling wait | tested on the Just Works build only: their scenarios pair with Just Works, which the Passkey Entry images refuse | Passkey Entry forms of those scenarios, as `sc-neg-repeated-passkey-v21-ble` is of `sc-neg-repeated` |
| parts of passing rows | a connection never established (6 events), a CONNECT_IND with a bad CRC and a packet with a corrupted header (CRC errors are not modelled), a 4 s connection interval, insufficient authentication on a Just Works key, the HID Control Point's Suspend and Exit Suspend writes | CONF |

## Scope limits (b)

What is left out on purpose, with the reason the code or its documents give:

- Legacy pairing is refused (Pairing Failed 0x03); only LE Secure Connections pairs (`ble_smp.c`). Passkey Entry is the default, Just Works a build option.
- No data length extension, no 2M or Coded PHY, no Connection Parameters Request procedure, no channel selection algorithm #2, no extended advertising, no LE Ping in the feature set (it is answered), no peripheral-initiated procedures (`ble_ll_ctrl.c`): what a keyboard's reports need.
- ATT_MTU stays 23 (`ble_att.c`): a keyboard report fits.
- The keyboard distributes no key of its own: its address is random static and never resolvable, so an IRK would tell a host nothing (`ble_smp.c`).
- One bond per profile, three profiles (`ble_bond.c`).
- A profile with a bond advertises with the flags LE General and Limited Discoverable cleared (0x04) and takes a CONNECT_IND only from its bonded host (`ble_adv.c`), so that no other host can take the profile over. A host that removed the keyboard from its list therefore cannot find it again (Android and Linux list only discoverable advertisers; host rules AND-DISC-05, BZ-DISC-02). Holding the profile's key 3 s (Fn + Q, W, E on the V75 Pro and V65 V3; Fn + KP1-KP3 on the V21) clears its bond: the profile then advertises discoverable from a new address, and the host pairs with it as with a new keyboard.
- While a profile without a bond waits for a pairing (`TLSR_BLE_PAIRING_TIMEOUT_S`, 60 s), it advertises LE General Discoverable, not Limited Discoverable, which HOGP 1.0 5.1.1 recommends ("should"): hosts list general discoverable advertisers in every discovery, and the 60 s window ends the advertising as limited discovery would (`ble_adv.c`).
- The GATT service sits at handles 48-51, above the other services (`ble_att.c` `GATT_HANDLE`), so that Service Changed keeps its handle while later images add up to seven attributes below it; more than that moves Service Changed, and hosts that kept its old handle would miss its indication.
- Connection parameters (`TLSR_BLE_CONN_*`): 7.5 ms, latency 44, 3 s asked for 1 s after the connection; a host that rejects them (Apple's hosts reject anything outside their Accessory Design Guidelines' limits) is asked once more, 30 s later, for 15 ms, latency 22, 6 s (`TLSR_BLE_CONN_FALLBACK_*`); the Connection Parameters Request procedure of the link layer is not used (`ble_link.c`, `ble.c`).
- The key that wakes the keyboard from deep sleep is not reported (`ble_sleep.c`, `p24.c`): the wake is a boot.
- No HID reports over USB in the BT and 2.4G positions; USB there is for tools and Studio (`tc32/Kconfig` `TLSR_USB_ON_REQUEST`).
- Updates arrive as SET_REPORT, about a minute per slot: the stock firmware's protocol (`tc32/README.md`).
- A fault inside the SoC setup is recoverable only over SWS: the boot guard runs right after that setup (`tlsr_boot_guard.c`).
- The 2.4G link does not mark a boot healthy: confirm over USB first (`tc32/README.md`).
- Only GD25LD40C/80C and ZB25WD40B/80B are unlocked for writes; another flash part gets no write (`TLSR_BOOT_GUARD_UNLOCK`).
- `TLSR_BOOT_GUARD_HEALTHY_UPTIME` counts a running image whose USB does not work as healthy (its Kconfig help gives the trade-off).
- A backlight change made within the save delay before a deep sleep or a cable pull is lost: the settings call chain is not put on the link thread's stack (`led_driver_spi.c`).
- Backlight keys are ignored for 3 s after boot: the recovery chord shares a key (`BACKLIGHT_BOOT_KEYS_MS`).
- `RGB_COLOR_HSB` returns -ENOTSUP (`behavior_backlight_rgb.c`); Studio does not offer it.
- Choosing USB without cable power is ignored (`behavior_link_mode.c`).
- A knob move whose middle state is missed is dropped (`detent_encoder.c`).
- A 2.4G consumer record carries one 16-bit usage; mouse and system records are not sent over 2.4G (`p24.c`).
- The stock firmware's other backlight effects and indicators are left out (`led_key_matrix_effects.c`); of its indicators only the host's Num Lock (V21) and Caps Lock (V75 Pro, V65 V3) are shown (`led_key_matrix_effects.c`, `led_driver_spi.c`).
- US-JIS: the same usage on two positions, `&kt` on a substitutable key and Mod-Morph are not supported (`docs/usjis-substitution.md`); Studio can still assign them, which leaves a key held on the host.
- The stock's tri-layer keys (VIA FN_MO13, FN_MO23) become plain `&mo` (tc32-keyboards `gen_*.py`); the keymaps have two layers.
- The stock's suspend delay, the USB interrupt lines (polled until measured), PKE test and RNG measurement images: as their Kconfig help and the board defconfigs say.

## Counts

| class | BLE stack | USB, 2.4G, flash, settings, app | boards and keymaps | test models |
|---|---|---|---|---|
| (a) | 9 | 8 | 4 | 8 |
| (b) | 10 | 15 | 1 | 30 (the model limits the keyboard emulator's `README.md` and `KEYBOARD.md` and TC32-devtools' `README.md` give with their reasons) |
| (c) | about 20 | about 60 | counted with zephyr-tc32's | about 70 |

zephyr-tc32's are counted in its `TC32-gaps.md`.
