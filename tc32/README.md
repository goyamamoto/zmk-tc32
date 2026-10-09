# tc32: ZMK on the Telink TLSR8278

The Zephyr module of zmk-tc32 ([TC32.md](../TC32.md)): the code a TLSR8278 keyboard needs beyond ZMK and zephyr-tc32, and the tools and tests for it. Paths below are relative to this folder. The keyboards themselves (board files, keymaps, their READMEs) are in the [tc32-keyboards](https://github.com/goyamamoto/tc32-keyboards) repository; the CIDOO V75 Pro, V65 V3 and V21 run this firmware.

| Path | What |
|---|---|
| `src/tlsr_slots.c`, `tlsr_slots.h` | The two firmware slots of the Telink boot ROM: the running slot, image checks, the journalled first-sector rewrite that makes the other slot boot (`CONFIG_TLSR_SLOTS`) |
| `src/tlsr_usb_ota.c` | Firmware updates over USB on a second HID interface, with the protocol the keyboards' original firmware uses (`CONFIG_TLSR_USB_OTA`), [below](#firmware-updates-over-usb) |
| `src/tlsr_boot_guard.c` | The watchdog, the boot count and the return to the other slot after 3 boots without a healthy one (`CONFIG_TLSR_BOOT_GUARD`), [below](#the-two-slots-and-the-way-back) |
| `src/behavior_prev_firmware.c` | `&prev_fw` (`zmk,behavior-prev-firmware`): a key that boots the other slot's image |
| `src/tlsr_spi_flash.c`, `tlsr_spi_flash_io.S`, `flash_tlsr_spi.c` | The SPI flash transport and the Zephyr flash driver over it (`CONFIG_TLSR_SPI_FLASH`), [below](#flash-access) |
| `drivers/usb/device/usb_dc_b87.c`, `include/zephyr/drivers/usb/usb_dc_b87.h` | The TLSR8278's USB device controller driver for Zephyr's legacy USB device stack (`CONFIG_USB_DC_TELINK_B87`), in Zephyr's library of legacy device controller drivers: the 8-byte EP0 FIFO, data endpoints 1-4, 7 and 8 IN and 5 and 6 OUT in the 256-byte USB RAM, suspend, resume and remote wakeup, the 1 ms poll from a board's timer interrupt (`USB_DC_TELINK_B87_EXTERNAL_POLL`) or a slower one around a low-power state (`USB_DC_TELINK_B87_SLOW_POLL`), and a register model for host tests (`USB_DC_TELINK_B87_SIM`) |
| `src/usb_suspend.c`, `usb_poll_timer.c` | The chip's suspend while the host suspends USB (`CONFIG_TLSR_USB_SUSPEND`); the 1 ms USB poll from a timer interrupt (`CONFIG_TLSR_USB_POLL_TIMER`) |
| `src/detent_encoder.c` | `cidoo,detent-encoder`: a knob counted per detent (`CONFIG_CIDOO_DETENT_ENCODER`) |
| `src/led_key_matrix.c`, `led_key_matrix.h` | `cidoo,led-key-matrix` and `cidoo,led-key-matrix-backlight`: a key matrix and an RGB backlight on the same columns, multiplexed from Timer1 with the key scan in its own slot (`CONFIG_CIDOO_LED_KEY_MATRIX`; the V21) |
| `src/led_key_matrix_effects.c`, `behavior_backlight_rgb.c` | `zmk,backlight-effects`: wave, breathe and light, colours, brightness levels and speeds on that backlight, kept in flash (`CONFIG_LED_KEY_MATRIX_EFFECTS`, `CONFIG_BACKLIGHT_SAVE`); `zmk,behavior-backlight-rgb`: ZMK's RGB commands for them |
| `src/led_driver_spi.c` | The backlight of the V75 Pro and the V65 V3 through their two LED driver chips (`CONFIG_LED_DRIVER_SPI`); each board's LED wiring table (`led_driver_table.h`, `led_driver_table.h`) is in its board directory |
| `src/battery_adc.c` | `cidoo,battery`: one battery measurement (the ADC, millivolts, percentage, the charger pins) for the BLE level, the status display and the update receiver's battery command (`CONFIG_BATTERY_ADC`) |
| `src/status_display.c`, `behavior_status.c` | `&sts`: the keyboard's state shown on the backlight for a few seconds (`CONFIG_TLSR_STATUS_DISPLAY`); which LEDs show what is each board's part (`v75pro_status.c`, `v65v3_status.c` in the board directories), [below](#the-status-display) |
| `src/ble/` | The own BLE stack (`CONFIG_TLSR_BLE`) and the proprietary 2.4G link to the keyboards' USB dongle (`CONFIG_TLSR_P24`), [below](#ble-and-24g) |
| `src/behavior_link_mode.c`, `behavior_usb_request.c`, `behavior_p24_pair.c` | `&link_mode` (the link chosen by key on a keyboard without a mode switch), `&usb_on` (USB brought up on request in a wireless position), `&p24_pair` (pair the 2.4G link again) |
| `src/tlsr_settings_log.c` | The settings backend in the storage partition, for what ZMK Studio saves (`CONFIG_TLSR_SETTINGS_LOG`) |
| `src/usjis.c`, `usjis_resolver.c`, `behavior_usjis.c`, `docs/` | US-JIS substitution: a US layout typed on a host set to a Japanese keyboard (`CONFIG_ZMK_USJIS`; `docs/usjis-substitution.md`, `docs/usjis-architecture.md`) |
| `src/layer_keep.c`, `tlsr_cpu_left.c`, `tlsr_crash_log.c`, `gpio_matrix_sleep.c` | A layer kept over a boot (the Windows/Mac mode), the CPU share left over for the lowest-priority thread, a crash record for the tool, the matrix's low-power state |
| `compat/` | What ZMK main needs on Zephyr 4.4, a module listed before ZMK's own ([TC32.md](../TC32.md#what-this-fork-changes-in-zmk)) |
| `scripts/build.sh` | Builds a board ([TC32.md](../TC32.md#build)) |
| `scripts/telink_ota.py` | The host side of the updates: `image`, `check`, `flash`, `info`, `confirm`, `battery`, `link`, `p24`, `crash` |
| `scripts/hot_slots.py`, `behavior_ids_check.py`, `backlight_curves.py` | Build checks (the flash cache slots of the key scan's constants; no two behaviours with the same ID) and the backlight's curve tables |
| `boot_path_units.txt`, `scripts/boot_path_check.py` | The compile units of the code executed from reset until the boot is counted and the power-on chord can take the keyboard back to the other slot (the boot guard's early stage), which keep the classic Thumb returns when the rest returns with `pop {..., pc}` (`CONFIG_TC32_POP_PC_RETURNS`) and stay unfolded with identical code folding (`CONFIG_TC32_ICF_SAFE`), and the build check of that code's instructions against the board's recorded listing (`<board>_studio_boot_path.txt` in tc32-keyboards) |
| `tests/` | `slots/`, `tool/`, `usb-sim/`, `settings-log/`, `ble_crypto/`, `usjis/`, `zmk-tests/`, `zmk-tests-tc32/`, `buildcheck/` ([Tests](#tests)); `docker/` holds the Linux image the native_sim tests run in |

## Firmware updates over USB

The keyboards' original firmware takes updates over USB HID output report ID 5, as Telink's OTA over a BLE-style packet. `src/tlsr_usb_ota.c` implements the same protocol on a second HID interface, so that either firmware can install the other, and `scripts/telink_ota.py` is the host side. Installing this firmware the first time needs nothing but the cable: no debug probe, no opening of the case.

The flash holds two 128 KB firmware slots, A at 0x00000 and B at 0x20000. The boot ROM starts the slot whose byte 8 is 0x4b (the `KNLT` word at bytes 8-11). An update is written to the slot not running; while it is written its byte 8 stays 0xff, so a cut transfer never leaves a bootable half image. The end command marks the new slot bootable, clears the flag word of the old slot and reboots. So after the first install the other slot holds the original firmware, and after an update from this firmware it holds the previous build: the image the ways back go to.

The image (`zmk.ota.bin`, made by `telink_ota.py image` from `zmk.bin`): the size word at 0x18 counts the whole image including the CRC-32 at its end and is 16n + 4; the CRC-32 is reflected, init 0xffffffff, no final XOR (`zlib.crc32() ^ 0xffffffff` over everything before it). `telink_ota.py check` verifies both and names the model the image is for.

The report (33 bytes): report ID 5; payload length fields; bytes 9-10 the chunk index or a command (0xff00 version, 0xff01 start, 0xff02 end; this receiver adds 0xff03 confirm, 0xff05 unlock, 0xff06 battery, 0xff07 flash test and a few read-only queries); bytes 11-26 sixteen image bytes at index × 16; bytes 27-28 a CRC-16/MODBUS over bytes 9-26. The device answers each report on its interrupt IN endpoint with the same report, the index set to the next one expected, and a status in byte 29 (0 on success; 1-6 the protocol's error numbers for index, CRC-16, verify, end, size or CRC-32; 7 flash; 9 running slot unclear; 10 confirm without a passed flash test; 12 the running image does not check; 13 an update over an image that checks, not unlocked; 14 confirm while the other slot holds no image that checks).

What this receiver does beyond the protocol:
- **Version command.** The answer carries the boot guard's state: the measured system clock, the watchdog capture, the boot count, flags (confirmed, flash test passed, the planned-reboot mark), whether the other slot's image checks (every way back needs it), whether this image runs from slot B, the CPU share left over and the uptime. `telink_ota.py info` prints it. The command writes nothing.
- **Running-slot check.** Before chunk 0 the receiver checks that the slot the hardware names as running is marked bootable and that 512 bytes of code read through XIP equal the same bytes read from that slot through the flash controller; otherwise nothing is written (status 9).
- **Erase per sector.** Each 4 KB sector is erased when its first chunk arrives. Every chunk is written and read back; a bad CRC-16 is not acknowledged and the transfer goes on.
- **The update gate** (`CONFIG_TLSR_USB_OTA_GATE`, on by default). While the other slot holds an image that checks, the way back, chunk 0 of an update is refused (status 13) unless the unlock command came within `CONFIG_TLSR_USB_OTA_ARM_MS` (10 s) before it. `telink_ota.py flash --overwrite-other-slot` sends the unlock. With nothing that checks there (after a cut transfer, say) an update goes ahead without it: it is then the way back. No key is needed, because this receiver is also the rescue path when the keys do not work.
- **The flash test and the confirm** (0xff07, 0xff03; `telink_ota.py confirm`). A newly installed image must be confirmed within three boots, or the boot guard takes the keyboard back to the other slot ([below](#the-two-slots-and-the-way-back)). The confirm is taken only after the flash test passed in the same boot and only while the other slot's image checks when the confirm arrives. The flash test compacts the bond log's two sectors (the first two of the storage partition) with every erase and write read back; it writes nothing in either slot, so the image may fill its slot. What a confirm shows: the image can erase, write and read its flash, the way back is intact, and, checked by hand before it, the keys that go back work.
- **Battery, link counters, crash record** (0xff06, `telink_ota.py battery`; `link`, `p24`, `crash`): read-only queries for the tools.

Speed: the receiver's HID interface has an IN endpoint only, so each report arrives as a SET_REPORT control transfer; an update of a full slot takes about a minute.

### The tool

`scripts/telink_ota.py` needs the Python `hid` package (hidapi). On macOS run it in a terminal that has Input Monitoring permission (System Settings, Privacy & Security); the tool opens devices shared, since a keyboard interface cannot be opened exclusively without root.

```sh
python3 scripts/telink_ota.py image  zmk.bin zmk.ota.bin               # build.sh does this
python3 scripts/telink_ota.py check  zmk.ota.bin
python3 scripts/telink_ota.py flash  zmk.ota.bin --vid 320f --pid 5055 --product "CIDOO V75" --yes   # over the original firmware
python3 scripts/telink_ota.py info   --vid 1d50 --pid 615e
python3 scripts/telink_ota.py confirm --vid 1d50 --pid 615e
python3 scripts/telink_ota.py flash  zmk.ota.bin --vid 1d50 --pid 615e --yes                      # a newer build
python3 scripts/telink_ota.py flash  original.bin --vid 1d50 --pid 615e --yes --overwrite-other-slot  # the original firmware, over the way back
```

`flash` finds the interface whose report descriptor has output report 5, sends the image only to a keyboard of the model the image is for (the image's model is the product string it holds, the keyboard's the one it answers with; nothing is sent otherwise), sends start, waits for each chunk's acknowledgement with retries, and finishes with the end command, after which the keyboard reboots. To a keyboard running this firmware it first sends the version command and stops when the other slot holds an image that checks, naming `&prev_fw` as the way to go back to that image, unless given `--overwrite-other-slot`. `--dry-run` goes through everything but the writes.

The original firmware of several Cidoo models answers as USB 320F:5055, so for that ID `flash` also needs `--product` with the keyboard's exact product string ("CIDOO V75", "CIDOO V65 V3", "CIDOO V21"). A keyboard running this firmware answers as 1D50:615E with its board's name ("V75 Pro ZMK", "V65 V3 ZMK", "V21 ZMK"). With several keyboards connected under one ID the tool sends nothing unless `--product` picks one.

Before the first install, charge the keyboard: the original firmware's updater can refuse to write on a low battery.

If an installed image does not boot far enough for the boot guard to run, only the SWS debug interface can recover the keyboard. The boot guard runs before any device or thread, so this needs a fault in the SoC setup itself.

## The two slots and the way back

After the first install the other slot holds the original firmware with its flag word cleared; after an update from this firmware, the previous build. `src/tlsr_slots.c` makes that image boot again without transferring it: it cross-checks the running slot, checks the other image (size word and CRC-32, with bytes 8-11 taken as `KNLT`), forgets the boot count, restores the other slot's `KNLT` by rewriting its first 4 KB sector, clears the running slot's flag word and reboots. It does nothing when the other slot holds no valid image.

The sector rewrite goes through a journal (a copy of the sector and a record at 0x69000-0x6afff) so that a reset at any point leaves the keyboard bootable: the flag bytes of a slot being rewritten stay 0xff until the rest of the sector is written and read back, the rewritten image must check before its flag is written, and a boot that finds the record finishes the job. Every flash write and erase of the slot code is read back; a write that does not take is an error, never silence.

Three things take the keyboard to the other image:

- **The boot guard** (`src/tlsr_boot_guard.c`, `CONFIG_TLSR_BOOT_GUARD`). It runs from the board's early-init hook, right after the SoC setup and before any device or thread (`CONFIG_TLSR_BOOT_GUARD_EARLY`), with the SoC's early watchdog (4 s) running, and later keeps a 4 s Timer2 watchdog fed from the lowest-priority thread. Each boot the firmware did not ask for adds one to a counter in the flash (0x68000), tagged with the running image's CRC-32; a counter left by another image is dropped. A reboot the firmware asked for (an update, a revert, `&prev_fw`) leaves a mark in analog register 0x3c, which a watchdog or software reset keeps and only a power-on reset clears, so that boot is not counted and does not read the power-on chord. A healthy boot clears the counter: the host configuring the USB device (`CONFIG_TLSR_BOOT_GUARD_HEALTHY=USB`, the default), a bonded BLE host's encrypted link, or 15 s of running (`UPTIME`). **Until the host has confirmed the image (`telink_ota.py confirm`) only the confirm clears it, so every new image must be confirmed within 3 boots, or the keyboard goes back to the other slot.** A confirmed image counts no boot at all: it writes nothing at boot, and no revert starts on its own after a reset nobody asked for; its ways back are the chord and `&prev_fw`. A boot on which no host configures the device (a charger, a KVM switched away, a host asleep) counts only for an unconfirmed image. A count the guard cannot keep (a flash that does not take the write) makes an unconfirmed image go back at once; a revert whose writes do not take stops where it is, with the slots consistent, and the next boot with the flash writable finishes it from the journal. After a revert to the original firmware, that firmware erases the slot it came from within seconds of booting: the revert is one way, and this firmware comes back by a new install.
- **A key chord held at power-on** (`CONFIG_TLSR_BOOT_GUARD_CHORD`; which keys, in each keyboard's README). The early stage reads the two matrix positions three times 1 ms apart and reverts whatever the count. Not on a boot the firmware asked for, so with this firmware in both slots the keys held through a revert do not take the keyboard back and forth. The chord is not the keys of a running binding, and its keys do nothing else when held through the boot that follows. So an image whose USB does not work can still be left without a tool, as long as it reaches the early stage.
- **`&prev_fw`** (`zmk,behavior-prev-firmware`): a key held for a few seconds while the firmware runs (3 s on the V75 Pro and V65 V3, 15 s on the V21, where a shorter hold could follow a slip during a pairing hold). It works when the keyboard scans keys but USB does not, so the update path is closed.

## BLE and 2.4G

`CONFIG_TLSR_BLE` runs a BLE keyboard of its own in one thread (`src/ble/`): advertising (a scan request is answered T_IFS after it, as a host that scans actively needs to list the keyboard), the connection, LE Secure Connections pairing with bonding (Passkey Entry by default: the host shows a passkey, the keyboard types it and Enter; Just Works with `CONFIG_TLSR_BLE_SC_PASSKEY=n`; P-256 from zephyr-tc32's `tc32_p256.h`, on the chip's public key engine), link encryption (AES-CCM on the chip's AES block), the GATT server (GAP, Device Information, Battery, HID with ZMK's report map) and ZMK's endpoints and `&bt` on it. Zephyr's Bluetooth host is not used: it does not fit the 32 KB of RAM. Three profiles, each with its own address and bond, are selected by ZMK's `&bt` keys; the bonds live in the storage partition's first two sectors, never where the original firmware keeps its own. The link is chosen by the mode switch (`cidoo,mode-switch`, read at boot and watched while running; a move reboots the chip into the new position) or, on a keyboard without one, by key (`&link_mode`, kept with the profiles).

Power: deep sleep (`CONFIG_TLSR_BLE_DEEP_SLEEP`) after 20 s of advertising to a bonded host or 60 s of discoverable advertising with no connection, or at 0 % on battery; a key, the mode switch or power from the cable wakes the chip. A low-power state while connected (`CONFIG_TLSR_BLE_LOW_POWER`) after 300 s without a key: the matrix sleeps with each row a wake pad, the link skips events up to its peripheral latency, and the idle thread suspends the chip between kernel timeouts. Neither while a USB host has the keyboard over the cable. The connection parameters asked for 1 s after a connection: 7.5 ms, latency 44, 3 s supervision (`CONFIG_TLSR_BLE_CONN_*`).

`CONFIG_TLSR_P24` runs the proprietary 2.4G link to the keyboards' USB dongle from the same thread (`src/ble/p24.c`, the radio's 2M mode in zephyr-tc32's `tlsr_radio.c`). Pairing when no dongle ID is kept or after `&p24_pair` is held 3 s; ZMK's keyboard and consumer reports go as records sent until the dongle answers, with a heartbeat when nothing is queued; the dongle ID and its settings command are kept as records in the storage partition's fourth sector. The same power rules as BLE: 300 s without a key suspends the chip until a key, 30 min ends in deep sleep, so does the dongle's sleep command.

The battery (`cidoo,battery`): one measurement is the ADC with the input against ground, eight samples through DFIFO2 with the middle four averaged, mV = (avg × 590 >> 9) + 71; the ADC is powered down after each. The level starts from the first five samples, then only goes down on battery and only up while charging, so it does not flicker.

The boot guard with the links: a bonded BLE host's encrypted link marks the boot healthy, the 2.4G link does not; a confirmed image counts no boot, so power-ons with no host in reach never add up to a revert. Confirm an image over USB before using it on 2.4G.

## Flash access

Every flash transaction (reads, writes and erases of the slot code, the update receiver, the boot guard and the settings) runs one SPI flash transport in RAM code (`src/tlsr_spi_flash_io.S`, explained assembly in Telink's syntax that zephyr-tc32's `tc32_asm_sources()` translates for a Thumb build); `src/tlsr_spi_flash.c` calls it page by page and `src/flash_tlsr_spi.c` is the Zephyr flash driver over it (`compatible = "telink,tlsr-spi-flash"`). The wrappers are plain functions usable before the kernel runs. Data written must not reside in the flash itself (the bytes are fetched while the SPI transaction is open), so a buffer in the flash is copied to the stack 32 bytes at a time.

The flash supply trims (analog registers 0x09 and 0x0c) are set at boot from the calibration bytes at 0x771c0, raised while an update writes a Zbit flash and put back after it; every write of a trim runs from RAM code with interrupts off, since the supply of the flash the code is fetched from changes under it. `build.sh` checks that the trim functions lie in the RAM code, branch only into one another and hold no literal pointing into the flash.

Block protection: the original firmware locks the low 256 KB of the flash (status register 0x18) at boot and clears it when its updater starts. Before its first flash write the boot guard reads the status register and, when block-protect bits are set, clears them for the parts it knows by JEDEC ID (GD25LD40C/80C, ZB25WD40B/80B; `CONFIG_TLSR_BOOT_GUARD_UNLOCK`); the unlock is read back and tried up to three times. This firmware does not lock the flash again. `telink_ota.py info` shows the JEDEC ID and the register as the guard found it and as it left it. Both flash makers, GigaDevice and Zbit, are handled; nothing tells which part a unit has before the firmware is on it.

## The status display

`&sts` (`CONFIG_TLSR_STATUS_DISPLAY`, `src/status_display.c`) shows the keyboard's state on the backlight for `CONFIG_TLSR_STATUS_DISPLAY_MS` after the key, whatever the backlight's own setting: the battery level and whether it charges, the three Bluetooth profiles (connected, advertising for a pairing, looking for the bonded host; other bonded profiles dimmed), the 2.4G link (linked, pairing, looking for the dongle), the CPU share left over for the lowest-priority thread, and the Windows/Mac mode. Which keys show what is the board's part (`v75pro_status.c`, `v65v3_status.c` in the board directories) and is in each keyboard's README. With the backlight toggled off the display stays dark.

## Tests

| Run | What |
|---|---|
| `tests/slots/run.sh` | ztest on native_sim with the flash simulator: the boot count, the healthy mark, the revert after 3 boots, the journal cut after every chunk of the sector rewrite, the planned-reboot mark, a locked flash, the watchdog capture (35 tests) |
| `python3 -m unittest discover -s tests/tool` | `telink_ota.py` against fake devices: the original firmware's receiver and this one, the update gate, the model check, the macOS shared open, the waits (58 tests) |
| `tests/usb-sim/run.sh` | The TLSR8278 USB driver and the update receiver against a register model of the controller on native_sim, in Docker: enumeration, HID, the update in both directions, the gate, the flash test and the confirm, `&prev_fw` ([TC32.md](../TC32.md#usb-driver-tc32testsusb-sim)) |
| `tests/settings-log/run.sh`, `tests/ble_crypto/run.sh` | The settings backend; AES, c1/s1, the session key and AES-CCM against the Core specification's vectors |
| `tests/usjis/run.sh` | The US-JIS substitution's snapshot tests on native_sim |
| `tests/zmk-tests/run.sh`, `tests/zmk-tests-tc32/build.sh`, `tests/buildcheck/` | ZMK's own snapshot tests on native_sim against zephyr-tc32, the same on the TC32 image in the emulator of [tc32-devtools](https://github.com/goyamamoto/tc32-devtools), and build checks ([TC32.md](../TC32.md#tests)) |

The Bluetooth requirements the BLE stack meets, and the tests that check each, are in `docs/ble-conformance.md`; what is known to be missing or wrong, with a test that fails until it is fixed, in `docs/known-gaps.md`.

## Licence

The code in this folder is free software under the GNU General Public License, version 3 or later (`SPDX-License-Identifier: GPL-3.0-or-later`; the text is in [LICENSE-GPL](../LICENSE-GPL)), except `compat/` (Zephyr code and code written against it), which is Apache-2.0 (the text is in [LICENSE-APACHE](../LICENSE-APACHE)), and `tests/zmk-tests-tc32/zmk_test_mock.overlay` and `zmk_test_mock.conf`, which are MIT (they repeat the settings of ZMK's native_sim test board). ZMK itself, in the rest of the repository, is MIT.

### Why these licences

Our aim is simple: good keyboards that people can keep using for a long time.

For that, the people who use a keyboard need four rights:

- **Customisability**: to change it to fit how they work.
- **Transparency**: to check that it does nothing they do not want.
- **Reliability**: to depend on it as a tool, every day, for years.
- **Longevity**: to fix it, also when its maker no longer does.

Source code is what makes these rights real. So the firmware (zmk-tc32, tc32-keyboards) is licensed under the GNU General Public License, version 3 or later. We welcome makers who build better keyboards with this work, and we ask them to develop in the open, so that the people who buy those keyboards keep these rights.
