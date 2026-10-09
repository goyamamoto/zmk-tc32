#!/usr/bin/env python3
# Copyright (c) 2026 Go Yamamoto
# SPDX-License-Identifier: GPL-3.0-or-later
"""Telink OTA images and USB updates for TLSR8278 keyboards (CIDOO V21, V75 Pro, V65 V3).

  image RAW OUT   Make an OTA image from a raw build (zephyr/zmk.bin): pad to 16 bytes,
                  set the size word at 0x18, append the CRC-32.
  check IMAGE     Check an OTA image: boot flag, size word, CRC-32, and the model it is
                  for (the product string it holds).
  flash IMAGE     Send an image over USB HID report ID 5, the stock firmware's OTA
                  protocol (also taken by tc32/src/tlsr_usb_ota.c). Needs the
                  Python "hid" package (hidapi).
  info            Ask zmk-tc32 firmware (not the stock firmware) for its system clock and
                  watchdog setup with the version command, which writes nothing.
  battery         Ask zmk-tc32 firmware built with the battery measurement for one
                  measurement: the millivolts and ADC code, the percentage, and
                  the charger pins (power in, charging). The firmware writes nothing;
                  older builds answer status 8, builds without it status 16.
  confirm
                  zmk-tc32 firmware only: first the flash test (the firmware reads,
                  erases, writes and reads back the two sectors of its bond log; neither
                  slot is touched), then, if it passed, the confirm command. The
                  firmware takes the confirm only after a passed test in the same boot
                  and only while the image in the other slot (the way back) checks,
                  which it reads when the confirm arrives (status 10 and 14 otherwise;
                  nothing is written then; when the version answer already says no
                  image checks there, no test is sent). What a confirm shows: the image
                  can erase, write and read its flash, the way back is intact, and,
                  checked by hand before it, the keys that go back to the other image
                  work.

  rng-samples     zmk-tc32 measurement images only (built with TLSR_RNG_MEASURE): raw samples of
                  tc32_rng's two credited sources, the 32 kHz jitter and the ADC's codes of VBAT,
                  into files NIST SP 800-90B's ea_non_iid and ea_restart read (one sample per
                  octet). continuous: runs of one source until --count samples; restart: this
                  power-on's restart batch (the first 1000 jitter samples and 1001 ADC codes after
                  the boot) as one row of each restart file, for --rows power-ons (--cycle CMD
                  between them, or a prompt); status: tc32_rng_collect() as the BLE stack calls
                  it, then whether the generator is ready, its credited bits, and the ADC windows
                  credited and refused since boot. The firmware writes nothing, but for the seed
                  record a generator stores when it gets ready (as in a daily image).

Image format, the one the stock firmware's OTA takes: "KNLT" at 8 (byte 8 = 0x4b marks the slot
bootable), the image size at 0x18 including a 4-byte CRC-32 at the end (16n + 4), and that
CRC (reflected, init 0xffffffff, no final XOR) over everything before it. The stock firmware
writes an update to the flash slot it is not running from (0x00000 or 0x20000, 128 KB each),
then marks the new slot bootable and the old one not.

flash refuses to run without --yes. The stock Cidoo firmware of several models answers as
320F:5055, so for that ID it also wants --product with the device's exact product string.
Every zmk-tc32 keyboard answers as 1D50:615E. Each command talks to one keyboard only: when
more than one connected keyboard has the OTA interface under the ID (after --product, if
given), it sends nothing and stops.

flash sends an image only to a keyboard of the model it is for (the V21, the V75 Pro or the
V65 V3): the image's model is the one whose USB product string it holds, the keyboard's the
one whose product string it answers with (the stock's or zmk-tc32's). An image that holds no
model's string, or more than one model's, is not sent; neither is any image to a keyboard of
another product string, or of none of these.
"""
import argparse
import os
import struct
import sys
import time
import zlib

SLOT_SIZE = 0x20000
REPORT_ID = 0x05
REPORT_LEN = 33
CMD_VERSION, CMD_START, CMD_END, CMD_CONFIRM = 0xFF00, 0xFF01, 0xFF02, 0xFF03
CMD_FLASH_TEST = 0xFF07  # zmk-tc32: the bond log's two sectors read, erased, written and read back
CMD_BATTERY = 0xFF06  # zmk-tc32: one battery measurement, nothing written
CMD_CRASH_LOG = 0xFF0D  # zmk-tc32 with TLSR_CRASH_LOG: the boot before's crash record
CMD_P24_DIAG = 0xFF0B  # zmk-tc32 with TLSR_P24_DIAG: the 2.4G link's counters, nothing written
CMD_LINK_STATS = 0xFF0C  # zmk-tc32 with the own BLE stack: the BLE and 2.4G links' counters, nothing written
CMD_RNG_SAMPLES = 0xFF0E  # zmk-tc32 measurement images (TLSR_RNG_MEASURE): raw samples of tc32_rng's sources
CMD_PKE_TEST = 0xFF0F  # zmk-tc32 measurement images (TLSR_PKE_TEST): the P-256 engine's known answers
CMD_UNLOCK = 0xFF05  # zmk-tc32: the next update may go over an image that checks in the other slot
# The version reply's diagnostics (tlsr_usb_ota.c diag_info()); request byte 32 = 1 asks for the CPU figure.
DIAG_FLAGS, DIAG_CPU, DIAG_UPTIME = 30, 31, 32
DIAG_OTHER_OK, DIAG_UNLOCKED, DIAG_GATE, DIAG_PRESENT, DIAG_SLOT_B, DIAG_ASK_CPU = 0x01, 0x02, 0x04, 0x40, 0x80, 0x01
DIAG_PLANNED, DIAG_MARK_OTHER, DIAG_MARK = 0x08, 0x10, 0x20  # the planned-reboot mark found at boot
STOCK_ID = (0x320F, 0x5055)
# Each model's USB product strings: the stock firmware's and zmk-tc32's (the board's ZMK_KEYBOARD_NAME). An
# image drives and reads the pads of its own model's board; on the other board its keys, the power-on chord
# and &prev_fw among them, are not where it reads them. flash therefore sends an image only to
# a keyboard of the model it is for: the image holds its model's product string (the stock firmware's as a USB
# string descriptor, zmk-tc32's as CONFIG_USB_DEVICE_PRODUCT, ASCII ending in a NUL), and the keyboard answers
# with one of its model's.
MODELS = {"V21": ("CIDOO V21", "V21 ZMK"), "V75 Pro": ("CIDOO V75", "V75 Pro ZMK"),
          "V65 V3": ("CIDOO V65 V3", "V65 V3 ZMK")}
PRODUCT_HELP = ("only the keyboard whose USB product string is exactly this (e.g. 'CIDOO V75' for the stock, "
                "'V75 Pro ZMK' for zmk-tc32 on it); needed when two keyboards answer as the same ID")
NO_WAY_BACK_FIX = ("; to put a way back there, flash the stock image (it goes into the other slot, which "
                   "then needs no --overwrite-other-slot, and the keyboard boots it), then flash this image "
                   "from the stock firmware and confirm it again")
STATUS = {0: "ok", 1: "chunk index", 2: "chunk CRC-16", 3: "flash verify", 4: "end command",
          6: "image size or CRC-32", 7: "flash error", 8: "bad report", 9: "running slot unclear, or the flash test came during an update",
          10: "the flash test has not passed in this boot",
          12: "the running image's size word or CRC-32 does not check",
          13: "the other slot holds an image that checks, and the update was not unlocked "
              "(--overwrite-other-slot)",
          14: "the other slot holds no image that checks: the boot guard, the power-on chord and &prev_fw "
              "have nothing to go back to, so the image is not confirmed (nothing written)" + NO_WAY_BACK_FIX,
          16: "this firmware has no battery measurement",
          15: "not a Telink image (no \"KNLT\" at 8): an update installs only a bootable image"}
# The flash test erases two sectors and reads them back (up to about 1 s each on a worn part).
FLASH_TEST_MS = 5000

# The answers a zmk-tc32 receiver gives only after it has checked an image (the version command's check of
# the other slot, chunk 0's of the target or of the running image: up to 128 KB read and CRC-32, then an
# erase; the confirm's check of the other slot and its erase): about 0.1 s in the emulator, just over 1 s at
# its most pessimistic cache model. Each try waits this long for them, so that a slow answer is not asked for
# again.
CHECK_MS = 3000


def check_timeout(a):
    return max(a.timeout, CHECK_MS)


OVERWRITE_HELP = (
    "the other slot holds an image that checks: the stock firmware after the install (or the previous "
    "build), the one the boot guard, the power-on chord and &prev_fw go back to, and this update would "
    "overwrite it. To go back to it, use &prev_fw instead (the V75 Pro: Fn + Space, the V21: Fn + KP0, held and "
    "then released; the board README says for how long). To overwrite it "
    "anyway (when the keys do not work, say), run again with --overwrite-other-slot")


def telink_crc32(data):
    return zlib.crc32(data) ^ 0xFFFFFFFF


def make_image(raw):
    if raw[8:12] != b"KNLT":
        raise ValueError("no KNLT boot flag at offset 8: not a Telink TC32 image")
    body = bytearray(raw)
    body += b"\xff" * (-len(body) % 16)
    size = len(body) + 4
    if size > SLOT_SIZE:
        raise ValueError(f"image is {size} bytes; a slot holds {SLOT_SIZE}")
    struct.pack_into("<I", body, 0x18, size)
    return bytes(body) + struct.pack("<I", telink_crc32(body))


def check_image(img):
    """Size from the header; raises ValueError on any problem. The CRC-32 counts bytes 8..11 as "KNLT", as
    the firmware's check does."""
    if len(img) < 0x24:
        raise ValueError("too short")
    if img[8:12] != b"KNLT":
        raise ValueError(f"boot flag {img[8:12].hex()} at offset 8, expected 4b4e4c54 (KNLT)")
    size = struct.unpack_from("<I", img, 0x18)[0]
    if size % 16 != 4 or size < 0x24:
        raise ValueError(f"size word {size} is not 16n + 4")
    if size > len(img):
        raise ValueError(f"size word {size} is past the end of the file ({len(img)} bytes)")
    if size > SLOT_SIZE:
        raise ValueError(f"size {size} is larger than a slot ({SLOT_SIZE})")
    stored = struct.unpack_from("<I", img, size - 4)[0]
    calc = telink_crc32(img[:8] + b"KNLT" + img[12:size - 4])
    if stored != calc:
        raise ValueError(f"CRC-32 0x{stored:08x}, computed 0x{calc:08x}")
    return size


def string_descriptor(s):
    """s as a USB string descriptor: its length, type 3, the text in UTF-16LE."""
    text = s.encode("utf-16le")
    return bytes([2 + len(text), 3]) + text


def image_models(img):
    """The models (MODELS) whose product string the image holds."""
    return {m for m, (stock, zmk) in MODELS.items()
            if string_descriptor(stock) in img or zmk.encode() + b"\0" in img}


def image_model(img):
    """The one model the image is for; ValueError when it holds no model's product string, or more than one
    model's."""
    found = image_models(img)
    if not found:
        known = ", ".join(repr(s) for names in MODELS.values() for s in names)
        raise ValueError(f"the image holds no known model's USB product string ({known}): the keyboard it "
                         "is for is unknown")
    if len(found) > 1:
        raise ValueError(f"the image holds the USB product strings of more than one model ({sorted(found)})")
    return found.pop()


def product_model(product):
    """The model whose stock or zmk-tc32 product string this is, or None."""
    return next((m for m, names in MODELS.items() if product in names), None)


def same_model(model, product, what):
    """Stop (nothing sent) unless the product string is one of the model's; what names its source."""
    have = product_model(product)
    if have is None:
        known = ", ".join(repr(s) for names in MODELS.values() for s in names)
        raise SystemExit(f"{what} {product!r} is no known model's ({known}): the image, for the {model}, "
                         "is not sent")
    if have != model:
        raise SystemExit(f"the image is for the {model}, and {what} {product!r} is the {have}'s: not sent "
                         "(an image runs right only on its own model's board)")


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(payload):
    n = len(payload)
    r = bytes([REPORT_ID, n + 9, 0x01, n + 7, n + 3, 0x04, 0x52, 0x28, 0x00]) + payload
    return r + bytes(REPORT_LEN - len(r))


def chunk_frame(idx, data):
    p = struct.pack("<H", idx) + data
    return frame(p + struct.pack("<H", crc16_modbus(p)))


def command_frame(cmd, args=b""):
    return frame(struct.pack("<H", cmd) + args)


def frames(img, check=True, start=CMD_START):
    """START, chunks 0..last (last holds the CRC), END. Returns (frames, last index)."""
    size = check_image(img) if check else struct.unpack_from("<I", img, 0x18)[0]
    img = img[:size]
    last = size // 16
    out = [command_frame(start)]
    for i in range(last + 1):
        d = img[i * 16:(i + 1) * 16]
        out.append(chunk_frame(i, d + b"\xff" * (16 - len(d))))
    out.append(command_frame(CMD_END, struct.pack("<HH", last, last ^ 0xFFFF)))
    return out, last


def has_report5_output(desc):
    """Whether a HID report descriptor has an output item in report ID 5 (short items only)."""
    i, report_id = 0, 0
    while i < len(desc):
        prefix = desc[i]
        size = (0, 1, 2, 4)[prefix & 3]
        value = int.from_bytes(desc[i + 1:i + 1 + size], "little")
        if prefix & 0xFC == 0x84:  # Report ID
            report_id = value
        elif prefix & 0xFC == 0x90 and report_id == REPORT_ID:  # Output
            return True
        i += 1 + size
    return False


def open_shared(hid):
    """macOS: hidapi's Darwin backend opens a device exclusively (it seizes the IOHIDDevice) by default, and
    seizing a keyboard interface, such as the stock's that carries report 5, needs root: "privilege
    violation" even with Input Monitoring. Opens are made shared where the binding has the switch (the
    `hid` package's hidapi library, 0.12 or later); on other systems, with other bindings or older
    libraries nothing changes."""
    if sys.platform != "darwin":
        return
    set_exclusive = getattr(getattr(hid, "hidapi", None), "hid_darwin_set_open_exclusive", None)
    if set_exclusive is not None:
        set_exclusive(0)


def open_device(hid, vid, pid, interface, product=None):
    """(device, enumeration entry) of the interface with output report 5; with product, only on a
    keyboard whose product string is that. More than one such interface is refused: two keyboards
    answer as one ID (several Cidoo models as the stock's 320F:5055, every zmk-tc32 keyboard as
    1D50:615E), and a command meant for one must not reach the other."""
    open_shared(hid)
    found, errors, hits, paths = [], [], [], set()
    for d in hid.enumerate(vid, pid):
        if interface is not None and d["interface_number"] != interface:
            continue
        # hidapi on macOS lists an interface once per top-level collection, each time with the same path (the
        # stock's interface 2 seven times): one path is one interface of one keyboard.
        if d["path"] in paths:
            continue
        paths.add(d["path"])
        try:
            dev = hid.Device(path=d["path"])
        except Exception as e:  # noqa: BLE001 - e.g. no Input Monitoring permission on macOS
            errors.append(f"interface {d['interface_number']}: {e}")
            continue
        try:
            desc = dev.get_report_descriptor()
        except Exception:  # noqa: BLE001 - not every backend can read descriptors
            desc = b""
        if (has_report5_output(desc) or (interface is not None and not desc)) and (
                product is None or dev.product == product):
            hits.append((dev, d))
            continue
        found.append(d["interface_number"] if product is None or dev.product == product
                     else f"{d['interface_number']} of {dev.product!r}")
        dev.close()
    if len(hits) == 1:
        return hits[0]
    if hits:
        names = [dev.product for dev, _ in hits]
        for dev, _ in hits:
            dev.close()
        raise SystemExit(f"{len(hits)} keyboards on {vid:04x}:{pid:04x} have the OTA interface ({names}): "
                         + ("unplug all but the one meant" if product is not None else
                            "pass --product with the exact product string of the one meant, or unplug the others"))
    hint = ("; on macOS the terminal app needs Input Monitoring permission, and a keyboard interface opens "
            "only shared: hidapi seizes a device by default ('privilege violation'), which this tool switches "
            "off where the hid package has hid_darwin_set_open_exclusive (hidapi 0.12 or later)"
            if errors else "")
    raise SystemExit(f"no interface with output report 5 on {vid:04x}:{pid:04x}"
                     + (f" with the product string {product!r}" if product is not None else "")
                     + f" (interfaces seen: {found}; could not open: {errors}){hint}; "
                     "pass --interface to choose one")


class Refused(SystemExit):
    """A refusal status (byte 29) from the device."""

    def __init__(self, message, status):
        super().__init__(message)
        self.status = status


def exchange(dev, req, want_idx, timeout_ms, retries):
    """Send req until a response whose bytes 9..10 are in want_idx arrives; returns it."""
    if isinstance(want_idx, int):
        want_idx = (want_idx,)
    for _ in range(retries):
        dev.write(req)
        deadline = time.monotonic() + timeout_ms / 1000
        while True:
            left = int((deadline - time.monotonic()) * 1000)
            if left <= 0:
                break
            r = dev.read(64, left)
            if not r or r[0] != REPORT_ID:
                continue
            if r[9] | r[10] << 8 in want_idx:
                return r
            if len(r) > 29 and r[29] not in (0, 2):
                raise Refused(f"device refused index {req[9] | req[10] << 8}: "
                              f"{STATUS.get(r[29], r[29])}", r[29])
    raise SystemExit(f"no acknowledgement for index {req[9] | req[10] << 8}")


def _hid_module():
    import hid  # noqa: PLC0415 - only flash needs it

    return hid


def cmd_image(a):
    with open(a.raw, "rb") as f:
        img = make_image(f.read())
    with open(a.out, "wb") as f:
        f.write(img)
    print(f"{a.out}: {len(img)} bytes, CRC-32 0x{struct.unpack_from('<I', img, len(img) - 4)[0]:08x}")


def cmd_check(a):
    with open(a.image, "rb") as f:
        img = f.read()
    size = check_image(img)
    model = image_model(img[:size])
    print(f"{a.image}: OK, {size} bytes of {len(img)}, {size // 16 + 1} chunks, for the {model}")


def cmd_battery(a):
    """One battery measurement from zmk-tc32 firmware (0xff06); the firmware writes nothing."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("battery is for zmk-tc32 firmware")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    try:
        r = exchange(dev, command_frame(CMD_BATTERY), CMD_BATTERY, check_timeout(a), a.retries)
    except Refused as e:
        raise SystemExit(f"battery refused: {STATUS.get(e.status, e.status)}") from None
    if bytes(r[21:23]) != b"ZC" or r[29] != 0:
        raise SystemExit(f"battery refused: {STATUS.get(r[29], r[29])}")
    mv, raw = struct.unpack_from("<HH", r, 11)
    power, charging = bool(r[15] & 1), bool(r[15] & 2)
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    print(f"battery: {mv} mV (ADC code {raw}), {r[16]} %")
    print("power in: " + ("yes, " + ("charging" if charging else "charge complete") if power else "no"))


RNG_KIND = {0: "nothing", 1: "the restart batch", 2: "a jitter run", 3: "an ADC run"}
RNG_RESTART_JITTER, RNG_RESTART_ADC, RNG_ROW = 1000, 1001, 1000


def rng_ask(dev, a, args, timeout_ms=None):
    """One 0xff0e exchange; the reply."""
    try:
        r = exchange(dev, command_frame(CMD_RNG_SAMPLES, args), CMD_RNG_SAMPLES,
                     timeout_ms or check_timeout(a), a.retries)
    except Refused as e:
        raise SystemExit(f"rng-samples refused: {STATUS.get(e.status, e.status)} (not a measurement image?)") \
            from None
    if r[29] != 0 or bytes(r[21:23]) != b"ZC" and args[:1] != b"\x02":
        raise SystemExit(f"rng-samples refused: {STATUS.get(r[29], r[29])} (not a measurement image?)")
    return r


def rng_readiness(dev, a):
    r = rng_ask(dev, a, bytes([3]), max(check_timeout(a), 2000))
    err = struct.unpack_from("<b", r, 11)[0]
    before, state, bits, record = r[12], r[13], r[14], r[15]
    ms, windows = struct.unpack_from("<2H", r, 16)
    out_of_range, health = struct.unpack_from("<2H", r, 23)
    names = {0: "not ready", 1: "ready", 2: "failed (the AES known-answer test)"}
    print(f"tc32_rng: {names.get(state, state)}; collect {err} after {ms} ms, "
          f"{'ready' if before == 1 else 'not ready'} before it; {bits} credited bits; "
          f"the seed record at init {'counted' if record else 'did not count'}")
    print(f"ADC windows since boot: {windows} credited, {out_of_range} refused for a code out of VBAT's range, "
          f"{health} refused by the health tests")
    return state == 1 and err == 0


def rng_status(r):
    """The status reply's fields (src/tlsr_rng_measure.c)."""
    used, first, second, size = struct.unpack_from("<4H", r, 12)
    return {"kind": r[11], "used": used, "first": first, "second": second, "size": size, "reads": r[20],
            "ms": struct.unpack_from("<I", r, 23)[0], "takes": struct.unpack_from("<H", r, 27)[0]}


def rng_read(dev, a, n):
    """The buffer's first n octets, 18 a page."""
    out = b""
    for page in range((n + 17) // 18):
        out += bytes(rng_ask(dev, a, struct.pack("<BH", 2, page))[11:29])
    return out[:n]


def rng_adc_files(codes):
    """The raw codes (2 octets each, little-endian), their low octets, and the low 4 bits of the
    difference of each code and the one before (one octet each, 0-15): one run."""
    raw = struct.pack(f"<{len(codes)}H", *codes)
    low8 = bytes(c & 0xFF for c in codes)
    diff4 = bytes((b - a) & 0x0F for a, b in zip(codes, codes[1:]))
    return raw, low8, diff4


def rng_continuous(dev, a):
    src = {"jitter": 0, "adc": 1}[a.source]
    names = ([f"{a.out}-jitter.bin"] if src == 0 else
             [f"{a.out}-adc.u16", f"{a.out}-adc-low8.bin", f"{a.out}-adc-diff4.bin"])
    for n in names:
        if os.path.exists(n):
            raise SystemExit(f"{n} exists: give another --out")
    files = [open(n, "xb") for n in names]
    got = runs = short = 0
    full = st = None
    try:
        while got < a.count:
            st = rng_status(rng_ask(dev, a, bytes([1, src]), max(check_timeout(a), 2000)))
            if st["kind"] != 2 + src:
                raise SystemExit(f"rng-samples: the take left {RNG_KIND.get(st['kind'], st['kind'])}")
            if st["first"] == 0:
                raise SystemExit(f"rng-samples: a take of {a.source} gave nothing (the source stopped)")
            full = st["size"] if src == 0 else st["size"] // 2
            short += st["first"] < full
            data = rng_read(dev, a, st["used"])
            k = min(st["first"], a.count - got)
            if src == 0:
                files[0].write(data[:k])
            else:
                for f, part in zip(files, rng_adc_files(struct.unpack_from(f"<{k}H", data))):
                    f.write(part)
            got += k
            runs += 1
    finally:
        for f in files:
            f.close()
    print(f"{a.source}: {got} samples in {runs} runs of up to {full}"
          + (f" ({short} cut short: the source stopped in them)" if short else "") + ": " + ", ".join(names))


def rng_restart_row(dev, a):
    st = rng_status(rng_ask(dev, a, bytes([0])))
    if st["kind"] != 1 or st["reads"] != 0 or st["first"] != RNG_RESTART_JITTER or st["second"] != RNG_RESTART_ADC:
        raise SystemExit(f"rng-samples: no unread restart batch ({RNG_KIND.get(st['kind'], st['kind'])}, read "
                         f"{st['reads']} times): power-cycle the keyboard for the next row")
    data = rng_read(dev, a, st["used"])
    codes = struct.unpack_from(f"<{RNG_RESTART_ADC}H", data, RNG_RESTART_JITTER)
    raw, low8, diff4 = rng_adc_files(codes)
    parts = {"restart-jitter.bin": data[:RNG_ROW], "restart-adc.u16": raw, "restart-adc-low8.bin": low8[:RNG_ROW],
             "restart-adc-diff4.bin": diff4}
    for name, part in parts.items():
        with open(f"{a.out}-{name}", "ab") as f:
            f.write(part)
    return st


def rng_wait(hid, a, present, timeout_s):
    """Until the keyboard's update interface is (present) or is not listed."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if bool(hid.enumerate(a.vid, a.pid)) == present:
            return True
        time.sleep(0.2)
    return False


def cmd_rng_samples(a):
    """Raw samples of tc32_rng's sources from a zmk-tc32 measurement image (0xff0e); nothing is written."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("rng-samples is for zmk-tc32 measurement images")
    hid = _hid_module()
    if a.mode == "status":
        dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
        print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
        if not rng_readiness(dev, a):
            raise SystemExit("rng-samples status: the generator is not ready")
        return
    if a.out is None:
        raise SystemExit(f"rng-samples {a.mode}: give --out")
    if a.mode == "continuous":
        dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
        print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
        rng_continuous(dev, a)
        return
    for row in range(a.rows):
        if row:
            if a.cycle:
                import shlex  # noqa: PLC0415
                import subprocess  # noqa: PLC0415
                subprocess.run(shlex.split(a.cycle), check=True)
            else:
                print("power-cycle the keyboard (unplug it in the wired position, plug it in again)")
            if not rng_wait(hid, a, False, 60) or not rng_wait(hid, a, True, 60):
                raise SystemExit(f"rng-samples: the keyboard did not go and come back after row {row}")
            time.sleep(1.0)
        dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
        st = rng_restart_row(dev, a)
        dev.close()
        print(f"restart row: {RNG_ROW} jitter samples and {RNG_RESTART_ADC} ADC codes taken {st['ms']} ms after "
              f"the boot; {a.out}-restart-*: "
              f"{os.path.getsize(a.out + '-restart-jitter.bin') // RNG_ROW} rows")


PKE_TEST_RECORD = 188
PKE_TEST_CHECKS = ("the debug private key times G is the debug public key",
                   "the recorded responder's private key times G is its public key",
                   "that key times the recorded initiator's public key is their DHKey",
                   "the debug public key is on the curve", "a point off the curve is refused",
                   "a scalar of 0 is refused")
PKE_TEST_OPS = ("dA.G", "dB.G", "dB.PA", "verify A", "verify off-curve", "0.G")


def cmd_pke_test(a):
    """The P-256 engine's known answers from a zmk-tc32 measurement image (0xff0f); nothing is written."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("pke-test is for zmk-tc32 measurement images")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    blob = b""
    for page in range((PKE_TEST_RECORD + 17) // 18):
        # Page 0 runs the six operations first (up to the driver's timeout each).
        try:
            r = exchange(dev, command_frame(CMD_PKE_TEST, bytes([page])), CMD_PKE_TEST,
                         max(check_timeout(a), 15000) if page == 0 else check_timeout(a), a.retries)
        except Refused as e:
            raise SystemExit(f"pke-test refused: {STATUS.get(e.status, e.status)} (not a measurement image?)") \
                from None
        if r[29] != 0:
            raise SystemExit(f"pke-test refused: {STATUS.get(r[29], r[29])} (not a measurement image?)")
        blob += bytes(r[11:29])
    flags, rc, us = blob[0], struct.unpack_from("<6b", blob, 1), struct.unpack_from("<5I", blob, 8)
    ax, ay, bx, by, dh = (blob[o:o + 32] for o in (28, 60, 92, 124, 156))
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    for i, name in enumerate(PKE_TEST_CHECKS):
        print(f"{'ok  ' if flags >> i & 1 else 'FAIL'} {name}")
    for i, name in enumerate(PKE_TEST_OPS):
        print(f"  {name}: rc {rc[i]}" + (f", {us[i]} us" if i < 5 else ""))
    print(f"  Ax {ax[::-1].hex()}\n  Ay {ay[::-1].hex()}\n  Bx {bx[::-1].hex()}\n  By {by[::-1].hex()}\n"
          f"  DHKey {dh[::-1].hex()}")
    if flags != 0x3f:
        raise SystemExit("pke-test: not every check passed")

P24_DIAG_FORMAT = "<13I4H2B3s7BII40s24s6I4I4I4I5I"
P24_REJECT = ("under 20 octets", "octet 0 not 0x03", "octet 1 not 0x57", "octet 3 not 0x80",
              "the echo of the keyboard ID differs", "a dongle ID of 0 or 0xffffffff")


def p24_diag_text(blob):
    """The lines cmd_p24 prints for the counters' octets (src/ble/p24.c, the diag struct)."""
    (exchanges, tx_irq, rx_irq, rx_good, rx_crc, rx_len, rx_late, to_irq, pair_ok, pair_reject, answered,
     failures, stores, tx_min, tx_max, rx_min, rx_max, state, chan, kid, level_beacon, level_data, unit13,
     ana_8a, why, unit9, unit10, dongle, image_crc, last_rx, last_rej, *more) = struct.unpack(
        P24_DIAG_FORMAT, blob[:struct.calcsize(P24_DIAG_FORMAT)])
    tries, (sent_fast, got_fast, sent_normal, got_normal) = more[0:6], more[6:10]
    sent_chan, got_chan, gap = more[10:14], more[14:18], more[18:23]
    span = lambda lo, hi: "none" if lo == 0xFFFF else f"{lo}-{hi} us after the packet's start"  # noqa: E731
    reasons = [t for i, t in enumerate(P24_REJECT) if why >> i & 1]
    return [
        f"state: {state} ({ {1: 'pairing', 2: 'reconnecting', 3: 'linked'}.get(state, 'not started') }), "
        f"channel index {chan}, dongle ID 0x{dongle:08x}",
        f"keyboard ID {kid.hex()}, beacon CRC field 0x{image_crc:08x}",
        f"per-unit bytes 9, 10, 13: 0x{unit9:02x} 0x{unit10:02x} 0x{unit13:02x}; levels used: beacon "
        f"0x{level_beacon:02x}, data 0x{level_data:02x}; analog 0x8a: 0x{ana_8a:02x}",
        f"packets started: {exchanges}; TX interrupts: {tx_irq} ({span(tx_min, tx_max)})",
        f"RX interrupts: {rx_irq} ({span(rx_min, rx_max)}): good {rx_good}, CRC error {rx_crc}, "
        f"length mismatch {rx_len}, after the exchange's end {rx_late}; RX timeouts: {to_irq}",
        f"pairing replies taken: {pair_ok}; frames not taken while pairing: {pair_reject}"
        + (f" (last: {', '.join(reasons)})" if reasons else ""),
        f"linked packets answered: {answered}; failures: {failures}; records written: {stores}",
        "linked packets answered at try 1, 2, 3, 4, 5-8, later: " + ", ".join(str(n) for n in tries),
        f"fast settle: {got_fast} of {sent_fast} packets got a frame; normal settle: {got_normal} of {sent_normal}",
        "per channel 2405, 2422, 2440, 2460 MHz: "
        + ", ".join(f"{g} of {n}" for g, n in zip(got_chan, sent_chan)),
        "a packet's start after a failed one's, under 1.5, 2.5, 4, 8 ms, longer: " + ", ".join(str(n) for n in gap),
        f"last RX buffer: {last_rx.hex()}",
        f"last frame not taken: {last_rej.hex()}",
    ]


CRASH_FORMAT = "<II" + "IIII" * 16 + "IIIIII"


def cmd_crash(a):
    """The record of the boot before (0xff0d, zmk-tc32 built with TLSR_CRASH_LOG); nothing is written.
    --oops: a fatal error now, to try the record (the watchdog resets the keyboard 4 s later)."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("crash is for zmk-tc32 firmware")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    if a.oops or a.hang:
        dev.write(command_frame(CMD_CRASH_LOG, bytes([0xEE, 0x5A if a.oops else 0x5B])))
        print("asked for a " + ("fatal error" if a.oops else "hang of the update thread")
              + "; the keyboard resets through the watchdog about 4 s later")
        return
    size, blob = struct.calcsize(CRASH_FORMAT), b""
    for page in range((size + 17) // 18):
        r = exchange(dev, command_frame(CMD_CRASH_LOG, bytes([page])), CMD_CRASH_LOG, check_timeout(a), a.retries)
        if r[29] != 0:
            raise SystemExit(f"crash refused: {STATUS.get(r[29], r[29])}")
        blob += bytes(r[11:29])
    v = struct.unpack(CRASH_FORMAT, blob[:size])
    magic, nxt = v[0], v[1]
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    if magic != 0x43524C47:
        print("no record of the boot before (a power-on, or the first boot of this image)")
        return
    samples = [v[2 + 4 * i:6 + 4 * i] for i in range(16)]
    order = [(nxt + i) % 16 for i in range(16)] if nxt >= 16 else list(range(nxt))
    print(f"the boot before: {nxt} samples taken, the last {len(order)} (ms, thread, interrupted pc, "
          "watchdog count since its last feed, ms at 48 MHz):")
    for i in order:
        ms, th, pc, wd = samples[i]
        print(f"  {ms:8d} ms  thread 0x{th:08x}  pc 0x{pc:08x}  watchdog {wd} ({wd / 48000:.0f} ms)")
    reason, th, pc, lr, ms, crc = v[66:72]
    if reason == 0xFFFFFFFF:
        print("no fatal error recorded (a reset without one: the watchdog, or a reset asked for)")
    else:
        print(f"fatal error {reason} at {ms} ms: thread 0x{th:08x}, pc 0x{pc:08x}, lr 0x{lr:08x}")


# The counters of zmk-tc32's own links, as their structures have them (32-bit words, little-endian):
# struct cidoo_ble_stats (src/ble/ble_internal.h) and struct cidoo_p24_stats (src/ble/p24.c).
BLE_STATS = ("adv_events", "adv_rx", "adv_tx_timeouts", "connections", "conn_events", "conn_missed", "conn_rx",
             "conn_rx_dup", "conn_rx_bad", "conn_late", "conn_ended", "last_reason", "ll_ctrl_rx", "ll_ctrl_tx",
             "data_rx", "l2cap_dropped", "encryptions", "mic_failures", "pairings", "smp_failures", "conn_held",
             "hold_timeouts", "conn_skipped", "timer_lat_max", "conn_extended", "adv_refused", "conn_rx_timer",
             "conn_anchor_late", "scan_rsps", "ll_ctrl_last", "ll_ctrl_seen", "ll_ctrl_answered")
HEX_WORDS = ("last_reason", "ll_ctrl_last", "ll_ctrl_seen", "ll_ctrl_answered")  # reasons, opcodes and bit masks
BLE_REASONS = {0x05: "authentication failure", 0x06: "PIN or key missing", 0x08: "connection timeout",
               0x13: "remote user terminated", 0x16: "terminated by this side (a profile change or disconnect)",
               0x22: "LL response timeout", 0x3b: "unacceptable connection parameters", 0x3d: "MIC failure",
               0x3e: "connection failed to be established"}
LL_CTRL_NAMES = {0x00: "CONNECTION_UPDATE_IND", 0x01: "CHANNEL_MAP_IND", 0x02: "TERMINATE_IND", 0x03: "ENC_REQ",
                 0x04: "ENC_RSP", 0x05: "START_ENC_REQ", 0x06: "START_ENC_RSP", 0x07: "UNKNOWN_RSP",
                 0x08: "FEATURE_REQ", 0x09: "FEATURE_RSP", 0x0a: "PAUSE_ENC_REQ", 0x0b: "PAUSE_ENC_RSP",
                 0x0c: "VERSION_IND", 0x0d: "REJECT_IND", 0x0e: "PERIPHERAL_FEATURE_REQ",
                 0x0f: "CONNECTION_PARAM_REQ", 0x10: "CONNECTION_PARAM_RSP", 0x11: "REJECT_EXT_IND",
                 0x12: "PING_REQ", 0x13: "PING_RSP", 0x14: "LENGTH_REQ", 0x15: "LENGTH_RSP", 0x16: "PHY_REQ",
                 0x17: "PHY_RSP", 0x18: "PHY_UPDATE_IND", 0x19: "MIN_USED_CHANNELS_IND"}


def ll_ctrl_names(mask):
    """The LL control opcodes of a bit mask (bit n = opcode n), named."""
    return ", ".join(f"0x{n:02x} {LL_CTRL_NAMES.get(n, '?')}" for n in range(32) if mask >> n & 1) or "none"

P24_STATS = ("exchanges", "answered", "failures", "low_enters", "low_exits", "tx_while_low", "pairings", "state",
             "stores")


def link_stats_text(blob):
    """The lines of a link counters blob: two 16-bit sizes, the BLE link's words, the 2.4G link's."""
    if len(blob) < 4:
        return ["no counters"]
    n_ble, n_p24 = struct.unpack_from("<HH", blob)
    out = []
    for title, names, at, size in (("BLE link", BLE_STATS, 4, n_ble), ("2.4G link", P24_STATS, 4 + n_ble, n_p24)):
        words = struct.unpack_from(f"<{size // 4}I", blob, at) if len(blob) >= at + size else ()
        if not words:
            continue
        v = dict(zip(names, words))
        out.append(f"{title}: " + ", ".join(
            f"{names[i] if i < len(names) else f'word {i}'} {f'0x{w:02x}' if i < len(names) and names[i] in HEX_WORDS else w}"
            for i, w in enumerate(words)))
        if title == "BLE link" and "last_reason" in v:
            r = v["last_reason"]
            out.append(f"  the last link's end: reason 0x{r:02x} ({BLE_REASONS.get(r, 'a reason code of the Core, Vol 1 Part F')})")
        if title == "BLE link" and v.get("conn_events"):
            e = v["conn_events"]
            out.append(f"  of {e} connection events: {v['conn_missed']} with no packet from the host "
                       f"({100 * v['conn_missed'] / e:.2f} %), {v.get('conn_late', 0)} started too late and "
                       f"{v.get('conn_held', 0)} held for a flash write (not listened to), "
                       f"{v.get('conn_skipped', 0)} skipped for the peripheral latency; the event timer came at most "
                       f"{v.get('timer_lat_max', 0)} system ticks late")
        if title == "BLE link" and "ll_ctrl_last" in v:
            out.append(f"  LL control opcodes received: {ll_ctrl_names(v['ll_ctrl_seen'])}; answered: "
                       f"{ll_ctrl_names(v['ll_ctrl_answered'])}; the last control PDU received: opcode "
                       f"0x{v['ll_ctrl_last']:02x} {LL_CTRL_NAMES.get(v['ll_ctrl_last'], '?')}")
        if title == "BLE link" and "scan_rsps" in v and v.get("adv_events"):
            out.append(f"  of {v['adv_events']} advertising events: {v['adv_rx']} packets received (scan and "
                       f"connection requests), {v['scan_rsps']} scan responses sent")
        if title == "2.4G link" and v.get("exchanges"):
            e = v["exchanges"]
            out.append(f"  of {e} exchanges: {v['answered']} answered ({100 * v['answered'] / e:.2f} %), "
                       f"{v['failures']} link failures")
    return out or ["no counters"]


def cmd_link(a):
    """The BLE and 2.4G links' counters since the boot, from zmk-tc32 firmware (0xff0c); nothing is written."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("link is for zmk-tc32 firmware")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    blob, size = b"", 4
    page = 0
    while len(blob) < size and page < 64:
        try:
            r = exchange(dev, command_frame(CMD_LINK_STATS, bytes([page])), CMD_LINK_STATS, check_timeout(a),
                         a.retries)
        except Refused as e:
            raise SystemExit(f"link refused: {STATUS.get(e.status, e.status)}") from None
        if r[29] != 0:
            raise SystemExit(f"link refused: {STATUS.get(r[29], r[29])}")
        blob += bytes(r[11:29])
        if page == 0:
            size = 4 + sum(struct.unpack_from("<HH", blob))
        page += 1
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    print("\n".join(link_stats_text(blob[:size])))


def cmd_p24(a):
    """The 2.4G link's counters from zmk-tc32 firmware built with TLSR_P24_DIAG (0xff0b); nothing is written."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("p24 is for zmk-tc32 firmware")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    size, blob = struct.calcsize(P24_DIAG_FORMAT), b""
    for page in range((size + 17) // 18):
        try:
            r = exchange(dev, command_frame(CMD_P24_DIAG, bytes([page])), CMD_P24_DIAG, check_timeout(a),
                         a.retries)
        except Refused as e:
            raise SystemExit(f"p24 refused: {STATUS.get(e.status, e.status)}") from None
        if r[29] != 0:
            raise SystemExit(f"p24 refused: {STATUS.get(r[29], r[29])}")
        blob += bytes(r[11:29])
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    print("\n".join(p24_diag_text(blob)))


def cmd_info(a):
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("info is for zmk-tc32 firmware; the stock firmware answers the version "
                         "command with an echo")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    req = bytearray(command_frame(CMD_VERSION))
    req[DIAG_UPTIME] = DIAG_ASK_CPU  # the receiver measures the CPU left over (about 100 ms)
    r = exchange(dev, bytes(req), CMD_VERSION, check_timeout(a), a.retries)
    if bytes(r[21:23]) != b"ZC":
        raise SystemExit("the answer carries no zmk-tc32 information")
    sclk, capture = struct.unpack_from("<I", r, 11)[0], struct.unpack_from("<H", r, 15)[0]
    resets, flags = r[17], r[18]
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    if not flags & 2:
        print("boot guard: not built in")
        return
    how = "measured against the system timer" if flags & 1 else "not measured; fallback value"
    print(f"system clock: {sclk} Hz ({how})")
    print(f"watchdog: capture {capture}, period {capture * 262144 / sclk:.2f} s" if sclk else
          f"watchdog: capture {capture}")
    print(f"boots counted at this boot without a healthy one: {resets}")
    print("image confirmed by the host: " + ("yes (the automatic healthy rule applies)" if flags & 4
                                             else "no (only 'confirm' clears the count)"))
    st_boot, st_now, ff = struct.unpack_from("<H", r, 19)[0], struct.unpack_from("<H", r, 23)[0], r[25]
    if ff & 16:
        mid = r[26] | r[27] << 8 | r[28] << 16
        print(f"flash: JEDEC ID 0x{mid:06x}; status register 0x{st_boot:04x} at boot, 0x{st_now:04x} now"
              + (" (the boot guard cleared the block protection)" if ff & 1 else "")
              + (" (STILL LOCKED after the boot guard's writes)" if ff & 2 else "")
              + (" (read as busy or write-enabled, a misread: that write not made)" if ff & 32 else "")
              + (" (a part the SDK's tables do not cover: not unlocked)" if ff & 8 else ""))
    else:
        print("flash status register: not read (the boot guard's unlock is not built in)")
    if ff & 4:
        print("a flash write of the boot guard or the revert did not read back in this boot")
    if flags & 16:
        print("the planned-reboot mark register (analog 0x3c) did not read back its cleared value at this "
              "boot: the boot was taken as unplanned")
    print_diagnostics(r)


def print_diagnostics(r):
    """Bytes 30..32 of a zmk-tc32 version reply (tlsr_usb_ota.c diag_info()), if it has them."""
    if len(r) <= DIAG_UPTIME or not r[DIAG_FLAGS] & DIAG_PRESENT:
        return
    f = r[DIAG_FLAGS]
    print("running from slot " + ("B (0x20000)" if f & DIAG_SLOT_B else "A (0x00000)"))
    print("other slot: " + ("its image checks (size word and CRC-32): the ways back have somewhere to go"
                            if f & DIAG_OTHER_OK else
                            "NO image that checks: the boot guard, the chord and &prev_fw cannot go back"))
    if f & DIAG_GATE:
        print("update gate: an update over the other slot's image needs flash --overwrite-other-slot"
              + (" (unlocked now)" if f & DIAG_UNLOCKED else ""))
    if f & DIAG_MARK:
        print("this boot: " + ("a reboot the firmware asked for, into this image" if f & DIAG_PLANNED
                               else "not a reboot the firmware asked for, into this image"))
        if f & DIAG_MARK_OTHER:
            print("planned-reboot mark (analog 0x3c) at this boot: another image's, left by a reboot into that "
                  "image and kept through the resets since (a power-on clears it)")
    print("CPU left for the lowest-priority thread: "
          + (f"{r[DIAG_CPU]}% over 100 ms" if r[DIAG_CPU] <= 100 else "not measured"))
    print("uptime: " + (f"{r[DIAG_UPTIME]} s" if r[DIAG_UPTIME] < 255 else "255 s or more"))


def flash_test(dev, a):
    """The flash test: the firmware reads, erases, writes and reads back the two sectors of its bond log."""
    try:
        r = exchange(dev, command_frame(CMD_FLASH_TEST), CMD_FLASH_TEST, max(a.timeout, FLASH_TEST_MS), 1)
    except Refused as e:
        raise SystemExit(f"flash test failed: {STATUS.get(e.status, e.status)}") from None
    if len(r) > 29 and r[29] == 8:
        # A receiver from before the command echoes it with status 8 (bad report).
        raise SystemExit("this firmware has no flash test (a receiver from before it): not confirmed")
    if len(r) <= 29 or r[29] != 0 or bytes(r[21:23]) != b"ZC":
        raise SystemExit("flash test failed: " + (STATUS.get(r[29], str(r[29])) if len(r) > 29 else "no status"))
    print("flash test: passed (the bond log's two sectors erased, written and read back)")


def cmd_confirm(a):
    """The flash test, then tell the boot guard this image is healthy (zmk-tc32 only)."""
    if (a.vid, a.pid) == STOCK_ID:
        raise SystemExit("confirm is for zmk-tc32 firmware")
    hid = _hid_module()
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    r = exchange(dev, command_frame(CMD_VERSION), CMD_VERSION, check_timeout(a), a.retries)
    if bytes(r[21:23]) != b"ZC" or not r[18] & 2:
        raise SystemExit("the firmware has no boot guard to confirm to")
    if len(r) > DIAG_UPTIME and r[DIAG_FLAGS] & DIAG_PRESENT and not r[DIAG_FLAGS] & DIAG_OTHER_OK:
        # The confirm would get status 14: no test first (the firmware checks again at the confirm).
        raise SystemExit(f"not confirmed: {STATUS[14]}")
    before = r[17]
    flash_test(dev, a)
    try:
        r = exchange(dev, command_frame(CMD_CONFIRM), CMD_CONFIRM, check_timeout(a), a.retries)
    except Refused as e:
        raise SystemExit(f"confirm refused: {STATUS.get(e.status, e.status)}") from None
    if bytes(r[21:23]) != b"ZC" or r[29] != 0:
        raise SystemExit(f"confirm refused: {STATUS.get(r[29], r[29])}")
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}")
    if len(r) > DIAG_UPTIME and r[DIAG_FLAGS] & DIAG_PRESENT:
        print("other slot: " + ("its image checks (read by the firmware at the confirm)"
                                if r[DIAG_FLAGS] & DIAG_OTHER_OK else "the answer says no image checks there"))
    print(f"boots counted without a healthy one: {before} before, {r[17]} after the confirm; "
          f"image confirmed: {'yes' if r[18] & 4 else 'no'}")


def guard_other_slot(dev, a):
    """Before an update of a zmk-tc32 keyboard: an update writes its other slot, which holds the image every
    way back goes to. Refuse unless --overwrite-other-slot, while that slot holds an image that checks (or
    the receiver is too old to say); with the flag, unlock the receiver's gate (tlsr_usb_ota.c) first.
    Returns whether the receiver is zmk-tc32's."""
    r = exchange(dev, command_frame(CMD_VERSION), CMD_VERSION, check_timeout(a), a.retries)
    if bytes(r[21:23]) != b"ZC":
        return False
    diag = len(r) > DIAG_UPTIME and r[DIAG_FLAGS] & DIAG_PRESENT
    if a.overwrite_other_slot:
        if diag and r[DIAG_FLAGS] & DIAG_GATE:
            # Whatever bit 0 says: the receiver's check of the other slot for the version answer can be
            # older than the slot's content, and the gate decides at chunk 0 by a check of its own.
            r = exchange(dev, command_frame(CMD_UNLOCK), CMD_UNLOCK, check_timeout(a), a.retries)
            if len(r) <= DIAG_UPTIME or not r[DIAG_FLAGS] & DIAG_UNLOCKED:
                raise SystemExit("the receiver did not confirm the unlock")
        print("the update may overwrite the other slot's image (--overwrite-other-slot)")
        return True
    if diag and not r[DIAG_FLAGS] & DIAG_OTHER_OK:
        print("the other slot holds no image that checks: nothing to lose there, the update goes ahead")
        return True
    raise SystemExit(OVERWRITE_HELP if diag else
                     "this receiver does not tell what its other slot holds; if it holds the stock "
                     "firmware, this update would overwrite it. Run again with --overwrite-other-slot "
                     "if that is meant")


def cmd_flash(a):
    with open(a.image, "rb") as f:
        img = f.read()
    out, last = frames(img)
    model = image_model(img[:check_image(img)])
    print(f"{a.image}: {last + 1} chunks to {a.vid:04x}:{a.pid:04x}, an image for the {model}")
    if a.product is not None:
        same_model(model, a.product, "--product")
    if a.dry_run:
        for f in out[:3] + out[-2:]:
            print(f.hex(" "))
        return
    if not a.yes:
        raise SystemExit("writing firmware: pass --yes to go ahead")
    hid = _hid_module()

    if (a.vid, a.pid) == STOCK_ID and a.product is None:
        raise SystemExit("320F:5055 is shared by several Cidoo models: pass --product with the "
                         "keyboard's exact product string")
    dev, entry = open_device(hid, a.vid, a.pid, a.interface, a.product)
    print(f"device: {dev.product!r}, release 0x{entry.get('release_number', 0):04x}, "
          f"interface {entry.get('interface_number')}")
    if a.product is not None and dev.product != a.product:
        raise SystemExit(f"product string is {dev.product!r}, not {a.product!r}")
    same_model(model, dev.product, "the keyboard's product string")
    if a.release is not None and entry.get("release_number") != a.release:
        raise SystemExit(f"release is 0x{entry.get('release_number', 0):04x}, not 0x{a.release:04x}")
    zmk = (a.vid, a.pid) != STOCK_ID and guard_other_slot(dev, a)
    # The stock firmware answers START with index 0; tlsr_usb_ota.c too.
    exchange(dev, out[0], (0, CMD_START), a.timeout, a.retries)
    t0 = time.monotonic()
    for i, f in enumerate(out[1:-1]):
        exchange(dev, f, i + 1, check_timeout(a) if i == 0 and zmk else a.timeout, a.retries)
        if i % 256 == 0 or i == last:
            print(f"\r{i + 1}/{last + 1}", end="", flush=True)
    print(f"\n{last + 1} chunks in {time.monotonic() - t0:.1f} s; sending the end command")
    dev.write(out[-1])
    # The stock firmware reboots without answering; the device may vanish mid-read.
    try:
        r = dev.read(64, 1000)
    except Exception:  # noqa: BLE001 - the device disconnected
        r = None
    if r and r[0] == REPORT_ID and len(r) > 29 and r[29] not in (0, 2):
        raise SystemExit(f"end command refused: {STATUS.get(r[29], r[29])}")
    print("done; the keyboard restarts into the new image")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("image")
    p.add_argument("raw")
    p.add_argument("out")
    p.set_defaults(fn=cmd_image)
    p = sub.add_parser("check")
    p.add_argument("image")
    p.set_defaults(fn=cmd_check)
    p = sub.add_parser("flash")
    p.add_argument("image")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP + " (required for 320F:5055)")
    p.add_argument("--release", type=lambda s: int(s, 16), help="bcdDevice to require, e.g. 0101")
    p.add_argument("--timeout", type=int, default=1000, help="ms per try (erase takes up to ~300)")
    p.add_argument("--retries", type=int, default=5)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--overwrite-other-slot", action="store_true",
                   help="a zmk-tc32 keyboard only: go ahead although the other slot holds an image that checks "
                        "(the stock firmware after the install), which the update overwrites; unlocks the "
                        "receiver's gate")
    p.set_defaults(fn=cmd_flash)
    p = sub.add_parser("info")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_info)
    p = sub.add_parser("crash", help="the boot before's crash record (zmk-tc32 built with TLSR_CRASH_LOG; "
                       "writes nothing)")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.add_argument("--oops", action="store_true", help="a fatal error now, to try the record")
    p.add_argument("--hang", action="store_true", help="the update thread busy for good (a hang above the "
                   "watchdog feeder's priority), to try the watchdog")
    p.set_defaults(fn=cmd_crash)
    p = sub.add_parser("p24", help="the 2.4G link's counters (zmk-tc32 built with TLSR_P24_DIAG; the firmware "
                       "writes nothing)")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_p24)
    p = sub.add_parser("link", help="the BLE and 2.4G links' counters since the boot (zmk-tc32 with its own BLE "
                       "stack; the firmware writes nothing)")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_link)
    p = sub.add_parser("rng-samples", help="raw samples of tc32_rng's sources from a measurement image "
                       "(zmk-tc32 built with TLSR_RNG_MEASURE; the firmware writes nothing)")
    p.add_argument("mode", choices=("continuous", "restart", "status"))
    p.add_argument("--out", help="the files' prefix: PREFIX-jitter.bin; PREFIX-adc.u16, "
                   "-adc-low8.bin, -adc-diff4.bin; PREFIX-restart-jitter.bin, -restart-adc.u16, "
                   "-restart-adc-low8.bin, -restart-adc-diff4.bin")
    p.add_argument("--source", choices=("jitter", "adc"), default="jitter", help="continuous: the source")
    p.add_argument("--count", type=int, default=1000000, help="continuous: the samples to take")
    p.add_argument("--rows", type=int, default=1, help="restart: the power-ons, one row each (appended)")
    p.add_argument("--cycle", help="restart: a command that power-cycles the keyboard between rows "
                   "(e.g. a hub's port power switch); without it, a prompt")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_rng_samples)
    p = sub.add_parser("pke-test", help="the P-256 engine's known answers from a measurement image "
                       "(zmk-tc32 built with TLSR_PKE_TEST; the firmware writes nothing)")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_pke_test)
    p = sub.add_parser("battery", help="one battery measurement (zmk-tc32 with the battery measurement; "
                       "the firmware writes nothing)")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_battery)
    p = sub.add_parser("confirm", help="the flash test (the firmware reads, erases, writes and reads back its "
                       "bond log's two sectors), then declare the running image healthy: the firmware checks "
                       "the other slot's image first, and the boot guard's counter is cleared")
    p.add_argument("--vid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--pid", type=lambda s: int(s, 16), required=True)
    p.add_argument("--interface", type=int)
    p.add_argument("--product", help=PRODUCT_HELP)
    p.add_argument("--timeout", type=int, default=1000)
    p.add_argument("--retries", type=int, default=3)
    p.set_defaults(fn=cmd_confirm)
    a = ap.parse_args(argv)
    try:
        a.fn(a)
    except ValueError as e:
        sys.exit(f"error: {e}")


if __name__ == "__main__":
    main()
