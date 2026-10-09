#!/usr/bin/env python3
"""telink_ota.py flash against fake devices.

FakeStock models the stock firmware's OTA receiver: START is answered with
index 0; a chunk is answered with index + 1 only when, after processing, it is
the last chunk written (so a repeat of the last chunk is acknowledged again and
older repeats are echoed unchanged); a bad CRC-16, a gap or a bad END erases and reboots without an
answer; a good END marks the slot and reboots without an answer.

Run: python3 -m unittest discover -s tests/tool (from tc32/).

SPDX-License-Identifier: GPL-3.0-or-later
"""
import contextlib
import io
import os
import random
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
import telink_ota as t  # noqa: E402


class Gone(OSError):
    pass


def model_image(seed, n, product=b"V75 Pro ZMK\0"):
    """An image of n pseudo-random bytes holding a model's product string at 0x40 (zmk-tc32's ASCII one by
    default; a stock image's is a string descriptor, telink_ota.string_descriptor()): flash and confirm RESCUE
    send an image only to a keyboard of its model, and the fakes below answer as the V75 Pro."""
    raw = bytearray(random.Random(seed).randbytes(n))
    raw[8:12] = b"KNLT"
    raw[0x40:0x40 + len(product)] = product
    return t.make_image(bytes(raw))


def temp_image(img):
    fd, path = tempfile.mkstemp(suffix=".bin")
    os.write(fd, img)
    os.close(fd)
    return path


class FakeStock:
    def __init__(self, drop_every=0, stale_every=0, interfaces=((2, True),)):
        self.slot = bytearray(b"\xff" * t.SLOT_SIZE)
        self.last = -1
        self.end = 0
        self.crc = 0
        self.crc_ok = False
        self.rebooted = None
        self.replies = []
        self.sent = 0
        self.drop_every = drop_every
        self.stale_every = stale_every
        self.prev = None
        self.interfaces = interfaces
        self.product = "CIDOO V75"
        self.writes = 0  # reports written to the device

    # hid module API
    def enumerate(self, vid, pid):
        return [{"path": f"if{n}".encode(), "interface_number": n, "release_number": 0x0101}
                for n, _ in self.interfaces]

    def Device(self, path):  # noqa: N802 - hid API
        n = int(path[2:])
        ok = dict(self.interfaces)[n]
        if ok is None:
            raise OSError("open failed (permission)")
        return FakeHandle(self, has_report5=ok)

    def reboot(self, why):
        self.rebooted = why

    def handle(self, r):
        if self.rebooted:
            raise Gone("device gone")
        r = bytearray(r)
        if r[2] != 1:
            self.reply(r)
            return
        idx = r[9] | r[10] << 8
        if idx == t.CMD_START:
            self.last, self.crc, self.crc_ok, self.end = -1, 0, False, 0
            r[9] = r[10] = 0
            self.reply(r)
            return
        if idx == t.CMD_VERSION:
            self.reply(r)
            return
        if idx == t.CMD_END:
            n, inv = struct.unpack_from("<HH", r, 11)
            if r[4] == 9 and (n ^ inv) == 0xFFFF and n != self.last or not self.crc_ok:
                return self.reboot("bad end")
            self.slot[8] = 0x4B
            return self.reboot("done")
        if idx != self.last + 1:
            if idx <= self.last:
                if idx == self.last:
                    struct.pack_into("<H", r, 9, idx + 1)
                self.reply(r)
                return
            return self.reboot("gap")
        if t.crc16_modbus(bytes(r[9:27])) != struct.unpack_from("<H", r, 27)[0]:
            return self.reboot("crc16")
        data = bytes(r[11:27])
        if idx == 1:
            size = struct.unpack_from("<I", data, 8)[0]
            if size % 16 != 4 or size - 1 > 0x3FFFE:
                return self.reboot("size")
            self.end = size // 16
        if self.end and idx == self.end:
            if struct.unpack_from("<I", data)[0] != self.crc ^ 0xFFFFFFFF:
                return self.reboot("crc32")
            self.crc_ok = True
        else:
            import zlib
            # ZMK's receiver counts chunk 0's bytes 8..11 as "KNLT" (the stock's images have it there).
            self.crc = zlib.crc32(data[:8] + b"KNLT" + data[12:] if idx == 0 else data, self.crc)
        chunk = bytearray(data)
        if idx == 0:
            chunk[8] = 0xFF
        self.slot[idx * 16:idx * 16 + 16] = chunk
        self.last = idx
        struct.pack_into("<H", r, 9, idx + 1)
        self.reply(r)

    def reply(self, r):
        self.sent += 1
        if self.drop_every and self.sent % self.drop_every == 0:
            return
        if self.stale_every and self.sent % self.stale_every == 0 and self.prev is not None:
            self.replies.append(self.prev)  # an old answer arrives first
        self.replies.append(bytes(r))
        self.prev = bytes(r)


class FakeHandle:
    def __init__(self, dev, has_report5):
        self.dev = dev
        self.product = dev.product
        self.has_report5 = has_report5

    def get_report_descriptor(self):
        if self.has_report5:
            return bytes.fromhex("05010900a101850515002600ff7508952009018102090291 02c0".replace(" ", ""))
        return bytes.fromhex("05010906a101850105071900297015002501750195788102c0")

    def write(self, data):
        self.dev.writes += 1
        self.dev.handle(data)

    def read(self, size, timeout=None):
        if self.dev.rebooted:
            raise Gone("device gone")
        return self.dev.replies.pop(0) if self.dev.replies else b""

    def close(self):
        pass


def run_flash(fake, img_path, *extra):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(["flash", img_path, "--vid", "320f", "--pid", "5055", "--product", "CIDOO V75",
                    "--timeout", "20", "--yes", *extra])
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class FlashTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.img = model_image(75, 3000)
        cls.path = temp_image(cls.img)

    @classmethod
    def tearDownClass(cls):
        os.unlink(cls.path)

    def installed(self, fake):
        size = len(self.img)
        want = bytearray(self.img)
        self.assertEqual(bytes(fake.slot[:size]), bytes(want))

    def test_stock_happy_path(self):
        fake = FakeStock()
        code, out = run_flash(fake, self.path)
        self.assertEqual(code, 0, out)
        self.assertEqual(fake.rebooted, "done")
        self.installed(fake)
        self.assertIn("release 0x0101", out)

    def test_dropped_replies(self):
        fake = FakeStock(drop_every=17)
        code, out = run_flash(fake, self.path)
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.installed(fake)

    def test_stale_replies(self):
        fake = FakeStock(stale_every=13)
        code, out = run_flash(fake, self.path)
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.installed(fake)

    def test_unopenable_interface_is_skipped(self):
        fake = FakeStock(interfaces=((0, None), (1, False), (2, True)))
        code, out = run_flash(fake, self.path)
        self.assertEqual((code, fake.rebooted), (0, "done"), out)

    def test_no_report5_interface(self):
        fake = FakeStock(interfaces=((0, None), (1, False)))
        code, out = run_flash(fake, self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("Input Monitoring", out)
        self.assertIsNone(fake.rebooted)

    def test_stock_id_needs_product(self):
        fake = FakeStock()
        t._hid_module = lambda: fake
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as e:
            t.main(["flash", self.path, "--vid", "320f", "--pid", "5055", "--yes"])
        self.assertIn("--product", str(e.exception.code))
        self.assertEqual(fake.sent, 0)

    def test_wrong_release_refused(self):
        fake = FakeStock()
        code, out = run_flash(fake, self.path, "--release", "0102")
        self.assertNotEqual(code, 0)
        self.assertEqual(fake.sent, 0)

    def test_refusal_status_stops(self):
        """tlsr_usb_ota.c reports refusals in byte 29; the tool must stop."""
        fake = FakeStock()
        orig = fake.handle

        def refuse(r):
            r = bytearray(r)
            if r[9] | r[10] << 8 == 0:
                r[29] = 9
                fake.replies.append(bytes(r))
                return
            orig(r)
        fake.handle = refuse
        code, out = run_flash(fake, self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("running slot unclear", out)
        self.assertIsNone(fake.rebooted)




class FakeZmk(FakeStock):
    """tlsr_usb_ota.c's version answer: clock, capture, resets, flags, "ZC"."""

    def __init__(self, sclk=16_000_000, capture=244, resets=1, flags=3, flash=None, diag=None):
        super().__init__()
        self.product = "V75 Pro ZMK"
        self.info = (sclk, capture, resets, flags)
        self.flash = flash   # (status at boot, status now, flags byte 25, JEDEC mid) or None: nothing read
        self.diag = diag     # bytes 30, 31, 32 (diag_info()) or None: an older receiver, which echoes them
        self.version_request = None

    def handle(self, r):
        r = bytearray(r)
        if r[9] | r[10] << 8 == t.CMD_VERSION:
            self.version_request = bytes(r)
            sclk, capture, resets, flags = self.info
            struct.pack_into("<IHBB", r, 11, sclk, capture, resets, flags)
            r[21:23] = b"ZC"
            if self.flash:
                st_boot, st_now, ff, mid = self.flash
                struct.pack_into("<H", r, 19, st_boot)
                struct.pack_into("<H", r, 23, st_now)
                r[25] = ff
                r[26:29] = bytes((mid & 0xFF, mid >> 8 & 0xFF, mid >> 16 & 0xFF))
            if self.diag:
                r[30:33] = bytes(self.diag)
            self.reply(r)
            return
        super().handle(r)


class FakeZmkConfirm(FakeZmk):
    """tlsr_usb_ota.c's flash test (0xff07: the bond log's sectors erased, written and read back, the
    version answer with byte 18 bit 3 set on a pass) and its confirm, refused (status 10) unless the test
    passed in the same boot and (status 14) while the other slot holds no image that checks."""

    def __init__(self, test_status=0, other_ok=True):
        super().__init__()
        self.test_status = test_status
        self.other_ok = other_ok
        self.tests = []        # statuses of the flash tests answered
        self.confirms = []     # statuses of the confirm commands answered

    def handle(self, r):
        r = bytearray(r)
        idx = r[9] | r[10] << 8
        if idx == t.CMD_FLASH_TEST:
            self.tests.append(self.test_status)
            sclk, capture, resets, flags = self.info
            if self.test_status == 0:
                self.info = (sclk, capture, resets, flags | 8)
                sclk, capture, resets, flags = self.info
            struct.pack_into("<IHBB", r, 11, sclk, capture, resets, flags)
            r[21:23] = b"ZC"
            r[29] = self.test_status
            self.reply(r)
            return
        if idx == t.CMD_START:
            self.info = (*self.info[:3], self.info[3] & ~8)
        if idx == t.CMD_CONFIRM:
            status = 10 if not self.info[3] & 8 else 0 if self.other_ok else 14
            self.confirms.append(status)
            sclk, capture, resets, flags = self.info
            if status == 0:
                self.info = (sclk, capture, 0, flags | 4)
                sclk, capture, resets, flags = self.info
            struct.pack_into("<IHBB", r, 11, sclk, capture, resets, flags)
            r[21:23] = b"ZC"
            r[29] = status
            self.reply(r)
            return
        super().handle(bytes(r))


class FakeZmkGate(FakeZmk):
    """tlsr_usb_ota.c's gate: while the other slot holds an image that checks, chunk 0 of an update is
    refused with status 13 unless the unlock (0xff05) came first, and it uses the unlock up; chunk 0
    erases the slot's first sector, after which nothing there checks. gate=False: a receiver from
    before the gate, which has the diagnostics but answers 0xff05 with status 8."""

    def __init__(self, other_ok=True, gate=True, report_other_ok=None, confirm_unlock=True):
        super().__init__()
        self.other_ok = other_ok
        self.report_other_ok = report_other_ok  # what the version reply says, if not other_ok (a stale check)
        self.gate = gate
        self.confirm_unlock = confirm_unlock
        self.unlocked = False
        self.cmds = []  # every index received, commands and chunks

    def set_diag(self):
        ok = self.other_ok if self.report_other_ok is None else self.report_other_ok
        self.diag = (t.DIAG_PRESENT | (t.DIAG_OTHER_OK if ok else 0) | (t.DIAG_GATE if self.gate else 0)
                     | (t.DIAG_UNLOCKED if self.unlocked and self.confirm_unlock else 0), 0xFF, 7)

    def handle(self, r):
        r = bytearray(r)
        idx = r[9] | r[10] << 8
        self.cmds.append(idx)
        self.set_diag()
        if idx == t.CMD_UNLOCK:
            if not self.gate:
                r[29] = 8
                self.reply(r)
                return
            self.unlocked = True
            self.set_diag()
            struct.pack_into("<H", r, 9, t.CMD_VERSION)  # answered as the version command, index kept
            super().handle(bytes(r))
            self.replies[-1] = self.replies[-1][:9] + struct.pack("<H", t.CMD_UNLOCK) + self.replies[-1][11:]
            return
        if idx == 0 and self.gate and self.other_ok:
            if not self.unlocked:
                r[29] = 13
                self.reply(r)
                return
            self.unlocked = False
        if idx == 0:
            self.other_ok = False
        super().handle(bytes(r))


def run_flash_zmk(fake, img_path, *extra):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(["flash", img_path, "--vid", "1d50", "--pid", "615e", "--timeout", "20", "--yes", *extra])
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class GateTest(unittest.TestCase):
    """flash to a zmk-tc32 keyboard: the other slot's image (the stock after the install) is overwritten
    only with --overwrite-other-slot, which unlocks the receiver's gate first."""

    @classmethod
    def setUpClass(cls):
        cls.img = model_image(13, 1500)
        cls.path = temp_image(cls.img)

    @classmethod
    def tearDownClass(cls):
        os.unlink(cls.path)

    def test_refused_without_the_flag(self):
        fake = FakeZmkGate()
        code, out = run_flash_zmk(fake, self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("--overwrite-other-slot", out)
        self.assertIn("&prev_fw", out)
        self.assertEqual(fake.cmds, [t.CMD_VERSION], "sent more than the version command")
        self.assertIsNone(fake.rebooted)

    def test_flag_unlocks_then_installs(self):
        fake = FakeZmkGate()
        code, out = run_flash_zmk(fake, self.path, "--overwrite-other-slot")
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.assertEqual(fake.cmds[:4], [t.CMD_VERSION, t.CMD_UNLOCK, t.CMD_START, 0])
        self.assertEqual(bytes(fake.slot[9:len(self.img)]), self.img[9:])
        self.assertIn("may overwrite the other slot's image", out)

    def test_flag_unlocks_whatever_the_reply_says(self):
        """The version answer's check of the other slot can be older than the slot (one made during an update
        that then got all its chunks but no end command): with the flag, the unlock goes anyway, since the
        receiver's own check at chunk 0 decides."""
        fake = FakeZmkGate(report_other_ok=False)
        code, out = run_flash_zmk(fake, self.path, "--overwrite-other-slot")
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.assertEqual(fake.cmds[:4], [t.CMD_VERSION, t.CMD_UNLOCK, t.CMD_START, 0])

    def test_nothing_that_checks_goes_ahead(self):
        """The recovery case (after a cut transfer, say): no flag needed, no unlock sent."""
        fake = FakeZmkGate(other_ok=False)
        code, out = run_flash_zmk(fake, self.path)
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.assertNotIn(t.CMD_UNLOCK, fake.cmds)
        self.assertIn("nothing to lose there", out)

    def test_receiver_without_the_gate(self):
        """A receiver with the diagnostics but no gate: the tool asks for the flag; with it, no unlock is
        sent (it would be refused with status 8)."""
        code, out = run_flash_zmk(FakeZmkGate(gate=False), self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("--overwrite-other-slot", out)
        fake = FakeZmkGate(gate=False)
        code, out = run_flash_zmk(fake, self.path, "--overwrite-other-slot")
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.assertNotIn(t.CMD_UNLOCK, fake.cmds)

    def test_receiver_without_diagnostics(self):
        """An older receiver echoes bytes 30-32: it cannot say what its other slot holds."""
        fake = FakeZmk()
        code, out = run_flash_zmk(fake, self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("does not tell what its other slot holds", out)
        self.assertIsNone(fake.rebooted)
        fake = FakeZmk()
        chunks = []
        handle = fake.handle

        def log(r):
            chunks.append(r[9] | r[10] << 8)
            handle(r)
        fake.handle = log
        code, out = run_flash_zmk(fake, self.path, "--overwrite-other-slot")
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.assertNotIn(t.CMD_UNLOCK, chunks)

    def test_status_13_is_readable(self):
        """The receiver's own check at chunk 0 decides: a version reply that said nothing checks there
        (a stale check) gets the refusal, in words, and nothing is written."""
        fake = FakeZmkGate(report_other_ok=False)
        code, out = run_flash_zmk(fake, self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("device refused index 0: the other slot holds an image that checks, and the update "
                      "was not unlocked (--overwrite-other-slot)", out)
        self.assertEqual(bytes(fake.slot[:16]), b"\xff" * 16)
        self.assertIsNone(fake.rebooted)

    def test_unlock_not_confirmed(self):
        fake = FakeZmkGate(confirm_unlock=False)
        code, out = run_flash_zmk(fake, self.path, "--overwrite-other-slot")
        self.assertNotEqual(code, 0)
        self.assertIn("did not confirm the unlock", out)
        self.assertEqual(fake.cmds, [t.CMD_VERSION, t.CMD_UNLOCK])

    def test_stock_id_gets_no_version_command(self):
        """The stock's OTA (320F:5055) is not asked anything before START: the install path is unchanged."""
        fake = FakeStock()
        cmds = []
        handle = fake.handle

        def log(r):
            cmds.append(r[9] | r[10] << 8)
            handle(r)
        fake.handle = log
        code, out = run_flash(fake, self.path)
        self.assertEqual((code, fake.rebooted), (0, "done"), out)
        self.assertEqual(cmds[:2], [t.CMD_START, 0])


class SharedOpenTest(unittest.TestCase):
    """macOS: hidapi seizes a device when it opens it, and seizing a keyboard interface (the stock's, which
    carries report 5) needs root. The tool switches hidapi to a shared open before any device is opened,
    where the binding has the call."""

    @classmethod
    def setUpClass(cls):
        cls.path = temp_image(model_image(29, 600))

    @classmethod
    def tearDownClass(cls):
        os.unlink(cls.path)

    @staticmethod
    def recorded(fake, log, with_call=True):
        """The fake's opens logged; with_call: a hidapi library with the switch, logging its calls too."""
        device = fake.Device

        def opened(path):
            log.append(("Device", path))
            return device(path)
        fake.Device = opened
        if with_call:
            fake.hidapi = types.SimpleNamespace(
                hid_darwin_set_open_exclusive=lambda value: log.append(("set_open_exclusive", value)))
        return fake

    def test_macos_opens_shared_before_any_open(self):
        for name, run, fake in (("flash", lambda f: run_flash(f, self.path), FakeStock()),
                                ("info", run_info, FakeZmk())):
            log = []
            with mock.patch.object(t.sys, "platform", "darwin"):
                code, out = run(self.recorded(fake, log))
            self.assertEqual(code, 0, f"{name}: {out}")
            self.assertEqual(log[0], ("set_open_exclusive", 0), f"{name}: {log[:3]}")
            self.assertIn("Device", [kind for kind, _ in log[1:]], name)

    def test_without_the_call_nothing_fails(self):
        """No hidapi attribute (another binding), or a library without the switch (hidapi before 0.12)."""
        for fake in (FakeStock(), self.recorded(FakeStock(), [], with_call=False)):
            with mock.patch.object(t.sys, "platform", "darwin"):
                code, out = run_flash(fake, self.path)
            self.assertEqual((code, fake.rebooted), (0, "done"), out)
        fake = FakeStock()
        fake.hidapi = object()
        with mock.patch.object(t.sys, "platform", "darwin"):
            code, out = run_flash(fake, self.path)
        self.assertEqual((code, fake.rebooted), (0, "done"), out)

    def test_other_systems_leave_it(self):
        log = []
        with mock.patch.object(t.sys, "platform", "linux"):
            code, out = run_flash(self.recorded(FakeStock(), log), self.path)
        self.assertEqual(code, 0, out)
        self.assertNotIn(("set_open_exclusive", 0), log)


class WaitTest(unittest.TestCase):
    """The answers a zmk-tc32 receiver gives only after checking an image (the version command, chunk 0, the
    confirm) get at least CHECK_MS per try: just over 1 s in the emulator's most pessimistic cache model.
    The other chunks, and every exchange with the stock, keep --timeout (20 ms here)."""

    @classmethod
    def setUpClass(cls):
        cls.path = temp_image(model_image(3, 700))

    @classmethod
    def tearDownClass(cls):
        os.unlink(cls.path)

    def waits(self, run, *args):
        calls = []
        real = t.exchange

        def spy(dev, req, want_idx, timeout_ms, retries):
            calls.append((req[9] | req[10] << 8, timeout_ms))
            return real(dev, req, want_idx, timeout_ms, retries)
        t.exchange = spy
        try:
            code, out = run(*args)
        finally:
            t.exchange = real
        self.assertEqual(code, 0, out)
        return calls

    def test_zmk_flash(self):
        calls = self.waits(run_flash_zmk, FakeZmkGate(), self.path, "--overwrite-other-slot")
        self.assertEqual(calls[:5], [(t.CMD_VERSION, t.CHECK_MS), (t.CMD_UNLOCK, t.CHECK_MS),
                                     (t.CMD_START, 20), (0, t.CHECK_MS), (1, 20)])
        self.assertEqual({w for i, w in calls[4:]}, {20})

    def test_stock_flash(self):
        calls = self.waits(run_flash, FakeStock(), self.path)
        self.assertEqual({w for i, w in calls}, {20})

    def test_confirm_and_info(self):
        calls = self.waits(run_confirm, FakeZmkConfirm())
        self.assertEqual(calls, [(t.CMD_VERSION, t.CHECK_MS), (t.CMD_FLASH_TEST, t.FLASH_TEST_MS),
                                 (t.CMD_CONFIRM, t.CHECK_MS)])
        calls = self.waits(run_info, FakeZmk())
        self.assertEqual(calls, [(t.CMD_VERSION, t.CHECK_MS)])


def run_confirm(fake):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(["confirm", "--vid", "1d50", "--pid", "615e", "--timeout", "20"])
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class ConfirmTest(unittest.TestCase):
    def test_test_then_confirm(self):
        fake = FakeZmkConfirm()
        code, out = run_confirm(fake)
        self.assertEqual(code, 0, out)
        self.assertIn("flash test: passed", out)
        self.assertIn("1 before, 0 after the confirm; image confirmed: yes", out)
        self.assertEqual(fake.tests, [0])
        self.assertEqual(fake.confirms, [0])
        self.assertIsNone(fake.rebooted)

    def test_receiver_without_the_test_gets_no_confirm(self):
        # A receiver from before 0xff07 echoes it with status 8: no confirm is sent.
        fake = FakeZmkConfirm()
        handle = fake.handle

        def old(r):
            if r[9] | r[10] << 8 == t.CMD_FLASH_TEST:
                r = bytearray(r)
                r[29] = 8
                fake.reply(r)
                return
            handle(r)
        fake.handle = old
        code, out = run_confirm(fake)
        self.assertNotEqual(code, 0)
        self.assertIn("no flash test", out)
        self.assertEqual(fake.confirms, [])
        self.assertIsNone(fake.rebooted)

    def test_other_slot_not_checking_is_said_in_words(self):
        fake = FakeZmkConfirm(other_ok=False)
        code, out = run_confirm(fake)
        self.assertNotEqual(code, 0)
        self.assertEqual(fake.confirms, [14])
        self.assertIn("confirm refused: the other slot holds no image that checks", out)
        self.assertIn("flash the stock image", out)  # what to do then
        self.assertNotIn("image confirmed: yes", out)

    def test_other_slot_known_bad_sends_no_test(self):
        # The version answer already says no image checks there: neither test nor confirm.
        fake = FakeZmkConfirm(other_ok=False)
        fake.diag = (t.DIAG_PRESENT, 0xFF, 5)
        code, out = run_confirm(fake)
        self.assertNotEqual(code, 0)
        self.assertEqual(fake.tests, [])
        self.assertEqual(fake.confirms, [])
        self.assertIn("the other slot holds no image that checks", out)
        self.assertIn("flash the stock image", out)
        fake = FakeZmkConfirm()
        fake.diag = (t.DIAG_PRESENT | t.DIAG_OTHER_OK, 0xFF, 5)
        code, out = run_confirm(fake)
        self.assertEqual(code, 0, out)
        self.assertEqual(fake.confirms, [0])

    def test_failed_test_sends_no_confirm(self):
        for status, text in ((3, "flash verify"), (7, "flash error")):
            fake = FakeZmkConfirm(test_status=status)
            code, out = run_confirm(fake)
            self.assertNotEqual(code, 0)
            self.assertIn(f"flash test failed: {text}", out)
            self.assertEqual(fake.tests, [status])
            self.assertEqual(fake.confirms, [])
            self.assertIsNone(fake.rebooted)

    def test_confirm_refused_untested(self):
        # A receiver whose test answer says pass but whose confirm says 10 (a START between, say).
        fake = FakeZmkConfirm()
        handle = fake.handle

        def untested(r):
            if r[9] | r[10] << 8 == t.CMD_CONFIRM:
                fake.info = (*fake.info[:3], fake.info[3] & ~8)
            handle(r)
        fake.handle = untested
        code, out = run_confirm(fake)
        self.assertNotEqual(code, 0)
        self.assertIn("confirm refused: the flash test has not passed in this boot", out)
        self.assertEqual(fake.confirms, [10])


def run_info(fake, vid="1d50", pid="615e"):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(["info", "--vid", vid, "--pid", pid, "--timeout", "20"])
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class FakeTwo:
    """Two keyboards that answer as one ID (the stocks of two Cidoo models on 320F:5055, or two
    zmk-tc32 keyboards on 1D50:615E), each a fake as above; the paths say which."""

    def __init__(self, a, b):
        self.boards = {"a": a, "b": b}

    def enumerate(self, vid, pid):
        return [dict(d, path=k.encode() + b":" + d["path"])
                for k, dev in self.boards.items() for d in dev.enumerate(vid, pid)]

    def Device(self, path):  # noqa: N802 - hid API
        k, rest = path.split(b":", 1)
        return self.boards[k.decode()].Device(rest.decode())


class FakeMacListing:
    """A keyboard as hidapi 0.15 lists it on macOS: an interface once per top-level collection of its report
    descriptor, each entry with the interface's one path (the stock's interface 2 is listed seven times)."""

    def __init__(self, dev, times=7):
        self.dev, self.times = dev, times

    def enumerate(self, vid, pid):
        out = []
        for d in self.dev.enumerate(vid, pid):
            out += [dict(d, usage=k) for k in range(self.times if d["interface_number"] == 2 else 1)]
        return out

    def Device(self, path):  # noqa: N802 - hid API
        return self.dev.Device(path)


def run_cmd(fake, *argv):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(list(argv))
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class FakeZmkBattery(FakeZmk):
    """tlsr_usb_ota.c's battery command (0xff06): mV, ADC code, flags, percent, "ZC"; or a status."""

    def __init__(self, reading=None, status=0):
        super().__init__()
        self.reading = reading  # (mv, raw, flags, percent)
        self.status = status

    def handle(self, r):
        r = bytearray(r)
        if r[9] | r[10] << 8 == t.CMD_BATTERY:
            if self.status:
                r[29] = self.status
            else:
                mv, raw, flags, pct = self.reading
                struct.pack_into("<HHBB", r, 11, mv, raw, flags, pct)
                r[21:23] = b"ZC"
            self.reply(r)
            return
        super().handle(bytes(r))


def run_battery(fake):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(["battery", "--vid", "1d50", "--pid", "615e", "--timeout", "20"])
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class TwoKeyboardsTest(unittest.TestCase):
    """A command reaches one keyboard only: with two on the ID, --product picks it, and without it (or
    with two of that product string) nothing is sent to either (the V21 and the V75 Pro on one host)."""

    @classmethod
    def setUpClass(cls):
        cls.path = temp_image(model_image(22, 2000))

    @classmethod
    def tearDownClass(cls):
        os.unlink(cls.path)

    def two_zmk(self):
        v21, v75 = FakeZmk(), FakeZmk()
        v21.product = "V21 ZMK"
        return v21, v75

    def test_info_needs_product(self):
        v21, v75 = self.two_zmk()
        code, out = run_cmd(FakeTwo(v21, v75), "info", "--vid", "1d50", "--pid", "615e", "--timeout", "20")
        self.assertNotEqual(code, 0)
        self.assertIn("2 keyboards on 1d50:615e", out)
        self.assertIn("--product", out)
        self.assertEqual((v21.sent, v75.sent), (0, 0))

    def test_info_with_product(self):
        v21, v75 = self.two_zmk()
        code, out = run_cmd(FakeTwo(v21, v75), "info", "--vid", "1d50", "--pid", "615e", "--timeout", "20",
                            "--product", "V75 Pro ZMK")
        self.assertEqual(code, 0, out)
        self.assertIn("device: 'V75 Pro ZMK'", out)
        self.assertEqual(v21.sent, 0)
        self.assertGreater(v75.sent, 0)

    def test_battery_with_product(self):
        v21, v75 = FakeZmkBattery((2466, 2079, 3, 50)), FakeZmkBattery((2466, 2079, 3, 50))
        v21.product = "V21 ZMK"
        code, out = run_cmd(FakeTwo(v21, v75), "battery", "--vid", "1d50", "--pid", "615e", "--timeout", "20")
        self.assertNotEqual(code, 0)
        self.assertIn("2 keyboards on 1d50:615e", out)
        self.assertEqual((v21.sent, v75.sent), (0, 0))
        code, out = run_cmd(FakeTwo(v21, v75), "battery", "--vid", "1d50", "--pid", "615e", "--timeout", "20",
                            "--product", "V21 ZMK")
        self.assertEqual(code, 0, out)
        self.assertGreater(v21.sent, 0)
        self.assertEqual(v75.sent, 0)

    def test_info_product_not_connected(self):
        v21, _ = self.two_zmk()
        code, out = run_cmd(v21, "info", "--vid", "1d50", "--pid", "615e", "--timeout", "20",
                            "--product", "V75 Pro ZMK")
        self.assertNotEqual(code, 0)
        self.assertIn("with the product string 'V75 Pro ZMK'", out)
        self.assertEqual(v21.sent, 0)

    def test_confirm_needs_product(self):
        a, b = FakeZmkConfirm(), FakeZmkConfirm()
        a.product = "V21 ZMK"
        code, out = run_cmd(FakeTwo(a, b), "confirm", "--vid", "1d50", "--pid", "615e", "--timeout", "20")
        self.assertNotEqual(code, 0)
        self.assertEqual((a.sent, b.sent, a.confirms, b.confirms), (0, 0, [], []))
        code, out = run_cmd(FakeTwo(a, b), "confirm", "--vid", "1d50", "--pid", "615e", "--timeout", "20",
                            "--product", "V75 Pro ZMK")
        self.assertEqual(code, 0, out)
        self.assertEqual((a.sent, a.confirms, b.confirms), (0, [], [0]))

    def test_flash_zmk_needs_product(self):
        a, b = FakeZmkGate(), FakeZmkGate()
        a.product = "V21 ZMK"
        code, out = run_cmd(FakeTwo(a, b), "flash", self.path, "--vid", "1d50", "--pid", "615e",
                            "--timeout", "20", "--yes", "--overwrite-other-slot")
        self.assertNotEqual(code, 0)
        self.assertEqual((a.sent, b.sent), (0, 0))

    def test_stock_flash_picks_the_product(self):
        v21, v75 = FakeStock(), FakeStock()
        v21.product = "CIDOO V21"
        code, out = run_cmd(FakeTwo(v21, v75), "flash", self.path, "--vid", "320f", "--pid", "5055",
                            "--product", "CIDOO V75", "--timeout", "20", "--yes")
        self.assertEqual(code, 0, out)
        self.assertEqual((v21.sent, v21.rebooted, v75.rebooted), (0, None, "done"))

    def test_one_keyboard_listed_per_collection(self):
        """macOS lists the stock's interface 2 seven times with one path: that is one keyboard."""
        v75 = FakeStock()
        code, out = run_cmd(FakeMacListing(v75), "flash", self.path, "--vid", "320f", "--pid", "5055",
                            "--product", "CIDOO V75", "--timeout", "20", "--yes")
        self.assertEqual((code, v75.rebooted), (0, "done"), out)
        zmk = FakeZmk()
        code, out = run_cmd(FakeMacListing(zmk), "info", "--vid", "1d50", "--pid", "615e", "--timeout", "20")
        self.assertEqual(code, 0, out)

    def test_two_keyboards_listed_per_collection(self):
        """The V21 and the V75 Pro stocks, each listed macOS-style: --product picks the V75 Pro; two V75 Pros
        are still two."""
        v21, v75 = FakeStock(), FakeStock()
        v21.product = "CIDOO V21"
        code, out = run_cmd(FakeTwo(FakeMacListing(v21), FakeMacListing(v75)), "flash", self.path, "--vid", "320f",
                            "--pid", "5055", "--product", "CIDOO V75", "--timeout", "20", "--yes")
        self.assertEqual(code, 0, out)
        self.assertEqual((v21.sent, v21.rebooted, v75.rebooted), (0, None, "done"))
        a, b = FakeStock(), FakeStock()
        code, out = run_cmd(FakeTwo(FakeMacListing(a), FakeMacListing(b)), "flash", self.path, "--vid", "320f",
                            "--pid", "5055", "--product", "CIDOO V75", "--timeout", "20", "--yes")
        self.assertNotEqual(code, 0)
        self.assertIn("2 keyboards on 320f:5055", out)
        self.assertEqual((a.sent, b.sent), (0, 0))

    def test_stock_flash_two_of_one_product(self):
        a, b = FakeStock(), FakeStock()
        code, out = run_cmd(FakeTwo(a, b), "flash", self.path, "--vid", "320f", "--pid", "5055",
                            "--product", "CIDOO V75", "--timeout", "20", "--yes")
        self.assertNotEqual(code, 0)
        self.assertIn("unplug all but the one meant", out)
        self.assertEqual((a.sent, b.sent), (0, 0))


def flash_args(path, vid_pid, product=None, *extra):
    return ["flash", path, "--vid", vid_pid[0], "--pid", vid_pid[1], *(["--product", product] if product else []),
            "--timeout", "20", "--yes", *extra]


STOCK, ZMK = ("320f", "5055"), ("1d50", "615e")


class ModelTest(unittest.TestCase):
    """flash and confirm RESCUE send an image only to a keyboard of the model it is for (a V21 image to the
    V21, stock or zmk-tc32, a V75 Pro image to the V75 Pro); otherwise nothing is written to the device."""

    @classmethod
    def setUpClass(cls):
        sd = t.string_descriptor
        cls.paths = {name: temp_image(model_image(seed, 2000, product)) for seed, (name, product) in enumerate((
            ("v21_zmk", b"V21 ZMK\0"), ("v21_stock", sd("CIDOO V21")),
            ("v75_zmk", b"V75 Pro ZMK\0"), ("v75_stock", sd("CIDOO V75")),
            ("none", b""), ("both", b"V21 ZMK\0" + sd("CIDOO V75")),
            # Not a model's string: the BLE name (no NUL after the product string), the text outside a
            # descriptor, another product's descriptor.
            ("ble_name", b"V21 ZMK 1\0"), ("bare", "CIDOO V75".encode("utf-16le")),
            ("plus", sd("CIDOO V75 PLUS")), ("v87", sd("CIDOO V87"))), start=40)}

    @classmethod
    def tearDownClass(cls):
        for p in cls.paths.values():
            os.unlink(p)

    def refused(self, fake, argv, why):
        code, out = run_cmd(fake, *argv)
        self.assertNotEqual(code, 0, out)
        self.assertIn(why, out)
        if fake is not None:
            self.assertEqual((fake.writes, fake.rebooted), (0, None), out)
        return out

    def test_check_names_the_model(self):
        for name, model in (("v21_zmk", "V21"), ("v21_stock", "V21"), ("v75_zmk", "V75 Pro"), ("v75_stock", "V75 Pro")):
            code, out = run_cmd(None, "check", self.paths[name])
            self.assertEqual(code, 0, out)
            self.assertIn(f"chunks, for the {model}", out)
        for name in ("none", "ble_name", "bare", "plus", "v87"):
            self.refused(None, ["check", self.paths[name]], "holds no known model's USB product string")
        self.refused(None, ["check", self.paths["both"]], "more than one model")

    def test_stock_of_the_other_model(self):
        """The product given for 320F:5055 decides before anything is opened."""
        self.refused(FakeStock(), flash_args(self.paths["v21_zmk"], STOCK, "CIDOO V75"),
                     "the image is for the V21, and --product 'CIDOO V75' is the V75 Pro's: not sent")
        v21 = FakeStock()
        v21.product = "CIDOO V21"
        self.refused(v21, flash_args(self.paths["v75_zmk"], STOCK, "CIDOO V21"),
                     "the image is for the V75 Pro, and --product 'CIDOO V21' is the V21's")
        self.refused(v21, flash_args(self.paths["v75_stock"], STOCK, "CIDOO V21"), "is the V21's")
        self.refused(None, flash_args(self.paths["v21_zmk"], STOCK, "CIDOO V75", "--dry-run"), "is the V75 Pro's")

    def test_zmk_of_the_other_model(self):
        """1D50:615E without --product: the keyboard's product string decides, before the version command."""
        v75 = FakeZmkGate()
        out = self.refused(v75, flash_args(self.paths["v21_zmk"], ZMK, None, "--overwrite-other-slot"),
                           "the image is for the V21, and the keyboard's product string 'V75 Pro ZMK' is the "
                           "V75 Pro's")
        self.assertEqual(v75.cmds, [], out)
        v21 = FakeZmkGate()
        v21.product = "V21 ZMK"
        self.refused(v21, flash_args(self.paths["v75_stock"], ZMK, None, "--overwrite-other-slot"),
                     "the image is for the V75 Pro")
        self.refused(v21, flash_args(self.paths["v75_zmk"], ZMK, "V21 ZMK", "--overwrite-other-slot"),
                     "the image is for the V75 Pro, and --product 'V21 ZMK' is the V21's")

    def test_own_model_goes_ahead(self):
        v21 = FakeStock()
        v21.product = "CIDOO V21"
        code, out = run_cmd(v21, *flash_args(self.paths["v21_zmk"], STOCK, "CIDOO V21"))
        self.assertEqual((code, v21.rebooted), (0, "done"), out)
        self.assertIn("an image for the V21", out)
        # Back to the stock from zmk-tc32: the stock image of the same model.
        zmk = FakeZmkGate()
        zmk.product = "V21 ZMK"
        code, out = run_cmd(zmk, *flash_args(self.paths["v21_stock"], ZMK, None, "--overwrite-other-slot"))
        self.assertEqual((code, zmk.rebooted), (0, "done"), out)
        code, out = run_cmd(None, *flash_args(self.paths["v21_stock"], STOCK, "CIDOO V21", "--dry-run"))
        self.assertEqual(code, 0, out)

    def test_unknown_keyboard_refused(self):
        v87 = FakeStock()
        v87.product = "CIDOO V87"
        self.refused(v87, flash_args(self.paths["v75_zmk"], STOCK, "CIDOO V87"),
                     "--product 'CIDOO V87' is no known model's")
        other = FakeZmkGate()
        other.product = "Pad ZMK"
        self.refused(other, flash_args(self.paths["v75_zmk"], ZMK, None, "--overwrite-other-slot"),
                     "the keyboard's product string 'Pad ZMK' is no known model's")

    def test_image_without_one_model_not_sent(self):
        for name, why in (("none", "no known model's"), ("both", "more than one model"), ("ble_name", "no known")):
            self.refused(FakeStock(), flash_args(self.paths[name], STOCK, "CIDOO V75"), why)
            self.refused(FakeZmkGate(), flash_args(self.paths[name], ZMK, None, "--overwrite-other-slot"), why)

    def test_confirm_takes_no_image(self):
        # The confirm's test is the firmware's own flash test: nothing is sent, so no image is taken.
        argv = ["--vid", "1d50", "--pid", "615e", "--timeout", "20"]
        v75 = FakeZmkConfirm()
        code, out = run_cmd(v75, "confirm", self.paths["v21_zmk"], *argv)
        self.assertNotEqual(code, 0)
        self.assertEqual((v75.sent, v75.tests, v75.confirms), (0, [], []))
        v21 = FakeZmkConfirm()
        v21.product = "V21 ZMK"
        code, out = run_cmd(v21, "confirm", *argv, "--product", "V21 ZMK")
        self.assertEqual((code, v21.tests, v21.confirms), (0, [0], [0]), out)

class BatteryTest(unittest.TestCase):
    def test_reading(self):
        code, out = run_battery(FakeZmkBattery((2466, 2079, 3, 50)))
        self.assertEqual(code, 0, out)
        self.assertIn("battery: 2466 mV (ADC code 2079), 50 %", out)
        self.assertIn("power in: yes, charging", out)
        code, out = run_battery(FakeZmkBattery((2180, 1831, 1, 9)))
        self.assertIn("power in: yes, charge complete", out)
        code, out = run_battery(FakeZmkBattery((2180, 1831, 0, 9)))
        self.assertIn("power in: no", out)

    def test_older_and_without(self):
        code, out = run_battery(FakeZmkBattery(status=8))
        self.assertNotEqual(code, 0)
        self.assertIn("bad report", out)
        code, out = run_battery(FakeZmkBattery(status=16))
        self.assertNotEqual(code, 0)
        self.assertIn("no battery measurement", out)

    def test_stock_id_refused(self):
        fake = FakeZmkBattery((2466, 2079, 3, 50))
        t._hid_module = lambda: fake
        with self.assertRaises(SystemExit):
            t.main(["battery", "--vid", "320f", "--pid", "5055"])


class FakeZmkRng(FakeZmk):
    """src/tlsr_rng_measure.c's command 0xff0e: the restart batch at boot (1000 jitter octets, 1001 codes), takes
    of one source filling a buffer of size octets (an ADC take cut short to the next count in short, the source
    stopped), status and 18-octet pages; or status 8 (a daily image)."""

    def __init__(self, size=3072, daily=False, seed=1, short=None, ready=(0, 0, 1, 128, 0, 12, 9, 0, 0)):
        super().__init__()
        self.size, self.daily, self.rnd = size, daily, random.Random(seed)
        self.short = list(short or [])
        # readiness: the collect's return, state before and after, bits, record, ms, windows credited, refused for
        # range and by the health tests
        self.ready = ready
        self.reads, self.takes = 0, 0
        jit = bytes(self.rnd.randrange(256) for _ in range(1000))
        codes = [0x1DF5 + self.rnd.randrange(-4, 5) for _ in range(1001)]
        self.buf, self.kind, self.first, self.second = jit + struct.pack("<1001H", *codes), 1, 1000, 1001
        self.jitter_out, self.codes_out = [jit], [codes]

    def handle(self, r):
        r = bytearray(r)
        if r[9] | r[10] << 8 != t.CMD_RNG_SAMPLES:
            super().handle(bytes(r))
            return
        op = r[11]
        if self.daily or op > 3:
            r[29] = 8
        elif op == 3:
            err, before, state, bits, record, ms, windows, rng_range, health = self.ready
            r[11:29] = bytes(18)
            struct.pack_into("<bBBBBHH", r, 11, err, before, state, bits, record, ms, windows)
            r[21:23] = b"ZC"
            struct.pack_into("<HH", r, 23, rng_range, health)
        elif op == 2:
            page = struct.unpack_from("<H", r, 12)[0]
            if page == 0 and self.kind == 1:
                self.reads += 1
            r[11:29] = self.buf[page * 18:page * 18 + 18].ljust(18, b"\0")
        else:
            if op == 1 and r[12] == 0:
                self.buf = bytes(self.rnd.randrange(256) for _ in range(self.size))
                self.kind, self.first, self.second = 2, self.size, 0
                self.jitter_out.append(self.buf)
                self.takes += 1
            elif op == 1:
                n = self.short.pop(0) if self.short else self.size // 2
                codes = [0x1DF5 + self.rnd.randrange(-4, 5) for _ in range(n)]
                self.buf, self.kind, self.first, self.second = struct.pack(f"<{len(codes)}H", *codes), 3, len(codes), 0
                self.codes_out.append(codes)
                self.takes += 1
            r[11] = self.kind
            struct.pack_into("<4H", r, 12, len(self.buf), self.first, self.second, self.size)
            r[20] = self.reads
            r[21:23] = b"ZC"
            struct.pack_into("<IH", r, 23, 7, self.takes)
        self.reply(r)


def slurp(path):
    with open(path, "rb") as f:
        return f.read()


def run_rng(fake, *args):
    t._hid_module = lambda: fake
    out = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out):
        try:
            t.main(["rng-samples", *args, "--vid", "1d50", "--pid", "615e", "--timeout", "20"])
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
            out.write(str(e.code))
    return code, out.getvalue()


class RngSamplesTest(unittest.TestCase):
    def test_continuous_jitter(self):
        fake = FakeZmkRng()
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "m")
            code, out = run_rng(fake, "continuous", "--source", "jitter", "--count", "7000", "--out", p)
            self.assertEqual(code, 0, out)
            data = slurp(p + "-jitter.bin")
            self.assertEqual(len(data), 7000)
            self.assertEqual(data, b"".join(fake.jitter_out[1:])[:7000])
            self.assertIn("7000 samples in 3 runs", out)
            code, out = run_rng(FakeZmkRng(), "continuous", "--count", "10", "--out", p)
            self.assertNotEqual(code, 0)
            self.assertIn("exists", out)

    def test_continuous_adc(self):
        fake = FakeZmkRng()
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "m")
            code, out = run_rng(fake, "continuous", "--source", "adc", "--count", "2000", "--out", p)
            self.assertEqual(code, 0, out)
            runs = fake.codes_out[1:]
            codes = [c for run in runs for c in run][:2000]
            raw = slurp(p + "-adc.u16")
            self.assertEqual(list(struct.unpack(f"<{len(raw) // 2}H", raw)), codes)
            self.assertEqual(slurp(p + "-adc-low8.bin"), bytes(c & 0xFF for c in codes))
            diff = slurp(p + "-adc-diff4.bin")
            # differences within each run only: 1536 codes give 1535, the 464 of the second run 463
            self.assertEqual(len(diff), 1535 + 463)
            first = runs[0]
            self.assertEqual(diff[:1535], bytes((b - a) & 15 for a, b in zip(first, first[1:])))

    def test_continuous_adc_cut_short(self):
        fake = FakeZmkRng(short=[100, 700])
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "m")
            code, out = run_rng(fake, "continuous", "--source", "adc", "--count", "2000", "--out", p)
            self.assertEqual(code, 0, out)
            self.assertIn("2000 samples in 3 runs of up to 1536 (2 cut short", out)
            codes = [c for run in fake.codes_out[1:] for c in run][:2000]
            raw = slurp(p + "-adc.u16")
            self.assertEqual(list(struct.unpack(f"<{len(raw) // 2}H", raw)), codes)
            self.assertEqual(os.path.getsize(p + "-adc-diff4.bin"), 99 + 699 + 1199)
            code, out = run_rng(FakeZmkRng(short=[0]), "continuous", "--source", "adc", "--count", "10",
                                "--out", os.path.join(d, "z"))
            self.assertNotEqual(code, 0)
            self.assertIn("gave nothing", out)

    def test_restart_rows(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "m")
            fakes = [FakeZmkRng(seed=s) for s in (1, 2)]
            for f in fakes:
                code, out = run_rng(f, "restart", "--out", p)
                self.assertEqual(code, 0, out)
                self.assertIn("restart row: 1000 jitter samples and 1001 ADC codes taken 7 ms after the boot", out)
            self.assertEqual(slurp(p + "-restart-jitter.bin"),
                             fakes[0].jitter_out[0] + fakes[1].jitter_out[0])
            self.assertEqual(os.path.getsize(p + "-restart-adc.u16"), 2 * 2 * 1001)
            low8 = slurp(p + "-restart-adc-low8.bin")
            self.assertEqual(low8[:1000], bytes(c & 0xFF for c in fakes[0].codes_out[0][:1000]))
            self.assertEqual(len(low8), 2000)
            self.assertEqual(os.path.getsize(p + "-restart-adc-diff4.bin"), 2000)
            # the same power-on again: the batch was read
            code, out = run_rng(fakes[1], "restart", "--out", p)
            self.assertNotEqual(code, 0)
            self.assertIn("no unread restart batch", out)
            self.assertEqual(os.path.getsize(p + "-restart-jitter.bin"), 2000)

    def test_status(self):
        code, out = run_rng(FakeZmkRng(ready=(0, 0, 1, 128, 0, 23, 4, 1, 2)), "status")
        self.assertEqual(code, 0, out)
        self.assertIn("tc32_rng: ready; collect 0 after 23 ms, not ready before it; 128 credited bits; "
                      "the seed record at init did not count", out)
        self.assertIn("ADC windows since boot: 4 credited, 1 refused for a code out of VBAT's range, 2 refused by "
                      "the health tests", out)
        code, out = run_rng(FakeZmkRng(ready=(-116, 0, 0, 40, 0, 251, 0, 0, 3)), "status")
        self.assertNotEqual(code, 0)
        self.assertIn("tc32_rng: not ready; collect -116 after 251 ms", out)
        self.assertIn("the generator is not ready", out)
        code, out = run_rng(FakeZmkRng(daily=True), "status")
        self.assertNotEqual(code, 0)
        self.assertIn("not a measurement image", out)
        code, out = run_rng(FakeZmkRng(), "continuous", "--count", "10")
        self.assertNotEqual(code, 0)
        self.assertIn("give --out", out)

    def test_daily_image_refused(self):
        with tempfile.TemporaryDirectory() as d:
            code, out = run_rng(FakeZmkRng(daily=True), "continuous", "--count", "10", "--out", os.path.join(d, "m"))
            self.assertNotEqual(code, 0)
            self.assertIn("not a measurement image", out)


class InfoTest(unittest.TestCase):
    def test_measured_clock(self):
        code, out = run_info(FakeZmk())
        self.assertEqual(code, 0, out)
        self.assertIn("system clock: 16000000 Hz (measured", out)
        self.assertIn("period 4.00 s", out)
        self.assertIn("boots counted at this boot without a healthy one: 1", out)

    def test_flash_status_bytes(self):
        """Bytes 19-20, 23-24 (status as found and as left), 25 (flags) and 26-28 (JEDEC ID), each in its place."""
        code, out = run_info(FakeZmk(flash=(0x0018, 0x0000, 0x11, 0x1360C8)))
        self.assertEqual(code, 0, out)
        self.assertIn("flash: JEDEC ID 0x1360c8; status register 0x0018 at boot, 0x0000 now (the boot guard cleared the block protection)", out)
        self.assertNotIn("did not read back", out)
        code, out = run_info(FakeZmk(flash=(0x0018, 0x0018, 0x12 | 0x04, 0x13325E)))
        self.assertIn("flash: JEDEC ID 0x13325e; status register 0x0018 at boot, 0x0018 now (STILL LOCKED after the boot guard's writes)", out)
        self.assertIn("did not read back", out)
        code, out = run_info(FakeZmk(flash=(0x00FF, 0x00FF, 0x10 | 0x20, 0x146085)))
        self.assertIn("status register 0x00ff at boot, 0x00ff now (read as busy or write-enabled, a misread: that write not made)", out)
        code, out = run_info(FakeZmk())
        self.assertIn("flash status register: not read", out)

    def test_fallback_clock(self):
        code, out = run_info(FakeZmk(flags=2))
        self.assertEqual(code, 0, out)
        self.assertIn("not measured", out)

    def test_diagnostics(self):
        """Bytes 30-32: the other slot's check, the running slot, the CPU left over and the uptime,
        each in its place; info asks for the CPU figure (request byte 32 = 1)."""
        fake = FakeZmk(diag=(0x40 | 0x01 | 0x80, 42, 17))
        code, out = run_info(fake)
        self.assertEqual(code, 0, out)
        self.assertEqual(fake.version_request[32], 1, "info did not ask for the CPU figure")
        self.assertIn("running from slot B (0x20000)", out)
        self.assertIn("other slot: its image checks", out)
        self.assertIn("CPU left for the lowest-priority thread: 42% over 100 ms", out)
        self.assertIn("uptime: 17 s", out)
        self.assertIn("boots counted at this boot without a healthy one: 1", out)  # the runners parse it
        code, out = run_info(FakeZmk(diag=(0x40, 0xFF, 255)))
        self.assertEqual(code, 0, out)
        self.assertIn("running from slot A (0x00000)", out)
        self.assertIn("other slot: NO image that checks", out)
        self.assertIn("CPU left for the lowest-priority thread: not measured", out)
        self.assertIn("uptime: 255 s or more", out)

    def test_planned_reboot_mark(self):
        """Byte 30 bits 3-5: the planned-reboot mark the boot guard found at this boot. Without bit 5
        (a receiver from before the mark moved to analog 0x3c) nothing is said about it."""
        code, out = run_info(FakeZmk(diag=(0x40 | 0x20 | 0x08, 0xFF, 3)))
        self.assertEqual(code, 0, out)
        self.assertIn("this boot: a reboot the firmware asked for, into this image", out)
        self.assertNotIn("another image's", out)
        self.assertNotIn("did not read back its cleared value", out)
        code, out = run_info(FakeZmk(diag=(0x40 | 0x20 | 0x10, 0xFF, 3)))
        self.assertEqual(code, 0, out)
        self.assertIn("this boot: not a reboot the firmware asked for, into this image", out)
        self.assertIn("planned-reboot mark (analog 0x3c) at this boot: another image's", out)
        code, out = run_info(FakeZmk(diag=(0x40 | 0x20, 0xFF, 3)))
        self.assertIn("this boot: not a reboot the firmware asked for", out)
        self.assertNotIn("another image's", out)
        code, out = run_info(FakeZmk(diag=(0x40 | 0x08 | 0x10, 0xFF, 3)))
        self.assertEqual(code, 0, out)
        self.assertNotIn("this boot:", out)
        self.assertNotIn("planned-reboot mark", out)

    def test_planned_reboot_mark_stuck(self):
        """Byte 18 bit 4: the mark register did not read back its cleared value, so the boot was taken as
        unplanned."""
        code, out = run_info(FakeZmk(flags=3 | 16, diag=(0x40 | 0x20, 0xFF, 3)))
        self.assertEqual(code, 0, out)
        self.assertIn("the planned-reboot mark register (analog 0x3c) did not read back its cleared value at "
                      "this boot: the boot was taken as unplanned", out)
        code, out = run_info(FakeZmk(diag=(0x40 | 0x20, 0xFF, 3)))
        self.assertNotIn("did not read back its cleared value", out)

    def test_receiver_without_diagnostics(self):
        """An older receiver echoes bytes 30-32 (byte 30 bit 6 clear): nothing is read into them."""
        code, out = run_info(FakeZmk())
        self.assertEqual(code, 0, out)
        self.assertNotIn("other slot:", out)
        self.assertNotIn("uptime:", out)

    def test_stock_echo_has_no_info(self):
        fake = FakeStock()
        code, out = run_info(fake)
        self.assertNotEqual(code, 0)
        self.assertIn("no zmk-tc32 information", out)

    def test_stock_id_refused(self):
        fake = FakeStock()
        code, out = run_info(fake, "320f", "5055")
        self.assertNotEqual(code, 0)
        self.assertEqual(fake.sent, 0, "sent a report to the stock ID")


if __name__ == "__main__":
    unittest.main()
