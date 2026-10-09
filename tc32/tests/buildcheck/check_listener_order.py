#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Read the keycode_state_changed subscriptions of a ZMK ELF in link order.

Prints the listeners subscribed to zmk_keycode_state_changed as they appear
between __event_subscriptions_start and __event_subscriptions_end, and checks
the condition zmk-usjis tests at startup (check_listener_order in
src/usjis.c): usjis after hold-tap and key repeat, and before hid_listener.

Usage: check_listener_order.py build/<dir>/zephyr/zmk.elf
"""
import struct
import sys

from elftools.elf.elffile import ELFFile

ENTRY = 8  # struct zmk_event_subscription: two 32-bit pointers


def main(path):
    with open(path, "rb") as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name(".symtab")
        addr = {}
        name_at = {}
        for sym in symtab.iter_symbols():
            if sym["st_info"]["type"] in ("STT_OBJECT", "STT_NOTYPE") and sym.name:
                addr[sym.name] = sym["st_value"]
                name_at.setdefault(sym["st_value"], []).append(sym.name)
        start, end = addr["__event_subscriptions_start"], addr["__event_subscriptions_end"]
        keycode_type = addr["zmk_event_zmk_keycode_state_changed"]
        data = None
        for sec in elf.iter_sections():
            base, size = sec["sh_addr"], sec["sh_size"]
            if sec["sh_type"] != "SHT_NOBITS" and base <= start and end <= base + size:
                data = sec.data()[start - base:end - base]
                break
        if data is None:
            sys.exit("subscription range not found in any section")

    order = []
    for i in range(0, len(data), ENTRY):
        event_type, listener = struct.unpack_from("<II", data, i)
        if event_type == keycode_type:
            names = [n for n in name_at.get(listener, []) if n.startswith("zmk_listener_")]
            order.append(names[0][len("zmk_listener_"):] if names else hex(listener))
    for i, name in enumerate(order):
        print(f"{i:2d} {name}")

    def idx(name):
        return order.index(name) if name in order else -1

    usjis, hid = idx("usjis"), idx("hid_listener")
    before = {n: idx(n) for n in ("behavior_hold_tap", "behavior_key_repeat")}
    ok = usjis >= 0 and hid >= 0 and usjis < hid and all(i < usjis for i in before.values())
    print(f"usjis={usjis} hid_listener={hid} " +
          " ".join(f"{n}={i}" for n, i in before.items()) + f" -> {'OK' if ok else 'WRONG'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
