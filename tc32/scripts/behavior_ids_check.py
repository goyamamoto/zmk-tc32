#!/usr/bin/env python3
# Copyright (c) 2026 Go Yamamoto
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check that the behaviors of a ZMK image have distinct local IDs.

With CONFIG_ZMK_BEHAVIOR_LOCAL_ID_TYPE_CRC16 a behavior's local ID, which ZMK
Studio and the saved keymap use to name it, is the CRC-16/ANSI of its device
name. Two behaviors with one ID would be taken for each other. This reads the
names from the image's local ID table (zmk.elf) and fails when two IDs agree.

Usage: behavior_ids_check.py zmk.elf
"""
import struct
import sys

from elftools.elf.elffile import ELFFile


def crc16_ansi(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def main(path):
    with open(path, "rb") as f:
        elf = ELFFile(f)
        sym = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
        sections = [(s["sh_addr"], s.data()) for s in elf.iter_sections()
                    if s["sh_type"] == "SHT_PROGBITS" and s["sh_flags"] & 2]

    def read(addr, size):
        for base, data in sections:
            if base <= addr and addr + size <= base + len(data):
                return data[addr - base:addr - base + size]
        raise SystemExit(f"behavior IDs: nothing loaded at 0x{addr:x}")

    def string(addr):
        out = b""
        while (c := read(addr + len(out), 1)) != b"\0":
            out += c
        return out

    try:
        start, end = sym["_zmk_behavior_local_id_map_list_start"], sym["_zmk_behavior_local_id_map_list_end"]
    except KeyError:
        raise SystemExit("behavior IDs: the image has no local ID table")
    ids = {}
    for entry in range(start, end, 8):  # struct zmk_behavior_local_id_map: device, local_id
        device, = struct.unpack("<I", read(entry, 4))
        name_at, = struct.unpack("<I", read(device, 4))  # struct device starts with the name
        name = string(name_at)
        ids.setdefault(crc16_ansi(name), []).append(name.decode())
    same = {i: n for i, n in ids.items() if len(n) > 1}
    for i, names in sorted(same.items()):
        print(f"behavior IDs: 0x{i:04x} is the ID of {', '.join(names)}")
    if same:
        raise SystemExit("behavior IDs: two behaviors with one local ID; rename one of them")
    print(f"behavior IDs: {sum(len(n) for n in ids.values())} behaviors, all local IDs distinct")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
