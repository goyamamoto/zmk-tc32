#!/usr/bin/env python3
# Copyright (c) 2026 Go Yamamoto
# SPDX-License-Identifier: GPL-3.0-or-later
"""Which lines of the 2 KB direct-mapped flash cache (64 slots of 32 bytes) the key scan's code and the constants
it reads fall in, for one zmk.elf: a constant in a slot of the scan's code is fetched again, and the code after
it, for every output the scan drives; and so are two of the constants that lie in one slot from different lines
(a GPIO call reads the API table, the port's device structure and its config: each evicts the other).

hot_slots.py NM ZMK_ELF [--pads]   (exit status 1 on a shared slot)

--pads: also the changes of CONFIG_TLSR_HOT_TEXT_PAD (bytes more or fewer than this image has, multiples of 32;
2048 more or fewer comes to the same) with which no slot would be shared. The pad follows the hot text (the symbol
_tlsr_hot_text_end of a board's hot_text.ld), so it moves what is linked from there on, the device structures
among it, and leaves the hot text where it is. Without that symbol in the image the constants not placed in the
hot text are taken as moved.

Code: kscan_matrix_work_handler, kscan_matrix_read, kscan_matrix_any_active, kscan_matrix_read_end,
kscan_matrix_schedule, kscan_matrix_read_continue, kscan_gpio_pin_get, gpio_pin_set_dt, gpio_port_get, the GPIO
driver's raw port functions, zmk_debounce_*. Constants: the GPIO ports' device structures (the first of the
device list, as many as there are gpio_b87_config_*), gpio_b87_config_*, gpio_b87_api,
kscan_matrix_config_*.
"""
import re
import subprocess
import sys

CODE = re.compile(r"^(kscan_matrix_(work_handler|read|any_active|read_end|schedule|read_continue)|kscan_gpio_pin_get|"
                  r"gpio_pin_set_dt|gpio_port_get|gpio_b87_port_(get_raw|set_bits_raw|clear_bits_raw)|"
                  r"zmk_debounce_\w+)$")
DATA = re.compile(r"^(gpio_b87_config_\d+|gpio_b87_api|kscan_matrix_config_\d+)$")
IN_HOT_TEXT = re.compile(r"^(gpio_b87_config_\d+|kscan_matrix_config_\d+)$")


def lines(addr, size, shift=0):
    return set(range(addr // 32 + shift, (addr + max(size, 1) - 1) // 32 + 1 + shift))


def main():
    nm, elf = sys.argv[1:3]
    code, data, devs, hot_end = {}, {}, {}, None
    out = subprocess.run([nm, "-S", "-n", elf], capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3 and p[2] == "_tlsr_hot_text_end":
            hot_end = int(p[0], 16)
        if len(p) != 4:
            continue
        a, n, name = int(p[0], 16) & ~1, int(p[1], 16), p[3]
        if a >= 0x800000:
            continue
        if p[2] in "Tt" and CODE.match(name):
            code[f"{name}@{a:x}"] = (a, n)
        elif DATA.match(name):
            data[name] = (a, n)
        elif name.startswith("__device_dts_ord_"):
            devs[name] = (a, n)
    # the GPIO ports: the first device structures of the device list, one for each of the driver's configs
    ports = sum(1 for name in data if name.startswith("gpio_b87_config_"))
    if not ports or not code:
        sys.exit(f"hot_slots.py: {elf}: {len(code)} functions of the scan and {ports} GPIO port configs found by"
                 f" their names: nothing to check")
    for name, (a, n) in sorted(devs.items(), key=lambda kv: kv[1][0])[:ports]:
        data[name] = (a, n)

    def moved(name, a):
        return a >= hot_end if hot_end is not None else not IN_HOT_TEXT.match(name.split("@")[0])

    def shared(shift):
        """name -> what the constant shares a slot with from another line, the pad shift lines larger"""
        used = {}
        for name, (a, n) in code.items():
            for x in lines(a, n, shift if hot_end is not None and moved(name, a) else 0):
                used.setdefault(x % 64, []).append((x, name.split("@")[0]))
        where = {name: lines(a, n, shift if moved(name, a) else 0) for name, (a, n) in data.items()}
        out = {}
        for name, s in where.items():
            # a constant in a line of the code itself is no clash
            hit = sorted({c for x in s for line, c in used.get(x % 64, []) if line != x})
            hit += sorted({other for other, t in where.items() if other != name
                           for x in s for y in t if x != y and x % 64 == y % 64})
            out[name] = (s, hit)
        return out, len(used)

    now, slots_used = shared(0)
    clashes = sum(bool(hit) for _, hit in now.values())
    for name, (s, hit) in sorted(now.items(), key=lambda kv: min(kv[1][0])):
        print(f"{name:26s} slots {sorted(x % 64 for x in s)}" + (f"  SHARED with {', '.join(hit)}" if hit else ""))
    print(f"the scan's code: {slots_used} of 64 slots; {clashes} of {len(data)} constants share one with it or with"
          f" another constant")
    if "--pads" in sys.argv[3:]:
        free = sorted(32 * k for k in range(-31, 33) if not any(hit for _, hit in shared(k)[0].values()))
        print(f"CONFIG_TLSR_HOT_TEXT_PAD: {', '.join(f'{p:+d}' for p in free) or 'no change'} would leave no slot"
              f" shared")
    return 1 if clashes else 0


if __name__ == "__main__":
    sys.exit(main())
