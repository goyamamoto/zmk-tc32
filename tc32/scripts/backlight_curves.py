#!/usr/bin/env python3
# Copyright (c) 2026 Go Yamamoto
# SPDX-License-Identifier: GPL-3.0-or-later
"""The curves of src/led_key_matrix_effects.c as C tables.

  backlight_curves.py    prints the C tables
"""

GAMMA = 1.8
RAMP_UP = [round(255 * ((i + 1) / 32) ** GAMMA) for i in range(32)]
RAMP_DOWN = [round(255 * ((31.5 - i) / 32) ** GAMMA) for i in range(32)]
CURVE = [max(k, round(255 * (k / 63) ** GAMMA)) for k in range(64)]
PRESETS = [0, 8, 12, 19, 20, 29, 31, 32, 34, 35, 39, 44, 50, 55, 63, 64, 72, 76, 83, 87, 91,
           95, 96, 98, 99, 103, 107, 112, 118, 127, 128, 136, 140, 146, 151, 155, 159, 160,
           161, 162, 163, 167, 172, 174, 178, 183]


def wheel(step):
    seg, i = divmod(step, 32)
    up, down = RAMP_UP[i], RAMP_DOWN[i]
    return [(255, up, 0), (down, 255, 0), (0, 255, up), (0, down, 255), (up, 0, 255),
            (255, 0, down)][seg]


def c_array(name, values):
    rows = [", ".join(f"{v}" for v in values[i:i + 16]) for i in range(0, len(values), 16)]
    return f"static const uint8_t {name}[{len(values)}] = {{\n\t" + ",\n\t".join(rows) + ",\n};"


if __name__ == "__main__":
    for name, values in (("ramp_up", RAMP_UP), ("ramp_down", RAMP_DOWN), ("curve", CURVE)):
        print(c_array(name, values))
