/*
 * Copyright (c) 2026 Go Yamamoto
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Pure substitution table lookup (spec section 4). No ZMK dependency. */

struct usjis_resolution {
    bool substituted;
    uint8_t output_usage;   /* keyboard page usage ID */
    bool output_shift;      /* add Left Shift on the output side */
    const char *id;         /* "C01".."C20" */
};

/* input_usage: keyboard page usage ID of the pressed key.
 * shift: whether Shift (left or right, physical or from the key's own binding)
 * is active at press time. */
struct usjis_resolution usjis_resolve(uint8_t input_usage, bool shift);
