/*
 * Copyright (c) 2026 Go Yamamoto
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "usjis_resolver.h"

/* Canonical substitution table, spec section 4. Usage IDs are HID keyboard
 * page values (HID Usage Tables 1.12, section 10). */
struct usjis_row {
    const char *id;
    uint8_t input_usage;
    bool input_shift;
    uint8_t output_usage;
    bool output_shift;
};

#define GRAVE 0x35
#define N2 0x1F
#define N6 0x23
#define N7 0x24
#define N8 0x25
#define N9 0x26
#define N0 0x27
#define MINUS 0x2D
#define EQUAL 0x2E
#define LBKT 0x2F
#define RBKT 0x30
#define BSLH 0x31
#define SEMI 0x33
#define SQT 0x34
/* JIS host positions (spec section 4, second table). */
#define JIS_CARET 0x2E
#define JIS_AT 0x2F
#define JIS_LBKT 0x30
#define JIS_RBKT 0x32
#define JIS_COLON 0x34
#define JIS_RO 0x87
#define JIS_YEN 0x89

static const struct usjis_row rows[] = {
    {"C01", GRAVE, true, JIS_CARET, true},
    {"C02", N2, true, JIS_AT, false},
    {"C03", N6, true, JIS_CARET, false},
    {"C04", N7, true, N6, true},
    {"C05", N8, true, JIS_COLON, true},
    {"C06", N9, true, N8, true},
    {"C07", N0, true, N9, true},
    {"C08", MINUS, true, JIS_RO, true},
    {"C09", EQUAL, false, MINUS, true},
    {"C10", EQUAL, true, SEMI, true},
    {"C11", LBKT, false, JIS_LBKT, false},
    {"C12", LBKT, true, JIS_LBKT, true},
    {"C13", RBKT, false, JIS_RBKT, false},
    {"C14", RBKT, true, JIS_RBKT, true},
    {"C15", BSLH, false, JIS_RO, false},
    {"C16", BSLH, true, JIS_YEN, true},
    {"C17", SEMI, true, JIS_COLON, false},
    {"C18", SQT, false, N7, true},
    {"C19", SQT, true, N2, true},
    {"C20", GRAVE, false, JIS_AT, true},
};

struct usjis_resolution usjis_resolve(uint8_t input_usage, bool shift) {
    for (unsigned i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        if (rows[i].input_usage == input_usage && rows[i].input_shift == shift) {
            return (struct usjis_resolution){.substituted = true,
                                             .output_usage = rows[i].output_usage,
                                             .output_shift = rows[i].output_shift,
                                             .id = rows[i].id};
        }
    }
    return (struct usjis_resolution){.substituted = false, .output_usage = input_usage};
}
