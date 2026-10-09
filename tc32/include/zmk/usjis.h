/*
 * Copyright (c) 2026 Go Yamamoto
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* US-JIS substitution: public API (docs/usjis-substitution.md, docs/usjis-architecture.md). */

enum zmk_usjis_op { ZMK_USJIS_OP_OFF = 0, ZMK_USJIS_OP_ON = 1, ZMK_USJIS_OP_TOGGLE = 2 };

/* Effective mode (spec section 3). Substitution also requires an active layer
 * in CONFIG_ZMK_USJIS_LAYER_MASK and none in CONFIG_ZMK_USJIS_LAYER_EXCLUDE_MASK. */
bool zmk_usjis_is_enabled(void);
/* True while a mode request waits for all non-modifier keys to be released. */
bool zmk_usjis_is_pending(void);
/* A mode operation (spec section 7). Returns the requested mode. */
bool zmk_usjis_request(enum zmk_usjis_op op);
/* Number of press records currently held (substituted or not). */
unsigned zmk_usjis_active_entries(void);
/* False if the startup listener-order check failed; the module then never substitutes. */
bool zmk_usjis_listener_order_ok(void);
/* Number of mode values written to settings since boot (0 without settings). */
unsigned zmk_usjis_settings_writes(void);
/* Called on the system work queue each time the effective mode changes (a
 * request applied at once or a pending one applied at the last release). The
 * default does nothing; a board may define it to show the new mode. */
void zmk_usjis_mode_applied(bool enabled);
