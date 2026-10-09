/*
 * One layer's on/off state kept over a boot (CONFIG_TLSR_KEPT_LAYER): a
 * Windows/Mac mode as a ZMK layer, kept in the settings.
 * A change of that layer's state is written to the settings a moment later
 * (one record of one octet, "tc32/layer"), and at boot, once the settings are
 * loaded, the layer is turned on again when it was on.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>

#define LAYER   ((zmk_keymap_layer_id_t)CONFIG_TLSR_KEPT_LAYER)
#define SAVE_MS 1000

static uint8_t kept;    /* the state in the settings */
static uint8_t wanted;  /* the state to write */

static void save(struct k_work *work)
{
	ARG_UNUSED(work);
	if (wanted != kept) {
		if (settings_save_one("tc32/layer", &wanted, 1) == 0) {
			kept = wanted;
		}
	}
}

static K_WORK_DELAYABLE_DEFINE(save_work, save);

static int on_layer(const zmk_event_t *eh)
{
	const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);

	if (ev != NULL && ev->layer == LAYER) {
		wanted = ev->state ? 1U : 0U;
		(void)k_work_reschedule(&save_work, K_MSEC(SAVE_MS));
	}
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(tlsr_layer_keep, on_layer);
ZMK_SUBSCRIPTION(tlsr_layer_keep, zmk_layer_state_changed);

static int set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "layer", &next) && next == NULL && len == 1U) {
		uint8_t v;

		if (read_cb(cb_arg, &v, 1) == 1) {
			kept = v != 0U;
			wanted = kept;
		}
	}
	return 0;
}

/* After the settings are loaded: the layer as it was kept. */
static int commit(void)
{
	if (kept != 0U && !zmk_keymap_layer_active(LAYER)) {
		(void)zmk_keymap_layer_activate(LAYER, true); /* locking, as &to leaves it */
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(tlsr_layer_keep, "tc32", NULL, set, commit, NULL);
