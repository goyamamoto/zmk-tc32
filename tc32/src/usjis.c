/* Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * US-JIS substitution: press records, modifier policy and mode control.
 * Spec: docs/usjis-substitution.md. Design: docs/usjis-architecture.md.
 *
 * The listener sits after the behaviors that re-raise keycode events (hold-tap,
 * sticky key, caps word, key repeat) and before hid_listener. While at least one
 * substituted key is held, it handles every keycode event itself so that the
 * report modifiers follow the conflict policy (spec section 6); otherwise it only
 * records presses and lets the baseline hid_listener work unchanged.
 */

#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/keys.h>
#include <zmk/keymap.h>
#include <zmk/usjis.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/modifiers.h>

#include "usjis_resolver.h"

LOG_MODULE_REGISTER(usjis, CONFIG_ZMK_USJIS_LOG_LEVEL);

#define SHIFT_BITS (MOD_LSFT | MOD_RSFT)

/* Press record (architecture section 4). */
struct entry {
    bool used;
    uint16_t input_usage;   /* keyboard page usage ID of the key as pressed */
    bool substituted;
    uint16_t output_usage;  /* usage pressed in the HID report */
    zmk_mod_flags_t added;  /* modifiers this key asks to add */
    zmk_mod_flags_t masked; /* modifiers this key asks to remove */
    uint32_t order;         /* press order, increasing */
    bool cleared;           /* HID state was cleared by an endpoint switch */
    bool non_shift_dropped; /* conflict rule 5: Ctrl/Alt/GUI request not restored */
};

static struct entry entries[CONFIG_ZMK_USJIS_MAX_ENTRIES];
static uint32_t next_order = 1;
static unsigned unrecorded_presses; /* presses beyond capacity (not substituted) */
/* The latest unrecorded press while any is held (0: none): its order and the
 * modifiers it asks to add. It takes part in the conflict policy as a record
 * that adds its own implicit modifiers and masks nothing. */
static uint32_t overflow_order;
static zmk_mod_flags_t overflow_added;

static bool enabled;          /* effective mode */
static bool pending;          /* a requested mode waits for all keys to be released */
static bool pending_enabled;  /* the requested mode */
static bool order_ok;         /* listener order check at startup */
static unsigned settings_writes;

static void persist_mode(bool value);

/* ------------------------------------------------------------ records */

static struct entry *find_entry(uint16_t input_usage) {
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (entries[i].used && entries[i].input_usage == input_usage) {
            return &entries[i];
        }
    }
    return NULL;
}

static struct entry *alloc_entry(void) {
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (!entries[i].used) {
            memset(&entries[i], 0, sizeof(entries[i]));
            entries[i].used = true;
            entries[i].order = next_order++;
            return &entries[i];
        }
    }
    return NULL;
}

static unsigned count_entries(void) {
    unsigned n = 0;
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        n += entries[i].used;
    }
    return n;
}

/* A substituted key that is not cleared is held (spec section 6, conflict rule 7). */
static bool substituted_held(void) {
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (entries[i].used && entries[i].substituted && !entries[i].cleared) {
            return true;
        }
    }
    return false;
}

/* The most recently pressed record that is not cleared. */
static struct entry *latest_entry(void) {
    struct entry *latest = NULL;
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (entries[i].used && !entries[i].cleared &&
            (latest == NULL || entries[i].order > latest->order)) {
            latest = &entries[i];
        }
    }
    return latest;
}

/* The latest press is an unrecorded one. */
static bool overflow_is_latest(void) {
    if (overflow_order == 0) {
        return false;
    }
    struct entry *latest = latest_entry();
    return latest == NULL || overflow_order > latest->order;
}

/* Another held, not cleared record still reports this output usage. */
static bool output_owned_by_other(const struct entry *self, uint16_t output_usage) {
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (entries[i].used && &entries[i] != self && !entries[i].cleared &&
            entries[i].output_usage == output_usage) {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------ modifiers */

/* reported = (physical & ~masked(latest)) | added(latest) while a substituted
 * key is held; otherwise the baseline processing (architecture section 5).
 * hid.c computes the report as (explicit & ~masked) | implicit, so the request
 * is applied through the implicit and masked modifier state. Returns whether
 * the report's modifiers changed. */
static bool apply_modifiers(void) {
    zmk_mod_flags_t before = zmk_hid_get_keyboard_report()->body.modifiers;
    if (!substituted_held()) {
        /* Conflict rule 7: back to the baseline. The baseline mask of held
         * Mod-Morph keys is not restored (Mod-Morph on substitutable keys is
         * not supported). */
        zmk_hid_implicit_modifiers_release();
        zmk_hid_masked_modifiers_clear();
    } else if (overflow_is_latest()) {
        zmk_hid_masked_modifiers_set(0);
        zmk_hid_implicit_modifiers_press(overflow_added);
    } else {
        struct entry *latest = latest_entry();
        zmk_mod_flags_t added = latest->added;
        if (latest->non_shift_dropped) {
            added &= SHIFT_BITS;
        }
        zmk_hid_masked_modifiers_set(latest->masked);
        zmk_hid_implicit_modifiers_press(added);
    }
    return zmk_hid_get_keyboard_report()->body.modifiers != before;
}

static void send_keyboard_report(void) {
    int err = zmk_endpoint_send_report(HID_USAGE_KEY);
    if (err < 0) {
        LOG_ERR("keyboard report failed (%d)", err);
    }
}

/* ------------------------------------------------------------ mode */

static void apply_mode(bool value) {
    if (value == enabled) {
        return;
    }
    enabled = value;
    LOG_INF("mode %s", enabled ? "ENABLED" : "DISABLED");
    persist_mode(enabled);
    zmk_usjis_mode_applied(enabled);
}

__weak void zmk_usjis_mode_applied(bool value) { ARG_UNUSED(value); }

static void apply_pending_if_idle(void) {
    if (pending && count_entries() == 0 && unrecorded_presses == 0) {
        pending = false;
        LOG_INF("applying pending mode");
        apply_mode(pending_enabled);
    }
}

bool zmk_usjis_request(enum zmk_usjis_op op) {
    bool basis = pending ? pending_enabled : enabled;
    bool requested;
    switch (op) {
    case ZMK_USJIS_OP_ON:
        requested = true;
        break;
    case ZMK_USJIS_OP_OFF:
        requested = false;
        break;
    default:
        requested = !basis;
        break;
    }
    if (requested == basis) {
        return requested; /* no-op (spec section 3) */
    }
    if (count_entries() > 0 || unrecorded_presses > 0) {
        if (requested == enabled) {
            pending = false; /* the request cancels the pending one (S16) */
        } else {
            pending = true;
            pending_enabled = requested;
        }
        LOG_INF("mode request %s pending", requested ? "ENABLED" : "DISABLED");
    } else {
        apply_mode(requested);
    }
    return requested;
}

bool zmk_usjis_is_enabled(void) { return enabled; }

/* Substitution applies only on the configured layers, and never while an
 * excluded layer is active (the Mac layers of a keyboard whose Mac mode is a
 * layer above the Win base layer). */
static bool on_substitution_layer(void) {
    zmk_keymap_layers_state_t state = zmk_keymap_layer_state();

    return (CONFIG_ZMK_USJIS_LAYER_MASK == 0 || (state & CONFIG_ZMK_USJIS_LAYER_MASK) != 0) &&
           (state & CONFIG_ZMK_USJIS_LAYER_EXCLUDE_MASK) == 0;
}
bool zmk_usjis_is_pending(void) { return pending; }
unsigned zmk_usjis_active_entries(void) { return count_entries(); }
bool zmk_usjis_listener_order_ok(void) { return order_ok; }
unsigned zmk_usjis_settings_writes(void) { return settings_writes; }

/* ------------------------------------------------------------ event handling */

extern const struct zmk_listener zmk_listener_usjis;
extern const struct zmk_listener zmk_listener_hid_listener;

/* Continue with the listeners after hid_listener (ble.c) so that they still
 * see every event, then tell the event manager that the event was taken. The
 * event lives in the raiser's stack frame, which is still active here. */
static int captured(const zmk_event_t *eh) {
    zmk_event_manager_raise_after((zmk_event_t *)eh, &zmk_listener_hid_listener);
    return ZMK_EV_EVENT_CAPTURED;
}

/* Press of a key that this module reports itself (a substituted key, or any
 * key while a substituted key is held). Mirrors hid_listener, with the
 * modifiers from the policy. */
static void report_press(uint16_t page, uint32_t usage_id, zmk_mod_flags_t explicit_mods) {
    uint32_t usage = ZMK_HID_USAGE(page, usage_id);
    if (!is_mod(page, usage_id) && zmk_hid_is_pressed(usage)) {
        /* Conflict rule 4: pre-release with the modifiers unchanged. */
        zmk_hid_release(usage);
        int err = zmk_endpoint_send_report(page);
        if (err < 0) {
            LOG_ERR("pre-release report failed (%d)", err);
        }
    }
    zmk_hid_press(usage);
    zmk_hid_register_mods(explicit_mods);
}

static void release_entry_output(struct entry *e) {
    if (!output_owned_by_other(e, e->output_usage)) {
        zmk_hid_release(ZMK_HID_USAGE(HID_USAGE_KEY, e->output_usage));
    }
}

/* A cleared substituted record goes away: its output usage is no longer in
 * the report, but hid.c still counts it as held for the USB boot report
 * (CONFIG_ZMK_USB_BOOT: more than six held keys give rollover errors), and
 * only a release lowers that count. Release it unless a later press put the
 * usage back into the report. */
static void drop_cleared_output(struct entry *e) {
    uint32_t usage = ZMK_HID_USAGE(HID_USAGE_KEY, e->output_usage);
    if (!zmk_hid_is_pressed(usage)) {
        zmk_hid_release(usage);
    }
}

static int on_keyboard_press(const zmk_event_t *eh, const struct zmk_keycode_state_changed *ev) {
    struct entry *e = find_entry(ev->keycode);
    if (e != NULL) {
        /* Same input usage pressed again (key repeat re-send, macro or a
         * second position): release the earlier record first. */
        LOG_DBG("re-press of usage 0x%02X", ev->keycode);
        if (substituted_held()) {
            release_entry_output(e);
            e->used = false;
            apply_modifiers();
            send_keyboard_report();
        } else {
            if (e->cleared && e->substituted) {
                drop_cleared_output(e);
            }
            e->used = false;
        }
    }
    e = alloc_entry();
    if (e == NULL) {
        unrecorded_presses++;
        overflow_order = next_order++;
        overflow_added = ev->implicit_modifiers;
        LOG_ERR("press record capacity exceeded; usage 0x%02X passed through unrecorded",
                ev->keycode);
        if (!substituted_held()) {
            return ZMK_EV_EVENT_BUBBLE;
        }
        /* Unsubstituted, but reported here as the latest press: its own
         * modifiers, while the records keep theirs for when it goes up. */
        report_press(HID_USAGE_KEY, ev->keycode, ev->explicit_modifiers);
        apply_modifiers();
        send_keyboard_report();
        return captured(eh);
    }
    e->input_usage = ev->keycode;
    e->output_usage = ev->keycode;
    e->added = ev->implicit_modifiers;
    e->masked = 0;

    /* The table has keyboard page usages below 0x100 only. */
    if (enabled && order_ok && ev->keycode <= UINT8_MAX && on_substitution_layer()) {
        bool shift = ((zmk_hid_get_explicit_mods() | ev->implicit_modifiers) & SHIFT_BITS) != 0;
        struct usjis_resolution r = usjis_resolve(ev->keycode, shift);
        if (r.substituted) {
            e->substituted = true;
            e->output_usage = r.output_usage;
            e->added = (ev->implicit_modifiers & ~SHIFT_BITS) | (r.output_shift ? MOD_LSFT : 0);
            e->masked = SHIFT_BITS;
            LOG_DBG("%s: usage 0x%02X -> 0x%02X add 0x%02X", r.id, ev->keycode, e->output_usage,
                    e->added);
        }
    }

    if (!substituted_held()) {
        /* Identity: the baseline hid_listener reports this press. */
        return ZMK_EV_EVENT_BUBBLE;
    }
    report_press(HID_USAGE_KEY, e->output_usage, ev->explicit_modifiers);
    apply_modifiers();
    send_keyboard_report();
    return captured(eh);
}

static int on_keyboard_release(const zmk_event_t *eh, const struct zmk_keycode_state_changed *ev) {
    struct entry *e = find_entry(ev->keycode);
    if (e == NULL) {
        bool overflow_was_latest = overflow_is_latest();

        if (unrecorded_presses > 0) {
            unrecorded_presses--;
        } else {
            LOG_WRN("release without press record: usage 0x%02X", ev->keycode);
        }
        if (unrecorded_presses == 0) {
            overflow_order = 0;
        }
        if (!substituted_held()) {
            /* The last key up applies a pending request, recorded or not. */
            apply_pending_if_idle();
            return ZMK_EV_EVENT_BUBBLE;
        }
        if (!output_owned_by_other(NULL, ev->keycode)) {
            zmk_hid_release(ZMK_HID_USAGE(HID_USAGE_KEY, ev->keycode));
        }
        zmk_hid_unregister_mods(ev->explicit_modifiers);
        if (overflow_was_latest && !overflow_is_latest()) {
            /* Conflict rule 5: an earlier record becomes the latest. */
            latest_entry()->non_shift_dropped = true;
        }
        apply_modifiers();
        send_keyboard_report();
        return captured(eh);
    }
    if (e->cleared && e->substituted) {
        /* Spec section 7: the output usage is gone already; no release report. */
        drop_cleared_output(e);
        e->used = false;
        if (apply_modifiers()) {
            send_keyboard_report();
        }
        apply_pending_if_idle();
        return captured(eh);
    }
    if (!substituted_held()) {
        e->used = false;
        apply_pending_if_idle();
        return ZMK_EV_EVENT_BUBBLE;
    }
    struct entry *was_latest = latest_entry();
    release_entry_output(e);
    zmk_hid_unregister_mods(ev->explicit_modifiers);
    e->used = false;
    struct entry *now_latest = latest_entry();
    if (was_latest == e && now_latest != NULL) {
        /* Conflict rule 5: an earlier key becomes the latest; only its Shift
         * request is restored. */
        now_latest->non_shift_dropped = true;
    }
    apply_modifiers();
    send_keyboard_report();
    apply_pending_if_idle();
    return captured(eh);
}

/* Modifier keys and consumer/system usages while a substituted key is held:
 * report them here so that hid_listener does not overwrite the policy. */
static int on_other_event(const zmk_event_t *eh, const struct zmk_keycode_state_changed *ev) {
    if (!substituted_held()) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    uint32_t usage = ZMK_HID_USAGE(ev->usage_page, ev->keycode);
    if (ev->state) {
        report_press(ev->usage_page, ev->keycode, ev->explicit_modifiers);
    } else {
        zmk_hid_release(usage);
        zmk_hid_unregister_mods(ev->explicit_modifiers);
    }
    bool changed = apply_modifiers();
    if (ev->usage_page == HID_USAGE_KEY) {
        /* Conflict rule 6: a physical modifier event always sends a report. */
        send_keyboard_report();
    } else {
        if (changed) {
            send_keyboard_report();
        }
        int err = zmk_endpoint_send_report(ev->usage_page);
        if (err < 0) {
            LOG_ERR("report for page 0x%02X failed (%d)", ev->usage_page, err);
        }
    }
    return captured(eh);
}

static int usjis_keycode_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->usage_page != HID_USAGE_KEY || is_mod(ev->usage_page, ev->keycode)) {
        return on_other_event(eh, ev);
    }
    return ev->state ? on_keyboard_press(eh, ev) : on_keyboard_release(eh, ev);
}

/* Endpoint switch: ZMK clears the HID state when the endpoint changes (spec
 * section 7). Detect the clear by checking whether the output usages of the
 * records are still pressed, so that this does not depend on which changes
 * clear. */
static int usjis_endpoint_listener(const zmk_event_t *eh) {
    if (as_zmk_endpoint_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    bool cleared = false;
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (entries[i].used && !entries[i].cleared &&
            !zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, entries[i].output_usage))) {
            cleared = true;
            break;
        }
    }
    if (!cleared) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    LOG_INF("HID state cleared by endpoint switch; %u press records marked cleared",
            count_entries());
    for (int i = 0; i < ARRAY_SIZE(entries); i++) {
        if (entries[i].used) {
            entries[i].cleared = true;
        }
    }
    overflow_order = 0; /* unrecorded presses still count for a pending mode, not as the latest */
    apply_modifiers(); /* no substituted key counts any more: back to the baseline */
    return ZMK_EV_EVENT_BUBBLE;
}

static int usjis_listener(const zmk_event_t *eh) {
    if (eh->event == &zmk_event_zmk_endpoint_changed) {
        return usjis_endpoint_listener(eh);
    }
    return usjis_keycode_listener(eh);
}

ZMK_LISTENER(usjis, usjis_listener);
ZMK_SUBSCRIPTION(usjis, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(usjis, zmk_endpoint_changed);

/* ------------------------------------------------------------ startup */

extern struct zmk_event_subscription __event_subscriptions_start[];
extern struct zmk_event_subscription __event_subscriptions_end[];
/* Present in the firmware, absent in the native simulation. */
extern const struct zmk_listener zmk_listener_behavior_key_repeat __attribute__((weak));
extern const struct zmk_listener zmk_listener_behavior_hold_tap __attribute__((weak));
extern const struct zmk_listener zmk_listener_behavior_sticky_key __attribute__((weak));
extern const struct zmk_listener zmk_listener_behavior_caps_word __attribute__((weak));

static int subscription_index(const struct zmk_listener *listener) {
    int len = __event_subscriptions_end - __event_subscriptions_start;
    for (int i = 0; i < len; i++) {
        struct zmk_event_subscription *sub = __event_subscriptions_start + i;
        if (sub->event_type == &zmk_event_zmk_keycode_state_changed && sub->listener == listener) {
            return i;
        }
    }
    return -1;
}

static int weak_index(const struct zmk_listener *listener) {
    return listener != NULL ? subscription_index(listener) : -1;
}

/* Architecture section 6: the listener must run after hold-tap, sticky key,
 * caps word and key repeat, and hid_listener must be the next keycode
 * listener: one in between would be skipped for every event this module
 * reports itself (captured() continues after hid_listener). If not, the
 * module never substitutes. */
static void check_listener_order(void) {
    int self = subscription_index(&zmk_listener_usjis);
    int hid = subscription_index(&zmk_listener_hid_listener);
    int before[] = {
        weak_index(&zmk_listener_behavior_hold_tap),
        weak_index(&zmk_listener_behavior_sticky_key),
        weak_index(&zmk_listener_behavior_caps_word),
        weak_index(&zmk_listener_behavior_key_repeat),
    };
    order_ok = self >= 0 && hid > self;
    for (int i = 0; i < ARRAY_SIZE(before); i++) {
        order_ok = order_ok && before[i] < self;
    }
    for (int i = self + 1; order_ok && i < hid; i++) {
        if (__event_subscriptions_start[i].event_type == &zmk_event_zmk_keycode_state_changed) {
            order_ok = false;
        }
    }
    if (order_ok) {
        LOG_INF("listener order ok: usjis %d, hid %d", self, hid);
    } else {
        LOG_ERR("listener order wrong (usjis %d, hid %d, hold-tap %d, sticky %d, caps word %d, "
                "key repeat %d); substitution disabled",
                self, hid, before[0], before[1], before[2], before[3]);
    }
}

/* ------------------------------------------------------------ settings */

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>

#define USJIS_SETTINGS_VERSION 1

struct usjis_settings_record {
    uint8_t version;
    uint8_t enabled;
};

static bool loaded_valid;
static bool loaded_enabled;

static int usjis_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb,
                                  void *cb_arg, void *param) {
    ARG_UNUSED(param);
    if (name == NULL || strcmp(name, "mode") != 0) {
        return 0;
    }
    struct usjis_settings_record rec;
    if (len != sizeof(rec)) {
        LOG_ERR("settings usjis/mode: invalid length %u; using DISABLED", (unsigned)len);
        return 0;
    }
    if (read_cb(cb_arg, &rec, sizeof(rec)) != sizeof(rec)) {
        LOG_ERR("settings usjis/mode: read failed; using DISABLED");
        return 0;
    }
    if (rec.version != USJIS_SETTINGS_VERSION || rec.enabled > 1) {
        LOG_ERR("settings usjis/mode: version %u value %u invalid; using DISABLED", rec.version,
                rec.enabled);
        return 0;
    }
    loaded_valid = true;
    loaded_enabled = rec.enabled;
    return 0;
}

static bool save_value;

static void save_work_cb(struct k_work *work) {
    struct usjis_settings_record rec = {.version = USJIS_SETTINGS_VERSION, .enabled = save_value};
    int err = settings_save_one("usjis/mode", &rec, sizeof(rec));
    settings_writes++;
    if (err) {
        LOG_ERR("settings usjis/mode: save failed (%d); the next boot may use the old mode", err);
    }
}

static K_WORK_DELAYABLE_DEFINE(save_work, save_work_cb);

static void persist_mode(bool value) {
    save_value = value;
    k_work_reschedule(&save_work, K_MSEC(CONFIG_ZMK_USJIS_SETTINGS_SAVE_DELAY_MS));
}

static void load_settings(void) {
    settings_subsys_init();
    int err = settings_load_subtree_direct("usjis", usjis_settings_load_cb, NULL);
    if (err) {
        LOG_ERR("settings usjis: load failed (%d); using DISABLED", err);
        return;
    }
    if (loaded_valid) {
        enabled = loaded_enabled;
        LOG_INF("mode restored: %s", enabled ? "ENABLED" : "DISABLED");
    }
}
#else
static void persist_mode(bool value) { ARG_UNUSED(value); }
static void load_settings(void) {}
#endif

static int usjis_init(void) {
    enabled = IS_ENABLED(CONFIG_ZMK_USJIS_DEFAULT_ENABLED);
    check_listener_order();
    load_settings();
    return 0;
}

SYS_INIT(usjis_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
