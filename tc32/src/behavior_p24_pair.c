/*
 * Behavior: pair the 2.4G link with a dongle again (src/ble/p24.c) once the
 * key has been held for hold-ms (3000 by default).
 * Outside the 2.4G position it does nothing. On a board without a mode
 * switch (CONFIG_TLSR_BLE_MODE_FROM_POWER_IN) a tap chooses the 2.4G link
 * (ble_link_request()).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT zmk_behavior_p24_pair

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "ble/ble_internal.h"

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_p24_pair_config {
    int32_t hold_ms;
};

static void held(struct k_work *work) {
    ARG_UNUSED(work);
    if (p24_running()) {
        p24_pair_request();
    }
}

static K_WORK_DELAYABLE_DEFINE(held_work, held);

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_p24_pair_config *cfg = dev->config;

    ARG_UNUSED(event);
    (void)k_work_reschedule(&held_work, K_MSEC(cfg->hold_ms));
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    /*
     * Released before hold-ms: the work is still pending. Read before the cancel, which returns the
     * state after it (0 for a pending work it cancelled).
     */
    bool tap = k_work_delayable_is_pending(&held_work);

    (void)k_work_cancel_delayable(&held_work);

    ARG_UNUSED(tap);
#if IS_ENABLED(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
    /* A tap chooses the 2.4G link (a board without a mode switch); held, the pairing above. */
    if (tap) {
        ble_link_request(LINK_P24);
    }
#endif
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_p24_pair_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define P24_PAIR_INST(n)                                                                           \
    static const struct behavior_p24_pair_config behavior_p24_pair_config_##n = {                  \
        .hold_ms = DT_INST_PROP(n, hold_ms),                                                       \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, &behavior_p24_pair_config_##n, POST_KERNEL,       \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p24_pair_driver_api);

DT_INST_FOREACH_STATUS_OKAY(P24_PAIR_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
