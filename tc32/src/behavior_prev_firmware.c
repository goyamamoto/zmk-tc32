/*
 * Behavior: go back to the image in the other slot when the key is released
 * after being held for hold-ms.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT zmk_behavior_prev_firmware

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "tlsr_slots.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_prev_firmware_config {
    int64_t hold_ms;
};

struct behavior_prev_firmware_data {
    int64_t pressed_at;
};

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_prev_firmware_data *data = dev->data;

    data->pressed_at = event.timestamp;
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_prev_firmware_config *cfg = dev->config;
    struct behavior_prev_firmware_data *data = dev->data;

    if (event.timestamp - data->pressed_at < cfg->hold_ms) {
        return ZMK_BEHAVIOR_OPAQUE;
    }
    LOG_WRN("going back to the firmware in the other slot");
    /* Reboots; returns only when the other slot has no valid image, or a write stopped it. */
    (void)tlsr_slot_revert();
    /* A stopped revert may have rewritten the other slot's first sector: info checks it again. */
    tlsr_slot_touched();
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_prev_firmware_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define PREV_FW_INST(n)                                                                            \
    static const struct behavior_prev_firmware_config behavior_prev_firmware_config_##n = {        \
        .hold_ms = DT_INST_PROP(n, hold_ms),                                                       \
    };                                                                                             \
    static struct behavior_prev_firmware_data behavior_prev_firmware_data_##n;                     \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_prev_firmware_data_##n,                       \
                            &behavior_prev_firmware_config_##n, POST_KERNEL,                       \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_prev_firmware_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PREV_FW_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
