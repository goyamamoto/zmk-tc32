/* Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * &usjis USJIS_OFF | USJIS_ON | USJIS_TOG: US-JIS substitution mode operations.
 * Acts on the press; the release does nothing. Nothing reaches the host.
 */
#define DT_DRV_COMPAT zmk_behavior_usjis

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/usjis.h>
#include <dt-bindings/zmk/usjis.h>

LOG_MODULE_DECLARE(usjis, CONFIG_ZMK_USJIS_LOG_LEVEL);

BUILD_ASSERT(USJIS_OFF == ZMK_USJIS_OP_OFF && USJIS_ON == ZMK_USJIS_OP_ON &&
             USJIS_TOG == ZMK_USJIS_OP_TOGGLE);

static int on_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case ZMK_USJIS_OP_OFF:
    case ZMK_USJIS_OP_ON:
    case ZMK_USJIS_OP_TOGGLE:
        zmk_usjis_request((enum zmk_usjis_op)binding->param1);
        return ZMK_BEHAVIOR_OPAQUE;
    default:
        LOG_ERR("unknown usjis operation %u", binding->param1);
        return -ENOTSUP;
    }
}

static int on_released(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata values[] = {
    {
        .display_name = "Toggle",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = USJIS_TOG,
    },
    {
        .display_name = "On",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = USJIS_ON,
    },
    {
        .display_name = "Off",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = USJIS_OFF,
    },
};

static const struct behavior_parameter_metadata_set set = {
    .param1_values = values,
    .param1_values_len = ARRAY_SIZE(values),
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = 1,
    .sets = &set,
};
#endif

static const struct behavior_driver_api behavior_usjis_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_CENTRAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

#define USJIS_INST(n)                                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_usjis_driver_api);

DT_INST_FOREACH_STATUS_OKAY(USJIS_INST)
