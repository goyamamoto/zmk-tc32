/*
 * Behavior: the link of a board without a mode switch
 * (CONFIG_TLSR_BLE_MODE_FROM_POWER_IN): the parameter is LINK_USB (0),
 * LINK_BLE (1) or LINK_P24 (2). On press it is kept with the BLE profiles;
 * running another link, the chip reboots into this one (src/ble/ble.c). USB
 * without the cable's power is ignored.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT zmk_behavior_link_mode

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "ble/ble_internal.h"

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata link_values[] = {
    {.display_name = "USB", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = LINK_USB},
    {.display_name = "Bluetooth", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = LINK_BLE},
#if IS_ENABLED(CONFIG_TLSR_P24)
    {.display_name = "2.4G", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = LINK_P24},
#endif
};
static const struct behavior_parameter_metadata_set link_set = {
    .param1_values = link_values,
    .param1_values_len = ARRAY_SIZE(link_values),
};
static const struct behavior_parameter_metadata metadata = {.sets_len = 1, .sets = &link_set};

static int get_metadata(const struct device *dev, struct behavior_parameter_metadata *out) {
    ARG_UNUSED(dev);
    *out = metadata;
    return 0;
}
#endif

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    ble_link_request(binding->param1 <= LINK_P24 ? (uint8_t)binding->param1 : LINK_USB);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_link_mode_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = get_metadata,
#endif
};

#define LINK_MODE_INST(n)                                                                          \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_link_mode_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LINK_MODE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
