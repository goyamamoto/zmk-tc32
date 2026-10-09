/*
 * Behavior: bring USB up in a position of the mode switch that has none at
 * boot (src/ble/ble.c, TLSR_USB_ON_REQUEST) when the key is pressed, for the
 * host tools. tlsr_usb_requested() tells the board, which may show it.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT zmk_behavior_usb_request

#include <zephyr/device.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "ble/tlsr_ble.h"

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

__weak void tlsr_usb_requested(void) {}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    if (ble_usb_request()) {
        tlsr_usb_requested();
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_usb_request_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define USB_REQUEST_INST(n)                                                                        \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_usb_request_driver_api);

DT_INST_FOREACH_STATUS_OKAY(USB_REQUEST_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
