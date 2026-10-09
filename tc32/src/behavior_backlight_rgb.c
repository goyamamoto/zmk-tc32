/*
 * ZMK's RGB commands (dt-bindings/zmk/rgb.h) for the backlight effects
 * (src/led_key_matrix_effects.c), with these meanings: hue steps
 * through the preset colours and multicolour, saturation takes white out
 * (SAI) or mixes it in (SAD), brightness steps the ten levels, speed the
 * five speeds. The relative commands reach this behaviour as they are (ZMK's
 * underglow behaviour turns them into absolute HSB colours first).
 * RGB_COLOR_HSB is not supported.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT zmk_behavior_backlight_rgb

#include <errno.h>

#include <zephyr/device.h>

#include <drivers/behavior.h>
#include <dt-bindings/zmk/rgb.h>
#include <zmk/behavior.h>

#include "led_key_matrix_effects.h"

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    /*
     * Keys already down when the keyboard starts are the power-on recovery
     * chord's (Fn + KP- on the V21, which is brightness down here), held a
     * moment too late for the boot guard: they must not step the backlight,
     * whose setting is now kept (repeated tries would dim it to dark, and a
     * dark board looks dead during a recovery).
     */
    if (event.timestamp < CONFIG_BACKLIGHT_BOOT_KEYS_MS) {
        return ZMK_BEHAVIOR_OPAQUE;
    }
    switch (binding->param1) {
    case RGB_TOG_CMD:
        return cidoo_backlight_toggle();
    case RGB_ON_CMD:
        return cidoo_backlight_set_on(true);
    case RGB_OFF_CMD:
        return cidoo_backlight_set_on(false);
    case RGB_HUI_CMD:
        return cidoo_backlight_step_colour(1);
    case RGB_HUD_CMD:
        return cidoo_backlight_step_colour(-1);
    case RGB_SAI_CMD:
        return cidoo_backlight_step_white(-1);
    case RGB_SAD_CMD:
        return cidoo_backlight_step_white(1);
    case RGB_BRI_CMD:
        return cidoo_backlight_step_level(1);
    case RGB_BRD_CMD:
        return cidoo_backlight_step_level(-1);
    case RGB_SPI_CMD:
        return cidoo_backlight_step_speed(1);
    case RGB_SPD_CMD:
        return cidoo_backlight_step_speed(-1);
    case RGB_EFF_CMD:
        return cidoo_backlight_cycle_effect(1);
    case RGB_EFR_CMD:
        return cidoo_backlight_cycle_effect(-1);
    case RGB_EFS_CMD:
        return cidoo_backlight_select_effect(binding->param2);
    default:
        return -ENOTSUP;
    }
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

/* The commands as ZMK Studio offers them (a command and no second parameter). */
static const struct behavior_parameter_value_metadata command_values[] = {
    {.display_name = "Toggle On/Off", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_TOG_CMD},
    {.display_name = "Turn On", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_ON_CMD},
    {.display_name = "Turn Off", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_OFF_CMD},
    {.display_name = "Next Colour", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_HUI_CMD},
    {.display_name = "Previous Colour", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_HUD_CMD},
    {.display_name = "Brightness Up", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_BRI_CMD},
    {.display_name = "Brightness Down", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_BRD_CMD},
    {.display_name = "Next Effect", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_EFF_CMD},
    {.display_name = "Previous Effect", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_EFR_CMD},
    {.display_name = "Whiter", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_SAD_CMD},
    {.display_name = "Less White", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_SAI_CMD},
    {.display_name = "Speed Up", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_SPI_CMD},
    {.display_name = "Speed Down", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = RGB_SPD_CMD},
};

static const struct behavior_parameter_metadata_set command_set = {
    .param1_values = command_values,
    .param1_values_len = ARRAY_SIZE(command_values),
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = 1,
    .sets = &command_set,
};

#endif /* CONFIG_ZMK_BEHAVIOR_METADATA */

static const struct behavior_driver_api behavior_cidoo_backlight_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_cidoo_backlight_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
