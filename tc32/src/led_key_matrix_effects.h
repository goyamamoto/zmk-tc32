/*
 * The backlight effects of "zmk,backlight-effects" (src/led_key_matrix_effects.c),
 * for the keymap's behaviour (src/behavior_backlight_rgb.c). Each returns 0,
 * or -EINVAL for an effect number out of range. Only the toggle acts while
 * the backlight is off, and a step past a limit does nothing.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_LED_KEY_MATRIX_EFFECTS_H_
#define TC32_LED_KEY_MATRIX_EFFECTS_H_

#include <stdbool.h>

#include <zephyr/sys/util.h>

int cidoo_backlight_toggle(void);
int cidoo_backlight_set_on(bool on);
/* The next (direction > 0) or the previous effect: wave, breathe, light. */
int cidoo_backlight_cycle_effect(int direction);
int cidoo_backlight_select_effect(int effect);
/* Through the 46 preset colours and the multicolour setting. */
int cidoo_backlight_step_colour(int direction);
/* White mixed into every colour: 0 to 240 in steps of 15. */
int cidoo_backlight_step_white(int direction);
/* Brightness levels 0 to 9. */
int cidoo_backlight_step_level(int direction);
/* Faster (direction > 0) or slower: five speeds. */
int cidoo_backlight_step_speed(int direction);

#if IS_ENABLED(CONFIG_BACKLIGHT_SAVE)
/*
 * A change not kept yet is written now: led_key_matrix_effects.c from any thread (before a deep sleep, a reboot of the
 * mode switch), led_driver_spi.c only when the caller is the system work queue's thread.
 */
void cidoo_backlight_save_pending(void);
#else
static inline void cidoo_backlight_save_pending(void)
{
}
#endif

#endif /* TC32_LED_KEY_MATRIX_EFFECTS_H_ */
