/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "kscan_gpio.h"

// An insertion sort: the lists are a keyboard's rows or columns, a few tens of pins, and the
// order of pins on the same port does not matter (kscan_gpio_pin_get reads each port once).
void kscan_gpio_list_sort_by_port(struct kscan_gpio_list *list) {
    for (size_t i = 1; i < list->len; i++) {
        const struct kscan_gpio g = list->gpios[i];
        size_t j = i;

        while (j > 0 && list->gpios[j - 1].spec.port > g.spec.port) {
            list->gpios[j] = list->gpios[j - 1];
            j--;
        }
        list->gpios[j] = g;
    }
}

int kscan_gpio_pin_get(const struct kscan_gpio *gpio, struct kscan_gpio_port_state *state) {
    if (gpio->spec.port != state->port) {
        state->port = gpio->spec.port;

        const int err = gpio_port_get(state->port, &state->value);
        if (err) {
            return err;
        }
    }

    return (state->value & BIT(gpio->spec.pin)) != 0;
}
