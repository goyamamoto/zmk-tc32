/*
 * Rotary encoder counted per detent, for the Cidoo V75 Pro's knob
 * (dts/bindings/sensor/cidoo,detent-encoder.yaml).
 *
 * The pins are read in their GPIO interrupt, not later in a work item: the
 * keymap work that a step starts can take longer than the time between two
 * pin changes on the TC32, and a change must not be missed. The reading only
 * needs to see which pin left the detent first; a move whose middle state is
 * missed, or that turns back through the other middle state, is dropped.
 * Moves wait in a short queue, runs of one direction together, and each
 * trigger handler run takes one run: a turn back and forth faster than the
 * handler still reports every move in order.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT cidoo_detent_encoder

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#define FULL_ROTATION 360
#define DIR_DROPPED 2 /* the move under way will not count */
#define DENC_RUNS 8 /* a power of two: the index is masked, not divided */
#define DENC_IDX(i) ((i) & (DENC_RUNS - 1))

struct denc_config {
    struct gpio_dt_spec a;
    struct gpio_dt_spec b;
    uint16_t steps;
};

struct denc_data {
    const struct device *dev;
    struct gpio_callback a_cb;
    struct gpio_callback b_cb;
    struct k_work work;
    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;
    uint8_t detent;   /* pins at the last detent: 0b00 or 0b11 (a << 1 | b) */
    int8_t dir;       /* move under way: +1, -1, DIR_DROPPED, or 0 if none */
    int8_t runs[DENC_RUNS]; /* moves not yet taken: runs of one direction */
    uint8_t head;
    uint8_t count;
    int16_t pulses;   /* pulses taken by sample_fetch, until channel_get */
};

static void denc_push(struct denc_data *data, int8_t dir) {
    int8_t *last = &data->runs[DENC_IDX(data->head + data->count - 1)];

    if (data->count > 0 && (*last > 0) == (dir > 0) && *last != dir * INT8_MAX) {
        *last += dir;
    } else if (data->count < DENC_RUNS) {
        data->runs[DENC_IDX(data->head + data->count)] = dir;
        data->count++;
    } else if (*last != dir * INT8_MAX) {
        *last += dir; /* full: keep the sum, lose the order of the last runs */
    }
}

static uint8_t denc_pins(const struct device *dev) {
    const struct denc_config *cfg = dev->config;

    return (gpio_pin_get_dt(&cfg->a) << 1) | gpio_pin_get_dt(&cfg->b);
}

/* Returns true when a move was counted. */
static bool denc_update(const struct device *dev) {
    struct denc_data *data = dev->data;
    unsigned int key = irq_lock();
    uint8_t ab = denc_pins(dev);
    bool counted = false;

    if (ab == data->detent) {
        data->dir = 0; /* back where it started (or bounced back) */
    } else if (ab == (data->detent ^ 0b11)) {
        /* Arrived at the other detent. */
        if (data->dir == 1 || data->dir == -1) {
            denc_push(data, data->dir);
            counted = true;
        }
        data->detent = ab;
        data->dir = 0;
    } else {
        int8_t dir = ((ab ^ data->detent) == 0b10) ? 1 : -1; /* a moved first: +1 */

        if (data->dir == 0) {
            data->dir = dir;
        } else if (data->dir != dir) {
            data->dir = DIR_DROPPED; /* went through the other middle state */
        }
    }

    irq_unlock(key);
    return counted;
}

static void denc_changed(struct denc_data *data) {
    if (denc_update(data->dev) && data->handler != NULL) {
        k_work_submit(&data->work);
    }
}

static void denc_a_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    denc_changed(CONTAINER_OF(cb, struct denc_data, a_cb));
}

static void denc_b_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    denc_changed(CONTAINER_OF(cb, struct denc_data, b_cb));
}

static void denc_work_cb(struct k_work *work) {
    struct denc_data *data = CONTAINER_OF(work, struct denc_data, work);
    sensor_trigger_handler_t handler = data->handler;

    if (handler != NULL) {
        handler(data->dev, data->trigger);
    }
}

static int denc_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct denc_data *data = dev->data;
    unsigned int key;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    denc_update(dev);
    key = irq_lock();
    if (data->count > 0) {
        data->pulses += 2 * data->runs[data->head];
        data->head = DENC_IDX(data->head + 1);
        data->count--;
        if (data->count > 0 && data->handler != NULL) {
            k_work_submit(&data->work); /* the next run gets its own handler call */
        }
    }
    irq_unlock(key);

    return 0;
}

static int denc_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct denc_data *data = dev->data;
    const struct denc_config *cfg = dev->config;
    int32_t deg;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    deg = data->pulses * FULL_ROTATION;
    data->pulses = 0;
    val->val1 = deg / cfg->steps;
    val->val2 = (deg % cfg->steps) * 1000000 / cfg->steps; /* fits: steps <= 720 */

    return 0;
}

static int denc_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                            sensor_trigger_handler_t handler) {
    struct denc_data *data = dev->data;
    unsigned int key = irq_lock();

    data->count = 0; /* moves before now are not reported */
    data->trigger = trig;
    data->handler = handler;
    irq_unlock(key);

    return 0;
}

static DEVICE_API(sensor, denc_api) = {
    .trigger_set = denc_trigger_set,
    .sample_fetch = denc_sample_fetch,
    .channel_get = denc_channel_get,
};

static int denc_init(const struct device *dev) {
    const struct denc_config *cfg = dev->config;
    struct denc_data *data = dev->data;
    uint8_t ab;
    int err;

    if (!gpio_is_ready_dt(&cfg->a) || !gpio_is_ready_dt(&cfg->b)) {
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&cfg->a, GPIO_INPUT);
    err = err ? err : gpio_pin_configure_dt(&cfg->b, GPIO_INPUT);
    if (err) {
        return err;
    }

    data->dev = dev;
    ab = denc_pins(dev);
    data->detent = (ab == 0b00) ? 0b00 : 0b11;
    k_work_init(&data->work, denc_work_cb);

    gpio_init_callback(&data->a_cb, denc_a_cb, BIT(cfg->a.pin));
    gpio_init_callback(&data->b_cb, denc_b_cb, BIT(cfg->b.pin));
    err = gpio_add_callback(cfg->a.port, &data->a_cb);
    err = err ? err : gpio_add_callback(cfg->b.port, &data->b_cb);
    err = err ? err : gpio_pin_interrupt_configure_dt(&cfg->a, GPIO_INT_EDGE_BOTH);
    err = err ? err : gpio_pin_interrupt_configure_dt(&cfg->b, GPIO_INT_EDGE_BOTH);

    return err;
}

#define DENC_INST(n)                                                                               \
    static struct denc_data denc_data_##n;                                                         \
    static const struct denc_config denc_config_##n = {                                            \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                    \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                    \
        .steps = DT_INST_PROP(n, steps),                                                           \
    };                                                                                             \
    BUILD_ASSERT(DT_INST_PROP(n, steps) > 0 && DT_INST_PROP(n, steps) <= 720,                     \
                 "steps must be 1-720: a move must report at least one degree");                           \
    DEVICE_DT_INST_DEFINE(n, denc_init, NULL, &denc_data_##n, &denc_config_##n, POST_KERNEL,       \
                          CONFIG_SENSOR_INIT_PRIORITY, &denc_api);

DT_INST_FOREACH_STATUS_OKAY(DENC_INST)
