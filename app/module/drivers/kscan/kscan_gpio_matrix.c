/*
 * Copyright (c) 2020-2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "kscan_gpio.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#include <zmk/debounce.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_gpio_matrix

#define INST_DIODE_DIR(n) DT_ENUM_IDX(DT_DRV_INST(n), diode_direction)
#define COND_DIODE_DIR(n, row2col_code, col2row_code)                                              \
    COND_CODE_0(INST_DIODE_DIR(n), row2col_code, col2row_code)

#define INST_ROWS_LEN(n) DT_INST_PROP_LEN(n, row_gpios)
#define INST_COLS_LEN(n) DT_INST_PROP_LEN(n, col_gpios)
#define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COLS_LEN(n))
#define INST_INPUTS_LEN(n) COND_DIODE_DIR(n, (INST_COLS_LEN(n)), (INST_ROWS_LEN(n)))

#if CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n)                                                                  \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_press_ms))
#endif

#if CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n)                                                                \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_release_ms))
#endif

#define USE_POLLING IS_ENABLED(CONFIG_ZMK_KSCAN_MATRIX_POLLING)
#define USE_INTERRUPTS (!USE_POLLING)

#define COND_INTERRUPTS(code) COND_CODE_1(CONFIG_ZMK_KSCAN_MATRIX_POLLING, (), code)
#define COND_POLL_OR_INTERRUPTS(pollcode, intcode)                                                 \
    COND_CODE_1(CONFIG_ZMK_KSCAN_MATRIX_POLLING, pollcode, intcode)

#define KSCAN_GPIO_ROW_CFG_INIT(idx, inst_idx)                                                     \
    KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), row_gpios, idx)
#define KSCAN_GPIO_COL_CFG_INIT(idx, inst_idx)                                                     \
    KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), col_gpios, idx)

enum kscan_diode_direction {
    KSCAN_ROW2COL,
    KSCAN_COL2ROW,
};

struct kscan_matrix_irq_callback {
    const struct device *dev;
    struct gpio_callback callback;
};

struct kscan_matrix_data {
    const struct device *dev;
    struct kscan_gpio_list inputs;
    kscan_callback_t callback;
    struct k_work_delayable work;
#if USE_INTERRUPTS
    /** Array of length config->inputs.len */
    struct kscan_matrix_irq_callback *irqs;
#endif
    /** Time of the current or scheduled scan, in kernel ticks. */
    int64_t scan_time;
    /**
     * Current state of the matrix as a flattened 2D array of length
     * (config->rows * config->cols)
     */
    struct zmk_debounce_state *matrix_state;
#if USE_POLLING
    /** The last full scan found every key released and no debouncer active. */
    bool idle;
#endif
    /**
     * The time the debouncers count at the next scan: the period it was
     * scheduled after, or one scan period for the scan that follows a poll or
     * an interrupt, or starts the scanning.
     */
    uint16_t scan_elapsed_ms;
};

struct kscan_matrix_config {
    struct kscan_gpio_list outputs;
    struct zmk_debounce_config debounce_config;
    size_t rows;
    size_t cols;
    int32_t debounce_scan_period_ms;
    int32_t debounce_press_scan_period_ms;
    int32_t poll_period_ms;
    enum kscan_diode_direction diode_direction;
};

/**
 * Get the index into a matrix state array from a row and column. The number
 * of rows and the diode direction are passed in, not read from the config
 * here: a scan reads them once, since the config lies in the flash.
 */
static inline int state_index_rc(const int rows, const int row, const int col) {
    return (col * rows) + row;
}

/**
 * Get the index into a matrix state array from input/output pin indices.
 */
static inline int state_index_io(const bool row2col, const int rows, const int input_idx,
                                 const int output_idx) {
    return row2col ? state_index_rc(rows, output_idx, input_idx)
                   : state_index_rc(rows, input_idx, output_idx);
}

static int kscan_matrix_set_all_outputs(const struct device *dev, const int value) {
    const struct kscan_matrix_config *config = dev->config;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct gpio_dt_spec *gpio = &config->outputs.gpios[i].spec;

        int err = gpio_pin_set_dt(gpio, value);
        if (err) {
            LOG_ERR("Failed to set output %i to %i: %i", i, value, err);
            return err;
        }
    }

    return 0;
}

#if USE_INTERRUPTS
static int kscan_matrix_interrupt_configure(const struct device *dev, const gpio_flags_t flags) {
    const struct kscan_matrix_data *data = dev->data;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct gpio_dt_spec *gpio = &data->inputs.gpios[i].spec;

        int err = gpio_pin_interrupt_configure_dt(gpio, flags);
        if (err) {
            LOG_ERR("Unable to configure interrupt for pin %u on %s", gpio->pin, gpio->port->name);
            return err;
        }
    }

    return 0;
}
#endif

#if USE_INTERRUPTS
static int kscan_matrix_interrupt_enable(const struct device *dev) {
    int err = kscan_matrix_interrupt_configure(dev, GPIO_INT_LEVEL_ACTIVE);
    if (err) {
        return err;
    }

    // While interrupts are enabled, set all outputs active so a pressed key
    // will trigger an interrupt.
    return kscan_matrix_set_all_outputs(dev, 1);
}
#endif

#if USE_INTERRUPTS
static int kscan_matrix_interrupt_disable(const struct device *dev) {
    int err = kscan_matrix_interrupt_configure(dev, GPIO_INT_DISABLE);
    if (err) {
        return err;
    }

    // While interrupts are disabled, set all outputs inactive so
    // kscan_matrix_read() can scan them one by one.
    return kscan_matrix_set_all_outputs(dev, 0);
}
#endif

#if USE_INTERRUPTS
static void kscan_matrix_irq_callback_handler(const struct device *port, struct gpio_callback *cb,
                                              const gpio_port_pins_t pin) {
    struct kscan_matrix_irq_callback *irq_data =
        CONTAINER_OF(cb, struct kscan_matrix_irq_callback, callback);
    struct kscan_matrix_data *data = irq_data->dev->data;

    // Disable our interrupts temporarily to avoid re-entry while we scan.
    kscan_matrix_interrupt_disable(data->dev);

    data->scan_time = k_uptime_ticks();

    k_work_reschedule(&data->work, K_NO_WAIT);
}
#endif

/**
 * Schedule the next read one period after the current one's time. When that
 * time has passed already (the work queue was held up, or the system slept),
 * the next read comes a full period from now instead: reads that run one
 * right after another to catch up would not be a period apart.
 */
static void kscan_matrix_schedule(const struct device *dev, int32_t period_ms) {
    struct kscan_matrix_data *data = dev->data;

    const int64_t period = k_ms_to_ticks_ceil32(period_ms);
    const int64_t now = k_uptime_ticks();

    data->scan_time += period;

    if (data->scan_time <= now) {
        data->scan_time = now + period;
    }

    k_work_reschedule(&data->work, K_TIMEOUT_ABS_TICKS(data->scan_time));
}

static void kscan_matrix_read_continue(const struct device *dev, int32_t period_ms) {
    struct kscan_matrix_data *data = dev->data;

    data->scan_elapsed_ms = period_ms;

    kscan_matrix_schedule(dev, period_ms);
}

static void kscan_matrix_read_end(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    // The first scan after this counts one scan period.
    data->scan_elapsed_ms = config->debounce_scan_period_ms;

#if USE_INTERRUPTS
    // Return to waiting for an interrupt.
    kscan_matrix_interrupt_enable(dev);
#else
    // Return to polling slowly.
    kscan_matrix_schedule(dev, config->poll_period_ms);
#endif
}

static int kscan_matrix_read(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    // The constants the loops over the keys use, read from the config once:
    // a read of the config for every key would be a read of the flash.
    const struct zmk_debounce_config debounce = config->debounce_config;
    const int rows = config->rows;
    const int cols = config->cols;
    const bool row2col = config->diode_direction == KSCAN_ROW2COL;
    const int elapsed_ms = data->scan_elapsed_ms;
    bool new_press = false;

    // Scan the matrix.
    for (int i = 0; i < config->outputs.len; i++) {
        const struct kscan_gpio *out_gpio = &config->outputs.gpios[i];

        int err = gpio_pin_set_dt(&out_gpio->spec, 1);
        if (err) {
            LOG_ERR("Failed to set output %i active: %i", out_gpio->index, err);
            return err;
        }

#if CONFIG_ZMK_KSCAN_MATRIX_WAIT_BEFORE_INPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_MATRIX_WAIT_BEFORE_INPUTS);
#endif
        struct kscan_gpio_port_state state = {0};

        for (int j = 0; j < data->inputs.len; j++) {
            const struct kscan_gpio *in_gpio = &data->inputs.gpios[j];

            const int index = state_index_io(row2col, rows, in_gpio->index, out_gpio->index);
            const int active = kscan_gpio_pin_get(in_gpio, &state);
            if (active < 0) {
                LOG_ERR("Failed to read port %s: %i", in_gpio->spec.port->name, active);
                return active;
            }

            if (active && !zmk_debounce_is_active(&data->matrix_state[index])) {
                // A released key read as pressed for the first time.
                new_press = true;
            }

            zmk_debounce_update(&data->matrix_state[index], active, elapsed_ms, &debounce);
        }

        err = gpio_pin_set_dt(&out_gpio->spec, 0);
        if (err) {
            LOG_ERR("Failed to set output %i inactive: %i", out_gpio->index, err);
            return err;
        }

#if CONFIG_ZMK_KSCAN_MATRIX_WAIT_BETWEEN_OUTPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_MATRIX_WAIT_BETWEEN_OUTPUTS);
#endif
    }

    // Process the new state.
    bool continue_scan = false;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            const int index = state_index_rc(rows, r, c);
            struct zmk_debounce_state *state = &data->matrix_state[index];

            if (zmk_debounce_get_changed(state)) {
                const bool pressed = zmk_debounce_is_pressed(state);

                LOG_DBG("Sending event at %i,%i state %s", r, c, pressed ? "on" : "off");
                data->callback(dev, r, c, pressed);
            }

            continue_scan = continue_scan || zmk_debounce_is_active(state);
        }
    }

#if USE_POLLING
    data->idle = !continue_scan;
#endif

    if (continue_scan) {
        // At least one key is pressed or the debouncer has not yet decided if
        // it is pressed. Poll quickly until everything is released; after a
        // key's first pressed reading, the read that can confirm it comes
        // after its own period.
        kscan_matrix_read_continue(dev, new_press ? config->debounce_press_scan_period_ms
                                                  : config->debounce_scan_period_ms);
    } else {
        // All keys are released. Return to normal.
        kscan_matrix_read_end(dev);
    }

    return 0;
}

#if USE_POLLING
/**
 * Polling while every key is released: drive every output in turn as
 * kscan_matrix_read() does, but only look whether any input is active. The
 * debouncers are left alone: updating a released one with an inactive input
 * changes nothing. With all the inputs on one port, each output takes a read
 * of that port and one mask; the port is read twice and the second read
 * taken, which comes no sooner after the output is driven than
 * kscan_matrix_read()'s read does.
 *
 * @return 1 if an input is active, 0 if none is, or a negative error.
 */
static int kscan_matrix_any_active(const struct device *dev) {
    const struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    const struct device *in_port = data->inputs.gpios[0].spec.port;
    gpio_port_pins_t in_mask = 0;

    for (int j = 0; j < data->inputs.len; j++) {
        const struct gpio_dt_spec *in = &data->inputs.gpios[j].spec;

        if (in->port != in_port) {
            in_port = NULL;
            break;
        }
        in_mask |= BIT(in->pin);
    }

    int found = 0;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct kscan_gpio *out_gpio = &config->outputs.gpios[i];
        int active = 0;

        int err = gpio_pin_set_dt(&out_gpio->spec, 1);
        if (err) {
            return err;
        }

#if CONFIG_ZMK_KSCAN_MATRIX_WAIT_BEFORE_INPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_MATRIX_WAIT_BEFORE_INPUTS);
#endif
        if (in_port != NULL) {
            gpio_port_value_t value;

            active = gpio_port_get(in_port, &value);
            if (active == 0) {
                active = gpio_port_get(in_port, &value);
            }
            if (active == 0) {
                active = (value & in_mask) != 0;
            }
        } else {
            struct kscan_gpio_port_state state = {0};

            for (int j = 0; j < data->inputs.len && active == 0; j++) {
                active = kscan_gpio_pin_get(&data->inputs.gpios[j], &state);
            }
        }

        err = gpio_pin_set_dt(&out_gpio->spec, 0);
        if (err) {
            return err;
        }

#if CONFIG_ZMK_KSCAN_MATRIX_WAIT_BETWEEN_OUTPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_MATRIX_WAIT_BETWEEN_OUTPUTS);
#endif
        if (active < 0) {
            return active;
        }
        found |= active;
    }

    return found;
}
#endif

static void kscan_matrix_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_matrix_data *data = CONTAINER_OF(dwork, struct kscan_matrix_data, work);

#if USE_POLLING
    if (data->idle && kscan_matrix_any_active(data->dev) == 0) {
        // Still nothing pressed: the next poll, without a full scan.
        kscan_matrix_read_end(data->dev);
        return;
    }
#endif

    kscan_matrix_read(data->dev);
}

static int kscan_matrix_configure(const struct device *dev, const kscan_callback_t callback) {
    struct kscan_matrix_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    return 0;
}

static int kscan_matrix_enable(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

#if USE_POLLING
    if (data->callback == NULL) {
        // Nothing takes the events yet: the polling starts when it is
        // enabled with a callback set.
        return 0;
    }

    data->scan_time = k_uptime_ticks();
    data->scan_elapsed_ms = config->debounce_scan_period_ms;
    data->idle = false;

    // The first read runs on the work queue, as every later one does, in
    // place of one still scheduled. It starts the polling once done.
    k_work_reschedule(&data->work, K_NO_WAIT);

    return 0;
#else
    data->scan_time = k_uptime_ticks();
    data->scan_elapsed_ms = config->debounce_scan_period_ms;

    // Read will automatically start interrupts once done: a matrix enabled as
    // a wake source has them armed when this returns.
    return kscan_matrix_read(dev);
#endif
}

static int kscan_matrix_disable(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;

    k_work_cancel_delayable(&data->work);

#if USE_INTERRUPTS
    return kscan_matrix_interrupt_disable(dev);
#else
    return 0;
#endif
}

static int kscan_matrix_init_input_inst(const struct device *dev, const struct kscan_gpio *gpio) {
    if (!device_is_ready(gpio->spec.port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->spec.port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&gpio->spec, GPIO_INPUT);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for input", gpio->spec.pin,
                gpio->spec.port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for input", gpio->spec.pin, gpio->spec.port->name);

#if USE_INTERRUPTS
    struct kscan_matrix_data *data = dev->data;
    struct kscan_matrix_irq_callback *irq = &data->irqs[gpio->index];

    irq->dev = dev;
    gpio_init_callback(&irq->callback, kscan_matrix_irq_callback_handler, BIT(gpio->spec.pin));
    err = gpio_add_callback(gpio->spec.port, &irq->callback);
    if (err) {
        LOG_ERR("Error adding the callback to the input device: %i", err);
        return err;
    }
#endif

    return 0;
}

static int kscan_matrix_init_inputs(const struct device *dev) {
    const struct kscan_matrix_data *data = dev->data;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct kscan_gpio *gpio = &data->inputs.gpios[i];
        int err = kscan_matrix_init_input_inst(dev, gpio);
        if (err) {
            return err;
        }
    }

    return 0;
}

static int kscan_matrix_init_output_inst(const struct device *dev,
                                         const struct gpio_dt_spec *gpio) {
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for output", gpio->pin, gpio->port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for output", gpio->pin, gpio->port->name);

    return 0;
}

static int kscan_matrix_init_outputs(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct gpio_dt_spec *gpio = &config->outputs.gpios[i].spec;
        int err = kscan_matrix_init_output_inst(dev, gpio);
        if (err) {
            return err;
        }
    }

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_matrix_disconnect_inputs(const struct device *dev) {
    const struct kscan_matrix_data *data = dev->data;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct gpio_dt_spec *gpio = &data->inputs.gpios[i].spec;
        int err = gpio_pin_configure_dt(gpio, GPIO_DISCONNECTED);
        if (err) {
            return err;
        }
    }

    return 0;
}

static int kscan_matrix_disconnect_outputs(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct gpio_dt_spec *gpio = &config->outputs.gpios[i].spec;
        int err = gpio_pin_configure_dt(gpio, GPIO_DISCONNECTED);
        if (err) {
            return err;
        }
    }

    return 0;
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static void kscan_matrix_setup_pins(const struct device *dev) {
    kscan_matrix_init_inputs(dev);
    kscan_matrix_init_outputs(dev);
    kscan_matrix_set_all_outputs(dev, 0);
}

static int kscan_matrix_init(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;

    data->dev = dev;

    // Sort inputs by port so we can read each port just once per scan.
    kscan_gpio_list_sort_by_port(&data->inputs);

    k_work_init_delayable(&data->work, kscan_matrix_work_handler);

#if IS_ENABLED(CONFIG_PM_DEVICE)
    pm_device_init_suspended(dev);

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#endif

#else
    kscan_matrix_setup_pins(dev);
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_matrix_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        kscan_matrix_disconnect_inputs(dev);
        kscan_matrix_disconnect_outputs(dev);

        return kscan_matrix_disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        kscan_matrix_setup_pins(dev);
        return kscan_matrix_enable(dev);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static const struct kscan_driver_api kscan_matrix_api = {
    .config = kscan_matrix_configure,
    .enable_callback = kscan_matrix_enable,
    .disable_callback = kscan_matrix_disable,
};

#define KSCAN_MATRIX_INIT(n)                                                                       \
    BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                                \
                 "ZMK_KSCAN_DEBOUNCE_PRESS_MS or debounce-press-ms is too large");                 \
    BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                              \
                 "ZMK_KSCAN_DEBOUNCE_RELEASE_MS or debounce-release-ms is too large");             \
    BUILD_ASSERT(DT_INST_PROP_OR(n, debounce_press_scan_period_ms,                                 \
                                 DT_INST_PROP(n, debounce_scan_period_ms)) > 0,                    \
                 "debounce-press-scan-period-ms and debounce-scan-period-ms must be at least 1");  \
                                                                                                   \
    static struct kscan_gpio kscan_matrix_rows_##n[] = {                                           \
        LISTIFY(INST_ROWS_LEN(n), KSCAN_GPIO_ROW_CFG_INIT, (, ), n)};                              \
                                                                                                   \
    static struct kscan_gpio kscan_matrix_cols_##n[] = {                                           \
        LISTIFY(INST_COLS_LEN(n), KSCAN_GPIO_COL_CFG_INIT, (, ), n)};                              \
                                                                                                   \
    static struct zmk_debounce_state kscan_matrix_state_##n[INST_MATRIX_LEN(n)];                   \
                                                                                                   \
    COND_INTERRUPTS(                                                                               \
        (static struct kscan_matrix_irq_callback kscan_matrix_irqs_##n[INST_INPUTS_LEN(n)];))      \
                                                                                                   \
    static struct kscan_matrix_data kscan_matrix_data_##n = {                                      \
        .inputs =                                                                                  \
            KSCAN_GPIO_LIST(COND_DIODE_DIR(n, (kscan_matrix_cols_##n), (kscan_matrix_rows_##n))),  \
        .matrix_state = kscan_matrix_state_##n,                                                    \
        COND_INTERRUPTS((.irqs = kscan_matrix_irqs_##n, ))};                                       \
                                                                                                   \
    static const struct kscan_matrix_config kscan_matrix_config_##n = {                            \
        .rows = ARRAY_SIZE(kscan_matrix_rows_##n),                                                 \
        .cols = ARRAY_SIZE(kscan_matrix_cols_##n),                                                 \
        .outputs =                                                                                 \
            KSCAN_GPIO_LIST(COND_DIODE_DIR(n, (kscan_matrix_rows_##n), (kscan_matrix_cols_##n))),  \
        .debounce_config =                                                                         \
            {                                                                                      \
                .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                                    \
                .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                                \
            },                                                                                     \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),                       \
        .debounce_press_scan_period_ms = DT_INST_PROP_OR(                                          \
            n, debounce_press_scan_period_ms, DT_INST_PROP(n, debounce_scan_period_ms)),           \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
        .diode_direction = INST_DIODE_DIR(n),                                                      \
    };                                                                                             \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_matrix_pm_action);                                           \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &kscan_matrix_init, PM_DEVICE_DT_INST_GET(n), &kscan_matrix_data_##n, \
                          &kscan_matrix_config_##n, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,       \
                          &kscan_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_MATRIX_INIT);
