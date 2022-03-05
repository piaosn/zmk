/*
 * Copyright (c) 2020-2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/kscan.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <zmk/event_manager.h>
#include <zmk/activity.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_hhkb

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct kscan_gpio_dt_spec {
    const struct device *port;
    gpio_pin_t pin;
    gpio_dt_flags_t dt_flags;
};

struct kscan_matrix_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_delayed_work work;
    /** Timestamp of the current or scheduled scan. */
    int64_t scan_time;

    /**
     * Current state of the matrix as a flattened 2D array of length
     * (config->rows.len * config->cols.len)
     */
    bool *matrix_state;
};

struct kscan_matrix_config {
    const struct kscan_gpio_dt_spec row0;
    const struct kscan_gpio_dt_spec row1;
    const struct kscan_gpio_dt_spec row2;
    const struct kscan_gpio_dt_spec row_en0;
    const struct kscan_gpio_dt_spec row_en1;
    const struct kscan_gpio_dt_spec col0;
    const struct kscan_gpio_dt_spec col1;
    const struct kscan_gpio_dt_spec col2;
    const struct kscan_gpio_dt_spec col_en;
    const struct kscan_gpio_dt_spec hys;
    const struct kscan_gpio_dt_spec key;

    const uint8_t row_len;
    const uint8_t col_len;
    int32_t poll_period_ms;
};

/**
 * Get the index into a matrix state array from a row and column.
 */
static int state_index_rc(const struct kscan_matrix_config *config, const int row, const int col) {
    return (col * config->col_len) + row;
}

static void kscan_matrix_read_continue(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;
    struct kscan_matrix_data *data = dev->data;

    // data->scan_time += config->debounce_scan_period_ms;
    data->scan_time += 1;

    // TODO (Zephyr 2.6): use k_work_reschedule()
    k_delayed_work_cancel(&data->work);
    k_delayed_work_submit(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void kscan_matrix_read_end(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    data->scan_time += config->poll_period_ms;

    // Return to polling slowly.
    // TODO (Zephyr 2.6): use k_work_reschedule()
    k_delayed_work_cancel(&data->work);
    k_delayed_work_submit(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static int useless_function() {
    int index = 0;
    for (int i = 0; i < 10000; i++) {
        index += i;
    }
    return index;
}

static int min(int a, int b) {
    return (a < b ? a : b);
}

static int kscan_matrix_read(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    // int err;
    bool prev_is_active = false;
    bool is_changed = false;

    bool continue_scan = false;
    int index = 0;
    bool active = false;

    // Select row
    for (uint8_t row = 0; row < config->row_len; row++) {
        gpio_pin_set(config->row0.port, config->row0.pin, (row>>0)&1);
        gpio_pin_set(config->row1.port, config->row1.pin, (row>>1)&1);
        gpio_pin_set(config->row2.port, config->row2.pin, (row>>2)&1);

        // Select col, 50us for each key
        for (uint8_t col = 0; col < config->col_len; col++) {
            gpio_pin_set(config->col0.port, config->col0.pin, (col >> 0) & 1);
            gpio_pin_set(config->col1.port, config->col1.pin, (col >> 1) & 1);
            gpio_pin_set(config->col2.port, config->col2.pin, (col >> 2) & 1);
            
            // nrf的tick默认为32768，大概每30us一个tick。所以计时的最小单位可能是30us。
            // Wait 5us
            k_busy_wait(5);

            // Set prev (clear prev state)
            if (prev_is_active) {
                gpio_pin_set(config->hys.port, config->hys.pin, 1);
                k_busy_wait(10);
            }

            // Enable read
            // enable read 之后大致需要400ns, key才会可读。
            gpio_pin_set(config->col_en.port, config->col_en.pin, 1);

            // 实测，这个sleep会休眠90us左右，完全不可用。
            // k_usleep(1);

            // 可以用硬等待, 实测大概花费1.5us
            k_busy_wait(1);

            // Read
            active = gpio_pin_get(config->key.port, config->key.pin);

            // Disable read and hys
            gpio_pin_set(config->col_en.port, config->col_en.pin, 0);
            gpio_pin_set(config->hys.port, config->hys.pin, 0);

            index = state_index_rc(config, row, col);
            is_changed = data->matrix_state[index] != active;
            data->matrix_state[index] = active;
            prev_is_active = active;

            // Trigger event
            if (is_changed) {
                continue_scan = true;
                LOG_DBG("Sending event at %i,%i state %s", row, col, active ? "on" : "off");
                data->callback(dev, row, col, active);
            }
        }
    }

    if (continue_scan) {
        // At least one key is pressed or the debouncer has not yet decided if
        // it is pressed. Poll quickly until everything is released.
        kscan_matrix_read_continue(dev);
    } else {
        // All keys are released. Return to normal.
        kscan_matrix_read_end(dev);
    }
    return 0;
}

static void kscan_matrix_work_handler(struct k_work *work) {
    struct k_delayed_work *dwork = CONTAINER_OF(work, struct k_delayed_work, work);
    struct kscan_matrix_data *data = CONTAINER_OF(dwork, struct kscan_matrix_data, work);
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

    data->scan_time = k_uptime_get();

    // Read will automatically start interrupts/polling once done.
    return kscan_matrix_read(dev);
}

static int kscan_matrix_disable(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;

    k_delayed_work_cancel(&data->work);

    return 0;
}

static int kscan_matrix_init_input_inst(const struct device *dev,
                                        const struct kscan_gpio_dt_spec *gpio) {
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure(gpio->port, gpio->pin, GPIO_INPUT | gpio->dt_flags);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for input", gpio->pin, gpio->port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for input", gpio->pin, gpio->port->name);

    return 0;
}

static int kscan_matrix_init_output_inst(const struct device *dev,
                                         const struct kscan_gpio_dt_spec *gpio) {
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure(gpio->port, gpio->pin, GPIO_OUTPUT | gpio->dt_flags);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for output", gpio->pin, gpio->port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for output", gpio->pin, gpio->port->name);

    return 0;
}

static int kscan_matrix_init(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    data->dev = dev;

    kscan_matrix_init_input_inst(dev, &config->key);

    kscan_matrix_init_output_inst(dev, &config->row0);
    kscan_matrix_init_output_inst(dev, &config->row1);
    kscan_matrix_init_output_inst(dev, &config->row2);
    kscan_matrix_init_output_inst(dev, &config->row_en0);
    kscan_matrix_init_output_inst(dev, &config->row_en1);
    kscan_matrix_init_output_inst(dev, &config->col0);
    kscan_matrix_init_output_inst(dev, &config->col1);
    kscan_matrix_init_output_inst(dev, &config->col2);
    kscan_matrix_init_output_inst(dev, &config->col_en);
    kscan_matrix_init_output_inst(dev, &config->hys);

    // gpio_pin_set(gpio->port, gpio->pin, value);

    k_delayed_work_init(&data->work, kscan_matrix_work_handler);

    return 0;
}

static int kscan_hhkb_event_listener(const zmk_event_t *eh) {
    if (as_zmk_activity_state_changed(eh)) {
        static bool prev_state = false;
        enum zmk_activity_state state = zmk_activity_get_state();
        // == ZMK_ACTIVITY_ACTIVE
        LOG_DBG("Keyboard active event: %d", state);
        return 0;
    }

    return -ENOTSUP;
}

ZMK_LISTENER(kscan_hhkb, kscan_hhkb_event_listener);
ZMK_SUBSCRIPTION(kscan_hhkb, zmk_activity_state_changed);


static const struct kscan_driver_api kscan_matrix_api = {
    .config = kscan_matrix_configure,
    .enable_callback = kscan_matrix_enable,
    .disable_callback = kscan_matrix_disable,
};

#define KSCAN_GPIO_DT_SPEC(node_id, prop)                                          \
{                                                                                  \
    .port = DEVICE_DT_GET(DT_GPIO_CTLR(node_id, prop)),                            \
    .pin = DT_GPIO_PIN(node_id, prop),                                             \
    .dt_flags = DT_GPIO_FLAGS(node_id, prop),                                      \
}

#define KSCAN_MATRIX_INIT(inst)                                                    \
    static struct kscan_matrix_config kscan_matrix_config_##inst = {               \
        .row_len = 8,\
        .col_len = 8,\
        .row0 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), row0_gpios),                 \
        .row1 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), row1_gpios),                 \
        .row2 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), row2_gpios),                 \
        .col0 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), col0_gpios),                 \
        .col1 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), col1_gpios),                 \
        .col2 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), col2_gpios),                 \
        .col_en = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), col_en_gpios),             \
        .hys = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), hys_gpios),                   \
        .key = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), key_gpios),                   \
        .row_en0 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), row_en0_gpios),           \
        .row_en1 = KSCAN_GPIO_DT_SPEC(DT_DRV_INST(inst), row_en1_gpios),           \
        .poll_period_ms = DT_INST_PROP(inst, poll_period_ms),                      \
    };                                                                             \
    static bool kscan_matrix_state_##inst[8*8] = {false};                                    \
    static struct kscan_matrix_data kscan_matrix_data_##inst = {                   \
        .matrix_state = kscan_matrix_state_##inst,                                 \
    };                                                                             \
                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, &kscan_matrix_init, device_pm_control_nop,         \
                          &kscan_matrix_data_##inst, &kscan_matrix_config_##inst, APPLICATION,   \
                          CONFIG_APPLICATION_INIT_PRIORITY, &kscan_matrix_api);

// 对设备树中每一个状态为ok的子节点执行 KSCAN_MATRIX_INIT 函数。
DT_INST_FOREACH_STATUS_OKAY(KSCAN_MATRIX_INIT);

// #endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
