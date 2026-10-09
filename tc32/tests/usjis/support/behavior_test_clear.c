/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Test-only: &test_clear does, on its press, what update_current_endpoint()
 * in ZMK's endpoints.c does when the endpoint changes (the B1 Pro's
 * connection switch moving between BT and cable): clear the reports, then
 * raise zmk_endpoint_changed. native_sim has no USB or Bluetooth endpoint to
 * change to. The empty report goes to the previous endpoint from inside
 * endpoints.c and is not logged by report_log.c; this logs "endpoint clear".
 */
#define DT_DRV_COMPAT zmk_behavior_test_clear

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/endpoints.h>
#include <zmk/events/endpoint_changed.h>

LOG_MODULE_DECLARE(report, LOG_LEVEL_INF);

static int on_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    LOG_INF("endpoint clear");
    zmk_endpoint_clear_reports();
    raise_zmk_endpoint_changed((struct zmk_endpoint_changed){.endpoint = zmk_endpoint_get_selected()});
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define TEST_CLEAR_INST(n)                                                                         \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(TEST_CLEAR_INST)
