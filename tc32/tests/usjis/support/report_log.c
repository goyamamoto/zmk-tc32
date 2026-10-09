/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Test-only: logs the HID state at every zmk_endpoint_send_report() call from
 * outside endpoints.c (hid_listener.c, usjis.c), as the host would receive it:
 *
 *   report: kbd {MM,[UU UU]}   modifiers, then the pressed usages in ascending order
 *   report: consumer [UUUU]
 *
 * The linker wraps the function (-Wl,--wrap in CMakeLists.txt); calls inside
 * endpoints.c itself (clearing on an endpoint change) are not logged.
 */

#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zmk/hid.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

LOG_MODULE_REGISTER(report, LOG_LEVEL_INF);

int __real_zmk_endpoint_send_report(uint16_t usage_page);

static void append(char *buf, size_t size, size_t *len, const char *fmt, unsigned value) {
    if (*len < size) {
        *len += snprintf(buf + *len, size - *len, fmt, value);
    }
}

static void log_keyboard(void) {
    const struct zmk_hid_keyboard_report *r = zmk_hid_get_keyboard_report();
    char buf[128];
    size_t len = 0;
    bool first = true;

    append(buf, sizeof(buf), &len, "{%02X,[", r->body.modifiers);
#if IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_HKRO)
    uint8_t keys[CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE];
    size_t n = 0;
    for (size_t i = 0; i < ARRAY_SIZE(r->body.keys); i++) {
        if (r->body.keys[i] != 0) {
            keys[n++] = r->body.keys[i];
        }
    }
    /* Ascending order: the slot order is not visible to the host. */
    for (size_t i = 1; i < n; i++) {
        for (size_t j = i; j > 0 && keys[j - 1] > keys[j]; j--) {
            uint8_t t = keys[j];
            keys[j] = keys[j - 1];
            keys[j - 1] = t;
        }
    }
    for (size_t i = 0; i < n; i++) {
        append(buf, sizeof(buf), &len, first ? "%02X" : " %02X", keys[i]);
        first = false;
    }
#else
    for (unsigned usage = 0; usage < 8 * sizeof(r->body.keys); usage++) {
        if (r->body.keys[usage / 8] & BIT(usage % 8)) {
            append(buf, sizeof(buf), &len, first ? "%02X" : " %02X", usage);
            first = false;
        }
    }
#endif
    LOG_INF("kbd %s]}", buf);
}

static void log_consumer(void) {
    const struct zmk_hid_consumer_report *r = zmk_hid_get_consumer_report();
    char buf[64] = "";
    size_t len = 0;
    bool first = true;

    for (size_t i = 0; i < ARRAY_SIZE(r->body.keys); i++) {
        if (r->body.keys[i] != 0) {
            append(buf, sizeof(buf), &len, first ? "%04X" : " %04X", r->body.keys[i]);
            first = false;
        }
    }
    buf[MIN(len, sizeof(buf) - 1)] = '\0';
    LOG_INF("consumer [%s]", buf);
}

int __wrap_zmk_endpoint_send_report(uint16_t usage_page) {
    switch (usage_page) {
    case HID_USAGE_KEY:
        log_keyboard();
        break;
    case HID_USAGE_CONSUMER:
        log_consumer();
        break;
    default:
        LOG_INF("page %04X", usage_page);
        break;
    }
    return __real_zmk_endpoint_send_report(usage_page);
}
