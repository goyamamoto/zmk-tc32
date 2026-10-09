/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Test-only, logged under "usjis_test" (only the cases whose events.patterns
 * keep these lines see them):
 *
 *   usjis_test: hook ENABLED|DISABLED   zmk_usjis_mode_applied() was called
 *   usjis_test: save usjis/... {VV VV}  settings_save_one() of a usjis key
 *
 * zmk_usjis_mode_applied() overrides the module's weak default (zmk-tc32's
 * copy); a module without the hook never calls it. settings_save_one() is
 * wrapped by the linker (-Wl,--wrap in CMakeLists.txt) when CONFIG_SETTINGS.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usjis_test, LOG_LEVEL_INF);

void zmk_usjis_mode_applied(bool enabled) { LOG_INF("hook %s", enabled ? "ENABLED" : "DISABLED"); }

#if IS_ENABLED(CONFIG_SETTINGS)
int __real_settings_save_one(const char *name, const void *value, size_t val_len);

int __wrap_settings_save_one(const char *name, const void *value, size_t val_len) {
    if (strncmp(name, "usjis", 5) == 0) {
        char buf[64];
        size_t len = 0;
        const uint8_t *bytes = value;
        for (size_t i = 0; i < val_len && len < sizeof(buf) - 4; i++) {
            len += snprintf(buf + len, sizeof(buf) - len, i ? " %02X" : "%02X", bytes[i]);
        }
        buf[MIN(len, sizeof(buf) - 1)] = '\0';
        LOG_INF("save %s {%s}", name, buf);
    }
    return __real_settings_save_one(name, value, val_len);
}
#endif
