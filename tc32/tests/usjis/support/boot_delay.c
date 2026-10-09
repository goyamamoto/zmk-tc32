/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Test only: CONFIG_KB1_TEST_BOOT_DELAY_MS (see Kconfig). CMakeLists.txt
 * places this source just before ZMK's keymap.c, so that this init runs
 * after the physical layouts have enabled the kscan and before keymap_init().
 */
#include <zephyr/init.h>
#include <zephyr/kernel.h>

static int boot_delay(void) {
    if (CONFIG_KB1_TEST_BOOT_DELAY_MS > 0) {
        k_msleep(CONFIG_KB1_TEST_BOOT_DELAY_MS);
    }
    return 0;
}

SYS_INIT(boot_delay, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
