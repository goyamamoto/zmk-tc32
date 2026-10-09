# SPDX-License-Identifier: Apache-2.0
# Restore Zephyr 4.1's kscan API header for ZMK.
zephyr_include_directories(include)
zephyr_syscall_header(${CMAKE_CURRENT_LIST_DIR}/include/zephyr/drivers/kscan.h)

# ZMK's module/drivers/kscan amends Zephyr's drivers__kscan library, which
# Zephyr 4.4 no longer creates. Create it here; this module must be listed
# before ZMK's module in ZEPHYR_MODULES.
if(CONFIG_KSCAN)
  zephyr_library_named(drivers__kscan)
endif()
