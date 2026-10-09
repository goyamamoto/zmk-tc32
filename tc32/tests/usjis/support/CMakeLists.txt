# SPDX-License-Identifier: GPL-3.0-or-later
# Test-only module, added by run.sh: logs every report that is sent
# (report_log.c), provides &test_clear (behavior_test_clear.c), can slow down
# the boot (boot_delay.c), and logs the mode hook and the settings writes
# (usjis_test_log.c).
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/report_log.c
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_test_clear.c
  ${CMAKE_CURRENT_SOURCE_DIR}/boot_delay.c
  ${CMAKE_CURRENT_SOURCE_DIR}/usjis_test_log.c
)
zephyr_ld_options(-Wl,--wrap=zmk_endpoint_send_report)
if(CONFIG_SETTINGS)
  zephyr_ld_options(-Wl,--wrap=settings_save_one)
endif()

# Init order within a priority is link order: move boot_delay.c to just
# before ZMK's keymap.c.
function(kb1_test_place_boot_delay)
  get_property(delay GLOBAL PROPERTY KB1_TEST_BOOT_DELAY_SOURCE)
  get_target_property(sources app SOURCES)
  list(REMOVE_ITEM sources ${delay})
  set(index -1)
  set(i 0)
  foreach(source ${sources})
    if(source MATCHES "(^|/)src/keymap\\.c$")
      set(index ${i})
    endif()
    math(EXPR i "${i} + 1")
  endforeach()
  if(index EQUAL -1)
    message(FATAL_ERROR "kb1 test support: keymap.c not found among the app sources")
  endif()
  list(INSERT sources ${index} ${delay})
  set_property(TARGET app PROPERTY SOURCES "${sources}")
endfunction()
set_property(GLOBAL PROPERTY KB1_TEST_BOOT_DELAY_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/boot_delay.c)
cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL kb1_test_place_boot_delay)
