# SPDX-License-Identifier: GPL-3.0-or-later

if(CONFIG_TLSR_SPI_FLASH)
  zephyr_library_named(tlsr_spi_flash)
  # Written in Telink's mnemonics: tc32_asm_sources() (zephyr-tc32
  # arch/tc32) translates it for a Thumb build.
  tc32_asm_sources(src/tlsr_spi_flash_io.S)
  zephyr_library_sources(src/tlsr_spi_flash.c src/flash_tlsr_spi.c)
  zephyr_include_directories(src)
endif()

if(CONFIG_TLSR_SLOTS)
  zephyr_library()
  zephyr_library_sources(src/tlsr_slots.c)
  zephyr_library_sources_ifdef(CONFIG_TLSR_USB_OTA src/tlsr_usb_ota.c)
  zephyr_library_sources_ifdef(CONFIG_TLSR_BOOT_GUARD src/tlsr_boot_guard.c)
  zephyr_include_directories(src)
endif()

# The legacy usb_dc driver of the TLSR8278's USB device controller
# (drivers/usb/device), and its interface to the board code
# (include/zephyr/drivers/usb/usb_dc_b87.h).
if(CONFIG_USB_DC_TELINK_B87)
  add_subdirectory(drivers/usb/device)
  zephyr_include_directories(include)
endif()

# ZMK behaviors join the ZMK app, whose include directories they need.
if(CONFIG_ZMK_BEHAVIOR_PREV_FIRMWARE)
  target_sources(app PRIVATE src/behavior_prev_firmware.c)
endif()

if(CONFIG_CIDOO_DETENT_ENCODER)
  target_sources(app PRIVATE src/detent_encoder.c)
endif()

if(CONFIG_CIDOO_LED_KEY_MATRIX)
  target_sources(app PRIVATE src/led_key_matrix.c)
endif()

if(CONFIG_TLSR_GPIO_MATRIX_SLEEP)
  target_sources(app PRIVATE src/gpio_matrix_sleep.c)
endif()

# US-JIS substitution (src/usjis.c, docs/usjis-substitution.md).
if(CONFIG_ZMK_USJIS)
  set(ZMK_USJIS_LISTENER ${CMAKE_CURRENT_SOURCE_DIR}/src/usjis.c)
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usjis_resolver.c
    ${ZMK_USJIS_LISTENER}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_usjis.c
  )

  # Listener order is link order (the .event_subscription section). Modules
  # are processed before ZMK adds its own sources, so usjis.c would come first
  # and run before hold-tap and key repeat. After ZMK's app directory has been
  # processed, move usjis.c to just before hid_listener.c
  # (docs/usjis-architecture.md section 6). The startup check in usjis.c
  # verifies the result.
  function(zmk_usjis_place_listener)
    get_property(listener GLOBAL PROPERTY ZMK_USJIS_LISTENER_SOURCE)
    get_target_property(sources app SOURCES)
    list(REMOVE_ITEM sources ${listener})
    set(index -1)
    set(i 0)
    foreach(source ${sources})
      if(source MATCHES "/hid_listener\\.c$")
        set(index ${i})
      endif()
      math(EXPR i "${i} + 1")
    endforeach()
    if(index EQUAL -1)
      message(FATAL_ERROR "usjis: hid_listener.c not found among the app sources")
    endif()
    list(INSERT sources ${index} ${listener})
    set_property(TARGET app PROPERTY SOURCES "${sources}")
    message(STATUS "usjis: listener placed before hid_listener.c (source ${index})")
  endfunction()
  set_property(GLOBAL PROPERTY ZMK_USJIS_LISTENER_SOURCE ${ZMK_USJIS_LISTENER})
  cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL zmk_usjis_place_listener)
endif()

if(CONFIG_TLSR_USB_POLL_TIMER)
  target_sources(app PRIVATE src/usb_poll_timer.c)
endif()

if(CONFIG_BATTERY_ADC)
  target_sources(app PRIVATE src/battery_adc.c)
endif()

if(CONFIG_LED_KEY_MATRIX_EFFECTS)
  target_sources(app PRIVATE src/led_key_matrix_effects.c)
endif()

if(CONFIG_LED_DRIVER_SPI)
  target_sources(app PRIVATE src/led_driver_spi.c)
endif()

if(DEFINED CONFIG_TLSR_KEPT_LAYER AND NOT CONFIG_TLSR_KEPT_LAYER EQUAL -1)
  target_sources(app PRIVATE src/layer_keep.c)
endif()

if(CONFIG_TLSR_CRASH_LOG)
  target_sources(app PRIVATE src/tlsr_crash_log.c)
endif()

if(CONFIG_TLSR_PKE_TEST)
  target_sources(app PRIVATE src/tlsr_pke_test.c)
endif()
if(CONFIG_TLSR_RNG_MEASURE)
  target_sources(app PRIVATE src/tlsr_rng_measure.c)
endif()

if(CONFIG_ZMK_BEHAVIOR_BACKLIGHT_RGB)
  target_sources(app PRIVATE src/behavior_backlight_rgb.c)
endif()
if(CONFIG_TLSR_CPU_LEFT)
  target_sources(app PRIVATE src/tlsr_cpu_left.c)
endif()
if(CONFIG_TLSR_STATUS_DISPLAY)
  target_sources(app PRIVATE src/status_display.c src/behavior_status.c)
endif()
# The board's part of the status display: the board's CMakeLists.txt names the
# source (TC32_BOARD_STATUS_SOURCE) and puts its directory on the include path.
if(CONFIG_STATUS_DISPLAY_CIDOO_V75PRO)
  get_property(board_status GLOBAL PROPERTY TC32_BOARD_STATUS_SOURCE)
  target_sources(app PRIVATE ${board_status})
endif()
if(CONFIG_TLSR_USB_ON_REQUEST)
  target_sources(app PRIVATE src/behavior_usb_request.c)
endif()
if(CONFIG_STATUS_DISPLAY_CIDOO_V65V3)
  get_property(board_status GLOBAL PROPERTY TC32_BOARD_STATUS_SOURCE)
  target_sources(app PRIVATE ${board_status})
endif()

if(CONFIG_TLSR_USB_SUSPEND)
  target_sources(app PRIVATE src/usb_suspend.c)
endif()

# The boot guard's healthy mark from ZMK's USB state (a ZMK listener).
if(CONFIG_TLSR_BOOT_GUARD_HEALTHY_USB)
  target_sources(app PRIVATE src/tlsr_boot_guard_usb.c)
endif()

# The own BLE stack (src/ble), on zephyr-tc32's radio driver (SOC_TLSR8278_RADIO, tlsr_radio.h).
if(CONFIG_TLSR_BLE)
  target_sources(app PRIVATE src/ble/ble.c src/ble/ble_adv.c
    src/ble/ble_conn.c src/ble/ble_ll_ctrl.c src/ble/ble_crypto.c src/ble/ble_link.c
    src/ble/ble_smp.c src/ble/ble_att.c src/ble/ble_hid.c src/ble/ble_zmk.c
    src/ble/ble_battery.c src/ble/ble_sleep.c)
  target_sources_ifdef(CONFIG_TLSR_BLE_SC_PASSKEY app PRIVATE src/ble/ble_passkey.c)
endif()
# The bond log, also without the stack: the confirm's flash test (tlsr_usb_ota.c).
if(CONFIG_TLSR_BOND_LOG)
  target_sources(app PRIVATE src/ble/ble_bond.c)
endif()
if(CONFIG_TLSR_P24)
  target_sources(app PRIVATE src/ble/p24.c src/behavior_p24_pair.c)
endif()
if(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
  target_sources(app PRIVATE src/behavior_link_mode.c)
endif()

# tc32_rng's seed record and device ID (src/tlsr_rng_store.c).
if(CONFIG_TLSR_RNG_STORE)
  target_sources(app PRIVATE src/tlsr_rng_store.c)
endif()

# The settings backend in the storage partition (src/tlsr_settings_log.c).
if(CONFIG_TLSR_SETTINGS_LOG)
  target_sources(app PRIVATE src/tlsr_settings_log.c)
endif()

# The bytes left free after a board's hot text (TLSR_HOT_TEXT_PAD), which its hot_text.ld places.
if(CONFIG_TLSR_HOT_TEXT_PAD GREATER 0)
  zephyr_sources(src/tlsr_hot_text_pad.S)
endif()

# With pop {..., pc} returns (TC32_POP_PC_RETURNS), the compile units of the code executed from
# reset until the boot is counted and the power-on chord can take the keyboard back to the other
# slot (the boot guard's early stage; boot_path_units.txt) keep the classic returns; with identical
# code folding (TC32_ICF_SAFE) the linker folds nothing they define or reference.
if(CONFIG_TC32_POP_PC_RETURNS OR CONFIG_TC32_ICF_SAFE OR CONFIG_TC32_ICF_ALL)
  file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/boot_path_units.txt boot_path_units REGEX "^[^#]")
  foreach(unit ${boot_path_units})
    if(unit MATCHES "^zephyr/(.+)")
      set(path ${ZEPHYR_BASE}/${CMAKE_MATCH_1})
    elseif(unit MATCHES "^tc32/(.+)")
      set(path ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_MATCH_1})
    elseif(unit MATCHES "^board/(.+)")
      set(path ${BOARD_DIR}/${CMAKE_MATCH_1})
    else()
      message(FATAL_ERROR "boot_path_units.txt: ${unit}: not under zephyr/, tc32/ or board/")
    endif()
    if(NOT EXISTS ${path})
      message(FATAL_ERROR "boot_path_units.txt: ${unit}: no such file (${path})")
    endif()
    tc32_classic_returns(${path})
  endforeach()
endif()
