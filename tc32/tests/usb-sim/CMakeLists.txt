# SPDX-License-Identifier: GPL-3.0-or-later

if(CONFIG_TLSR_USB_SIM_TEST)
  zephyr_library()
  zephyr_library_sources(src/tlsr_usb_model.c src/tlsr_usb_host.c)

  if(CONFIG_TLSR_SLOTS_SIM)
    # OTA frames made by tc32/scripts/telink_ota.py, replayed by the host.
    set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/gen)
    set(tool ${ZEPHYR_ZMK_TC32_MODULE_DIR}/scripts/telink_ota.py)
    add_custom_command(
      OUTPUT ${gen_dir}/ota_vectors.h
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/test/gen_ota_vectors.py
              ${tool} ${gen_dir}/ota_vectors.h
      DEPENDS ${CMAKE_CURRENT_LIST_DIR}/test/gen_ota_vectors.py ${tool}
    )
    add_custom_target(tlsr_usb_ota_vectors DEPENDS ${gen_dir}/ota_vectors.h)
    add_dependencies(${ZEPHYR_CURRENT_LIBRARY} tlsr_usb_ota_vectors)
    zephyr_library_include_directories(${gen_dir})
    zephyr_library_sources(src/tlsr_usb_ota_host.c)
  endif()
endif()
