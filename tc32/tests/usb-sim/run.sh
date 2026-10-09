#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build and run the test of the TLSR8278 usb_dc driver of tc32/drivers/usb/device
# (ZMK on native_sim//zmk_test_mock with the register model and scripted host of
# this module, and the USB OTA receiver of tc32/) in the Linux container of tc32/tests/docker. The image is
# built once (network needed); the test build runs without network. Exit
# status is the test result. The workspace (zephyr-tc32 as zephyr/, build/):
# see tc32/scripts/build.sh.
set -eu
R=$(cd "$(dirname "$0")/../../.." && pwd)
W=$(cd "${TC32_WORKSPACE:-$R/..}" && pwd)
IMG=$("$R/tc32/tests/docker/image.sh" "$W/zephyr")
docker run --rm --network none -v "$W":/w -v "$R":/zmk -w /w "$IMG" sh -c '
	set -e
	export ZEPHYR_BASE=/w/zephyr ZEPHYR_TOOLCHAIN_VARIANT=host
	C=/zmk/tc32/compat U=/zmk/tc32/tests/usb-sim b=/w/build/usbsim
	rm -rf "$b"
	cmake -S /zmk/app -B "$b" -GNinja \
		-DZephyr_DIR="$ZEPHYR_BASE/share/zephyr-package/cmake" \
		"-DZEPHYR_MODULES=$C;/zmk/app/module;/zmk/tc32;$U;/zmk/app/keymap-module" \
		-DBOARD=native_sim//zmk_test_mock \
		"-DEXTRA_DTC_OVERLAY_FILE=$U/test/native_sim.overlay;$U/test/usbsim.keymap" \
		"-DEXTRA_CONF_FILE=$C/zephyr44.conf;$U/test/usbsim.conf" \
		> "$b.cmake.log" 2>&1 || { tail -40 "$b.cmake.log"; exit 2; }
	cmake --build "$b" > "$b.build.log" 2>&1 || { grep -E "error|Error" "$b.build.log" | head -40; exit 2; }
	"$b/zephyr/zmk.exe" -stop_at=120 -flash_in_ram > "$b.run.log" 2>&1 || true
	grep -vE "^\s*$|<dbg>" "$b.run.log"
	grep -q "RESULT: PASS" "$b.run.log"
	# usb_write() warns on each -EAGAIN; the driver completes taken IN packets
	# itself, so a write should never find the endpoint still busy.
	if grep -q "Failed to write endpoint buffer" "$b.run.log"; then
		echo "FAIL: $(grep -c "Failed to write endpoint buffer" "$b.run.log") endpoint writes found the endpoint busy"
		exit 1
	fi
'
