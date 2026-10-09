#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build and run tc32/tests/settings-log (the settings backend on native_sim
# with the flash simulator) in the Linux container of tc32/tests/docker. Exit
# status is the test result. The workspace (zephyr-tc32 as zephyr/, build/):
# see tc32/scripts/build.sh.
set -eu
R=$(cd "$(dirname "$0")/../../.." && pwd)
W=$(cd "${TC32_WORKSPACE:-$R/..}" && pwd)
IMG=$("$R/tc32/tests/docker/image.sh" "$W/zephyr")
docker run --rm --network none -v "$W":/w -v "$R":/zmk -w /w "$IMG" sh -c '
	set -e
	export ZEPHYR_BASE=/w/zephyr ZEPHYR_TOOLCHAIN_VARIANT=host
	b=/w/build/tc32-settings-log
	rm -rf "$b"
	cmake -S /zmk/tc32/tests/settings-log -B "$b" -GNinja -DBOARD=native_sim/native/64 \
		-DZephyr_DIR="$ZEPHYR_BASE/share/zephyr-package/cmake" \
		-DZEPHYR_MODULES=/zmk/tc32 > "$b.cmake.log" 2>&1 || { tail -40 "$b.cmake.log"; exit 2; }
	cmake --build "$b" > "$b.build.log" 2>&1 || { grep -E "error|Error" "$b.build.log" | head -40; exit 2; }
	"$b/zephyr/zephyr.exe" -flash_in_ram > "$b.run.log" 2>&1 || true
	grep -vE "^\s*$" "$b.run.log" | tail -60
	grep -q "PROJECT EXECUTION SUCCESSFUL" "$b.run.log"
'
