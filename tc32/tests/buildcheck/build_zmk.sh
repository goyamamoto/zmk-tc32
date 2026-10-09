#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build-only check of ZMK on zephyr-tc32's generic boards with a mock key
# matrix (the key-processing core; no USB or BLE). Set ZMK_USJIS=<path to
# zmk-usjis> to add that module and its keymap/config (usjis.keymap,
# usjis.conf); plain ZMK otherwise.
# Usage: tc32/tests/buildcheck/build_zmk.sh [board]  (default tlsr8278_generic)
# Output: <workspace>/build/zmk_<board>[_usjis] (see tc32/scripts/build.sh for
# the workspace).
set -u
R=$(cd "$(dirname "$0")/../../.." && pwd)
W=$(cd "${TC32_WORKSPACE:-$R/..}" && pwd) || exit 1
T="$R/tc32/tests/buildcheck"
C="$R/tc32/compat"
board=${1:-tlsr8278_generic}
b="$W/build/zmk_$board"
modules="$C;$R/app/module;$R/app/keymap-module"
keymap="$T/tc32_mock.keymap"
conf="$T/tc32_mock.conf"
if [ -n "${ZMK_USJIS:-}" ]; then
  modules="$C;$R/app/module;$ZMK_USJIS;$R/app/keymap-module"
  keymap="$T/usjis.keymap"
  conf="$T/tc32_mock.conf;$T/usjis.conf"
  b="${b}_usjis"
fi
export PATH="$W/.venv-zephyr/bin:$PATH" ZEPHYR_BASE="$W/zephyr"
rm -rf "$b"
mkdir -p "$W/build"
cmake -S "$R/app" -B "$b" -GNinja \
  -DZephyr_DIR="$ZEPHYR_BASE/share/zephyr-package/cmake" \
  "-DZEPHYR_MODULES=$modules" \
  -DBOARD="$board" -DZEPHYR_TOOLCHAIN_VARIANT=host/llvm \
  -DLLVM_TOOLCHAIN_PATH="$W/toolchains/tc32-stage2" \
  -DCMAKE_READELF="$W/toolchains/shim/llvm-readelf" \
  -DPython3_EXECUTABLE="$W/.venv-zephyr/bin/python" \
  -DUSER_CACHE_DIR="$W/build/cache" \
  "-DEXTRA_DTC_OVERLAY_FILE=$T/tc32_mock.overlay;$keymap" \
  "-DEXTRA_CONF_FILE=$conf" > "$b.cmake.log" 2>&1 &&
cmake --build "$b" -- -k 0 > "$b.build.log" 2>&1
rc=$?
echo "board=$board exit=$rc (logs: $b.cmake.log, $b.build.log)"
exit $rc
