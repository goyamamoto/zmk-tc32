#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build ZMK's snapshot tests (app/tests: the directories with a
# native_sim.keymap) for the TC32, to run in tc32-devtools' emulator
# (run-boot --console reads the log). Each test is tlsr8278_generic with the
# native_sim test board's mock key matrix and settings (zmk_test_mock.overlay,
# zmk_test_mock.conf), the test's native_sim.keymap, native_sim.conf and
# extra-cmake-args, built with the clang for ARMv4T Thumb
# (toolchains/thumb-llvm) and written as a TC32 image by thumb2tc32.py, or with
# DIRECT=1 on the direct path (-mcpu=tc32, elf2bin.py; tc32/scripts/build.sh). The
# log goes to a register the chip does not have: these images are for the
# emulator only.
#
# Usage: tc32/tests/zmk-tests-tc32/build.sh [test path under app/tests, default: all] [jobs]
# Output: <workspace>/build/zmk-tests-tc32/<test>/zephyr/zmk.{elf,bin} with
# <test>/build.log, and build/zmk-tests-tc32/build-status[.<path>].txt, one
# line per test (BUILT or NOBUILD). The workspace and tc32-devtools: see
# tc32/scripts/build.sh.
set -u
R=$(cd "$(dirname "$0")/../../.." && pwd)
M="$R/tc32"
T="$M/tests/zmk-tests-tc32"
W=$(cd "${TC32_WORKSPACE:-$R/..}" && pwd) || exit 1
D=$(cd "${TC32_DEVTOOLS:-$W/../tc32-devtools}" && pwd) || exit 1
out="$W/build/zmk-tests-tc32"
direct=OFF writer=thumb2tc32.py
[ "${DIRECT:-0}" = 1 ] && direct=ON writer=elf2bin.py

if [ "${1:-}" != "--one" ]; then
  filter=${1:-}
  jobs=${2:-4}
  status="$out/build-status.txt"
  [ -n "$filter" ] && status="$out/build-status.$(echo "$filter" | tr / _).txt"
  mkdir -p "$out"
  find "$R/app/tests/$filter" -name native_sim.keymap -exec dirname {} \; | sort |
    xargs -P "$jobs" -I{} sh "$0" --one {} > "$status.tmp"
  sort -k2 "$status.tmp" > "$status" && rm -f "$status.tmp"
  echo "BUILT $(grep -c '^BUILT' "$status")  NOBUILD $(grep -c '^NOBUILD' "$status")  ($status)"
  exit 0
fi

src=$2
t=${src#"$R/app/tests/"}
b="$out/$t"
confs="$M/compat/zephyr44.conf;$T/zmk_test_mock.conf"
[ -f "$src/native_sim.conf" ] && confs="$confs;$src/native_sim.conf"
extra=""
[ -f "$src/extra-cmake-args" ] && extra=$(tr '\n' ' ' < "$src/extra-cmake-args")
export PATH="$W/.venv-zephyr/bin:$PATH" ZEPHYR_BASE="$W/zephyr"
rm -rf "$b"
mkdir -p "$b"
# shellcheck disable=SC2086  # extra holds separate -D arguments
{
  cmake -S "$R/app" -B "$b" -GNinja \
    -DZephyr_DIR="$ZEPHYR_BASE/share/zephyr-package/cmake" \
    "-DZEPHYR_MODULES=$M/compat;$R/app/module;$M;$R/app/keymap-module" \
    -DBOARD=tlsr8278_generic -DZEPHYR_TOOLCHAIN_VARIANT=host/llvm \
    -DLLVM_TOOLCHAIN_PATH="$W/toolchains/thumb-llvm" -DTC32_THUMB=ON -DTC32_DIRECT=$direct -DTC32_THUMB_TOOLS="$D/compiler" \
    -DCMAKE_READELF="$W/toolchains/thumb-llvm/bin/llvm-readelf" \
    -DPython3_EXECUTABLE="$W/.venv-zephyr/bin/python" \
    -DUSER_CACHE_DIR="$W/build/cache" \
    "-DZMK_CONFIG=$src" \
    "-DEXTRA_DTC_OVERLAY_FILE=$T/zmk_test_mock.overlay;$src/native_sim.keymap" \
    "-DEXTRA_CONF_FILE=$confs" $extra &&
  cmake --build "$b" &&
  python3 "$D/compiler/$writer" "$b/zephyr/zmk.elf" "$b/zephyr/zmk.bin" &&
  python3 "$D/compiler/image_check.py" "$b/zephyr/zmk.elf" "$b/zephyr/zmk.bin"
} > "$b/build.log" 2>&1
rc=$?
if [ $rc -eq 0 ]; then echo "BUILT $t"; else echo "NOBUILD $t (see $b/build.log)"; fi
