#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# One ZMK snapshot test in the container (called by run.sh). Prints one line:
# PASS, FAILED (events differ), NOBUILD or PENDING (differs, marked pending).
path="$1"
C=/zmk/tc32/compat
case=$(echo "$path" | sed -n -e "s|.*/tests/||p")
b=/w/build/zmk-tests/$case
rm -rf "$b"
mkdir -p "$b"
conf="$C/zephyr44.conf"
[ -f "$path/native_sim.conf" ] && conf="$conf;$path/native_sim.conf"
extra=""
[ -f "$path/extra-cmake-args" ] && extra=$(cat "$path/extra-cmake-args" | tr '\n' ' ')
# shellcheck disable=SC2086
if ! cmake -S /zmk/app -B "$b/build" -GNinja \
	-DZephyr_DIR="$ZEPHYR_BASE/share/zephyr-package/cmake" \
	"-DZEPHYR_MODULES=$C;/zmk/app/module;/zmk/app/keymap-module" \
	-DBOARD=native_sim//zmk_test_mock -DCONFIG_ASSERT=y -DZMK_CONFIG="$path" \
	"-DEXTRA_DTC_OVERLAY_FILE=$path/native_sim.keymap" "-DEXTRA_CONF_FILE=$conf" $extra \
	> "$b/cmake.log" 2>&1 || ! cmake --build "$b/build" > "$b/build.log" 2>&1; then
	echo "NOBUILD $case"
	exit 0
fi
timeout 60 "$b/build/zephyr/zmk.exe" -stop_at=30 2>&1 | sed -e "s/.*> //" > "$b/keycode_events_full.log" || true
sed -n -f "$path/events.patterns" "$b/keycode_events_full.log" > "$b/keycode_events.log"
if diff -auZ "$path/keycode_events.snapshot" "$b/keycode_events.log" > "$b/diff.log"; then
	echo "PASS $case"
elif [ -f "$path/pending" ]; then
	echo "PENDING $case"
else
	echo "FAILED $case"
fi
