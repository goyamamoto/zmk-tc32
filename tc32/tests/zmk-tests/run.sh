#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Run ZMK's own snapshot tests (app/tests: a mock key scan presses keys, the
# key events in the log are compared with keycode_events.snapshot) on
# native_sim, built against zephyr-tc32 (Zephyr 4.4) with tc32/compat, in the
# Linux container of tc32/tests/docker. This is what upstream ZMK runs with its
# Zephyr 4.1 (app/run-test.sh), minus west. Results:
# <workspace>/build/zmk-tests/pass-fail.log for a full run, or
# pass-fail.<path with / as _>.log for a partial one, one line per test. The
# exit status is non-zero if any test failed or did not build (as upstream).
# The workspace (zephyr-tc32 as zephyr/): see tc32/scripts/build.sh.
#
# Usage: tc32/tests/zmk-tests/run.sh [test path under app/tests, default: all] [jobs]
set -eu
R=$(cd "$(dirname "$0")/../../.." && pwd)
W=$(cd "${TC32_WORKSPACE:-$R/..}" && pwd)
filter=${1:-}
jobs=${2:-6}
IMG=$("$R/tc32/tests/docker/image.sh" "$W/zephyr")
log=pass-fail.log
[ -n "$filter" ] && log="pass-fail.$(echo "$filter" | tr / _).log"
docker run --rm --network none -v "$W":/w -v "$R":/zmk -w /w -e FILTER="$filter" -e JOBS="$jobs" -e LOG="$log" "$IMG" sh -c '
	set -e
	export ZEPHYR_BASE=/w/zephyr ZEPHYR_TOOLCHAIN_VARIANT=host
	out=/w/build/zmk-tests
	mkdir -p "$out"
	cases=$(find /zmk/app/tests/$FILTER -name native_sim.keymap -exec dirname {} \; | sort)
	echo "$cases" | xargs -P "$JOBS" -I{} sh /zmk/tc32/tests/zmk-tests/one.sh {} > "$out/$LOG.tmp" 2>&1 || true
	sort -k2 "$out/$LOG.tmp" > "$out/$LOG"
	rm -f "$out/$LOG.tmp"
	echo "PASS $(grep -c "^PASS" $out/$LOG)  FAIL $(grep -c "^FAILED" $out/$LOG)  BUILD $(grep -c "^NOBUILD" $out/$LOG)  PENDING $(grep -c "^PENDING" $out/$LOG)  ($out/$LOG)"
	! grep -qE "^(FAILED|NOBUILD)" "$out/$LOG"
'
