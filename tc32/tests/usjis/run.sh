#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# US-JIS module tests on ZMK's native_sim test board, for both copies of the
# module: the Keychron B1 Pro's (zmk-kb1-upstream) and zmk-tc32's (V75 Pro).
#
#   bash run.sh                      # all variants, all cases
#   bash run.sh tc32                 # one variant
#   bash run.sh tc32 usjis-hook      # one variant, named cases
#
# Variants:
#   baseline  zmk-kb1-upstream's own tests/zmk cases (unmodified), its module,
#             its pinned upstream ZMK app: what its scripts/run-zmk-tests.sh runs
#   kb1       this suite's cases (cases/), the kb1 module, upstream ZMK app
#   tc32      this suite's cases, the module files copied from the zmk-tc32
#             worktree (src, headers, binding, and the "US-JIS substitution"
#             Kconfig menu and CMake block cut out of tc32/Kconfig and
#             tc32/CMakeLists.txt unchanged), with zmk-tc32's own app/
#
# A case runs in a variant unless its directory has a `variants` file that
# does not list it. A case with a `boots` file runs its executable that many
# times on one simulated flash (erased before the first), "boot N" logged
# before each.
#
# Uses the zmk-kb1-upstream west workspace (KB1_REPO, its workspace/firmware
# prepared by its scripts) read-only, and the pinned image of its
# scripts/build-firmware.sh; runs without network. TC32_WORKTREE defaults to
# the zmk-tc32 tree this suite is in. Neither repository is
# written: build trees, outputs and the generated glue are under build/ here.
set -euo pipefail

suite="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
kb1="${KB1_REPO:-}"
tc32_wt="${TC32_WORKTREE:-$(cd "$suite/../../.." && pwd)}"

if [[ "${1:-}" == in-container ]]; then
  shift
  # ------------------------------------------------------------ container
  variant="$1"; shift
  mkdir -p "$HOME"
  git config --global --add safe.directory '*'
  export PYTHONDONTWRITEBYTECODE=1
  cd /workspace
  export ZEPHYR_BASE=/workspace/zephyr
  west zephyr-export >/dev/null
  case "$variant" in
    baseline) cases=/workspace/config/tests/zmk; support=/workspace/config/tests/zmk/support; app=/workspace/zmk/app ;;
    kb1)      cases=/suite/cases; support=/suite/support; app=/workspace/zmk/app ;;
    tc32)     cases=/suite/cases; support=/suite/support; app=/tc32zmk/app ;;
  esac
  run_case() {
    local dir="$1" name boots i
    name="$(basename "$dir")"
    local build_dir="/workspace/build/$name"
    if ! west build -s "$app" -d "$build_dir" -b native_sim//zmk_test_mock -p -- \
        -DCONFIG_ASSERT=y -DZMK_CONFIG="$dir" \
        -DZMK_EXTRA_MODULES="$support" >"/out/$name.build.log" 2>&1; then
      echo "FAILED: $variant/$name did not build (build/out-$variant/$name.build.log)"
      return 1
    fi
    if [[ -f "$dir/boots" ]]; then
      boots="$(cat "$dir/boots")"
      : >"/out/$name.full.log"
      for ((i = 1; i <= boots; i++)); do
        echo "boot $i" >>"/out/$name.full.log"
        (cd "$build_dir" && timeout 300 ./zephyr/zmk.exe --flash="$build_dir/flash.bin" \
          $([[ $i -eq 1 ]] && echo --flash_erase)) | sed -e 's/.*> //' >>"/out/$name.full.log"
      done
    else
      timeout 300 "$build_dir/zephyr/zmk.exe" | sed -e 's/.*> //' >"/out/$name.full.log"
    fi
    sed -n -f "$dir/events.patterns" "/out/$name.full.log" >"/out/$name.log"
    if diff -u "$dir/keycode_events.snapshot" "/out/$name.log" >"/out/$name.diff"; then
      echo "PASS: $variant/$name ($(wc -l <"/out/$name.log") lines)"
    else
      echo "FAILED: $variant/$name (build/out-$variant/$name.diff)"
      head -60 "/out/$name.diff"
      return 1
    fi
  }
  status=0
  for dir in "$cases"/*/; do
    dir="${dir%/}"
    [[ -f "$dir/native_sim.keymap" ]] || continue
    if [[ $# -gt 0 ]] && [[ ! " $* " == *" $(basename "$dir") "* ]]; then
      continue
    fi
    if [[ -f "$dir/variants" ]] && ! grep -qw "$variant" "$dir/variants"; then
      continue
    fi
    run_case "$dir" || status=1
  done
  exit $status
fi

# ---------------------------------------------------------------- host
case "${1:-all}" in
  all) variants=(baseline kb1 tc32); [[ $# -gt 0 ]] && shift ;;
  baseline|kb1|tc32) variants=("$1"); shift ;;
  *) echo "usage: $0 [all|baseline|kb1|tc32] [case...]" >&2; exit 2 ;;
esac

[[ -n "$kb1" ]] || { echo "KB1_REPO: a zmk-kb1-upstream checkout whose workspace/firmware is prepared" >&2; exit 2; }
python3 "$suite/generate.py" --check
image_ref="$(sed -n 's/^image_ref="\(.*\)"$/\1/p' "$kb1/scripts/build-firmware.sh")"
image_platform="$(sed -n 's/^image_platform="\(.*\)"$/\1/p' "$kb1/scripts/build-firmware.sh")"
workspace="$kb1/workspace/firmware"
[[ -d "$workspace/.west" ]] || { echo "kb1 workspace not prepared: $workspace" >&2; exit 1; }

# The TC32 glue: the module files as they are in the worktree, plus its
# Kconfig menu and CMake block cut out unchanged; it is also the west
# manifest repository (config) of the read-only kb1 workspace, so that the
# kb1 module is not picked up.
prepare_tc32() {
  local glue="$suite/build/glue-tc32" t="$tc32_wt/tc32" f
  rm -rf "$glue"
  mkdir -p "$glue/config" "$glue/zephyr" "$glue/src" "$glue/include/zmk" \
    "$glue/include/dt-bindings/zmk" "$glue/dts/bindings/behaviors"
  cp "$suite/manifest-tc32/config/west.yml" "$glue/config/west.yml"
  printf 'name: usjis-tc32-copy\nbuild:\n  cmake: .\n  kconfig: Kconfig\n  settings:\n    dts_root: .\n' \
    >"$glue/zephyr/module.yml"
  for f in src/usjis.c src/usjis_resolver.c src/usjis_resolver.h src/behavior_usjis.c \
      include/zmk/usjis.h include/dt-bindings/zmk/usjis.h \
      dts/bindings/behaviors/zmk,behavior-usjis.yaml; do
    cp "$t/$f" "$glue/$f"
  done
  awk '/^menu "US-JIS substitution"$/ {on = 1} on {print} on && /^endmenu$/ {exit}' \
    "$t/Kconfig" >"$glue/Kconfig"
  awk '/^if\(CONFIG_ZMK_USJIS\)$/ {on = 1} on {print} on && /^endif\(\)$/ {exit}' \
    "$t/CMakeLists.txt" >"$glue/CMakeLists.txt"
  grep -q '^endmenu$' "$glue/Kconfig" || { echo "US-JIS Kconfig menu not found" >&2; exit 1; }
  grep -q '^endif()$' "$glue/CMakeLists.txt" || { echo "US-JIS CMake block not found" >&2; exit 1; }
  {
    echo "worktree: $tc32_wt"
    echo "HEAD: $(git --no-optional-locks -C "$tc32_wt" rev-parse HEAD)"
    echo "changes in tc32/ against HEAD:"
    git --no-optional-locks -C "$tc32_wt" status --porcelain -- tc32/src tc32/include \
      tc32/dts tc32/Kconfig tc32/CMakeLists.txt app
    (cd "$glue" && shasum -a 256 src/* include/zmk/usjis.h include/dt-bindings/zmk/usjis.h \
      dts/bindings/behaviors/* Kconfig CMakeLists.txt)
  } >"$suite/build/tc32-provenance.txt"
}

mkdir -p "$suite/build/home"
status=0
for variant in "${variants[@]}"; do
  config_src="$kb1"
  extra=()
  if [[ "$variant" == tc32 ]]; then
    prepare_tc32
    config_src="$suite/build/glue-tc32"
    extra=(--mount "type=bind,source=$tc32_wt/app,target=/tc32zmk/app,readonly")
  fi
  rm -rf "$suite/build/ws-$variant" "$suite/build/out-$variant"
  mkdir -p "$suite/build/ws-$variant" "$suite/build/out-$variant"
  docker run --rm --platform "$image_platform" --network none \
    --user "$(id -u):$(id -g)" \
    --mount "type=bind,source=$workspace,target=/workspace,readonly" \
    --mount "type=bind,source=$suite/build/ws-$variant,target=/workspace/build" \
    --mount "type=bind,source=$suite/build/home,target=/workspace/.home" \
    --mount "type=bind,source=$config_src,target=/workspace/config,readonly" \
    --mount "type=bind,source=$suite,target=/suite,readonly" \
    --mount "type=bind,source=$suite/build/out-$variant,target=/out" \
    ${extra[@]+"${extra[@]}"} \
    --workdir /workspace \
    --env HOME=/workspace/.home \
    "$image_ref" bash /suite/run.sh in-container "$variant" "$@" || status=1
done
[[ -f "$suite/build/tc32-provenance.txt" ]] && sed -n 1,2p "$suite/build/tc32-provenance.txt"
exit $status
