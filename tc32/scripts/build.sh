#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build ZMK for a TC32 board of the tc32-keyboards repository ($TC32_BOARDS, by
# default <workspace>/tc32-keyboards; its boards/ holds the board directories).
# The boards have no flash runner; zmk.ota.bin goes through
# tc32/scripts/telink_ota.py flash.
#
# Usage: tc32/scripts/build.sh [board]   (default cidoo_v75pro)
# Output: zmk.elf, zmk.bin and zmk.ota.bin (the image for telink_ota.py flash),
# in <workspace>/build/zmk_<board>/zephyr, or in $BUILD_DIR/zephyr if set.
# $EXTRA_CONF adds Kconfig fragments (;-separated), for experiments; $KEYMAP replaces the
# board keymap (for example cidoo_v21_ble.keymap with TLSR_BLE).
# $EXTRA_MODULES adds Zephyr modules (;-separated): nanopb and zmk-studio-messages for ZMK Studio.
# $BOOT_PATH_REF names the board's recorded listing of the code executed from reset until the boot
# is counted and the power-on chord can take the keyboard back to the other slot (the boot guard's
# early stage; boot_path_check.py), which the build is checked against.
# $BUILD_VERSION replaces the build string in the boot banner (default: from the Zephyr tree's VERSION).
# THUMB=1 builds with the clang for ARMv4T Thumb in toolchains/thumb-llvm
# instead of the LLVM TC32 toolchain; thumb2tc32.py then writes zmk.bin. When the
# configuration sets TC32_POP_PC_RETURNS, that clang must be llvm-tc32 (TC32.md).
# DIRECT=1 (implies THUMB=1) takes the direct path with the same clang, which must
# be llvm-tc32: it writes TC32 code itself (-mcpu=tc32, zephyr-tc32's
# TC32_DIRECT), the assembly sources are their standard-syntax twins
# (<name>.ual.S), and elf2bin.py writes zmk.bin from the link without
# re-encoding. The checks below run on both paths.
#
# The workspace is the folder that holds this repository as zmk/ and
# zephyr-tc32 as zephyr/ (the layout west gives), with toolchains/ and the
# Python venv .venv-zephyr/: $TC32_WORKSPACE, by default the parent of this
# repository. The Thumb tools (tc32asm2thumb.py, thumb2tc32.py,
# image_check.py, forms_check.py) come from tc32-devtools: $TC32_DEVTOOLS, by
# default tc32-devtools next to the workspace.
set -u
R=$(cd "$(dirname "$0")/../.." && pwd)
M="$R/tc32"
W=$(cd "${TC32_WORKSPACE:-$R/..}" && pwd) || exit 1
D=$(cd "${TC32_DEVTOOLS:-$W/../tc32-devtools}" && pwd) || exit 1
B=$(cd "${TC32_BOARDS:-$W/tc32-keyboards}" && pwd) || exit 1
board=${1:-cidoo_v75pro}
bdir=$(dirname "$(find "$B/boards" -name board.yml -path "*/$board/*" | head -1)")
b=${BUILD_DIR:-"$W/build/zmk_$board"}
direct=OFF
[ "${DIRECT:-0}" = 1 ] && direct=ON
if [ "${THUMB:-0}" = 1 ] || [ $direct = ON ]; then
  tc="$W/toolchains/thumb-llvm" readelf="$W/toolchains/thumb-llvm/bin/llvm-readelf" thumb=ON
else
  tc="$W/toolchains/tc32-stage2" readelf="$W/toolchains/shim/llvm-readelf" thumb=OFF
fi
export PATH="$W/.venv-zephyr/bin:$PATH" ZEPHYR_BASE="$W/zephyr"
# The Zephyr build string the image carries in its boot banner: $BUILD_VERSION, by
# default v<major>.<minor>.<patchlevel>[-<extraversion>]-tc32 from the Zephyr
# tree's VERSION file. It is always passed to the build, so an image depends on
# the sources and the tools, not on a repository's history or tags.
bv=${BUILD_VERSION:-$(awk -F' *= *' '
  $1 == "VERSION_MAJOR" { a = $2 } $1 == "VERSION_MINOR" { i = $2 }
  $1 == "PATCHLEVEL" { p = $2 } $1 == "EXTRAVERSION" { e = $2 }
  END { printf "v%s.%s.%s%s-tc32", a, i, p, (e != "" ? "-" e : "") }' "$W/zephyr/VERSION")}
rm -rf "$b"
mkdir -p "$(dirname "$b")"
# ZMK's keymap lookup (boards/post_boards_shields.cmake) is called by a patch in
# ZMK's Zephyr fork, which zephyr-tc32 lacks, so pass the board keymap directly.
# tc32/compat must come before ZMK's app/module (tc32/compat/CMakeLists.txt).
cmake -S "$R/app" -B "$b" -GNinja \
  -DZephyr_DIR="$ZEPHYR_BASE/share/zephyr-package/cmake" \
  "-DZEPHYR_MODULES=$M/compat;$R/app/module;$M;$B;$R/app/keymap-module${EXTRA_MODULES:+;$EXTRA_MODULES}" \
  -DBOARD="$board" -DZEPHYR_TOOLCHAIN_VARIANT=host/llvm \
  -DLLVM_TOOLCHAIN_PATH="$tc" -DTC32_THUMB=$thumb -DTC32_DIRECT=$direct -DTC32_THUMB_TOOLS="$D/compiler" \
  -DCMAKE_READELF="$readelf" \
  -DPython3_EXECUTABLE="$W/.venv-zephyr/bin/python" \
  -DUSER_CACHE_DIR="$W/build/cache" \
  -DBUILD_VERSION="$bv" \
  -DEXTRA_DTC_OVERLAY_FILE="${KEYMAP:-$bdir/$board.keymap}" \
  -DEXTRA_CONF_FILE="$M/compat/zephyr44.conf${EXTRA_CONF:+;$EXTRA_CONF}" > "$b.cmake.log" 2>&1 &&
cmake --build "$b" -- -k 0 > "$b.build.log" 2>&1
rc=$?
echo "board=$board exit=$rc (logs: $b.cmake.log, $b.build.log)"
# The OTA image: size word and CRC-32 for the Telink OTA, and the 128 KB slot
# limit, which the linker does not check.
if [ $rc -eq 0 ] && [ $thumb = ON ]; then
  if [ $direct = ON ]; then
    python3 "$D/compiler/elf2bin.py" "$b/zephyr/zmk.elf" "$b/zephyr/zmk.bin" || rc=$?
  else
    python3 "$D/compiler/thumb2tc32.py" "$b/zephyr/zmk.elf" "$b/zephyr/zmk.bin" || rc=$?
  fi
  # Every byte of the written image against the ELF, laid out by llvm-objcopy
  # (on the direct path every byte is the ELF's).
  # With the SPI flash transport (src/tlsr_spi_flash_io.S), also that it sits
  # under a $d mapping symbol and its bytes equal the manifest's
  # (thumb2tc32.py left it).
  # With zephyr-tc32's setup/tlsr8278_startup.S (SOC_TLSR8278_STARTUP), also
  # that the image's startup instructions and the header's ROM copy size equal
  # its manifest's, and the pools hold the link's values.
  io="$M/src/tlsr_spi_flash_io.json"
  ss="$W/zephyr/soc/telink/tlsr/tlsr827x/setup/tlsr8278_startup.json"
  [ $rc -eq 0 ] && { python3 "$D/compiler/image_check.py" "$b/zephyr/zmk.elf" "$b/zephyr/zmk.bin" \
    $( [ -f "$io" ] && grep -q '^CONFIG_TLSR_SPI_FLASH=y' "$b/zephyr/.config" && echo --blob "$io" ) \
    $( [ -f "$ss" ] && grep -q '^CONFIG_SOC_TLSR8278_STARTUP=y' "$b/zephyr/.config" && echo --startup "$ss" ) || rc=$?; }
  # Every instruction form of the image against the forms Telink's own code
  # uses (tc32-devtools compiler/vendor_forms.txt): a form without that
  # evidence fails the build. The full report is in $b.forms.txt.
  [ $rc -eq 0 ] && { python3 "$D/compiler/forms_check.py" check "$b/zephyr/zmk.elf" $( [ $direct = OFF ] && echo --thumb ) \
    > "$b.forms.txt" 2>&1 \
    || { grep -E "NO VENDOR USE|form\(s\)" "$b.forms.txt"; rc=1; }; }
  [ $rc -eq 0 ] && tail -1 "$b.forms.txt"
fi
# The flash supply trims (tc32/src/tlsr_spi_flash.c) write while the supply
# of the flash the code runs from moves. Their functions, each by its size in
# the symbol table (literal pool included), must lie in the RAM code below
# _tc32_boot_copy_lma_end, branch only into one another (calls, plain and
# conditional branches, tail calls), write pc only to return (bx lr, pop with
# pc), and hold no literal that points into the flash above the RAM code.
if [ $rc -eq 0 ] && grep -q '^CONFIG_TLSR_SPI_FLASH=y' "$b/zephyr/.config"; then
  python3 - "$b/zephyr/zmk.elf" "$tc/bin/llvm-nm" "$tc/bin/llvm-objdump" <<'PY' || rc=$?
import re, subprocess, sys
elf, nm, objdump = sys.argv[1:4]
trim = ("trim_set", "trim_get", "trim_analog_read", "trim_analog_write")
REGS = 0x800000  # registers and SRAM start here; below is the flash (XIP)
sym = {}
for line in subprocess.run([nm, "-S", "-n", elf], capture_output=True, text=True, check=True).stdout.splitlines():
    p = line.split()
    if len(p) == 4:
        sym.setdefault(p[3], (int(p[0], 16), int(p[1], 16)))
    elif len(p) == 3:
        sym.setdefault(p[2], (int(p[0], 16), None))
end = sym.get("_tc32_boot_copy_lma_end", (None, None))[0]
bad = [] if end is not None else ["no _tc32_boot_copy_lma_end"]
spans = {}
for f in trim:
    if f not in sym or not sym[f][1]:
        bad.append(f"{f} missing, or without a size")
    else:
        spans[f] = (sym[f][0], sym[f][0] + sym[f][1])
inside = lambda v: any(s <= v < e for s, e in spans.values())
insn = re.compile(r"^\s*([0-9a-f]+):(?:\s+[0-9a-f]{2,4})+\s+(\S+)\s*([^@]*)")
branch = re.compile(r"b|bl|blx|b(?:eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)|tj\w*")
for f, (start, stop) in spans.items():
    if end is not None and stop > end:
        bad.append(f"{f} at 0x{start:x}-0x{stop:x} is not in the RAM code (ends 0x{end:x})")
    dis = subprocess.run([objdump, "-d", f"--start-address={start}", f"--stop-address={stop}", elf],
                         capture_output=True, text=True, check=True).stdout
    for l in dis.splitlines():
        m = insn.match(l)
        if not m:
            continue
        at, op, args = int(m.group(1), 16), m.group(2), m.group(3).strip()
        first = args.split(",")[0].split()[0] if args else ""
        if op == ".word":
            v = int(first, 16)
            if end is not None and end <= v < REGS:
                bad.append(f"{f} at 0x{at:x}: literal 0x{v:x} points into the flash above the RAM code")
        elif branch.fullmatch(op):
            if first.startswith("0x"):
                if not inside(int(first, 16)):
                    bad.append(f"{f} at 0x{at:x}: {op} to {first}, outside the trim's functions")
            elif first != "lr":
                bad.append(f"{f} at 0x{at:x}: {op} through {first}")
        elif op in ("bx", "tjex"):
            if first != "lr":
                bad.append(f"{f} at 0x{at:x}: {op} {first}")
        elif first == "pc" and not op.startswith(("pop", "tpop")):
            bad.append(f"{f} at 0x{at:x}: {op} writes pc")
if bad:
    sys.exit("flash supply trims: " + "; ".join(bad))
print(f"flash supply trims: {len(trim)} functions in the RAM code (below 0x{end:x}), branching only into one another")
PY
fi
# The code executed from reset until the boot is counted and the power-on chord can take the keyboard
# back to the other slot (the boot guard's early stage): each of its functions the same instructions as
# the board's recorded listing, $BOOT_PATH_REF (boot_path_check.py; its compile units are listed in
# tc32/boot_path_units.txt). The full report is in $b.boot_path.txt.
if [ $rc -eq 0 ] && [ $thumb = ON ] && [ -n "${BOOT_PATH_REF:-}" ]; then
  TC32_DEVTOOLS="$D" python3 "$M/scripts/boot_path_check.py" check "$tc/bin/llvm-objdump" "$tc/bin/llvm-nm" \
    "$b/zephyr/zmk.elf" "$b/zephyr/zmk.map" "$BOOT_PATH_REF" > "$b.boot_path.txt" 2>&1 \
    || { grep -E "^(DIFF|missing) " "$b.boot_path.txt"; rc=1; }
  tail -1 "$b.boot_path.txt"
fi
# With tc32_rng (TC32_RNG): only tc32_rng.c holds an address of the random bit generator's registers, and no
# code of Zephyr's sys_rand, sys_csrand or entropy APIs is linked (rng_users_check.py).
if [ $rc -eq 0 ] && grep -q '^CONFIG_TC32_RNG=y' "$b/zephyr/.config"; then
  python3 "$M/scripts/rng_users_check.py" "$tc/bin" "$b/zephyr/zmk.elf" "$b/zephyr/zmk.map" || rc=$?
fi
# The constants the key scan reads, each in a flash cache slot of its own (TLSR_HOT_TEXT_CHECK, hot_slots.py).
if [ $rc -eq 0 ] && grep -q '^CONFIG_TLSR_HOT_TEXT_CHECK=y' "$b/zephyr/.config"; then
  python3 "$M/scripts/hot_slots.py" "$tc/bin/llvm-nm" "$b/zephyr/zmk.elf" --pads | tail -2
  python3 "$M/scripts/hot_slots.py" "$tc/bin/llvm-nm" "$b/zephyr/zmk.elf" > /dev/null || rc=$?
fi
# Behaviors named by the CRC-16 of their names (ZMK Studio, the saved keymap): no two alike.
if [ $rc -eq 0 ] && grep -q '^CONFIG_ZMK_BEHAVIOR_LOCAL_ID_TYPE_CRC16=y' "$b/zephyr/.config"; then
  "$W/.venv-zephyr/bin/python" "$M/scripts/behavior_ids_check.py" "$b/zephyr/zmk.elf" || rc=$?
fi
if [ $rc -eq 0 ]; then
  python3 "$M/scripts/telink_ota.py" image "$b/zephyr/zmk.bin" "$b/zephyr/zmk.ota.bin" || rc=$?
fi
# What the image was built from, next to it (build-info.json): the commits of
# this repository, tc32-keyboards, zephyr-tc32 and tc32-devtools (-dirty when the tree had
# uncommitted changes; the full hash too, since describe depends on the tags a
# clone has), the compiler and host tools, the build string given to Zephyr
# (build_version) and the one read back from the image's boot banner, extra
# Kconfig fragments and the OTA image's SHA-256,
# so that a flashed image can be traced to its sources and rebuilt.
if [ $rc -eq 0 ]; then
  desc() { git -C "$1" describe --always --dirty --abbrev=12 2>/dev/null || echo unknown; }
  fullhash() { git -C "$1" rev-parse HEAD 2>/dev/null || echo unknown; }
  first() { out=$("$@" 2>/dev/null | sed -n 1p | tr -d '"\\'); echo "${out:-unknown}"; }
  sha=$(shasum -a 256 "$b/zephyr/zmk.ota.bin" | cut -d' ' -f1)
  cc=$(ls "$tc/bin/clang" 2>/dev/null || ls "$tc/bin/"*clang 2>/dev/null | sed -n 1p)
  cc=${cc:-/nonexistent-compiler}
  banner=$(strings -n 8 "$b/zephyr/zmk.bin" | sed -n 's/.*Booting Zephyr OS build \([^ ]*\).*/\1/p' | sed -n 1p)
  cat > "$b/zephyr/build-info.json" <<INFO
{"board": "$board", "thumb": $([ $thumb = ON ] && echo true || echo false), "direct": $([ $direct = ON ] && echo true || echo false), "extra_conf": "${EXTRA_CONF:-}",
 "zmk": "$(desc "$R")", "boards": "$(desc "$B")", "zephyr": "$(desc "$W/zephyr")", "devtools": "$(desc "$D")",
 "zmk_commit": "$(fullhash "$R")", "boards_commit": "$(fullhash "$B")", "zephyr_commit": "$(fullhash "$W/zephyr")", "devtools_commit": "$(fullhash "$D")",
 "build_version": "$bv", "zephyr_build_string": "${banner:-unknown}",
 "compiler": "$(first "$cc" --version)", "compiler_path": "$(first python3 -c 'import os, sys; p = sys.argv[1]; print(os.path.realpath(p) if os.path.exists(p) else "")' "$cc")",
 "cmake": "$(first cmake --version)", "ninja": "$(first ninja --version)", "python": "$(first python3 --version)",
 "dtc": "$(first dtc --version)",
 "zmk_ota_bin_sha256": "$sha", "built": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"}
INFO
  cat "$b/zephyr/build-info.json"
fi
exit $rc
