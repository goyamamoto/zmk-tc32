#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The instructions of the code executed from reset until the boot is counted and the power-on chord can take the
keyboard back to the other slot (the boot guard's early stage), against a recorded listing.

  boot_path_check.py check OBJDUMP NM ELF MAP REFERENCE
  boot_path_check.py record OBJDUMP NM ELF MAP FUNCTION... > REFERENCE

FUNCTION is a function's name, or NAME@OBJECT for a name that several compile units define (OBJECT is the
object file's name as the link map shows it, e.g. k_sem_give@flash_tlsr_spi.c.obj).

A function's listing is its Thumb disassembly (llvm-objdump) up to its symbol's size, without the data in it
(the literal pools), each instruction as text but for
- a branch or call target, written as every symbol that holds that address (name+offset), and for a target
  outside every function (a helper the compiler puts after one) its four bytes too, as Thumb halfwords (in an
  ELF built with -mcpu=tc32, the direct path, they are TC32's and are read back through tc32-devtools'
  common/tc32isa.py, found through $TC32_DEVTOOLS);
- a literal-pool load, written as the word it loads and every symbol that holds that word's value or ends
  there, a linker-script value equal to it (name=, such as __bss_end=), the constant without a symbol that
  holds it (a compiler's private constant as object:section+offset, a constant the linker merged as
  const:<its bytes>+offset), and the text it points at when it points at a string (which has no symbol of
  its own).
check compares each recorded function with the image's: the same number of instructions, the same text, a
target with a name in common, and a literal that is the same word or has a name in common (a literal may
move, but must point at the same thing). It prints one line per function, a DIFF line naming every
instruction that differs (and, when the counts differ, both counts; the instructions are compared up to the
shorter listing), and exits 1 when one differs or is missing. The listings are recorded from Thumb ELFs, where a move between low registers is movs rd, rm
(lsls rd, rm, #0), which thumb2tc32.py writes as adds rd, rm, #0; a TC32 ELF holds the adds itself, so
against one such a recorded movs is read as that adds.
"""
import os
import re
import struct
import subprocess
import sys


def thumb_word(word):
    """A word of TC32 code as the two Thumb halfwords with the same meaning (tc32-devtools' tc32isa)."""
    d = os.environ.get("TC32_DEVTOOLS")
    if not d:
        sys.exit("boot_path_check: the ELF names the core tc32; set TC32_DEVTOOLS to read its code as Thumb")
    sys.path.insert(0, os.path.join(d, "common"))
    import tc32isa
    return tc32isa.to_thumb(word & 0xFFFF) | tc32isa.to_thumb(word >> 16) << 16


class Image:
    def __init__(self, objdump, nm, elf, mapf):
        self.objdump, self.elf = objdump, elf
        out = subprocess.run([nm, "-S", "-n", elf], capture_output=True, text=True, check=True).stdout
        self.syms, self.funcs, self.absolute = [], {}, {}
        for line in out.splitlines():
            p = line.split()
            if (len(p) >= 3 and p[-2] == "A" and not p[-1].startswith(("$", "CONFIG_"))
                    and 0x840000 <= int(p[0], 16) < 0x850000):
                # a linker-script address in the SRAM (__bss_end); never a size, an offset or a setting, whose
                # value is the thing compared
                self.absolute.setdefault(int(p[0], 16), set()).add(p[-1])
            if len(p) == 4 and not p[3].startswith("$") and p[2] not in "AaNU":  # A: a value, not an address
                a, sz = int(p[0], 16), int(p[1], 16)
                self.syms.append((a, sz, p[3]))
                if p[2] in "TtWw":
                    self.funcs.setdefault(p[3], []).append((a, sz))
            elif len(p) == 3 and not p[2].startswith("$") and p[1] in "TtDdBbRrWw":
                self.syms.append((int(p[0], 16), 0, p[2]))
        self.sections, self.data = [], []
        self.tc32 = False
        data = open(elf, "rb").read()
        shoff, = struct.unpack_from("<I", data, 0x20)
        shentsize, shnum = struct.unpack_from("<HH", data, 0x2e)
        for i in range(shnum):
            _, kind, flags, addr, off, size = struct.unpack_from("<IIIIII", data, shoff + i * shentsize)
            if kind == 0x70000003:  # SHT_ARM_ATTRIBUTES: Tag_CPU_name (5) "tc32" marks TC32 code
                self.tc32 = b"\x05tc32\x00" in data[off:off + size].lower()
            if kind == 1 and flags & 2:  # PROGBITS, SHF_ALLOC
                self.sections.append((addr, data[off:off + size]))
                if not flags & 4:  # not SHF_EXECINSTR: data
                    self.data.append((addr, data[off:off + size]))
        self.objects, self.consts = [], []
        for line in open(mapf, errors="replace"):
            m = re.match(r"^\s*([0-9a-f]+)\s+[0-9a-f]+\s+([0-9a-f]+)\s+\d+\s+\S*?\(?([^()\s/]+\.obj|<internal>)\)?:\((\S+?)\)", line)
            # the input sections in the image (a debug section's addresses are offsets in that section)
            if m and int(m.group(2), 16) and not m.group(4).startswith((".debug", ".comment", ".ARM")):
                a, n, obj, sec = int(m.group(1), 16), int(m.group(2), 16), m.group(3), m.group(4)
                if obj != "<internal>":
                    self.objects.append((a, n, obj))
                # constants without a symbol: a compiler's private constant (.rodata..L__const.<function>.<name>,
                # in the object that defines it) and the linker's merged constants (.rodata.cst<size>)
                if sec.startswith(".rodata..L") and obj != "<internal>":
                    self.consts.append((a, n, f"{obj}:{sec}"))
                elif obj == "<internal>" and re.fullmatch(r"\.rodata\.cst\d+", sec):
                    self.consts.append((a, n, sec))

    def word(self, a):
        for s, b in self.sections:
            if s <= a and a + 4 <= s + len(b):
                return int.from_bytes(b[a - s:a - s + 4], "little")
        return None

    def string(self, v):
        """The NUL-terminated printable text at v in a data section (a string literal has no symbol of its own), or
        None."""
        for s, b in self.data:
            if s <= v < s + len(b):
                e = b.find(b"\0", v - s, v - s + 256)
                t = b[v - s:e] if e > v - s else b""
                return t if t and all(32 <= c < 127 for c in t) else None
        return None

    def names(self, v):
        """Every symbol that holds v, as name+offset, and every one that ends at v, as name+end (a stack's top);
        else the nearest symbol below v within 4 KB. A value that points at text also gets str:<the text in hex>.
        Besides those: a linker-script value equal to v (name=), and the constant without a symbol that holds v
        (object:section+offset for a compiler's private constant, const:<bytes>+offset for a merged one)."""
        got = {f"{n}+0x{v - a:x}" for a, sz, n in self.syms if (sz and a <= v < a + sz) or (not sz and a == v)}
        got |= {f"{n}+end" for a, sz, n in self.syms if sz and a + sz == v}
        t = self.string(v)
        if t is not None and len(t) >= 2:
            got.add("str:" + t.hex())
        if not got:
            below = [x for x in self.syms if x[0] <= v and v - x[0] < 0x1000]
            if below:
                a, _, n = max(below, key=lambda x: x[0])
                got.add(f"{n}+0x{v - a:x}")
        got |= {f"{n}=" for n in self.absolute.get(v, ())}
        for a, sz, name in self.consts:
            if a <= v < a + sz:
                k = re.fullmatch(r"\.rodata\.cst(\d+)", name)
                if k:  # a merged constant: named by its bytes, which is all that tells it apart
                    k = int(k.group(1))
                    o = a + (v - a) // k * k
                    b = b"".join((self.word(x) or 0).to_bytes(4, "little") for x in range(o, o + k, 4))
                    got.add(f"const:{b[:k].hex()}+0x{v - o:x}")
                else:
                    got.add(f"{name}+0x{v - a:x}")
        return got

    def find(self, func):
        name, _, obj = func.partition("@")
        defs = self.funcs.get(name, [])
        if obj:
            defs = [d for d in defs if any(o == obj and s <= d[0] < s + sz for s, sz, o in self.objects)]
        return defs[0] if len(defs) == 1 else None

    def listing(self, func):
        """The function's instructions: (text with <> for each target, [target names], (pool word, names))."""
        d = self.find(func)
        if d is None:
            return None
        a, sz = d
        out = subprocess.run([self.objdump, "-d", "--no-show-raw-insn", f"--start-address=0x{a:x}",
                              f"--stop-address=0x{a + sz:x}", self.elf], capture_output=True, text=True).stdout
        rows = []
        for line in out.splitlines():
            m = re.match(r"^\s*([0-9a-f]+):\s+(.*)$", line)
            if not m or int(m.group(1), 16) >= a + sz:
                continue
            text = m.group(2).strip()
            if re.match(r"(?:[0-9a-f]{2}\s+)*\.(?:word|short|byte)\b", text):
                continue
            pool = None
            m = re.search(r"\[pc, #(-?0x[0-9a-f]+|-?\d+)\]\s+@\s+0x([0-9a-f]+)", text)
            if m:
                w = self.word(int(m.group(2), 16))
                pool = (w, self.names(w) if w is not None else set())
                text = text[:m.start()] + "[pc, POOL]"
            text = re.sub(r"@ imm = #-?0x[0-9a-f]+", "", text)
            targets = []

            def target(mm):
                v = int(mm.group(1), 16)
                # a target outside every function (a call-through-register helper the compiler puts after
                # one): its four bytes too
                inside = any(sz and s <= v < s + sz for s, sz, _ in self.syms)
                w = 0 if inside else self.word(v & ~1) or 0
                extra = "" if inside else "=" + "%08x" % (thumb_word(w) if self.tc32 else w)
                targets.append({n + extra for n in self.names(v)})
                return "<>"
            text = re.sub(r"\b0x([0-9a-f]+) <[^>]+>", target, text)
            rows.append((re.sub(r"\s+", " ", text).strip(), targets, pool))
        return rows


def fmt(row):
    text, targets, pool = row
    parts = text.split("<>")
    s = parts[0] + "".join("<" + "|".join(sorted(t)) + ">" + p for t, p in zip(targets, parts[1:]))
    if pool is not None:
        w, names = pool
        s += f" ={'?' if w is None else f'0x{w:x}'}" + (" " + "|".join(sorted(names)) if names else "")
    return s


def parse(line):
    pool = None
    m = re.search(r" =(0x[0-9a-f]+|\?)(?: (\S+))?$", line)
    if m and "[pc, POOL]" in line:
        pool = (None if m.group(1) == "?" else int(m.group(1), 16), set(m.group(2).split("|")) if m.group(2) else set())
        line = line[:m.start()]
    targets = [set(t.split("|")) if t else set() for t in re.findall(r"<([^>]*)>", line)]
    return re.sub(r"<[^>]*>", "<>", line), targets, pool


def same(ref, got):
    (ta, ga, pa), (tb, gb, pb) = ref, got
    if ta != tb or len(ga) != len(gb) or not all(x & y for x, y in zip(ga, gb)):
        return False
    if (pa is None) != (pb is None):
        return False
    return pa is None or pa[0] == pb[0] or bool(pa[1] & pb[1])


def record(img, funcs):
    print("# boot_path_check.py's listing of the functions executed from reset until the boot is counted and the")
    print("# power-on chord can take the keyboard back to the other slot (the boot guard's early stage;")
    print("# tc32/boot_path_units.txt in zmk-tc32): instructions, branch targets and literals by symbol.")
    print("# Written by: boot_path_check.py record OBJDUMP NM zmk.elf zmk.map FUNCTION...")
    bad = 0
    for f in funcs:
        rows = img.listing(f)
        if rows is None:
            print(f"{f}: not found, or several definitions", file=sys.stderr)
            bad += 1
            continue
        print(f"== {f} {len(rows)}")
        for r in rows:
            print(fmt(r))
    return 1 if bad else 0


def check(img, ref):
    funcs, cur = {}, None
    for line in open(ref):
        line = line.rstrip("\n")
        if line.startswith("#") or not line:
            continue
        if line.startswith("== "):
            cur = line.split()[1]
            funcs[cur] = []
        else:
            row = parse(line)
            if img.tc32:
                row = (re.sub(r"^movs (r[0-7]), (r[0-7])$", r"adds \1, \2, #0x0", row[0]),) + row[1:]
            funcs[cur].append(row)
    n_same = n_diff = n_missing = 0
    for f, want in funcs.items():
        rows = img.listing(f)
        if rows is None:
            print(f"missing {f}: not found, or several definitions")
            n_missing += 1
            continue
        got = [parse(fmt(r)) for r in rows]
        diffs = []
        if len(got) != len(want):
            diffs.append(f"{len(want)} instructions recorded, {len(got)} built")
        for i, (a, b) in enumerate(zip(want, got)):
            if not same(a, b):
                diffs.append(f"at {i}: recorded {fmt(a)!r}, built {fmt(b)!r}")
        if diffs:
            print(f"DIFF {f}: {len(diffs)} difference(s): " + "; ".join(diffs))
            n_diff += 1
        else:
            print(f"same {f} ({len(got)} instructions)")
            n_same += 1
    print(f"boot path: {n_same} functions the same instructions as recorded, {n_diff} differ, {n_missing} missing")
    return 1 if n_diff or n_missing else 0


def main():
    mode, objdump, nm, elf, mapf = sys.argv[1:6]
    img = Image(objdump, nm, elf, mapf)
    if mode == "record":
        sys.exit(record(img, sys.argv[6:]))
    if mode == "check":
        sys.exit(check(img, sys.argv[6]))
    sys.exit(__doc__)


if __name__ == "__main__":
    main()
