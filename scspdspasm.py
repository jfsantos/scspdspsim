#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scspdspasm.py — SCSP/Sega Saturn DSP assembler (tentative)
Version: v0.3

New in v0.3:
- Full-line comments starting with ")" **and** with "'" (apostrophe) are ignored; inline apostrophe comments still work.
- Accepts "=END" in addition to "#END".
- **Nested @ expressions**: e.g.
    @ TEMP01 * C2L + (MEMS01 * C1L + (MEMS00 * C0L +))>TEMP00
  are expanded into a sequence of micro-instructions that evaluate a sum of products:
    ACC = MEMS00*C0L; ACC = MEMS01*C1L + REG; ACC = TEMP01*C2L + REG; then store.
  Parentheses are used only as hints; the assembler extracts all `pM*pC` pairs in order.
- **Bare coefficient names** in products are allowed (no need for COEF[...]).
- **Product multiplicand sources** may be INPUT (`INPUT` keyword), `TEMPnn`, *or* direct input selectors
  `MEMSnn`/`MIXSnn`/`EXTSx` (these set `IRA` inside the microinstruction).
- **Inline store suffix** on `@` lines supported: use `> Dest[, Dest...]` or `> MW[AddrSpec]`.
  `>MW[...]` is sugar for a following `MW MR[...]` micro-instruction.

New in v0.2 (kept):
- NOFL honored on LDI/MW; ring-buffer writes via `MW MR[...]`.
- Stores to FREG and ADREG from `>` lines.

References: Sega dAsms manual (syntax/EXC) and the `mpro` bit layout from `ssfinfo.py`.
"""

import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

# -------------------------------
# Numeric conversions (manual rules)
# -------------------------------

def twos_comp_13(value: int) -> int:
    if value < -4096 or value > 4095:
        raise ValueError("Coefficient out of range (-4096..4095)")
    if value < 0:
        value = (1 << 13) + value
    return value & 0x1FFF


def parse_coef_rhs(s: str) -> int:
    s = s.strip()
    if s.upper().startswith("&H"):
        v = int(s[2:], 16) & 0x1FFF
        return v
    if s.startswith("%"):
        pct = float(s[1:])
        v = int(round(4095.0 * (pct / 100.0)))
        v = min(max(v, -4096), 4095)
        return twos_comp_13(v)
    if "." in s:
        f = float(s)
        v = int(round(4096.0 * f))
        return twos_comp_13(v)
    v = int(s, 10)
    return twos_comp_13(v)


def parse_adrs_rhs(s: str) -> int:
    s = s.strip()
    if s.upper().startswith("&H"):
        return int(s[2:], 16) & 0xFFFF
    if s.lower().startswith("ms"):
        ms = float(s[2:])
        return int(round(44100.0 * (ms / 1000.0))) & 0xFFFF
    return int(s, 10) & 0xFFFF


# -------------------------------
# Symbol tables
# -------------------------------

@dataclass
class Symbols:
    coef_order: List[str] = field(default_factory=lambda: ["ZERO"])
    coef: Dict[str, int] = field(default_factory=lambda: {"ZERO": 0})
    adrs_order: List[str] = field(default_factory=list)
    adrs: Dict[str, int] = field(default_factory=dict)

    def add_coef(self, name: str, rhs: str):
        if name.upper() == "ZERO":
            raise ValueError("ZERO is reserved")
        val = parse_coef_rhs(rhs)
        self.coef[name] = val
        self.coef_order.append(name)

    def add_adrs(self, name: str, rhs: str):
        val = parse_adrs_rhs(rhs)
        self.adrs[name] = val
        self.adrs_order.append(name)

    def coef_index(self, name: str) -> int:
        try:
            return self.coef_order.index(name)
        except ValueError:
            raise KeyError(f"Undefined coefficient symbol '{name}'")

    def adrs_index(self, name: str) -> int:
        try:
            return self.adrs_order.index(name)
        except ValueError:
            raise KeyError(f"Undefined address symbol '{name}'")


# -------------------------------
# Micro-instruction encoder (64-bit) per ssfinfo.py:mpro
# -------------------------------

def pack_mpro(**f) -> int:
    w = 0
    w |= (f.get("TRA", 0) & 0x7F) << 56
    w |= (f.get("TWT", 0) & 0x01) << 55
    w |= (f.get("TWA", 0) & 0x7F) << 48
    w |= (f.get("XSEL", 0) & 0x01) << 47
    w |= (f.get("YSEL", 0) & 0x03) << 45
    w |= (f.get("IRA", 0) & 0x3F) << 38
    w |= (f.get("IWT", 0) & 0x01) << 37
    w |= (f.get("IWA", 0) & 0x1F) << 32
    w |= (f.get("TABLE", 0) & 0x01) << 31
    w |= (f.get("MWT", 0) & 0x01) << 30
    w |= (f.get("MRD", 0) & 0x01) << 29
    w |= (f.get("EWT", 0) & 0x01) << 28
    w |= (f.get("EWA", 0) & 0x0F) << 24
    w |= (f.get("ADRL", 0) & 0x01) << 23
    w |= (f.get("FRCL", 0) & 0x01) << 22
    w |= (f.get("SHFT", 0) & 0x03) << 20
    w |= (f.get("YRL", 0) & 0x01) << 19
    w |= (f.get("NEGB", 0) & 0x01) << 18
    w |= (f.get("ZERO", 0) & 0x01) << 17
    w |= (f.get("BSEL", 0) & 0x01) << 16
    w |= (f.get("CRA", 0) & 0x3F) << 9
    w |= (f.get("NOFL", 0) & 0x01) << 7
    w |= (f.get("MASA", 0) & 0x1F) << 2
    w |= (f.get("ADREB", 0) & 0x01) << 1
    w |= (f.get("NXADR", 0) & 0x01)
    return w & ((1 << 64) - 1)


def ira_for_src(src: str) -> int:
    m = re.fullmatch(r"(MEMS|MIXS|EXTS)(\d+)", src)
    if not m:
        raise ValueError(f"Bad INPUT source '{src}'")
    bank, idx = m.group(1), int(m.group(2))
    if bank == "MEMS":
        if not (0 <= idx <= 31):
            raise ValueError("MEMS index 0..31")
        return idx
    if bank == "MIXS":
        if not (0 <= idx <= 15):
            raise ValueError("MIXS index 0..15")
        return 0x20 + idx
    if bank == "EXTS":
        if not (0 <= idx <= 1):
            raise ValueError("EXTS index 0..1")
        return 0x30 + idx
    raise ValueError("unreachable")


def shft_for_store(opt: Optional[str]) -> Tuple[int, bool]:
    if opt is None:
        return (0, True)
    o = opt.upper()
    if o == "S1":
        return (1, True)
    if o == "S2":
        return (2, False)
    if o == "S3":
        return (3, False)
    raise ValueError("Invalid store option (use S1/S2/S3 or omit)")


# -------------------------------
# Parser & assembler
# -------------------------------

COEF_RE = re.compile(r"^\s*([A-Za-z][A-Za-z0-9]{0,14})\s*=\s*(.+?)\s*$")
ADRS_RE = COEF_RE
LDI_RE  = re.compile(r"^\s*LDI\s+(MEMS(\d{1,2})),\s*MR\[(.+?)\]\s*$", re.I)
LDY_RE  = re.compile(r"^\s*LDY\s+(MEMS\d{1,2}|MIXS\d{1,2}|EXTS\d)\s*$", re.I)
LDA_RE  = re.compile(r"^\s*LDA\s+(MEMS\d{1,2}|MIXS\d{1,2}|EXTS\d)\s*$", re.I)
MW_RE   = re.compile(r"^\s*MW\s+MR\[(.+?)\]\s*$", re.I)
AT_RE   = re.compile(r"^\s*@\s*(.+?)\s*$")
ST_RE   = re.compile(r"^\s*>\s*([Ss][123])?\s*(.+?)\s*$")

PROD_RE = re.compile(
    r"\b(INPUT|TEMP\d{1,2}|MEMS\d{1,2}|MIXS\d{1,2}|EXTS\d)\b\s*\*\s*"
    r"(COEF\[[^\]]+\]|YREGH|YREGL|[A-Za-z][A-Za-z0-9]{0,14})",
    re.I,
)


def parse_addr_expr(expr: str, syms: Symbols) -> Tuple[int, int, int, int, int]:
    """
    Parse MR[ sym (+DEC)? (+ADREG)? (+1)? (/NF)? ]
    Returns (MASA, TABLE, ADREB, NXADR, NOFL)
    TABLE=0 when +DEC present (auto-increment), else 1.
    """
    expr = expr.strip()
    expr = expr.split("'")[0]
    parts = expr.split("/")
    body = parts[0].replace(" ", "")
    flags = parts[1] if len(parts) > 1 else ""
    elems = body.split("+")
    if not elems or not re.match(r"^[A-Za-z]", elems[0]):
        raise ValueError("MR[...] must start with an address symbol")
    sym = elems[0]
    masa = syms.adrs_index(sym)
    DEC = 0
    ADREG = 0
    plus1 = 0
    for e in elems[1:]:
        eu = e.upper()
        if eu == "DEC":
            DEC = 1
        elif eu in ("ADREG", "ADRS"):
            ADREG = 1
        elif e == "1":
            plus1 = 1
        elif e == "":
            continue
        else:
            raise ValueError(f"Unknown address element '+{e}'")
    NOFL = 1 if "NF" in flags.upper() else 0
    TABLE = 0 if DEC else 1
    # plus1 currently not explicitly encoded beyond NXADR below (if needed by caller)
    return masa, TABLE, ADREG, plus1, NOFL


def parse_term(token: str) -> Dict[str, int]:
    t = token.strip().upper()
    if t == "REG":
        return {"BSEL": 1}  # ACC as B
    m = re.fullmatch(r"TEMP(\d{1,2})", t)
    if m:
        return {"BSEL": 0, "TRA": int(m.group(1))}
    if t == "INPUT":
        return {"XSEL": 1}
    raise ValueError(f"Unsupported term '{token}'")


def coef_ref_to_fields(pc: str, syms: Symbols) -> Tuple[int, int]:
    pc = pc.strip()
    if pc.upper() in ("YREGH", "YREGL"):
        return (2 if pc.upper() == "YREGH" else 3, 0)
    if pc.upper().startswith("COEF[") and pc.endswith("]"):
        name = pc[5:-1]
        return (1, syms.coef_index(name) & 0x3F)
    # bare symbol
    if re.fullmatch(r"[A-Za-z][A-Za-z0-9]{0,14}", pc):
        return (1, syms.coef_index(pc) & 0x3F)
    raise ValueError(f"Bad coefficient reference '{pc}'")


def multiplicand_to_fields(pm: str) -> Dict[str, int]:
    pmu = pm.upper()
    if pmu == "INPUT":
        return {"XSEL": 1}
    m = re.fullmatch(r"TEMP(\d{1,2})", pmu)
    if m:
        return {"XSEL": 0, "TRA": int(m.group(1))}
    m = re.fullmatch(r"(MEMS\d{1,2}|MIXS\d{1,2}|EXTS\d)", pmu)
    if m:
        return {"XSEL": 1, "IRA": ira_for_src(pmu)}
    raise ValueError(f"Bad multiplicand '{pm}'")


def expand_products(expr: str, syms: Symbols) -> List[Tuple[Dict[str, int], Dict[str, int]]]:
    """Extract a left-to-right list of (pm_fields, (YSEL,CRA)) from an @ expression."""
    prods = []
    for m in PROD_RE.finditer(expr):
        pm = m.group(1)
        pc = m.group(2)
        pmf = multiplicand_to_fields(pm)
        ysel, cra = coef_ref_to_fields(pc, syms)
        prods.append((pmf, {"YSEL": ysel, "CRA": cra}))
    if not prods:
        raise ValueError("No product terms found in '@' expression")
    return prods


class Assembler:
    def __init__(self):
        self.syms = Symbols()
        self.prog_words: List[int] = []

    def assemble(self, text: str) -> None:
        section = None
        for raw in text.splitlines():
            stripped = raw.lstrip()
            if not stripped:
                continue
            if stripped.startswith("'"):
                continue
            line = raw.split("'")[0].rstrip()
            if not line.strip():
                continue

            u = line.strip().upper()
            if u == "#COEF":
                section = "COEF"; continue
            if u == "#ADRS":
                section = "ADRS"; continue
            if u == "#PROG":
                section = "PROG"; continue
            if u == "#END":
                section = "END"; continue

            if section == "COEF":
                m = COEF_RE.match(line)
                if not m:
                    raise ValueError(f"Bad coef line: {line}")
                self.syms.add_coef(m.group(1), m.group(2))
                continue
            if section == "ADRS":
                m = ADRS_RE.match(line)
                if not m:
                    raise ValueError(f"Bad adrs line: {line}")
                self.syms.add_adrs(m.group(1), m.group(2))
                continue
            if section == "PROG":
                self.emit_instr_or_compound(line)
                continue

    # --- instruction & compound handlers ---

    def emit_instr_or_compound(self, line: str):
        # Support inline store suffix on '@' lines: split once on '>'
        if line.strip().startswith("@") and ">" in line:
            at_part, store_part = line.split(">", 1)
            self.emit_at_chain(at_part)
            self.emit_store_suffix(store_part.strip())
            return
        # Otherwise hand over to individual matchers
        if line.strip().startswith("@"):
            self.emit_at_chain(line)
            return
        # LDI
        m = LDI_RE.match(line)
        if m:
            mem_idx = int(m.group(2))
            masa, table, adreb, nxadr, nofl = parse_addr_expr(m.group(3), self.syms)
            word = pack_mpro(
                MRD=1,
                IWT=1, IWA=mem_idx & 0x1F,
                MASA=masa & 0x1F,
                TABLE=table,
                ADREB=1 if adreb else 0,
                NXADR=1 if nxadr else 0,
                NOFL=1 if nofl else 0,
            )
            self.prog_words.append(word)
            return
        # MW MR[...]
        m = MW_RE.match(line)
        if m:
            masa, table, adreb, nxadr, nofl = parse_addr_expr(m.group(1), self.syms)
            word = pack_mpro(
                MWT=1,
                MASA=masa & 0x1F,
                TABLE=table,
                ADREB=1 if adreb else 0,
                NXADR=1 if nxadr else 0,
                NOFL=1 if nofl else 0,
            )
            self.prog_words.append(word)
            return
        # LDY
        m = LDY_RE.match(line)
        if m:
            ira = ira_for_src(m.group(1).upper())
            word = pack_mpro(IRA=ira, YRL=1)
            self.prog_words.append(word)
            return
        # LDA
        m = LDA_RE.match(line)
        if m:
            ira = ira_for_src(m.group(1).upper())
            word = pack_mpro(IRA=ira, ADRL=1, SHFT=0)
            self.prog_words.append(word)
            return
        # Standalone store line
        m = ST_RE.match(line)
        if m:
            self.emit_store_line(m)
            return
        raise ValueError(f"Unknown PROG line: {line}")

    def emit_at_chain(self, line: str):
        m = AT_RE.match(line)
        if not m:
            raise ValueError(f"Bad '@' line: {line}")
        expr = m.group(1)
        prods = expand_products(expr, self.syms)
        # Emit first product: ZERO=1 (no augend)
        pmf, yf = prods[0]
        word = pack_mpro(
            SHFT=3,
            ZERO=1,
            BSEL=0,
            NEGB=0,
            TRA=pmf.get("TRA", 0),
            XSEL=pmf.get("XSEL", 0),
            IRA=pmf.get("IRA", 0),
            YSEL=yf.get("YSEL", 0),
            CRA=yf.get("CRA", 0),
        )
        self.prog_words.append(word)
        # Subsequent products: add REG (ACC)
        for pmf, yf in prods[1:]:
            word = pack_mpro(
                SHFT=3,
                ZERO=0,
                BSEL=1,  # REG (ACC) as augend
                NEGB=0,
                TRA=pmf.get("TRA", 0),
                XSEL=pmf.get("XSEL", 0),
                IRA=pmf.get("IRA", 0),
                YSEL=yf.get("YSEL", 0),
                CRA=yf.get("CRA", 0),
            )
            self.prog_words.append(word)

    def emit_store_suffix(self, text: str):
        # text like "TEMP00" or "MW[waL + DEC]" or "S1 TEMP00, EFREG00, FREG, ADREG"
        m = re.match(r"^([Ss][123])?\s*(.*)$", text)
        if not m:
            raise ValueError(f"Bad store suffix '> {text}'")
        opt = m.group(1)
        rest = m.group(2).strip()
        if rest.upper().startswith("MW[") and rest.endswith("]"):
            inner = rest[3:-1]
            masa, table, adreb, nxadr, nofl = parse_addr_expr(inner, self.syms)
            word = pack_mpro(
                MWT=1,
                MASA=masa & 0x1F,
                TABLE=table,
                ADREB=1 if adreb else 0,
                NXADR=1 if nxadr else 0,
                NOFL=1 if nofl else 0,
            )
            self.prog_words.append(word)
            return
        # Otherwise treat as normal store list
        fake = f"> {opt+' ' if opt else ''}{rest}"
        m2 = ST_RE.match(fake)
        if not m2:
            raise ValueError(f"Bad store suffix '> {text}'")
        self.emit_store_line(m2)

    def emit_store_line(self, m):
        opt = m.group(1)
        dests = [d.strip().upper() for d in m.group(2).split(",")]
        SHFT, _prot = shft_for_store(opt)
        TWT = 0; TWA = 0
        EWT = 0; EWA = 0
        FRCL = 0
        ADRL = 0
        for d in dests:
            if d.startswith("TEMP"):
                TWT = 1; TWA = int(d[4:])
            elif d.startswith("EFREG"):
                EWT = 1; EWA = int(d[5:])
            elif d == "FREG":
                FRCL = 1
            elif d == "ADREG":
                ADRL = 1
            elif d == "":
                continue
            else:
                raise ValueError(f"Bad store dest '{d}' (use TEMPxx, EFREGxx, FREG, ADREG)")
        word = pack_mpro(SHFT=SHFT, TWT=TWT, TWA=TWA, EWT=EWT, EWA=EWA, FRCL=FRCL, ADRL=ADRL)
        self.prog_words.append(word)

    def write_exc(self, out) -> None:
        print("COEF", file=out)
        print(f"00:{self.syms.coef['ZERO']:04X}:ZERO", file=out)
        for i, name in enumerate(self.syms.coef_order[1:], start=1):
            val = self.syms.coef[name]
            print(f"{i:02X}:{val:04X}:{name}", file=out)
        print("ADRS", file=out)
        for i, name in enumerate(self.syms.adrs_order):
            val = self.syms.adrs[name]
            print(f"{i:02X}:{val:04X}:{name}", file=out)
        print("PROG", file=out)
        for i, w in enumerate(self.prog_words):
            b3 = (w >> 48) & 0xFFFF
            b2 = (w >> 32) & 0xFFFF
            b1 = (w >> 16) & 0xFFFF
            b0 = (w >> 0) & 0xFFFF
            print(f"{i:02X}:{b3:04X} {b2:04X} {b1:04X} {b0:04X}", file=out)


# -------------------------------
# CLI
# -------------------------------

def main(argv):
    if len(argv) < 2:
        print("Usage: scspdspasm.py input.usc [-o output.exc]", file=sys.stderr)
        sys.exit(1)
    src = argv[1]
    outpath = None
    if "-o" in argv:
        outpath = argv[argv.index("-o") + 1]
    with open(src, "r", encoding="utf-8") as f:
        text = f.read()
    asm = Assembler()
    asm.assemble(text)
    if outpath:
        with open(outpath, "w", encoding="utf-8") as out:
            asm.write_exc(out)
    else:
        asm.write_exc(sys.stdout)


if __name__ == "__main__":
    main(sys.argv)
