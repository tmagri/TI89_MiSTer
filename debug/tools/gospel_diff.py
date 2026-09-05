#!/usr/bin/env python3
"""
gospel_diff.py — bounded byte-level comparison of a gospel state against a
hardware capture (Phase 2/3 workhorse).

Usage:
  gospel_diff.py <gospel.bin> <target.(bin|capture)> [--base HEX] [--len HEX]
                 [--label NAME] [--max N]

  * target starting with "$P" is parsed as a dbg_uart run-8-style capture
    automatically; anything else is treated as raw big-endian bytes.
  * --base/--len slice BOTH inputs at byte granularity (hex, no $).
  * Verdicts: MATCH, or diff classified by signature:
      word_swap  — bytes exchanged inside words (CPU write-lane bug shape)
      shift±1/2  — diff collapses under a 1/2-byte address shift (fetch
                   alignment / A0 misdecode shape)
      scatter    — true content difference

Writes gospel/reports/<label>.md and prints the summary. Exit 0 on MATCH.
"""

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent          # debug/
sys.path.insert(0, str(HERE / "tools"))
from gospel_diff_lib import parse_run8, parse_dump_stream, bounded_diff, signatures  # noqa: E402


def load(path: Path) -> bytes:
    data = path.read_bytes()
    # run-8-style captures carry boot status lines BEFORE the "$P" marker;
    # command-dump captures start with "$D"
    if data[:2] == b"$P" or data.find(b"$P", 0, 8192) != -1:
        return parse_run8(str(path))
    if data[:2] == b"$D" or data.find(b"$D", 0, 64) != -1:
        return parse_dump_stream(data, len(data))   # caller slices by --base/--len
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gospel")
    ap.add_argument("target")
    ap.add_argument("--base", default="0")
    ap.add_argument("--len", default=None)
    ap.add_argument("--label", default=None)
    ap.add_argument("--max", type=int, default=16)
    a = ap.parse_args()

    g = load(Path(a.gospel))
    t = load(Path(a.target))
    base = int(a.base, 16)
    ln = int(a.len, 16) if a.len else max(0, min(len(g), len(t)) - base)
    gS, tS = g[base:base + ln], t[base:base + ln]

    diffs = bounded_diff(gS, tS)
    sig = signatures(diffs, gS, tS)
    label = a.label or f"diff_{Path(a.gospel).stem}_vs_{Path(a.target).stem}"

    lines = []
    add = lines.append
    add(f"# {label} — {datetime.now(timezone.utc).isoformat(timespec='seconds')}")
    add("")
    add(f"- gospel: `{a.gospel}` @0x{base:X} +0x{ln:X}")
    add(f"- target: `{a.target}`")
    add(f"- differing bytes: **{len(diffs)}** of {ln} ({100 * len(diffs) / max(ln, 1):.4f}%)")
    if not diffs:
        add("")
        add("## Verdict: **MATCH**")
    else:
        add("")
        add("## Signature")
        for k in ("word_swap", "shift_pm1", "shift_pm2", "scatter"):
            if sig.get(k):
                add(f"- **{k}**")
        add("")
        add("## First differences")
        for o, gv, tv in diffs[:a.max]:
            add(f"- 0x{base + o:06X}: target {tv:02X} vs gospel {gv:02X}")
        if len(diffs) > a.max:
            add(f"- … {len(diffs) - a.max} more")
        add("")
        add("## Verdict: **DIFFER**")
    report = "\n".join(lines) + "\n"
    out = HERE / "gospel" / "reports" / f"{label}.md"
    out.write_text(report)
    print("\n".join(lines))
    print(f"report: {out}")
    return 0 if not diffs else 1


if __name__ == "__main__":
    sys.exit(main())
