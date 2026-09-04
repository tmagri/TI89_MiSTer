#!/usr/bin/env python3
"""
parse_readback.py — TI-89 MiSTer SDRAM image read-back analysis.

Parses the UART capture produced by the pre-boot dump (dbg_uart dump mode)
and compares every pass against the golden flash image (flash.hex produced
by convert89u.py, one 4-hex-digit word per line, big-endian byte order in
the stream: hi byte then lo byte per word).

Stream format (see rtl/dbg_uart.sv):
  "$P<n>\r\n"                 start of pass n
  "@<6-hex word addr>\r\n"    resync marker every 4096 words
  <hi> <lo>                   two raw bytes per word
  "[TI89] ...\r\n"            periodic status lines (skipped)

Verdict logic:
  * all passes bit-exact        -> CLEAN (load path + read path OK)
  * mismatches identical across passes (same addr, same values)
                                 -> DETERMINISTIC (load/write path)
  * mismatch sets differ        -> FLAKY (read capture / timing)

Usage: parse_readback.py <capture.bin> <flash.hex> [--max-report N]
"""

import sys
import argparse
from collections import Counter

FLASH_WORDS = 2 * 1024 * 1024  # 4 MB / 2


def load_golden(path):
    words = bytearray(2 * FLASH_WORDS)
    with open(path) as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            if i >= FLASH_WORDS:
                raise SystemExit(f"golden hex has more than {FLASH_WORDS} lines")
            w = int(line, 16)
            words[2 * i] = (w >> 8) & 0xFF
            words[2 * i + 1] = w & 0xFF
    return words


def parse_capture(path):
    """Returns {pass_number: bytearray(4MB)} with None where not covered.

    Framing of the CURRENT dbg_uart producer (verified against the
    run-8 capture, 2026-09-01): each pass is "$P<n>\\r\\n" followed by
    exactly 512 blocks of one 9-byte "@xxxxxx\\r\\n" sync line plus
    8192 raw big-endian data bytes (4096 words). Block syncs land at
    exact 8201-byte intervals, so parsing is predictive inside a pass;
    only the pre-pass bytes (boot status lines) are scanned, because
    data bytes can equal '@'/'$'/'['.

    The OLD producer left du_pos dangling between strings (5-byte sync
    remnants, stray LFs) and needed a quirks model; captures from that
    era (run 6) will simply report framing loss here.
    """
    data = open(path, "rb").read()
    passes = {}
    i = 0
    n = len(data)

    # Skip everything until the first "$P<n>\r\n"
    while i + 4 < n:
        if (data[i] == ord("$") and data[i + 1] == ord("P")
                and data[i + 2] in b"0123"
                and data[i + 3] == 0x0D and data[i + 4] == 0x0A):
            break
        i += 1

    while i + 5 <= n and data[i] == ord("$"):
        p = data[i + 2] - ord("0")
        i += 5
        img = bytearray(2 * FLASH_WORDS)
        passes[p] = img
        framing_errors = 0

        for blk in range(FLASH_WORDS // 4096):
            # 9-byte sync "@xxxxxx\r\n"
            if data[i:i + 1] != b"@" or data[i + 7:i + 9] != b"\r\n":
                framing_errors += 1
                break
            i += 9
            pos_word = blk * 4096
            # 4096 words = 8192 bytes
            take = min(8192, n - i)
            need = (FLASH_WORDS - pos_word) * 2
            if take > need:
                take = need
            img[pos_word * 2:pos_word * 2 + take] = data[i:i + take]
            i += take
            if take < 8192:
                break  # capture truncated mid-pass

        if framing_errors:
            print(f"note: pass {p}: framing lost at stream 0x{i:X} "
                  f"(old-producer capture?)")

        # after 512 blocks the next thing is the next "$P" (loop) or the
        # post-dump status lines (loop exits on the $-check)
    return passes


def analyze(passes, golden, max_report):
    results = {}
    for p in sorted(passes):
        img = passes[p]
        mism = []
        bit_hist = Counter()
        for wa in range(FLASH_WORDS):
            gw = (golden[2 * wa] << 8) | golden[2 * wa + 1]
            dw = (img[2 * wa] << 8) | img[2 * wa + 1]
            if gw != dw:
                mism.append((wa, dw, gw))
                d = gw ^ dw
                for bit in range(16):
                    if d & (1 << bit):
                        bit_hist[bit] += 1
        results[p] = (mism, bit_hist)

    # ---- report ----
    print(f"passes captured: {sorted(passes)}")
    for p in sorted(passes):
        mism, bit_hist = results[p]
        n_words = len(mism)
        print(f"\n=== pass {p}: {n_words} mismatched words "
              f"({100.0*n_words/FLASH_WORDS:.6f}%) ===")
        for wa, dw, gw in mism[:max_report]:
            print(f"  word 0x{wa:06X} (byte 0x{wa*2:06X}): "
                  f"got {dw:04X} expected {gw:04X}")
        if len(mism) > max_report:
            print(f"  ... {len(mism)-max_report} more")
        if n_words and bit_hist:
            print("  bit-flip histogram (word bit 15..0):")
            for bit in sorted(bit_hist, reverse=True):
                print(f"    bit {bit:2d} (DQ{bit if bit < 8 else bit-8}"
                      f"{' hi' if bit >= 8 else ' lo'}): {bit_hist[bit]}")

    # ---- verdict ----
    print("\n=== verdict ===")
    if not passes:
        print("NO PASS CAPTURED — no '$P' marker found in the stream")
        return 2
    if all(len(results[p][0]) == 0 for p in results):
        print("CLEAN: every captured pass is bit-exact vs flash.hex")
        print("  -> SDRAM image content AND read path verified on hardware.")
        return 0
    sets = {p: set(wa for wa, _, _ in results[p][0]) for p in results}
    plist = sorted(sets)
    if len(plist) == 1:
        print("SINGLE PASS captured — determinism not testable "
              "(rerun with DUMP_PASSES > 1 to distinguish load vs read faults)")
        return 3
    if all(sets[plist[0]] == sets[p] for p in plist):
        print("DETERMINISTIC: identical mismatch addresses across passes")
        print("  -> load/write path (rom_loader -> SDRAM write) is the prime suspect.")
        return 3
    print("FLAKY: mismatch sets differ between passes")
    print("  -> SDRAM read capture / timing is the prime suspect.")
    return 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("golden")
    ap.add_argument("--max-report", type=int, default=20)
    args = ap.parse_args()
    golden = load_golden(args.golden)
    passes = parse_capture(args.capture)
    sys.exit(analyze(passes, golden, args.max_report))


if __name__ == "__main__":
    main()
