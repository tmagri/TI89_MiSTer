#!/usr/bin/env python3
"""
gospel_diff_lib.py — shared helpers for gospel-vs-hardware comparisons.

parse_run8(path)            -> 4 MB bytes (big-endian word stream from the
                               dbg_uart pre-boot dump framing)
bounded_diff(a, b, base=0)  -> list of (offset, got, want) over min len
signatures(diffs)           -> detect byte-swap / 1-2 byte address-shift
                               structure in a diff list (Phase 2 garble
                               triage: a garble that collapses under swap
                               or shift is diagnosed mechanically)
"""

FLASH_WORDS = 2 * 1024 * 1024  # 4 MB / 2


def parse_run8(path):
    """Parse a run-8-style capture: "$P<n>\\r\\n", then per 4096-word block a
    9-byte "@xxxxxx\\r\\n" sync + 8192 raw big-endian bytes (the CURRENT
    dbg_uart producer framing, verified 2026-09-01)."""
    data = open(path, "rb").read()
    n = len(data)
    i = 0
    while i + 5 <= n:
        if (data[i:i + 1] == b"$" and data[i + 1:i + 2] == b"P"
                and data[i + 2:i + 3] in b"0123"
                and data[i + 3:i + 5] == b"\r\n"):
            break
        i += 1
    else:
        raise ValueError(f"no $P pass marker in {path}")
    i += 5
    img = bytearray(2 * FLASH_WORDS)
    for blk in range(FLASH_WORDS // 4096):
        if data[i:i + 1] != b"@" or data[i + 7:i + 9] != b"\r\n":
            raise ValueError(f"framing lost at stream 0x{i:X}")
        i += 9
        take = min(8192, n - i)
        pos = blk * 8192
        img[pos:pos + take] = data[i:i + take]
        i += take
        if take < 8192:
            break
    return bytes(img)


def parse_dump_stream(data, expect_bytes=None):
    """Parse a "$D\\r\\n" command-dump response: the 4-byte marker line, then
    blocks of one 9-byte "@xxxxxx\\r\\n" sync + up to 8192 raw bytes (same
    block framing as the $P stream, block-aligned to the command stream).
    Structural walk: each block is 8192 bytes unless it is the last (capture
    ends at the payload end). Leading junk (interleaved status characters)
    is skipped by the find()."""
    i = data.find(b"$D")
    if i < 0:
        raise ValueError("no $D marker in stream")
    i += 4                                    # "$D\r\n"
    out = bytearray()
    first = True
    while True:
        if not first or data[i:i + 1] == b"@":
            if data[i:i + 1] != b"@" or data[i + 7:i + 9] != b"\r\n":
                break                          # end of framed blocks
            i += 9
        first = False
        rem = len(data) - i
        take = min(8192, rem)
        if take <= 0:
            break
        out += data[i:i + take]
        i += take
        if take < 8192:
            break
    return bytes(out)


def bounded_diff(a, b, base=0):
    n = min(len(a), len(b))
    return [(base + i, a[i], b[i]) for i in range(n) if a[i] != b[i]]


def signatures(diffs, a=None, b=None):
    """Classify a byte-diff list. Returns dict with detected structure:
       word_swap   — every diff pair (even,odd) matches with bytes exchanged
       shift_pm1/2 — diff collapses when comparing a[i±k] == b[i]
       scatter     — neither (true content difference)
    """
    offs = {o for o, _, _ in diffs}
    out = {"count": len(diffs), "word_swap": False,
           "shift_pm1": False, "shift_pm2": False, "scatter": False}
    if a is not None and b is not None and diffs:
        pairs = [(o, o + 1) for o, _, _ in diffs if o % 2 == 0 and (o + 1) in offs]
        if pairs and all(
                a[o] == b[o + 1] and a[o + 1] == b[o] for o, _ in pairs):
            out["word_swap"] = True
        def shifted(k):
            n = min(len(a), len(b)) - abs(k)
            bad = sum(1 for o, _, _ in diffs
                      if 0 <= o + k < n and a[o] != b[o + k])
            extra = sum(1 for o, _, _ in diffs
                        if not (0 <= o + k < n and a[o] == b[o + k]))
            return extra == 0
        out["shift_pm1"] = shifted(1) or shifted(-1)
        out["shift_pm2"] = shifted(2) or shifted(-2)
        out["scatter"] = not (out["word_swap"] or out["shift_pm1"]
                              or out["shift_pm2"])
    return out
