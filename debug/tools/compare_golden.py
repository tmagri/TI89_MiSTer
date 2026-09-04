#!/usr/bin/env python3
"""
compare_golden.py — build the golden 4MB flash image from TI89Titanium_OS.89u
using the reference emulator recipe (n-89 convert.rs / v12.js handle_newromready,
as implemented by rtl/rom_loader.sv) and diff it against a captured read-back.

Golden image layout (flash chip offsets):
  0x00000-0x000FF  mirror of payload bytes 0x88..0x187 (256-byte boot block)
  0x00100          0xFEEDBABE
  0x00104          HWPB pointer 0x00800108
  0x00108-0x00121  HWPB: len=0x18, hardwareID=9, revision=2, boot 1.1.1,
                    gate array=3
  0x00122-0x11FFF  0xFF
  0x12000-...      .89u payload (file offset 0x4E to EOF)
  ...-0x3FFFFF     0xFF

Usage: compare_golden.py <capture.bin> <os.89u> [--max-report N]
Uses the clean framing of the current dbg_uart producer: "$P<n>\r\n" then,
per 4096-word block, a 9-byte "@xxxxxx\r\n" sync line followed by 8192 raw
data bytes (verified against dump_run8.bin: block syncs land exactly every
8201 bytes).
"""

import sys
import argparse
import struct

FLASH_WORDS = 2 * 1024 * 1024  # 4 MB / 2


def parse_capture(path):
    data = open(path, "rb").read()
    passes = {}
    i = 0
    n = len(data)
    while i + 5 <= n:
        # find next "$P<n>\r\n"
        if data[i:i + 1] != b"$" or data[i + 1:i + 2] != b"P":
            i += 1
            continue
        p = data[i + 2] - ord("0")
        if not 0 <= p <= 3 or data[i + 3:i + 5] != b"\r\n":
            i += 1
            continue
        i += 5
        img = bytearray(2 * FLASH_WORDS)
        ok = True
        for blk in range(FLASH_WORDS // 4096):
            # 9-byte sync "@xxxxxx\r\n"
            if data[i:i + 1] != b"@" or data[i + 7:i + 9] != b"\r\n":
                ok = False
                break
            i += 9
            take = min(8192, n - i)
            pos = blk * 8192
            img[pos:pos + take] = data[i:i + take]
            i += take
            if take < 8192:
                break
        passes[p] = img
        if not ok:
            print(f"note: pass {p}: framing lost at stream 0x{i:X}")
    return passes

ROM_SIZE = 4 * 1024 * 1024
SPP = 0x12000          # system privileged part (OS image base, chip offset)
HWID_PTR = 0x104       # HWPB pointer location
HWID_OFF = 0x108       # HWPB location


def build_golden(os_path):
    f = open(os_path, "rb").read()
    # payload starts at file 0x4E ("basecode" marker at 0x11 + 0x3D)
    start = f.find(b"basecode")
    assert start == 0x11, f"'basecode' marker at 0x{start:X}, expected 0x11"
    start += 0x3D
    payload = f[start:]

    img = bytearray(b"\xFF" * ROM_SIZE)

    # boot-block mirror: payload[0x88:0x188]
    img[0x000:0x100] = payload[0x88:0x188]

    # synthesized header
    struct.pack_into(">I", img, 0x100, 0xFEEDBABE)
    struct.pack_into(">I", img, HWID_PTR, 0x00800108)
    # HWPB — big-endian 16-bit words (the 68k reads them with move.w; v12.js
    # writes exactly these words, and the hardware-verified run-8 readback
    # matches. The n-89 convert.rs field structs resolve to the same bytes.)
    hwpb_words = [0x0018,        # len = 24
                  0x0000, 0x0009,  # hardware ID = 9 (TI-89 Titanium)
                  0x0000, 0x0002,  # hardware revision = 2
                  0x0000, 0x0001,  # boot major = 1
                  0x0000, 0x0001,  # boot revision = 1
                  0x0000, 0x0001,  # boot build = 1
                  0x0000, 0x0003]  # gate array = 3 (HW3)
    for i, w in enumerate(hwpb_words):
        struct.pack_into(">H", img, HWID_OFF + 2 * i, w)

    # payload at chip 0x12000
    n = min(len(payload), ROM_SIZE - SPP)
    img[SPP:SPP + n] = payload[:n]
    return img, len(payload)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("osfile")
    ap.add_argument("--max-report", type=int, default=20)
    args = ap.parse_args()

    golden, plen = build_golden(args.osfile)
    print(f"golden built: payload {plen} bytes (0x{plen:X}) "
          f"at chip 0x12000, spans to 0x{SPP + plen:X}")

    passes = parse_capture(args.capture)
    if not passes:
        print("NO PASS CAPTURED — no '$P' marker in stream")
        return 2

    for p in sorted(passes):
        img = passes[p]
        mism = []
        for wa in range(FLASH_WORDS):
            gw = (golden[2 * wa] << 8) | golden[2 * wa + 1]
            dw = (img[2 * wa] << 8) | img[2 * wa + 1]
            if gw != dw:
                mism.append((wa, dw, gw))
        print(f"pass {p}: {len(mism)} mismatched words "
              f"({100.0 * len(mism) / FLASH_WORDS:.6f}%)")
        for wa, dw, gw in mism[:args.max_report]:
            print(f"  word 0x{wa:06X} (chip byte 0x{wa * 2:06X}): "
                  f"got {dw:04X} expected {gw:04X}")
        if len(mism) > args.max_report:
            print(f"  ... {len(mism) - args.max_report} more")
        if not mism:
            print("  BIT-EXACT vs golden synthesis ✓")

    return 0


if __name__ == "__main__":
    sys.exit(main())
