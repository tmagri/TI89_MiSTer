#!/usr/bin/env python3
"""Convert a TI-89 Titanium .89u upgrade file into the 4MB flash image
the core's SDRAM must contain after a load. Mirrors references/n-89
(n-89/src/convert.rs) exactly. Simulation-only; never committed."""
import sys

SPP = 0x12000
BOOT_OFFSET = 0x88

src, dst = sys.argv[1], sys.argv[2]
data = open(src, "rb").read()

assert data[0:8] == b"**TIFL**", "missing **TIFL** signature"
dev_type, data_type = data[0x30], data[0x31]
data_size = int.from_bytes(data[0x4A:0x4E], "little")
print(f"device_type=0x{dev_type:02x} data_type=0x{data_type:02x} "
      f"data_size=0x{data_size:x} file_size=0x{len(data):x}")

payload = data[0x4E:0x4E + data_size]
assert len(payload) == data_size, "file truncated"

rom_base = payload[BOOT_OFFSET + 5] & 0xF0
print(f"rom_base=0x{rom_base:02x} ({'TI-89 Titanium' if rom_base == 0x80 else 'other'})")

img = bytearray(b"\xff") * (4 * 1024 * 1024)

# Boot block mirror: flash[0x000..0x0FF] = payload[0x88..0x187]
img[0x000:0x100] = payload[BOOT_OFFSET:BOOT_OFFSET + 256]
# 0xFEEDBABE
img[0x100:0x104] = (0xFEEDBABE).to_bytes(4, "big")
# rom_base pointer byte pair
img[0x104] = 0x00
img[0x105] = rom_base
# HWPB pointer 0x00800108
img[0x106:0x108] = (0x0108).to_bytes(2, "big")
# HwParamBlock (big endian): len, id, rev, boot_major, boot_rev, build, ga
hwpb = (24).to_bytes(2, "big") + (9).to_bytes(4, "big") + \
       (2).to_bytes(4, "big") + (1).to_bytes(4, "big") + \
       (1).to_bytes(4, "big") + (1).to_bytes(4, "big") + \
       (3).to_bytes(4, "big")
img[0x108:0x122] = hwpb
# The OS payload itself
img[SPP:SPP + data_size] = payload

# Diagnostics: the boot vectors the CPU will fetch after the boot copy
ssp = int.from_bytes(payload[0x88:0x8C], "big")
pc = int.from_bytes(payload[0x8C:0x90], "big")
marker = payload[0x84:0x88]
print(f"CC marker={marker.hex()} initial SSP=${ssp:06x} initial PC=${pc:06x}")

with open(dst, "w") as f:
    for i in range(0, len(img), 2):
        f.write(f"{img[i]:02x}{img[i+1]:02x}\n")
print(f"wrote {dst} ({len(img)//2} words)")
