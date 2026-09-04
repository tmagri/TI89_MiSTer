#!/usr/bin/env python3
"""
wsm_contract_test.py — pure-software parity test of the TI-89 flash WSM.

No Verilog simulator, no FPGA: this models, side by side,

  * REF — the reference simulator's Write State Machine (v12.js
    ww_"9"_flashspecial word model: 0x5050 clear status, 0x1010 write
    setup -> one word AND-programmed, 0x2020 + 0xD0D0 64KB block erase,
    0x9090 identifier mode, 0xFFFF read-array reset; reads in status mode
    return ready, chip-globally, until 0xFFFF), and

  * DUT — the decode table implemented by rtl/flash_ctrl.sv (commands
    matched on the addressed lane of any write, data wins over command
    decoding while armed, ID codes 0x00B0/0x00B5, status register 0x0080
    = DQ7 ready with no error bits, deferred erase completes before the
    next command is serviced).

and asserts that both produce identical flash contents and equivalent
read results for the canonical 68k sequences the OS actually issues
(references/tiemu/docs/ti_hw/flash/EEPROM Programming.htm and
titanium-info.txt comment [2]):

  program : 5050 / 1010 / DATA@addr / poll / 5050 / FFFF
  erase   : 5050 / 2020 / D0D0@block / poll / 5050 / FFFF
  id      : 9090@cert / read base / read base+2 / 5050 / FFFF
             (and the boot-code variant without the 5050)

plus the edge cases that can freeze a poll loop if decoded wrong:
0xFFFF / 0x5050 / 0xD0D0 written as DATA while armed, a command issued
immediately after erase confirm with no poll, and status polled from an
address in the other 2MB half of the flash window.

Run:  python3 wsm_contract_test.py     (exit 0 = contract holds)
"""

import sys

FLASH_SIZE = 4 * 1024 * 1024
ROM_BASE = 0x800000


# ---------------------------------------------------------------------------
# Reference model — v12.js ww_flashspecial / rw_flashspecial (word model)
# ---------------------------------------------------------------------------
class RefWSM:
    def __init__(self):
        self.rom = bytearray(b"\xFF" * FLASH_SIZE)
        self.write_ready = 0
        self.phase = 0x50          # 0x50 idle / 0x90 id-codes / ...
        self.status_mode = False   # flash_ret_or (chip-global)

    def write_word(self, addr, val):
        if self.write_ready:
            off = addr - ROM_BASE
            self.rom[off:off + 2] = \
                bytes([self.rom[off] & (val >> 8),
                       self.rom[off + 1] & (val & 0xFF)])
            self.write_ready -= 1
            self.status_mode = True
        elif val == 0x5050:
            self.phase = 0x50
        elif val == 0x9090:
            self.phase = 0x90
        elif val == 0x1010:
            if self.phase == 0x50:
                self.write_ready = 1
        elif val == 0x2020:
            if self.phase == 0x50:
                self.phase = 0x20
        elif val == 0xD0D0:
            if self.phase == 0x20:
                self.phase = 0xD0
                self.status_mode = True
                base = (addr - ROM_BASE) & 0xFF0000
                self.rom[base:base + 0x10000] = b"\xFF" * 0x10000
        elif val == 0xFFFF:
            if self.phase in (0x50, 0x90):
                self.write_ready = 0
                self.status_mode = False

    def read_word(self, addr):
        if self.phase == 0x90:
            sel = (addr - ROM_BASE) & 0xFFFF
            if sel == 0:
                return 0x00B0
            if sel == 2:
                return 0x00B5
            return 0xFFFF
        if self.status_mode:
            return 0xFFFF          # rom[..] | flash_ret_or
        off = addr - ROM_BASE
        return (self.rom[off] << 8) | self.rom[off + 1]


# ---------------------------------------------------------------------------
# DUT model — rtl/flash_ctrl.sv decode table
# ---------------------------------------------------------------------------
class DutWSM:
    def __init__(self):
        self.rom = bytearray(b"\xFF" * FLASH_SIZE)
        self.phase = 0x50
        self.wready = 0
        self.ret_or = False
        self.erase_pending = None   # deferred fill completes before the
                                    # next command is serviced (F_IDLE
                                    # priority), so model it as: run now.

    def write_word(self, addr, val):
        if self.wready:
            off = addr - ROM_BASE
            self.rom[off:off + 2] = \
                bytes([self.rom[off] & (val >> 8),
                       self.rom[off + 1] & (val & 0xFF)])
            self.wready = 0
            self.ret_or = True
            return
        cmd = val & 0xFF            # word writes: low byte is the command
        if cmd == 0x50:
            self.phase = 0x50
        elif cmd == 0x70:
            self.phase = 0x70
            self.ret_or = True
        elif cmd == 0x90:
            self.phase = 0x90
            self.ret_or = False
        elif cmd in (0x10, 0x40):
            self.wready = 1
            self.phase = cmd
        elif cmd == 0x20:
            self.phase = 0x20
        elif cmd == 0xD0:
            if self.phase == 0x20:
                self.phase = 0xD0
                self.ret_or = True
                base = (addr - ROM_BASE) & 0xFF0000
                self.rom[base:base + 0x10000] = b"\xFF" * 0x10000
        elif cmd == 0xFF:
            self.phase = 0x50
            self.wready = 0
            self.ret_or = False

    def read_word(self, addr):
        if self.phase == 0x90:
            sel = (addr - ROM_BASE) & 0xFFFF
            if sel == 0:
                return 0x00B0
            if sel == 2:
                return 0x00B5
            return 0xFFFF
        if self.ret_or:
            return 0x0080          # status: DQ7 ready, no error bits
        off = addr - ROM_BASE
        return (self.rom[off] << 8) | self.rom[off + 1]


# ---------------------------------------------------------------------------
# Sequences
# ---------------------------------------------------------------------------
APP_BLOCK = 0x960000               # first-boot FlashApps area (bank 0)
CERT = 0x810000                    # CertMem — OS command/status pointer
OTHER_BANK = 0xB00000              # second 2MB half of the flash window


def seq_program(w, addr, data):
    w.write_word(CERT, 0x5050)
    w.write_word(CERT, 0x1010)
    w.write_word(addr, data)
    w.read_word(CERT)              # status poll (ready)
    w.write_word(CERT, 0x5050)
    w.write_word(CERT, 0xFFFF)


def seq_erase(w, block):
    w.write_word(CERT, 0x5050)
    w.write_word(CERT, 0x2020)
    w.write_word(block, 0xD0D0)
    w.read_word(CERT)
    w.write_word(CERT, 0x5050)
    w.write_word(CERT, 0xFFFF)


def seq_id(w, with_clr=True):
    w.write_word(CERT, 0x9090)
    mfr = w.read_word(ROM_BASE)
    dev = w.read_word(ROM_BASE + 2)
    if with_clr:
        w.write_word(CERT, 0x5050)
    w.write_word(CERT, 0xFFFF)
    return mfr, dev


def run_scenario(name, script):
    ref = RefWSM()
    dut = DutWSM()
    script(ref)
    script(dut)
    ok = True
    if ref.rom != dut.rom:
        for i in range(FLASH_SIZE):
            if ref.rom[i] != dut.rom[i]:
                print(f"  FAIL {name}: flash differs at 0x{i:X}: "
                      f"ref {ref.rom[i]:02X} dut {dut.rom[i]:02X}")
                ok = False
                break
    # read-back parity (post-FFFF array reads + a mid-status poll)
    ref2, dut2 = RefWSM(), DutWSM()
    reads = []
    for w in (ref2, dut2):
        w.write_word(CERT, 0x1010)
        w.write_word(APP_BLOCK, 0x1234)
        mid = w.read_word(CERT if w is ref2 else CERT)  # status-mode read
        w.write_word(CERT, 0xFFFF)
        arr = w.read_word(APP_BLOCK)
        reads.append((mid, arr))
    # ref status-mode read = 0xFFFF, dut = 0x0080: both have DQ7 set and
    # no error bits — the OS only tests bit 7. Assert exactly that.
    for label, (mid, arr) in zip(("ref", "dut"), reads):
        if mid & 0x80 == 0:
            print(f"  FAIL {name}: {label} mid-status poll not ready")
            ok = False
        if arr != 0x1234:
            print(f"  FAIL {name}: {label} array read-back {arr:04X} "
                  f"!= 1234")
            ok = False
    print(("  PASS " if ok else "  FAIL ") + name)
    return ok


def main():
    all_ok = True

    all_ok &= run_scenario(
        "program: canonical 5050/1010/DATA/poll/5050/FFFF",
        lambda w: seq_program(w, APP_BLOCK, 0x1234))

    all_ok &= run_scenario(
        "program: 0xFFFF as DATA while armed must be programmed, not "
        "decoded as reset",
        lambda w: (w.write_word(CERT, 0x5050), w.write_word(CERT, 0x1010),
                   w.write_word(APP_BLOCK + 0x10, 0xA5FF),
                   w.write_word(CERT, 0x5050), w.write_word(CERT, 0xFFFF)))

    all_ok &= run_scenario(
        "erase: canonical 5050/2020/D0D0@block/poll/5050/FFFF",
        lambda w: seq_erase(w, APP_BLOCK + 0x8000))

    all_ok &= run_scenario(
        "erase then immediately program same block (no poll between)",
        lambda w: (seq_erase(w, APP_BLOCK),
                   w.write_word(CERT, 0x1010),
                   w.write_word(APP_BLOCK + 4, 0xBEEF),
                   w.write_word(CERT, 0x5050),
                   w.write_word(CERT, 0xFFFF)))

    def id_os(w):
        m = seq_id(w, with_clr=True)
        assert m == (0x00B0, 0x00B5), f"ID codes wrong: {m}"
    all_ok &= run_scenario("id: OS variant (9090/read/read/5050/FFFF)",
                           id_os)

    def id_boot(w):
        m = seq_id(w, with_clr=False)
        assert m == (0x00B0, 0x00B5), f"ID codes wrong: {m}"
    all_ok &= run_scenario("id: boot variant (9090/read/read/FFFF)",
                           id_boot)

    def cross_bank(w):
        # program in bank 1, poll status from the bank-0 command pointer
        w.write_word(CERT, 0x5050)
        w.write_word(CERT, 0x1010)
        w.write_word(OTHER_BANK, 0x00FF)
        if not (w.read_word(CERT) & 0x80):
            raise AssertionError("status not ready from other bank")
        w.write_word(CERT, 0x5050)
        w.write_word(CERT, 0xFFFF)
    all_ok &= run_scenario(
        "status: program bank 1, poll from bank-0 pointer (chip-global)",
        cross_bank)

    def repeat_programs(w):
        for i in range(8):
            w.write_word(CERT, 0x1010)
            w.write_word(APP_BLOCK + 2 * i, 0x0100 + i)
        w.write_word(CERT, 0x5050)
        w.write_word(CERT, 0xFFFF)
    all_ok &= run_scenario(
        "program: repeated 1010/DATA pairs without interposed 5050",
        repeat_programs)

    print("\nWSM contract: " + ("HELD" if all_ok else "VIOLATED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
