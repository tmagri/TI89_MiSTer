# TI-89 Titanium Flash WSM — the software contract

Pure software-level reference for what AMS 3.10 requires of the flash
Write State Machine (Sharp LH28F320BF, Intel-style value-only CUI — **no
unlock address pair**; any write decoding to the flash window is a command,
matched on data, at any address). Derived exclusively from the reference
emulators and their documentation:

- `references/ti89-simulator/js/v12.js` — `ww_*_flashspecial` /
  `rw_*_flashspecial` word model (the 89T-corrected emulator)
- `references/tiemu/src/core/ti_hw/flash.c` — `FlashWriteByte` WSM
  (`wsm.ret_or`, ID codes, erase/program semantics)
- `references/tiemu/docs/ti_hw/flash/EEPROM Programming.htm` — canonical
  68k sequences incl. the DQ7 poll loop ("ROM cannot be read while write
  operations are being performed... writing code must execute from RAM;
  Trap 11 handles this")
- `references/tiemu/docs/mails/Olivier Armand/titanium-info.txt` [2] —
  the exact ID-code sequence the OS issues (`#$9090` to CertMem,
  reads `ROM_BASE`/`+2`, `#$5050`, `#$FFFF`; expects manufacturer `0x89`
  or device `0xB5`; Titanium's manufacturer is `0xB0`)

## Command table (word writes; the OS writes words only)

| Word    | Meaning                    | State effect |
|---------|----------------------------|--------------|
| `0x5050`| Clear status register      | `phase = 0x50` from **any** state; does **not** exit status mode |
| `0x1010`| Write setup                | arms exactly one following word as data |
| *data*  | Program word @ addr        | **AND**-programmed (`rom &= data`; bits only 1→0); data wins over command decoding while armed — including `0xFFFF`/`0x5050`/`0xD0D0` values; enters status mode |
| `0x2020`| Erase setup                | only from `phase 0x50` |
| `0xD0D0`| Erase confirm @ block addr | 64 KB block at `addr & 0xFF0000` → `0xFF`; only from `phase 0x20`; enters status mode |
| `0x9090`| Read identifier codes      | reads: `block+0 → 0x00B0` (manufacturer), `block+2 → 0x00B5` (device) |
| `0xFFFF`| Read array / reset         | exits status and ID mode; accepted from any phase |

## Status mode (the freeze hazard)

- Entered by: any program completion, erase confirm, (DUT only: `0x7070`).
- While active, **all** flash reads return the status register,
  **chip-globally** — not scoped to the block or 2 MB half that was
  written. The OS's flash routines poll status through a command/status
  pointer (`a2`, typically CertMem) that can sit in a different half than
  the data pointer (`a3`); a bank-scoped status returns *array data*
  there, and any programmed word with bit 7 = 0 deadlocks the poll:
  `move.w (a2),d0 / btst #7,d0 / beq loop`.
- **Busy vs ready — the deferred-implementation rule (corrected
  2026-09-05):** the reference emulators are *instantaneous* (writes
  complete in zero time), so their status is always ready — there is no
  fill window to be busy in. A hardware-accurate deferred implementation
  (ours erases 64 KB via a background fill) **must report BUSY (DQ7=0)
  while the fill runs and READY (DQ7=1) after** — exactly like real
  silicon. Reporting ready during the fill let the OS read/verify and
  program blocks while the fill was still overwriting them: corrupted
  decompressed data (doubled banner glyphs), then the post-decompression
  derail. Verified on hardware 2026-09-05 (build p2d failure → p3c fix).
- Only `0xFFFF` exits status mode (`0x5050` alone does not — all three
  implementations agree).

## Canonical 68k sequences

```
program: 5050 / 1010 / DATA@addr / poll(a2) until DQ7=1 / 5050 / FFFF
erase:   5050 / 2020 / D0D0@block / poll / 5050 / FFFF
id (OS): 9090@CertMem / read ROM_BASE / read ROM_BASE+2 / 5050 / FFFF
id (boot): same without the 5050
```

## DUT (rtl/flash_ctrl.sv) status

Audited 2026-09-02 against the table above. Accepted extensions (harmless,
no OS-visible difference): byte writes decoded by lane; `0x4040` accepted
as setup alias; `0x7070` read-status command (real WSM has it, references
don't); erase fill physically completes before the next deferred command is
serviced (so the 68000's DTACK stretches during a fill — architecturally
legal, ordering preserved).

Fixes applied 2026-09-02 (both were freeze-class hazards):

1. **Status mode is now chip-global** (was bank-scoped on `ret_or_bank`).
2. **Status is now always ready** (`0x0080`; was `0x0000` "busy" during
   the erase fill — an invented busy state absent from both references).

## Checks

- `debug/readback/wsm_contract_test.py` — models the v12 reference WSM and
  the DUT decode side by side and asserts parity (flash contents + poll
  results) over the canonical sequences and the freeze edge cases
  (data-looking commands, cross-bank status polls, poll-less erase→program).
  Current status: **8/8 PASS**.
- `debug/readback/compare_golden.py` — golden 4 MB image synthesis from a
  `.89u` (payload @ chip `0x12000`, boot mirror, `FEEDBABE` + HWPB) vs the
  UART readback capture.
