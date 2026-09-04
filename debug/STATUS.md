# TI-89 MiSTer — debug status (current)

Single entry point for debugging state. Supersedes `archive/SESSION_STATUS.md`
(2026-08-31) and `archive/DEBUG_STATUS.md` (2026-08-20, §1–§22 history).
Updated 2026-09-05.

## ACTIVE: tactical debug plan (gospel-diff approach)

Two open failures: (1) garbled banner rendering, (2) install never completes.
Executing the approved 4-phase plan — **P0 COMPLETE, see
`gospel/reports/P0_provenance.md`**:

- **Gospel proven (P0 ✅):** reference emulator == local synthesis ==
  hardware-loaded flash, three-way (`gospel/provenance.md`), AND behavioral
  proof — real-Chrome v12 boots our `.89u`, renders the banner, runs the AI7
  soft-reboot, reaches a drawn HOME screen.
- **Gospel states captured:** S1 (pristine flash + reset RAM) and S2 (banner
  moment: pc=$95C5E0, banner framebuffer, vectors, OS vars) in
  `gospel/states/`. S3 (post-install flash) pending a rerun with a
  stability-based HOME detector (the pc==`$962226` signature is sim-only).
- **v12 landmines** (must-know, in P0 report): pause_emulator is a no-op +
  resume stacks intervals (never pause); loadrom's TIB fill ($1400 head)
  corrupts the cert area and derails (use setRom of the RTL-identical
  synthesis); page wiring is closure-scoped (replicate loadSimulator calls;
  link.setEmu required).
- **Next:** P2 framebuffer diff (`D R 4C00 1000` over UART once the RX-command
  RTL lands, vs `S2_ram_fb_1000.bin`) decides CPU-write-path vs display-path.


## RTL state (uncommitted working tree)

Six files modified vs HEAD (97a5b29), 527 insertions / 90 deletions:

| File | Contents |
|---|---|
| `rtl/mem_ctrl.sv` | AI7 soft-reboot write-block (2026-08-31 root cause: TiEmu blocks the protected write below `$000120`; performing it corrupted the OS's NMI reboot context) + pre-boot image dump FSM |
| `rtl/dbg_uart.sv` | Fault-triggered trace dump: 1 header + 32-entry bus ring + **64-entry flash-watch ring** (every completed CPU cycle in the flash window — the tool for WSM-handshake divergences) |
| `rtl/rom_loader.sv` | Payload-word skid buffer (fixes b_wr/b_wait race that dropped words) |
| `rtl/io_ports.sv`, `TI89.sv` | I/O addressing + IACK/VPA latch fixes (see archive/DEBUG_STATUS.md §13–§15) |
| `rtl/flash_ctrl.sv` | **2026-09-04 WSM contract fixes (NOT yet compiled or deployed):** status mode is now chip-global (was bank-scoped on `ret_or_bank` — could freeze the OS's `btst #7` poll when its status pointer sits in the other 2 MB half) and always-ready `0x0080` (was invented `0x0000`-busy during erase fill; both references complete instantly) |

## Verification ledger

- **Flash image loads bit-exact on hardware.** Run-8 UART readback (2026-09-01,
  `readback/dump_run8.bin`) vs the reference synthesis: identical except one
  cosmetic word at chip `0x167C86` (`9600`/`96FF` — the dangling-payload-byte
  flush's pad didn't land; past the OS's declared end).
  Reproduce: `python3 tools/parse_readback.py readback/dump_run8.bin ti89_sim/archive/flash.hex`
- **WSM software contract holds.** `tools/wsm_contract_test.py` models the v12
  reference WSM and our decode side by side: **8/8 PASS** (canonical program /
  erase / ID sequences + freeze edge cases). Contract documented in
  `WSM_CONTRACT.md`.
- **Sim boots end-to-end** (historical, pre-Sep-2 RTL): archive/DEBUG_STATUS.md
  §16, run 12 — banner-phase flash format (~262 K words) completes, HOME screen
  at `$962226`, `protect=1`, interrupts healthy.
- **Not yet validated anywhere:** the 2026-08-31 AI7 write-block fix on
  hardware (runs 6–8 were spent on load-path verification; sims can't reach
  the AI7 in budget), and the 2026-09-04 flash_ctrl fixes (unbuilt).

## What a healthy boot looks like (UART monitor, ~4 Hz)

The "Installation in progress… Do not interrupt" banner is the OS's legitimate
first-boot archive/FlashApps formatting — expect **tens of seconds** of it
(`FLW` climbing to ~262 K). Healthy end state: `ST=4 L=1 P=1`, `PC` near
`962226`, `INT` incrementing, no `F…` fault dumps.

## Open items

1. Hardware validation of the AI7 fix + the flash_ctrl WSM fixes (build with
   `ti89_build_debug.sh --compile`, then watch the flash-watch ring output).
2. RAM-ghost discrepancy: TiEmu `mem89tm.c` mirrors RAM at `$200000/$400000`
   (what our `mem_ctrl` implements); v12.js fixed 89T boot by having **no**
   ghosts beyond `$40000`, and AMS 3.10 checks ghost space (`$82241C`).
3. The single-word flush quirk at chip `0x167C86` (cosmetic; see ledger).

## Folder layout

```
debug/
  STATUS.md                     this file
  WSM_CONTRACT.md               flash WSM software contract (reference-cited)
  TI89.qsf.known-good-20260831  ONLY copy of the full DE10 pinout — do not archive
  gospel/                       gospel reference (P0 complete)
    states/                       S1/S2 dumps + manifests (+ HOME proof)
    reports/                      P0_provenance.md
    cdp_gospel.mjs                real-Chrome capture driver (observer-in-page)
    cdp_probe.mjs                 ad-hoc CDP evaluator
    gospel_harness.mjs            headless node driver (kept; see landmines)
    provenance.md                 three-way flash proof — PASS
  hw/                           timestamped SSH/UART captures (hw_session.sh)
  tools/                        current analysis tooling
    compare_golden.py             golden synthesis (BE HWPB) + diff vs capture
    parse_readback.py             capture parser + CLEAN/DETERMINISTIC/FLAKY verdict
    gospel_provenance.py          three-way proof runner (exit 0 = PASS)
    gospel_diff_lib.py            run-8 parser + swap/shift signature detection
    wsm_contract_test.py          WSM contract parity test (8/8)
    hw_session.sh                 bounded SSH/UART capture wrapper + manifests
    convert89u.py                 .89u -> flash.hex golden generator (copy)
    session_status.sh             60 s progress watcher (paths updated)
  readback/                     current captures (run 8 = latest, verified)
  ti89_sim/                     Verilator sim sources — outputs in ti89_sim/archive/
  archive/                      superseded docs, photos, logs, run 6–7 captures
  .buildlock/                   created/removed by ti89_build_debug.sh (compile lock)
```
