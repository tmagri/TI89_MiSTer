# P0 — Workspace protocol + gospel validity — COMPLETE (2026-09-05)

**Verdict: GOSPEL PROVEN.** Three-way flash agreement + behavioral proof
(banner rendered, HOME reached) in the reference emulator with our exact
`.89u`. Phase gate 0→15% satisfied.

## Deliverables
- `gospel/states/S1_*` — pristine flash (4 MB) + reset-state RAM (256 KB),
  manifest: pc=$812188 sr=$2700 a7=$4C00 (exact reset entry).
- `gospel/states/S2_*` — banner-moment state: pc=$95C5E0 (format scan),
  framebuffer rendering the two-line "… in progress / Do not interrupt!"
  banner, vectors + OS vars + flash $812000–$95FFFF.
- `gospel/provenance.md` — **PASS**: gospel flash == local synthesis
  (1 expected byte: gospel drops the odd trailing payload byte; RTL/hardware
  flush it as `$96FF`) == hardware run-8 readback (0 non-benign words).
- `tools/hw_session.sh`, `tools/gospel_provenance.py`, `tools/gospel_diff_lib.py`,
  `gospel/cdp_gospel.mjs`, `gospel/cdp_probe.mjs`, `gospel/gospel_harness.mjs`.

## Behavioral proof (the P0 stop-gate)
The reference emulator (ti89-simulator v12, real Chrome, unmodified JS):
1. Boots our OS from the RTL-identical flash synthesis,
2. renders the banner (S2 framebuffer ASCII render in this report's history
   and reproducible from `S2_ram_fb_1000.bin`),
3. executes the documented AI7 soft-reboot choreography (~T+95 s, pc=$812644),
4. reaches a fully drawn, stable HOME screen (framebuffer verified; idle PCs
   in $958Cxx–$95C5xx — the $962226 signature is sim-specific, not a law).

## Reference-emulator landmines (mandatory knowledge for P1–P3)
1. **`pause_emulator()` is a no-op** (its `clearInterval` is commented out in
   v12.js) while `resume_emulator()` stacks a SECOND `emu_main_loop` interval.
   Every pause/resume cycle doubles the drivers → OS timing shredded →
   deterministic derail (a7=$1D6, ASCII stack, pc in $3xxxxxx). NEVER pause;
   snapshot via atomic in-page reads instead.
2. **`loadrom()`'s TIB converter fills flash 0x0000–0x11FFF with $1400** —
   garbage in the certificate area (chip 0x10000). Booting from that image
   derails identically in headless-node AND real-browser hosts. The
   RTL-identical synthesis (0xFF-erased head + boot mirror + FEEDBABE/HWPB)
   boots cleanly. → Gospel load path = `setRom(img) + initemu()`.
3. **Page wiring is closure-scoped** (`calccontainer.js` jQuery-ready): emu/
   ui/link are not globals and there is no file input. `loadSimulator()`'s
   exact call sequence must be replicated at global scope, including
   `link.setEmu(emu)` (the link module keeps module-local `emu`; without the
   setter `emu.raise_interrupt` throws).
4. **HWPB layout settled**: len word @$108, then interleaved BE words
   (00 00 / 00 09 / 00 02 …) — hardware ID word @$10C = $0009. Matches
   rom_loader.sv, v12, and the hardware readback exactly. The earlier
   "endianness discrepancy" was a bug in compare_golden.py's LE packing
   (fixed).

## Consequences for the remaining phases
- P1 is effectively complete for S1/S2 (captured + proven); S3 (post-install
  flash) needs one rerun with a framebuffer- or stability-based HOME detector
  (pc==home signature retired).
- P2 (garbled banner) can now diff the hardware framebuffer against
  `S2_ram_fb_1000.bin` — the exact gospel pixels of the same screen.
- The plan's "hashing the decompressed OS in emulator RAM" idea is
  superseded: with the uncompressed Titanium image, flash-state equality is
  the stronger proof (RAM is derivable runtime state, not an identity).
