# P2 — Garbled-banner diagnosis — fork decision reached (2026-09-05)

**Fork verdict: the garble is CPU-WRITE-SIDE.** The corrupted framebuffer
bytes are stable across reads (two dumps bit-identical), so the display path
and the SDRAM read path are exonerated: the junk is genuinely in RAM — the
OS wrote it. The bytes the CPU *executes* are correct (boot header verified
exact on hardware).

## What was built (P2 prerequisite RTL, deployed as build p2d)
- `rtl/dbg_uart.sv`: UART RX (8N1) + command parser (`D <F|R> <start> <len>`,
  `T`), `$D` command-dump marker, forced-trace support, and a `C=xx` parser
  diagnostic field appended to every status line.
- `rtl/mem_ctrl.sv`: command-dump FSM (B_CPASS/B_CREQ/B_CWAITW/B_CEND) —
  bounded range dumps of the flash image or calc RAM, lowest-priority RAM
  slot, CPU keeps running.
- `tools/hw_session.sh`: reader-first bounded capture (a reader attached
  after `printf` loses the `$D` marker and shifts the capture — found the
  hard way), 6-hex zero-padding, remote-tmp staging.
- `tools/gospel_diff_lib.py`: structural `$D`/`$P` stream parser.

## Hardware findings (all bounded dumps, manifests in `debug/hw/`)

1. **Framebuffer (RAM $4C00, 4096 B) vs gospel S2:** 356/4096 bytes differ,
   signature *scatter*. Rendered side-by-side: the hardware holds the SAME
   banner text, but every row is shifted left ~8 px (= **1 byte**) and each
   row carries 2 junk columns at the left edge; rows 0–3 are a solid bar
   (possibly a legitimate later-sub-phase progress bar).
2. **Stability:** two consecutive dumps bit-identical ⇒ content is written
   state, not a capture/read artifact. Display path + SDRAM read path
   exonerated. The LCD DMA fetches the same junked RAM ⇒ the visible garble
   is fully explained.
3. **Executed bytes are correct:** the boot header at chip $12088 (SSP/PC)
   and the first 8 KB of OS code match the gospel exactly (the CPU booted
   from correct bytes).
4. **Flash window diverges from the pre-boot gospel:** chip $12000+ content
   now differs substantially (rewrite = the AMS first-boot in-place flash
   decompression; rewritten bytes look like plausible decompressed code —
   consistent with the 2026-08-31 hardware traces of the "big decompression
   at $820700"). **The gospel v12 does NOT model this** (its post-install
   flash changed only 79 words) — documented gospel boundary: v12 is valid
   for pre-decompression states and static semantics, not for post-
   decompression flash state.
5. **Live state at capture:** `ST=4`, `FLW=$A1` (161 WSM strobes, frozen),
   `INT=$616` (frozen), CPU derailed into $1xxxxx reading $1414; the last
   flash-window cycles (flash-watch ring, via the `T` command) show a tight
   read loop at CPU $81269C–$8126A4 fetching correct gospel bytes — the
   boot-handler region ($8126xx = the AI7 soft-reboot handler).
6. **Determinism:** the same derail reproduces on every boot at the same
   FLW/INT marks (p2b and p2d builds).

## Interpretation
The install reaches the scan phase (~161 WSM writes), enters the AI7
soft-reboot choreography, and derails. The banner RAM damage (1-byte row
shift + edge junk) is written by the OS around the same era. Two candidate
mechanisms, both now testable at byte level (the rules' simulation gate is
satisfied for the framebuffer discrepancy):
- the AI7-trigger write handling (our RTL blocks the write, TiEmu-style;
  v12 — the behavioral gospel that COMPLETES the install — lets it land),
- the decompression write path (which on hardware writes through the WSM
  into the flash window while v12 never does).

## Next (P3)
1. `D R 0 400` vectors diff (captured, pending analysis) + `D R 5B00 100`
   OS vars.
2. Cert block `D F 10000 80` (captured; parse with the `$P`-marker path).
3. Targeted µs simulation (first one justified): replay the recorded
   flash-watch sequence around the AI7 write; second target: the fb write
   of one banner row (proven 1-byte shift).
