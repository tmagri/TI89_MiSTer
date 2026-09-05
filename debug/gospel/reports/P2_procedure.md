# P2 — Garbled-banner diagnosis: execution procedure

Run AFTER the P2 bitstream (WSM fixes + UART RX command interface) is
deployed. All commands bounded; results land in `debug/hw/` with manifests.

## Preconditions
- `ti89_build_debug.sh --compile` completed (P2 build: flash_ctrl status
  fixes, dbg_uart RX `D`/`T` commands, mem_ctrl command-dump FSM).
- Core auto-launches with the OS (`auto_boot.mgl`); the banner is the long
  lived state while the install hangs — ideal capture window.

## Step 1 — framebuffer diff (the fork decision)
```
debug/tools/hw_session.sh dump R 4C00 1000
python3 debug/tools/gospel_diff.py gospel/states/S2_ram_fb_1000.bin \
        debug/hw/dump_R_4C00_1000_<ts>.bin --label P2_fb_diff
```
Decision tree (`P2_fb_diff.md`):
- **MATCH** — the OS drew the banner correctly; corruption is DOWNSTREAM of
  the framebuffer (lcd_ctrl DMA fetch / video scaler). Next bounded probe:
  HDMI capture vs a PBM render of `S2_ram_fb_1000.bin`; a divergence there
  justifies the first targeted µs simulation (lcd_ctrl row fetch only).
- **word_swap** — CPU write-lane bug (UDS/LDS mapping on RAM writes).
- **shift±1/2** — address/A0 misdecode on the write path.
- **scatter** — compare text rows 0–20 only (the S2 gospel capture is from
  format-loop ENTRY; the hardware banner may sit at a later sub-phase, so
  the progress-bar region may legitimately differ). If the text rows match
  and only the bar region differs, re-capture and diff again at a matched
  sub-phase; if the text rows differ, same decision tree as above.

## Step 2 — code/font integrity (CPU fetch path)
```
debug/tools/hw_session.sh dump F 12000 140000      # chip $12000-$15FFFF
python3 tools/gospel_diff.py gospel/states/S2_flash_12000_140000.bin \
        debug/hw/dump_F_12000_140000_<ts>.bin --label P2_code_integrity
```
MATCH (expected — the gospel proves this region never changes) ⇒ the CPU is
executing correct bytes ⇒ any garble is display-side. ANY mismatch = the
proven byte-level discrepancy → eligible for the targeted µs simulation.

## Step 3 — WSM handshake at the stall (feeds P3)
```
debug/tools/hw_session.sh send 'T'                 # trace rings, no fault needed
```
The F/E/L lines show the last 32 completed CPU bus cycles + last 64
flash-window cycles — the exact WSM command/poll sequence at the stall.

## Step 4 — installer progress (feeds P3)
The gospel installer signature (P1) says only 79 words ever change:
- cert block: `dump F 10000 80`  → after install, word 0 = $FFF8, word 1 = 0
- 39 block markers: `dump F 190010 2` … one word at chip $190000+last word
  of block (exact offsets in P1 report)
If the cert block still reads all-$FF on hardware, the installer never got
to write it — bounds the stall to before FL_addCert.
