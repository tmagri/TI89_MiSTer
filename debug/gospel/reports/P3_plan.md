# P3 — Root-cause & fix plan (2026-09-05)

## Decisive new evidence
RAM dump `R 4B00 200` (p2d build, hardware): calculator RAM $4D00–$5166
contains dozens of **verbatim `[TI89] PC=… A=… D=…` debug status lines as
ASCII**, sequentially appended — INSIDE the framebuffer region
(fb = $4C00 + 100×30 B = $4C00–$5757; $4D00 is row 5).

**⇒ The "garbled banner" IS our own UART debug text written into calculator
RAM and rendered as pixels.** And the same writer corrupting OS state
(vector pages $130+ junk; INT frozen; FLW=$A1; cert ALL $FF ⇒ installer
stalled before its first program op; boot-critical vectors $0–$7F clean).

## Watch-ring fingerprint (build p3b, 2026-09-05)
The `T` command now emits the fb-write watch ring (M lines). Captured on
the derailed state (INT=$615):
- Last 32 fb writes = **clean sequential `$FFFF` WORD writes ascending
  $4CE0→$4CEE** (the banner top-row white fill — matches the solid rows
  0–3 seen in the fb dump). These are legitimate banner-era CPU writes.
- The CPU is derailed in unmapped $1xxxxx and cannot be the author of the
  growing status text; the bus rings show no text-writing CPU cycles.
- **⇒ The status-text writer is a NON-CPU hardware path** that lands
  TX-correlated bytes into SDRAM at sequentially increasing addresses,
  ~1 status line per several seconds (still growing: fb dump at 13:09 had
  text to ~row 10; at 15:02 to ~row 28).
- p3a regression found & fixed: `tr_line` was 7-bit while TR_LINES=129
  (truncates to 1) ⇒ every trace dump "completed" after 1 line. Widened to
  8 bits (p3b) — full 129-line traces verified (1 F + 32 E + 64 L + 32 M).

## Root-cause hypothesis (to be proven, not assumed)
The dbg_uart status TRANSMISSION is coupled to spurious SDRAM WRITES whose
data = the TX shift content and whose addresses advance per byte. Likely
coupling points (ordered):
1. sd_wr glitch on the mem_ctrl→SDRAM interface correlated with TX activity
   (du_send/S_SEND toggling shares the clk with sdram_wr sampling).
2. The du producer's dump_pass_stb/dump_stb path aliasing into a write
   (dump_stb is combinational on dump_rdy — a read strobe should never
   assert sd_wr, but the arbiter's sd_wr default and grant FSM timing must
   be audited).
3. lcd_dma/req handshake aliasing writes with dbg activity.

## Fix steps
1. **Observability (RTL, DONE in p3a/p3b):** fb-write watch ring (M lines) ✓;
   OSD O7 "UART Status Line" mute (status_mute input) ✓.
2. **Coupling experiment (needs human at OSD):** set O7=Off (status muted)
   → wait 60 s → `D R 4C00 1000` twice → diff. If the text stops growing
   while muted, the TX-coupling is CONFIRMED; if it keeps growing, the
   writer is independent of the TX path.
3. **Targeted µs simulation (justified: byte discrepancy proven):** replay
   the write ring in tb_boot.sv — a few hundred cycles — to pin the exact
   glitching cycle once the coupling is confirmed.
4. **Fix per evidence:** expected shapes — (a) sd_wr asserted during
   non-write grants (tighten grant FSM defaults), (b) du/TX engine
   drive contention on a shared bus, (c) SDRAM DQ drive conflict. The
   mute test + ring fingerprint decide.
5. **Re-verify:** banner renders clean (fb diff vs gospel S2 = MATCH on
   text rows), install proceeds past FLW=$A1, cert block becomes non-$FF,
   boot reaches HOME (`ST=4 L=1 P=1`, INT climbing).
6. **Commit:** RTL fixes + tools + reports; update STATUS.md.

## CORRECTION (2026-09-05, after OSD O7 mute + reader-first captures)

The earlier "status text written into the framebuffer" finding was a
**capture artifact**: with the UART unmuted, status lines interleaved into
the raw dump stream on the wire and my parser walked them as payload. With
the reader-first capture + status muted, the dumps are byte-perfect
(marker + exactly 4096 bytes) and contain **zero status text**.

**Superseded conclusions:** the "1-byte row shift + edge junk" fb analysis
(tainted by wire junk) is retracted; the display path is NOT exonerated by
that evidence. What remains solid:
- fb content is STABLE (repeat dumps identical, muted and unmuted windows).
- The TRUE fb (muted capture) shows: rows 0–7 solid white (a filled bar or
  title area — not present in gospel S2), then sparse readable text rows.
  It is NOT the gospel S2 banner state — the machine is somewhere else
  (consistent with the post-derail frozen state, INT=$615, FLW=$A1).
- Cert block ALL $FF (installer stalled pre-first-program) ✓ stands.
- Boot-critical vectors $0–$7F match gospel ✓ stands.
- Deterministic derail at the same FLW/INT marks ✓ stands.
- v12 in-place-decompression gospel boundary ✓ stands.

## SIMULATION RESULT (2026-09-05, build p3d semantics, behavioral SDRAM)

Rebuilt tb_boot on the CURRENT RTL (busy-fill + AI7-write-lands + all
fixes) with Verilator --binary. Two TB defects found and fixed en route
(missing dump_rdy tie-off had silently disabled all sim boots since the
dump interface landed; fx68k_now needed -Wno-BLKANDNBLK).

**Result: NO DERAIL in simulation.** 2.85G+ cycles: boot, install (archive
writes at $990000 per the P1 signature), then a stable scheduler loop at
$8228xx. Same RTL on hardware derails into the $1414 sweep at FLW=$A1.

**⇒ The remaining defect is a hardware-vs-behavioral-model timing
difference — the real SDRAM chip's read path under CPU+LCD burst load**
(the §21/§22 family; the negedge-capture fix cured the SLOW pre-boot dump
path but the failure persists under the CPU/LCD back-to-back burst load of
the decompression/install era).

## Revised next steps
1. **Real-chip-model sim on current RTL:** swap the TB's behavioral sdmem
   for `sdram_chip.sv` (the §21/§22 methodology) — expected to reproduce
   the HW-only derail in simulation, giving cycle-exact visibility.
2. **AI7 A/B verdict (done):** write-lands semantics = REAL PROGRESS,
   kept permanently (INT climbing vs frozen; OS runs much further).
3. **Busy-during-fill (p3c):** correct real-silicon semantics, kept; did
   not alone fix the hang.
4. **Human visual check (done, 17:11 screenshot):** the screen shows the
   doubled-stroke banner ⇒ the corruption IS in the fb content (the OS
   drew it from corrupted decompression-era data), and the display path
   renders the fb faithfully at 4× (all screenshot strokes are 4-px
   multiples — display path quantitatively exonerated).
5. After the real-chip-model sim reproduces the derail: cycle-exact fix,
   hardware A/B, then the full verify chain (clean banner, cert non-$FF,
   FLW past $A1, HOME).
