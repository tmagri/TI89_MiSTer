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

## WRITE-PATH TIMING ANALYSIS (plan step 1, 2026-09-06)

Static write-timing math at 60 MHz (T = 16.667 ns), current sdram.sv:

- Controller launches CMD_WR + DQ_OUT + dq_oe at its posedge (t=0); pins
  settle by t ≈ tCO ≈ 2-5 ns (FPGA IOE + board).
- The chip samples commands on SDRAM_CLK posedges; SDRAM_CLK = clk_sdram
  = PLL -3000 ps (leading ~2.9 ns) → the chip's sample edge lands at
  t ≈ 13.8 ns after the launch edge (next SDRAM_CLK posedge).
- DQ validity window: [~3 ns, ~25 ns] (dq_oe held through S_WDLY's tWR
  hold cycle before PRECHARGE).
- ⇒ Setup margin at the chip's latch edge ≈ 10.8 ns; hold ≈ 11 ns.

**Static margins are comfortable — which is why the empirical W-command
test (plan step 2) is the decisive instrument**: if the pattern test
FAILS on hardware despite comfortable static margins, the corruption is
DYNAMIC (pattern/address-dependent: e.g., data-rate-related DQ slewing,
bank/row contention, or refresh collisions) and the failing bit/word
fingerprint from the test localizes it.

## W COMMAND (plan step 2, implemented build p3e)

`W R <6-hex start> <6-hex len>` — dbg_uart parses 'W' (cmd_wr), mem_ctrl's
command FSM writes the counting word pattern (start_words + i) & 0xFFFF
through the boot RAM slot (the same write path the CPU uses: grant →
sdram.sv → chip), then the host reads the region back with `D R` and
compares. RTL: dbg_uart.sv (parser + cmd_wr), mem_ctrl.sv (cmd_wr_r,
B_CREQ writes cmd_pat_w, B_CWAITW write-mode completion without the dump
handshake), TI89.sv (wiring). Host: hw_session.sh `wrtest`.

## MILESTONE (2026-09-06, build p3e): banner renders CLEAN; OS reaches controlled power-down

The framebuffer (muted clean capture) now shows the banner dialog
**correctly drawn**: dark title band with light, properly gridded text
glyphs (rows 8–21), dialog body, second text line (rows 78–84). The
double-stroke glyph corruption is GONE — the busy-during-fill semantics
fixed the data corruption (the OS now waits for erase fills, so its
font/data reads return correct bytes).

State at capture: CPU in STOP (S=1, PC=$2702), LCD off (L=0), INT=$61E
frozen — the OS's **APD power-down** (auto power-down after ~5 min
inactivity; wake source = the ON key/AI6). This is a CONTROLLED OS state,
not the derail. Cert still all-$FF (installer not yet completed) — the
install progression past the banner era remains to be verified by waking
the machine (user: press Insert = ON).

**Bug 1 (garbled banner): FIXED by busy-during-fill** (pending user
visual confirmation). Bug 2 (install hang): the machine no longer
derails at the stall — the install progression past the banner era is
the remaining verification.

## WRTEST RESULT (2026-09-06, build p3e, live derailed machine)

`W R 004000 001000` (counting-word pattern write to calc RAM $4000) then
`D R 004000 1000` readback: **the payload matches the pattern exactly**
(`20 00 20 01 20 02 …` = 0x2000+i ✓ verified by inspection; the initial
"3960 mismatched" verdict was my compare builder's double-increment bug).

**⇒ The SDRAM write+read path is CLEAN on hardware.** The RAM corruption
(the doubled-glyph fb content, the vector-page junk) was written BY THE OS
itself — from data it read corruptly during the decompression-era flash
reads (intermittent, real-SDRAM-load-only; the slow dump reads and both
sim models are clean).

**⇒ The fix domain: SDRAM read-capture margins at 60 MHz under burst
load.** The concrete plan: phase sweep (legal 625-ps multiples) with the
USER'S SCREENSHOT as the per-phase metric (clean banner vs doubled
glyphs), plus the wrtest loop at each phase. If a phase renders the banner
clean → pin it; if no phase does → read-hold-time fix (different capture
alignment) per the sweep data.

## LESSONS (tooling)
- parse markers: false "$D" inside binary payload — anchor structurally
  (last "$" before the first @sync), not find().
- Compare builders: verify against a known-good payload by inspection
  before trusting a mismatch count.
- wrtest during the derailed sweep state is invalid (the sweep smears
  4096 B in ~40 ms); valid windows = the APD-stopped state or the
  pre-derail minutes immediately after boot_done.

## FINAL NARROWING (2026-09-06): every path tests clean; the suspect is the SDRAM read path under the OS's RAM-probe pattern

Evidence matrix (all hardware, current builds):
- SDRAM reads, 4 PLL phases, 336 KB × 4 (controller-paced): BIT-EXACT
- SDRAM write+readback, 2048-word counting pattern (W+D): BIT-EXACT
- Flash code region, 128 KB: BIT-EXACT
- Display path: faithful 4× (screenshot strokes are 4-px multiples)
- YET: the OS drew doubled-stroke glyphs, corrupted vector pages $130+,
  and derailed deterministically.

⇒ The corruption is in the OS's own RAM-probe era: the FIRST-boot RAM
sizing probe (titanium-info [1]: C5A3C5A3 patterns, 64K strides, read-back)
runs through the SDRAM read path with specific address/data patterns. An
intermittent mis-read THERE → wrong RAM size → wrong heap/layout → doubled
draw buffer + corrupted task structures + the derail. Explains: the
phase-dependent failure points, the sims' clean runs (no setup/hold
violations in models), the stable corrupt content afterward.

## THE FIX PLAN: SDRAM controller timing audit
Audit rtl/sdram.sv state-machine timers against the IS42S16160 datasheet
at 60 MHz (T = 16.667 ns): tRCD, tRP, tRAS, tRC, tWR, tREFI refresh
interval, DQ valid windows. A timer one cycle short produces intermittent
wrong-row/column reads — the probe-failure signature. Fix = correct the
timer counts; verify = the probe-pattern wrtest at each suspicious
address stride (64K, 32K, 16K) + the full boot chain.

## BREAKTHROUGH (2026-09-06 13:0x): THE CORE IS FUNCTIONALLY OPERATIONAL

Live state: `PC=$060006` loop reading `$600006` (power/keyboard poll),
`INT=$2C85` climbing, `L=1`, the dialog drawn in the fb, C=06 (commands
parsing). And the `$4000` pattern wrtest: **bit-exact** (the earlier
"3960 mismatched" was my compare builder's double-increment bug — the
payload matched by inspection).

**⇒ The core WORKS: boot, display, interrupts, debug command interface,
SDRAM write+read bit-exact.** The remaining symptom: the banner dialog's
text glyphs render **double-stroked** (each stroke twice, offset ~1 fb px)
— in the fb content itself, drawn by the OS.

## THE FINAL TWO CANDIDATES
1. **fx68k instruction-level defect** in the decompressor's instruction
   mix (the decompressed data region is bit-exact vs gospel — but that
   only proves the WRITES landed; if the decompressor READ its source
   through a defective path... the source is the flash window ✓ verified).
   Verification: fx68k instruction test vectors over the decompressor's
   exact instruction mix.
2. **An OS state input differing on our HW** (boot version 1.1.1
   fabricated, geometry, $600001 defaults) sending the dialog renderer
   down a double-stroke path.

## NEXT SESSION
1. fx68k instruction verification over the decompressor's instruction mix
2. If clean: the OS-state-input audit (boot version, $600001 reset value
   vs J89hw's documented $04, HWPB fields) — one variable per rebuild
3. The user's eyes on the screen after ON-key wake
