# P1 — Gospel reference binaries — COMPLETE (2026-09-05)

**Verdict: all three states captured and cross-verified** in the reference
emulator (real Chrome, RTL-identical synthesis, no pausing). Phase gate
15%→40% satisfied.

## Captured states (`gospel/states/`)

| State | When | Content | Proof |
|---|---|---|---|
| S1 post-load | pc=$812188 sr=$2700 a7=$4C00 (reset entry) | full 4 MB flash + 256 KB RAM | three-way provenance PASS (`provenance.md`) |
| S2 banner | pc=$95C5E0 sr=$2704 a7=$4B56 | vectors, framebuffer (banner rendered), OS vars, flash $812000–$95FFFF | code region **bit-pristine** vs synthesis: 0/1,310,720 bytes (`P1_S2_flash_pristine_check.md`) |
| S3 post-install | flash-quiescence 30 s after banner (pc=$841EE2) | vectors, framebuffer (HOME), OS vars, **full 4 MB flash** | see installer signature below |

Captured by `gospel/cdp_gospel.mjs` run 11 — observer-in-page, never pausing
(v12 pause/resume is sabotaged, see P0 report). HOME detection = 30 s of
archive-flash-signature quiescence after the banner (the AMS idle wanders
across many code regions, so a pc window does not work).

## The installer's complete net-flash signature (S1 vs S3)

**79 words total** — this is everything the whole install changes:

1. **CertMem, chip $10000 (CPU $810000): 40 words**
   `$FFFF→$FFF8` (the OS writes the certificate marker ITSELF — the $FFF8
   check is therefore satisfied only AFTER install in the direct-boot flow),
   `$0000` at +2 (boot-gate cleared), then the certificate bytes.
2. **39 archive blocks × 1 word each**, chip $190000→$3F0000
   (CPU $990000→$BF0000): a per-64KB allocation marker. $990000 is exactly
   the archive-start address in titanium-info.txt — confirms the zone math.

**The ~262K flash writes observed during the banner are erase-fills writing
$FF over $FF** (net-invisible). Implications:
- P3's bounded flash-progress dump = cert block (chip $10000, 128 B) + the
  39 marker words (1 word per block, chip $190000+$n*$10000) — a few hundred
  bytes total instead of megabytes.
- A WSM defect in bulk-erase, $FF-program, cert writes, or the marker writes
  is sufficient to hang the banner forever (candidate set for bug 2).
- The banner's code+font region is proven pristine ⇒ the garbled banner
  (bug 1) is NOT bad code/font bytes — P2's framebuffer diff decides between
  CPU write-path vs display-path.

## Diff tooling

`tools/gospel_diff.py <gospel> <target> [--base HEX --len HEX --label N]` —
auto-parses run-8-style captures, classifies diffs (word_swap / shift±1,2 /
scatter), writes `gospel/reports/<label>.md`. Self-tested:
- S1 vs itself → MATCH
- S1 vs run-8 hardware capture → only the known flush-pad word, classified
  `shift_pm2` (`selftest_run8.md`)
- S2 code region vs synthesis → MATCH (`P1_S2_flash_pristine_check.md`)
- S2 fb vs S3 fb → DIFFER (banner → HOME) (`P1_S2S3_fb_changed.md`)
