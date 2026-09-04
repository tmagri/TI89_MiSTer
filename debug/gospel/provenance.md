# Gospel provenance — 2026-09-04T17:58:20+00:00

- OS file: TI89Titanium_OS.89u  sha256 `39440a0864b936520026fc674f9b35260405d8696fcfcc7a2770b6f79b6a80ee`
- A gospel S1 flash: `70912348789b47221546e174ab6f70b2f931ea4f933f6d5beeb125612a991ea3` (4194304 B)
- B local synthesis: `2ea4dbebac76802bc35252b73731b0de6bbaaa85517ac39192b82d6ec6a9caec` (4194304 B)
- C hardware run-8: parsed from dump_run8.bin (single pass)

## Invariants (gospel S1 flash)
- ✅ initial SSP @0x12088 == 0x00004C00
- ✅ initial PC  @0x1208C == 0x00812188
- ✅ magic @0x100 == 0xFEEDBABE
- ✅ HWPB pointer @0x104 == 0x00800108
- ✅ hardware ID word @0x10C == 0x0009 (BE)

## A vs B (gospel vs local synthesis): 1 differing bytes, 0 non-expected
- 0x167C86: gospel FF vs synthesis 96 (expected: gospel drops odd trailing payload byte)

## A vs C (gospel vs hardware run-8): 1 differing words, 0 non-benign
- word 0x0B3E43 (chip 0x167C86): hw 9600 vs gospel FFFF (known benign flush-pad)

## Verdict
**PASS — three-way agreement.** The gospel starting state is proven:
reference emulator flash == deterministic synthesis == hardware-loaded
flash (modulo the one known cosmetic pad word).
