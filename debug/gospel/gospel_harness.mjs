#!/usr/bin/env node
// gospel_harness.mjs — headless TI-89 gospel capture (Phase 0/1).
//
// Drives the UNMODIFIED ti89-simulator core (references/ti89-simulator/js/v12.js)
// in node, loads our TI89Titanium_OS.89u through its real loader path
// (loadrom -> handle_newromready -> initemu), fast-forwards execution and
// captures bounded gospel dumps:
//
//   S1  post-load, pre-execution : full flash (4 MB) + full RAM (256 KB)
//   S2  banner phase             : RAM $0000-$03FF, $4C00-$5BFF, $5B00-$5CFF
//                                  + flash chip $012000-$15FFFF
//   S3  HOME screen              : S2 bounds + full flash (4 MB)
//
// All files big-endian byte streams in debug/gospel/states/, each with a
// .manifest.json (pc/sr/cycles/lcd_base/hashes). The references/ tree is
// never modified: the factory source is sliced in memory, and ui/link are
// no-op shims (their full member list was enumerated from v12.js).
//
// Usage: node gospel_harness.mjs [--max-minutes 30]

import fs from "node:fs";
import path from "node:path";
import crypto from "node:crypto";

const HERE = path.dirname(new URL(import.meta.url).pathname);
const PROJ = path.resolve(HERE, "../..");
const V12_PATH = path.join(PROJ, "references/ti89-simulator/js/v12.js");
const ROM_PATH = path.join(PROJ, "TI89Titanium_OS.89u");
const OUT = path.join(HERE, "states");

const maxMinutes = (() => {
    const i = process.argv.indexOf("--max-minutes");
    return i > 0 ? Number(process.argv[i + 1]) : 30;
})();

fs.mkdirSync(OUT, { recursive: true });
const t0 = Date.now();

// ---------------------------------------------------------------- shims ---
// ui: canvas-less no-ops (enumerated from the core's call sites).
const uiShim = new Proxy({}, { get: () => () => undefined });
globalThis.ui = uiShim;
// link: NO SHIM — the real TI68kEmulatorLinkModule is instantiated below and
// wired with setLink(). (An early no-op shim returned 0 from
// compute_link_status where the real one returns $50 STX when
// link_config&2 — AMS's early $60000D polls then saw a never-ready link and
// derailed within the first 400K instructions.)

// loadrom() -> new FileReader(); reader.onload; reader.readAsArrayBuffer(file)
// Synchronous on purpose: initemu() must have completed before we dump S1
// or start the fast-forward loop (the browser's async reader would defer
// the whole init to a microtask that cannot run inside our sync loop).
class FileReaderShim {
    readAsArrayBuffer(file) {
        this.result = file._buf;
        this.readyState = 2;
        this.onload();
    }
}
globalThis.FileReader = FileReaderShim;

// ------------------------------------------------------- load the factory ---
const src = fs.readFileSync(V12_PATH, "utf8");
const a = src.indexOf("function TI68kEmulatorCoreModule");
const b = src.indexOf("function TI68kEmulatorLinkModule");
if (a < 0 || b < 0 || b <= a) {
    console.error("FATAL: cannot locate TI68kEmulatorCoreModule in v12.js");
    process.exit(2);
}
const factory = new Function(src.slice(a, b) + "\nreturn TI68kEmulatorCoreModule;")();
const emu = factory(globalThis); // stdlib = globalThis (console/setInterval/clearInterval)
// The REAL link module (slice from its factory to the next top-level fn).
const linkFactory = new Function(
    src.slice(b, b + 10 + src.slice(b + 10).search(/\nfunction /) + 1) +
    "\nreturn TI68kEmulatorLinkModule;")();
const realLink = linkFactory(globalThis);
// Wire the module-local ui/link vars — exactly what the page's
// calccontainer.js does (emu.setUI(ui); emu.setLink(link);). Without this,
// bare `ui`/`link` inside the factory are unwired and every ui.* call throws.
emu.setUI(uiShim);
emu.setLink(realLink);
// The link module calls back through the free global `emu` (emu.to_hex,
// emu.raise_interrupt) exactly as the page's global `emu` does — expose ours.
globalThis.emu = emu;
globalThis.link = realLink;
// setReset wires the module-local `reset` skip-ahead hook (initialize_
// calculator calls it after reset_calculator; the page's reset only clears
// its own UI vars). No-op = run the full unmodified boot.
emu.setReset(() => {});
// The link module keeps module-local `emu`/`ui` vars wired by the page via
// setEmu/setUI (calccontainer.js:51,59); without setEmu every
// emu.raise_interrupt call inside link_handling throws.
realLink.setEmu(emu);
realLink.setUI(uiShim);
realLink.setCalculatorModel(9);

// ------------------------------------------------------------ load the OS ---
const romBytes = fs.readFileSync(ROM_PATH);
emu.loadrom({ name: path.basename(ROM_PATH), size: romBytes.length, _buf: romBytes.buffer });

// Stop the 11 ms wall-clock interval; we drive emu_main_loop() ourselves.
emu.pause_emulator();

const rom = emu.rom();          // live Uint16Array (2^21 words = 4 MB)
const ram = emu.ram();          // live Uint16Array (2^17 words = 256 KB)
const rwNow = () => emu.rw();   // current bus read function (mode-dependent)
const pcOf = () => emu.pc() >>> 0;
const srOf = () => emu.sr() & 0xffff;
// NB: the exported cycles getter returns the per-opcode timing TABLE, not a
// counter — progress is measured in main_loop calls (each = 12800 instr +
// 200 timer ticks) and wall time.
const sha = (buf) => crypto.createHash("sha256").update(buf).digest("hex");

// words (Uint16Array view) -> big-endian byte string
const be = (words, wordOff, wordLen) => {
    const out = Buffer.alloc(wordLen * 2);
    for (let i = 0; i < wordLen; i++) out.writeUInt16BE(words[wordOff + i], i * 2);
    return out;
};

function dumpState(tag, ranges) {
    callsAtDump = loopCalls;
    const manifest = {
        state: tag,
        date: new Date().toISOString(),
        pc: pcOf().toString(16), sr: srOf().toString(16),
        main_loop_calls: callsAtDump, elapsed_s: (Date.now() - t0) / 1000,
        lcd_base: null, files: {},
    };
    for (const [name, kind, base, len] of ranges) {
        let buf;
        if (kind === "ram") buf = be(ram, base >>> 1, len >>> 1);
        else buf = be(rom, base >>> 1, len >>> 1);
        const f = path.join(OUT, `${tag}_${name}.bin`);
        fs.writeFileSync(f, buf);
        manifest.files[name] = { range: `0x${base.toString(16)}-0x${(base + len - 1).toString(16)}`, sha256: sha(buf) };
    }
    const lcdSel = rwNow()(0x700017) & 3;
    manifest.lcd_base = (0x4c00 + 0x1000 * lcdSel).toString(16);
    // framebuffer snapshot for eyeballing (240x128, 1bpp MSB-first, stride 30 B)
    const fbBase = parseInt(manifest.lcd_base, 16);
    const ppm = Buffer.alloc(240 * 128 * 3);
    for (let y = 0; y < 128; y++) {
        for (let x = 0; x < 240; x++) {
            const byte = ram[((fbBase + y * 30 + (x >> 3)) >>> 1)];
            const bit = (byte >> (7 - (x & 7))) & 1;
            const v = bit ? 0 : 255, o = (y * 240 + x) * 3;
            ppm[o] = ppm[o + 1] = ppm[o + 2] = v;
        }
    }
    const pf = path.join(OUT, `${tag}_fb.ppm`);
    fs.writeFileSync(pf, Buffer.concat([Buffer.from(`P6\n240 128\n255\n`), ppm]));
    fs.writeFileSync(path.join(OUT, `${tag}.manifest.json`), JSON.stringify(manifest, null, 2));
    console.log(`[dump] ${tag} pc=${manifest.pc} sr=${manifest.sr} calls=${manifest.main_loop_calls} lcd=${manifest.lcd_base}`);
}

let callsAtDump = 0, loopCalls = 0;

// ------------------------------------------------------------- S1 capture ---
dumpState("S1", [
    ["flash_4mb", "rom", 0x000000, 0x400000],
    ["ram_256k", "ram", 0x000000, 0x040000],
]);

// ------------------------------------------------------------ fast-forward ---
const BANNER_PC_LO = 0x95c5e0, BANNER_PC_HI = 0x95c5ff; // first-boot format loop
const HOME_PC = 0x962226;                               // idle node-walk
let bannerHits = 0, homeHits = 0, stoppedSeen = 0;

// Instruction-granular derail finder: wrap the LIVE handler table (exported
// `t` — the same object the interpreter dispatches through) so that any
// instruction executing outside the valid 89T maps (flash $800000-$BFFFFF,
// RAM $000000-$03FFFF, I/O $600000-$71FFFF) is logged with its source pc.
const weird = [];
const pcTail = new Int32Array(4096); let pcTailI = 0;
{
    const t = emu.t();
    let lastPc = pcOf();
    const okPc = (p) => (p >= 0x800000 && p < 0xc00000) || p < 0x40000 ||
                        (p >= 0x600000 && p < 0x720000);
    for (let i = 0; i < 65536; i++) {
        const h = t[i];
        if (typeof h !== "function") continue;
        t[i] = function () {
            const pc = pcOf();
            pcTail[pcTailI++ & 4095] = pc | 0;
            if (!okPc(pc) && weird.length < 24)
                weird.push(`$${lastPc.toString(16)} -> $${pc.toString(16)} sr=$${srOf().toString(16)}`);
            lastPc = pc;
            return h.apply(this, arguments);
        };
    }
}
let s2done = false, s3done = false;
const BUDGET_CALLS = 20_000_000;                        // ~256G instructions worst case
const pcRing = new Array(32).fill(0); let pcRingI = 0;

process.stderr.write("fast-forwarding (banner loop >= $95C5E0, HOME at $962226)...\n");
for (loopCalls = 1; loopCalls <= BUDGET_CALLS; loopCalls++) {
    try {
        pcRing[pcRingI++ & 31] = pcOf();
        emu.emu_main_loop();
    } catch (e) {
        if (e === "STOP") { stoppedSeen++; if (stoppedSeen > 100000) {
            console.error(`\nFATAL: CPU stuck in STOP at pc=$${pcOf().toString(16)}`); process.exit(4);
        } continue; }
        console.error(`\nFATAL: js exception at pc=$${pcOf().toString(16)}:`, e && e.message || e);
        console.error("first invalid-map instruction executions (from -> to):");
        for (const w of weird) console.error("   " + w);
        console.error("last 64 executed pcs (instruction-granular):");
        {
            const out = [];
            for (let k = 64; k >= 1; k--)
                out.push("$" + (pcTail[(pcTailI - k) & 4095] >>> 0).toString(16));
            console.error(out.join(" "));
        }
        console.error("pc ring (oldest->newest, per main_loop call):",
            pcRing.map(p => "$" + p.toString(16)).join(" "));
        try {
            emu.print_status();
            const rl = emu.rl(), a7 = emu.a7();
            console.error(`stack @a7=$${a7.toString(16)}:`,
                [0, 4, 8, 12, 16].map(o => "$" + rl(a7 + o).toString(16)).join(" "));
            console.error("instr word at pc:", "$" + rl(pcOf() & ~1).toString(16));
        } catch (e2) { console.error("(diag failed:", e2.message, ")"); }
        fs.writeFileSync(path.join(HERE, "crash.json"), JSON.stringify({
            pc: pcOf().toString(16), sr: srOf().toString(16), calls: loopCalls,
            pc_ring: pcRing.map(p => p.toString(16)),
            weird,
        }, null, 2));
        process.exit(3);
    }

    const pc = pcOf();
    if (!s2done && pc >= BANNER_PC_LO && pc <= BANNER_PC_HI) {
        if (++bannerHits === 100) { dumpState("S2", [
            ["ram_vectors_0400", "ram", 0x000000, 0x000400],
            ["ram_fb_1000", "ram", 0x004c00, 0x001000],
            ["ram_osvars_0200", "ram", 0x005b00, 0x000200],
            ["flash_12000_140000", "rom", 0x012000, 0x140000],
        ]); s2done = true; }
    } else bannerHits = 0;

    if (pc === HOME_PC) {
        if (++homeHits === 500) {
            dumpState("S3", [
                ["ram_vectors_0400", "ram", 0x000000, 0x000400],
                ["ram_fb_1000", "ram", 0x004c00, 0x001000],
                ["ram_osvars_0200", "ram", 0x005b00, 0x000200],
                ["flash_4mb", "rom", 0x000000, 0x400000],
            ]); s3done = true; break;
        }
    } else homeHits = 0;

    if ((loopCalls & 0x3fff) === 0) {
        process.stderr.write(`  call ${loopCalls} pc=$${pc.toString(16)} ` +
            `(banner ${bannerHits}, home ${homeHits}, stop ${stoppedSeen})   `);
        if ((Date.now() - t0) > maxMinutes * 60_000) {
            console.error(`\nFATAL: ${maxMinutes} min budget exhausted before HOME`); process.exit(6);
        }
    }
}

if (s3done) {
    fs.writeFileSync(path.join(OUT, "HOME_REACHED"), new Date().toISOString());
    console.log(`\nHOME REACHED in ${((Date.now() - t0) / 1000).toFixed(0)} s — gospel behavioral proof OK`);
    process.exit(0);
}
console.error("\nFATAL: call budget exhausted before HOME (banner captured: " + s2done + ")");
process.exit(6);
