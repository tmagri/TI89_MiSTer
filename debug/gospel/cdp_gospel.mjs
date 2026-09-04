#!/usr/bin/env node
// cdp_gospel.mjs — BROWSER-FAITHFUL gospel capture via Chrome DevTools
// Protocol (v2, observer-in-page).
//
// Hard-won constraints this design respects (all empirically established
// 2026-09-05, see gospel/reports/P0_provenance.md):
//   * v12's pause_emulator() is a no-op (its clearInterval is commented out)
//     and resume_emulator() stacks a SECOND emu_main_loop interval — every
//     pause/resume cycle doubles the drivers and derails the OS. NEVER pause.
//   * v12's own loadrom() TIB converter fills flash 0x0000-0x11FFF with
//     0x1400 (garbage certificate area at chip 0x10000) and the OS derails
//     from that image in ANY host. Feeding the image exactly as our
//     rtl/rom_loader.sv synthesizes it (boot mirror @0, FEEDBABE + HWPB,
//     0xFF-erased cert area, payload @ chip 0x12000) boots cleanly.
//   * The page scopes emu/ui/link inside its jQuery-ready closure — the
//     driver replicates loadSimulator()'s wiring at global scope instead.
//   * In-page read-only dumps are safe and atomic (JS event loop).
//
// The observer runs as ONE async in-page evaluate; it stages state dumps
// (base64) in window._gospelDumps and reports progress in
// window._gospelStatus. The node driver only reads status and pulls the
// staged dumps after the run.

import { spawn } from "node:child_process";
import { writeFileSync, mkdirSync } from "node:fs";
import path from "node:path";

const HERE = path.dirname(new URL(import.meta.url).pathname);
const PROJ = path.resolve(HERE, "../..");
const OUT = path.join(HERE, "states");
const PAGE = "http://127.0.0.1:8791/references/ti89-simulator/index.html";
const ROMURL = "/TI89Titanium_OS.89u";
const CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";

const argv = process.argv.slice(2);
const opt = (k, d) => { const i = argv.indexOf(k); return i >= 0 ? Number(argv[i + 1]) : d; };
const MAX_MIN = opt("--minutes", 30);
const CDP_PORT = opt("--port", 9223);

mkdirSync(OUT, { recursive: true });
const t0 = Date.now();
const log = (m) => console.log(`[${((Date.now() - t0) / 1000).toFixed(0)}s] ${m}`);
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

// ------------------------------------------------- serve the project root ---
async function serverUp() {
    try { const r = await fetch(PAGE.replace("index.html", "")); return r.ok; }
    catch { return false; }
}
if (!(await serverUp())) {
    var httpSrv = spawn("python3", ["-m", "http.server", "8791", "--bind", "127.0.0.1"],
        { cwd: PROJ, stdio: "ignore" });
    process.on("exit", () => httpSrv.kill());
}
for (let i = 0; i < 40 && !(await serverUp()); i++) await sleep(500);
if (!(await serverUp())) throw new Error("local http server never came up on :8791");

// ---------------------------------------------------------- launch Chrome ---
const chrome = spawn(CHROME, [
    "--headless=new", `--remote-debugging-port=${CDP_PORT}`,
    "--user-data-dir=/tmp/ti89_gospel_chrome_" + Date.now(), "--no-first-run",
    "--window-size=1280,900", "about:blank",
], { stdio: "ignore" });
process.on("exit", () => chrome.kill());

async function getCdpWs() {
    for (let i = 0; i < 40; i++) {
        try {
            const res = await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`);
            const tabs = await res.json();
            const page = tabs.find(t => t.type === "page");
            if (page) return page.webSocketDebuggerUrl;
        } catch { /* chrome not up yet */ }
        await sleep(500);
    }
    throw new Error("Chrome CDP never came up");
}

const ws = new WebSocket(await getCdpWs());
await new Promise(r => ws.onopen = r);
let msgId = 0;
const pending = new Map();
ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
};
function cdp(method, params = {}) {
    const id = ++msgId;
    return new Promise((resolve, reject) => {
        pending.set(id, (m) => m.error ? reject(new Error(m.error.message)) : resolve(m.result));
        ws.send(JSON.stringify({ id, method, params }));
    });
}
async function evalJs(expression) {
    const r = await cdp("Runtime.evaluate", { expression, returnByValue: true, awaitPromise: true });
    if (r.exceptionDetails) throw new Error("page: " + (r.exceptionDetails.exception?.description || r.exceptionDetails.text));
    return r.result.value;
}

await cdp("Page.enable");
await cdp("Page.navigate", { url: PAGE });
await sleep(6000);

// ------------------------------------------------------ the in-page observer ---
const OBSERVER = `
(async () => {
    window._gospelStatus = { phase: "wiring" };
    const S = window._gospelStatus;
    const b64Of = (a, baseW, lenW) => {
        let s = ""; const total = lenW * 2;
        for (let off = 0; off < total; off += 61440) {
            let chunk = "";
            for (let i = 0; i < 61440 && off + i < total; i += 2) {
                const w = a[baseW + ((off + i) >> 1)];
                chunk += String.fromCharCode((w >> 8) & 255, w & 255);
            }
            s += btoa(chunk);
        }
        return s;
    };
    const stage = (tag, jobs) => {
        window._gospelDumps = window._gospelDumps || {};
        window._gospelDumps[tag] = { manifest: JSON.stringify({
            state: tag, when: new Date().toISOString(),
            pc: emu.pc().toString(16), sr: emu.sr().toString(16),
            a7: (emu.a7() >>> 0).toString(16) }), files: {} };
        for (const [name, kind, base, len] of jobs)
            window._gospelDumps[tag].files[name] = b64Of(emu[kind](), base, len);
    };
    try {
        window.emu = TI68kEmulatorCoreModule(window);
        window.ui  = TI68kEmulatorUIModule(window);
        window.link = TI68kEmulatorLinkModule(window);
        emu.setReset(function () {});
        ui.setEmu(emu); ui.setLink(link);
        emu.setUI(ui); emu.setLink(link);
        link.setEmu(emu); link.setUI(ui);

        S.phase = "loading";
        const resp = await fetch(${JSON.stringify(ROMURL)});
        if (!resp.ok) throw new Error("fetch " + resp.status);
        const buf = new Uint8Array(await resp.arrayBuffer());
        const start = 0x4E;
        const img = new Uint16Array(0x200000).fill(0xFFFF);
        const rd = (o) => (buf[o] << 8) | buf[o + 1];
        for (let i = 0; i < 0x80; i++) img[i] = rd(start + 0x88 + i * 2);
        img[0x80] = 0xFEED; img[0x81] = 0xBABE;
        img[0x82] = 0x0080; img[0x83] = 0x0108;
        const hwpb = [0x0018, 0, 9, 0, 2, 0, 1, 0, 1, 0, 1, 0, 3];
        for (let i = 0; i < hwpb.length; i++) img[0x84 + i] = hwpb[i];
        const pw = (buf.length - start) >> 1;
        for (let i = 0; i < pw; i++) img[0x9000 + i] = rd(start + i * 2);
        emu.setRom(img);
        S.phase = "booting";
        emu.initemu();

        // S1: flash-pristine snapshot (in-page, atomic, no pause)
        S.phase = "S1";
        stage("S1", [["flash_4mb", "rom", 0, 0x200000],
                     ["ram_256k", "ram", 0, 0x20000]]);

        const B_LO = 0x95c5e0, B_HI = 0x95c5ff, HOME = 0x962226;
        let s2 = false;
        const t0 = Date.now(), BUDGET = ${MAX_MIN} * 60000;
        while (Date.now() - t0 < BUDGET) {
            await new Promise(r => setTimeout(r, 400));
            const p = emu.pc() >>> 0;
            const ok = (p >= 0x800000 && p < 0xc00000) || p < 0x40000 ||
                       (p >= 0x600000 && p < 0x720000);
            if (!ok) {
                S.phase = "derailed";
                const a7 = emu.a7() >>> 0, rl = emu.rl(), stack = [];
                for (let o = 0; o < 40; o += 4) stack.push(rl(a7 + o).toString(16));
                S.detail = JSON.stringify({ pc: p.toString(16),
                    sr: emu.sr().toString(16), a7: a7.toString(16), stack });
                return "derailed";
            }
            if (!s2 && p >= B_LO && p <= B_HI) {
                S.phase = "S2";
                stage("S2", [["ram_vectors_0400", "ram", 0, 0x200],
                             ["ram_fb_1000", "ram", 0x2600, 0x800],
                             ["ram_osvars_0200", "ram", 0x2D80, 0x100],
                             ["flash_12000_140000", "rom", 0x9000, 0xA0000]]);
                s2 = true;
                S.phase = "booting";
            }
            if (p === HOME) {
                S.phase = "S3";
                stage("S3", [["ram_vectors_0400", "ram", 0, 0x200],
                             ["ram_fb_1000", "ram", 0x2600, 0x800],
                             ["ram_osvars_0200", "ram", 0x2D80, 0x100],
                             ["flash_4mb", "rom", 0, 0x200000]]);
                window._gospelHome = new Date().toISOString();
                S.phase = "home";
                return "home";
            }
            S.pc = p.toString(16);
        }
        S.phase = "budget";
        return "budget";
    } catch (e) {
        S.phase = "error"; S.detail = String(e && e.message || e);
        return "error";
    }
})()`;

log("launching in-page observer (boots OS, stages dumps; never pauses)…");
const observerPromise = evalJs(OBSERVER).catch(e => "driver-error: " + e.message);

// ------------------------------------------------ status poll + dump pull ---
let lastPhase = "";
let done = false;
while (!done && Date.now() - t0 < (MAX_MIN + 5) * 60000) {
    await sleep(2000);
    let st;
    try { st = await evalJs(`JSON.stringify(window._gospelStatus || {})`); }
    catch (e) { log("status read failed: " + e.message); continue; }
    const s = JSON.parse(st);
    if (s.phase !== lastPhase) {
        log(`phase: ${s.phase}${s.detail ? " — " + s.detail : ""}${s.pc ? " pc=$" + s.pc : ""}`);
        lastPhase = s.phase;
    }
    done = ["home", "derailed", "budget", "error", "driver-error"].includes(s.phase);
}

const phase = (await evalJs(`window._gospelStatus.phase`));
log(`final phase: ${phase}`);

// pull staged dumps (chunked per-file reads; page is idle or dead by now)
const tags = await evalJs(`JSON.stringify(Object.keys(window._gospelDumps || {}))`);
for (const tag of JSON.parse(tags)) {
    const entry = await evalJs(`(function(){ var d = window._gospelDumps["${tag}"];
        return JSON.stringify({ manifest: d.manifest, files: Object.keys(d.files) }); })()`);
    const { manifest, files } = JSON.parse(entry);
    const dir = path.join(OUT, tag === "S1" ? "." : ".");
    for (const name of files) {
        const b64 = await evalJs(`window._gospelDumps["${tag}"].files["${name}"]`);
        writeFileSync(path.join(OUT, `${tag}_${name}.bin`), Buffer.from(b64, "base64"));
    }
    writeFileSync(path.join(OUT, `${tag}.manifest.json`), manifest);
    log(`pulled ${tag}: ${files.join(", ")}`);
}
if (phase === "home") {
    writeFileSync(path.join(OUT, "HOME_REACHED"), new Date().toISOString());
    log("HOME REACHED — gospel behavioral proof OK");
    process.exit(0);
}
process.exit(phase === "derailed" ? 4 : phase === "home" ? 0 : 3);
