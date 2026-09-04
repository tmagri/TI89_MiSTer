#!/usr/bin/env node
// cdp_probe.mjs — ad-hoc CDP evaluator for the persistent Chrome on :9223.
// Usage: node cdp_probe.mjs '<expression>' [url-to-navigate-first]
//        node cdp_probe.mjs --file probe.js [url]
// Connects to the FIRST page target, optionally navigates, evaluates, prints.
import { readFileSync } from "node:fs";

const args = process.argv.slice(2);
const useFile = args[0] === "--file";
const expr = useFile ? readFileSync(args[1], "utf8") : args[0];
const nav = !useFile ? args[1] : args[2];

const tabs = await (await fetch("http://127.0.0.1:9223/json/list")).json();
const page = tabs.find(t => t.type === "page");
const ws = new WebSocket(page.webSocketDebuggerUrl);
await new Promise(r => ws.onopen = r);
let id = 0; const pend = new Map();
ws.onmessage = ev => { const m = JSON.parse(ev.data);
    if (m.id && pend.has(m.id)) { pend.get(m.id)(m); pend.delete(m.id); } };
const cdp = (method, params = {}) => new Promise((res, rej) => {
    const i = ++id; pend.set(i, m => m.error ? rej(new Error(m.error.message)) : res(m.result));
    ws.send(JSON.stringify({ id: i, method, params })); });
if (nav) { await cdp("Page.enable"); await cdp("Page.navigate", { url: nav });
    await new Promise(r => setTimeout(r, 6000)); }
const r = await cdp("Runtime.evaluate", { expression: expr, returnByValue: true, awaitPromise: true });
if (r.exceptionDetails)
    console.log("EXCEPTION:", r.exceptionDetails.exception?.description || r.exceptionDetails.text);
else
    console.log(typeof r.result.value === "string" ? r.result.value : JSON.stringify(r.result.value, null, 1));
process.exit(0);
