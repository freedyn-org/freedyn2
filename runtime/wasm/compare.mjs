// Simulate .fdm models with freedyn2.wasm and with the native freedyn2_run; compare the results.
//
//   node runtime/wasm/compare.mjs build-wasm/freedyn2.wasm build/freedyn2_run build/generated
//
// Models: .fdm files or directories (all .fdm files in them; no shell globbing needed, e.g. in
// PowerShell). The native runtime: on Windows e.g. build/Release/freedyn2_run.exe.
//
// Models with a plugin line (expression forces) are skipped: WebAssembly has no model plugins.
//
// Fails if the number of rows or steps differs, or if a relative difference exceeds --rtol
// (default 1e-3). The native runtime uses the platform's math library (sin, cos, atan), the
// module its own; they can differ in the last bit, which a chaotic model amplifies (double
// pendulum against MSVC: 3.4e-5 after 10 s; against glibc: 1e-16). The module itself gives the
// same bits everywhere: the printed fingerprint (SHA-256 of the result) is the same on every
// platform and in every browser.

import { execFileSync } from "node:child_process";
import { createHash } from "node:crypto";
import { mkdtempSync, readdirSync, readFileSync, statSync } from "node:fs";
import { tmpdir } from "node:os";
import { basename, join } from "node:path";
import { loadFreedyn2 } from "./fd_wasm.mjs";

const argv = process.argv.slice(2);
let rtol = 1e-3;
const r_ = argv.indexOf("--rtol");
if (r_ >= 0) rtol = Number(argv.splice(r_, 2)[1]);
const [wasmPath, nativeRun, ...args] = argv;
if (!nativeRun || !args.length) {
  console.error("usage: node compare.mjs freedyn2.wasm freedyn2_run[.exe] model.fdm|directory ...");
  process.exit(2);
}
const models = args.flatMap((a) =>
  statSync(a).isDirectory() ? readdirSync(a).filter((f) => f.endsWith(".fdm")).sort().map((f) => join(a, f)) : [a]);
const fd = await loadFreedyn2(readFileSync(wasmPath));
const work = mkdtempSync(join(tmpdir(), "fd-wasm-"));
let worst = 0;

for (const model of models) {
  const text = readFileSync(model, "utf8");
  if (/^plugin\s/m.test(text)) {
    console.log(`${basename(model)}: skipped (model plugin)`);
    continue;
  }
  const csv = join(work, basename(model, ".fdm") + ".csv");
  let t0 = performance.now();
  const log = execFileSync(nativeRun, [model, "-o", csv, "--linear-solver", "sparselu"], { encoding: "utf8" });
  const nativeMs = performance.now() - t0;
  const nativeSteps = Number(/(\d+) steps/.exec(log)?.[1]);
  const lines = readFileSync(csv, "utf8").trim().split("\n");
  const native = lines.slice(1).map((l) => l.split(",").map(Number));

  t0 = performance.now();
  const r = fd.run(text);
  const wasmMs = performance.now() - t0;

  if (r.rows !== native.length || r.cols !== native[0].length || r.steps !== nativeSteps) {
    console.log(`${basename(model)}: DIFFERENT RUN wasm ${r.rows}x${r.cols}, ${r.steps} steps;` +
      ` native ${native.length}x${native[0].length}, ${nativeSteps} steps`);
    worst = Infinity;
    continue;
  }
  const fingerprint = createHash("sha256").update(new Uint8Array(r.data.buffer)).digest("hex").slice(0, 16);
  // per column: largest difference relative to the column's magnitude
  let rel = 0;
  for (let j = 0; j < r.cols; j++) {
    let scale = 0, diff = 0;
    for (let i = 0; i < r.rows; i++) {
      scale = Math.max(scale, Math.abs(native[i][j]));
      diff = Math.max(diff, Math.abs(r.data[i * r.cols + j] - native[i][j]));
    }
    rel = Math.max(rel, diff / Math.max(scale, 1e-12));
  }
  worst = Math.max(worst, rel);
  console.log(`${basename(model).padEnd(24)} ${String(r.rows).padStart(6)} rows  ${String(r.steps).padStart(6)} steps` +
    `  rel. diff ${rel.toExponential(1).padStart(8)}  native ${nativeMs.toFixed(0).padStart(5)} ms (process)` +
    `  wasm ${wasmMs.toFixed(0).padStart(5)} ms  ${fingerprint}`);
}
console.log(`largest relative difference: ${worst.toExponential(1)} (limit ${rtol})`);
process.exit(worst <= rtol ? 0 : 1);
