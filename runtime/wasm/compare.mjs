// Simulate .fdm models with freedyn2.wasm and with the native freedyn2_run; compare the results.
//
//   node runtime/wasm/compare.mjs build-wasm/freedyn2.wasm build/freedyn2_run build/generated/*.fdm
//
// Models with a plugin line (expression forces) are skipped: WebAssembly has no model plugins.

import { execFileSync } from "node:child_process";
import { mkdtempSync, readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { basename, join } from "node:path";
import { loadFreedyn2 } from "./fd_wasm.mjs";

const [wasmPath, nativeRun, ...models] = process.argv.slice(2);
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
  execFileSync(nativeRun, [model, "-o", csv, "--linear-solver", "sparselu"], { stdio: "ignore" });
  const nativeMs = performance.now() - t0;
  const lines = readFileSync(csv, "utf8").trim().split("\n");
  const native = lines.slice(1).map((l) => l.split(",").map(Number));

  t0 = performance.now();
  const r = fd.run(text);
  const wasmMs = performance.now() - t0;

  if (r.rows !== native.length || r.cols !== native[0].length) {
    console.log(`${basename(model)}: DIFFERENT SHAPE wasm ${r.rows}x${r.cols}, native ${native.length}x${native[0].length}`);
    worst = Infinity;
    continue;
  }
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
    `  wasm ${wasmMs.toFixed(0).padStart(5)} ms`);
}
console.log(`largest relative difference: ${worst.toExponential(1)}`);
process.exit(worst < 1e-6 ? 0 : 1);
