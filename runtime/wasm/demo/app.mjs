// freedyn2 in the browser: YAML -> .fdm (fdgen in Pyodide) -> simulation (freedyn2.wasm) -> viewer.
//
// Files next to this page (tools/build_web_demo.py): fd_wasm.mjs, freedyn2.wasm, fdgen.zip (the
// Python package fdgen with the viewer and the schema), examples.json. Pyodide from its CDN, or
// from ?pyodide=<url of a Pyodide distribution directory>.

import { loadFreedyn2 } from "./fd_wasm.mjs";

const PYODIDE_CDN = "https://cdn.jsdelivr.net/pyodide/v0.27.7/full/";
const $ = (id) => document.getElementById(id);
const ui = { example: $("example"), yaml: $("yaml"), run: $("run"), download: $("download"),
  status: $("status"), timings: $("timings"), view: $("view") };

function status(text, kind = "") {
  ui.status.textContent = text;
  ui.status.className = kind;
}
const frame = () => new Promise((r) => requestAnimationFrame(() => setTimeout(r, 0)));  // let the page paint
const ms = (t0) => `${(performance.now() - t0).toFixed(0)} ms`;

async function boot() {
  const t0 = performance.now();
  const base = new URLSearchParams(location.search).get("pyodide") || PYODIDE_CDN;
  const [{ loadPyodide }, examples, fdgenZip, fd] = await Promise.all([
    import(new URL("pyodide.mjs", new URL(base, location.href)).href),
    fetch("./examples.json").then((r) => r.json()),
    fetch("./fdgen.zip").then((r) => r.arrayBuffer()),
    loadFreedyn2(new URL("./freedyn2.wasm", import.meta.url)),
  ]);
  const py = await loadPyodide({ indexURL: new URL(base, location.href).href });
  await py.loadPackage(["numpy", "pyyaml", "jsonschema"]);
  py.unpackArchive(fdgenZip, "zip", { extractDir: "/lib/freedyn2" });
  py.runPython("import sys; sys.path.insert(0, '/lib/freedyn2')");
  const web = py.pyimport("fdgen.web");
  const boot = ms(t0);

  for (const name of Object.keys(examples)) ui.example.add(new Option(name, name));
  const pick = () => { ui.yaml.value = examples[ui.example.value]; };
  ui.example.addEventListener("change", pick);
  ui.example.value = examples.double_pendulum ? "double_pendulum" : Object.keys(examples)[0];
  pick();

  ui.download.addEventListener("click", () => {
    const name = (/^model:\s*(\S+)/m.exec(ui.yaml.value)?.[1] || "model") + ".yaml";
    const a = Object.assign(document.createElement("a"), {
      href: URL.createObjectURL(new Blob([ui.yaml.value], { type: "application/yaml" })), download: name });
    a.click();
    URL.revokeObjectURL(a.href);
  });

  let pageUrl = null;
  ui.run.addEventListener("click", async () => {
    ui.run.disabled = true;
    try {
      status("translating the model ...", "live");
      await frame();
      let t = performance.now();
      const fdm = web.prepare(ui.yaml.value);
      const tPrepare = ms(t);

      status("simulating ...", "live");
      await frame();
      t = performance.now();
      const r = fd.run(fdm);
      const tRun = ms(t);

      status("building the view ...", "live");
      await frame();
      t = performance.now();
      const lines = [r.names.join(",")];
      for (let i = 0; i < r.rows; i++) lines.push(Array.from(r.data.subarray(i * r.cols, (i + 1) * r.cols)).join(","));
      const html = web.page(lines.join("\n") + "\n");
      if (pageUrl) URL.revokeObjectURL(pageUrl);
      pageUrl = URL.createObjectURL(new Blob([html], { type: "text/html" }));
      ui.view.replaceChildren(Object.assign(document.createElement("iframe"), { src: pageUrl, title: "viewer" }));
      const tView = ms(t);

      status(`done: ${r.steps} steps${r.rejected ? ` (${r.rejected} rejected)` : ""}, ${r.rows} output rows`);
      ui.timings.textContent = `start (python, solver)  ${boot}\nyaml -> model file      ${tPrepare}\n` +
        `simulation (wasm)       ${tRun}\nviewer page             ${tView}`;
    } catch (e) {
      // a Python error: its last line is the message (fdgen.errors.ModelError and the like)
      const msg = String(e.message || e).trim().split("\n").filter(Boolean).pop();
      status(msg.replace(/^[\w.]*(Error|Exception): /, ""), "error");
    } finally {
      ui.run.disabled = false;
    }
  });

  ui.run.disabled = ui.download.disabled = false;
  status(`ready (${boot})`);
  ui.run.click();  // the first example right away
  window.freedyn2Demo = { py, web, fd };  // for the console and tests
}

boot().catch((e) => {
  status(`cannot start: ${e.message || e}`, "error");
  console.error(e);
});
