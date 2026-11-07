// freedyn2 in the browser: YAML -> .fdm (fdgen in Pyodide) -> simulation (freedyn2.wasm) -> viewer.
//
// Meshes: the files a model reads live in Pyodide's file system next to it (fdgen.web.add_file);
// the viewer page carries them (as freedyn2 view --export does).
//
// Files next to this page (tools/build_web_demo.py): fd_wasm.mjs, freedyn2.wasm, fdgen.zip (the
// Python package fdgen with the viewer and the schema), examples.json. Pyodide from its CDN, or
// from ?pyodide=<url of a Pyodide distribution directory>.

import { loadFreedyn2 } from "./fd_wasm.mjs";

const PYODIDE_CDN = "https://cdn.jsdelivr.net/pyodide/v0.27.7/full/";
const $ = (id) => document.getElementById(id);
const ui = { example: $("example"), yaml: $("yaml"), run: $("run"), download: $("download"),
  open: $("open"), file: $("file"), files: $("files"), status: $("status"), timings: $("timings"),
  view: $("view"), aside: document.querySelector("aside") };
const OWN = "(your files)";

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

  const b64 = (text) => Uint8Array.from(atob(text), (c) => c.charCodeAt(0));
  const showFiles = () => {
    const list = web.files().toJs();
    ui.files.replaceChildren(...list.map((f) => Object.assign(document.createElement("li"), { textContent: f })));
  };
  for (const name of Object.keys(examples)) ui.example.add(new Option(name, name));
  const pick = () => {
    const ex = examples[ui.example.value];
    if (!ex) return;
    web.clear_files();
    for (const [path, data] of Object.entries(ex.files)) web.add_file(path, b64(data));
    ui.yaml.value = ex.yaml;
    showFiles();
  };
  ui.example.addEventListener("change", () => { pick(); ui.run.click(); });
  ui.example.value = examples.two_links ? "two_links" : Object.keys(examples)[0];
  pick();

  // Own files: a YAML model with its meshes, or a zip with all of them; meshes alone are added.
  async function openFiles(list) {
    list = [...list];
    const isModel = (f) => /\.(ya?ml|zip)$/i.test(f.name);
    if (!list.length) return;
    try {
      if (list.some(isModel)) web.clear_files();
      let yaml = null;
      for (const f of list) {
        const data = new Uint8Array(await f.arrayBuffer());
        if (/\.zip$/i.test(f.name)) {
          const models = web.add_zip(data).toJs();
          if (models.size > 1) throw new Error(`${f.name}: more than one YAML model in the zip`);
          for (const text of models.values()) yaml = text;
        } else if (/\.ya?ml$/i.test(f.name)) {
          yaml = new TextDecoder().decode(data);
        } else {
          web.add_file(f.name, data);  // next to the model: the YAML names it by its file name
        }
      }
      if (yaml !== null) ui.yaml.value = yaml;
      if (![...ui.example.options].some((o) => o.value === OWN)) ui.example.add(new Option(OWN, OWN), 0);
      ui.example.value = OWN;
      showFiles();
      if (yaml !== null) ui.run.click();
      else status(`added ${list.length} file(s); simulate to see them`);
    } catch (e) {
      status(String(e.message || e).trim().split("\n").filter(Boolean).pop(), "error");
    }
  }
  ui.open.addEventListener("click", () => ui.file.click());
  ui.file.addEventListener("change", () => { openFiles(ui.file.files); ui.file.value = ""; });
  ui.aside.addEventListener("dragover", (e) => { e.preventDefault(); ui.aside.classList.add("drop"); });
  ui.aside.addEventListener("dragleave", (e) => { if (!ui.aside.contains(e.relatedTarget)) ui.aside.classList.remove("drop"); });
  ui.aside.addEventListener("drop", (e) => { e.preventDefault(); ui.aside.classList.remove("drop"); openFiles(e.dataTransfer.files); });

  ui.download.addEventListener("click", () => {
    const stem = /^model:\s*(\S+)/m.exec(ui.yaml.value)?.[1] || "model";
    const withFiles = web.files().toJs().length > 0;  // the model and its meshes as one zip
    const blob = withFiles
      ? new Blob([web.bundle(ui.yaml.value, `${stem}.yaml`).toJs()], { type: "application/zip" })
      : new Blob([ui.yaml.value], { type: "application/yaml" });
    const a = Object.assign(document.createElement("a"), {
      href: URL.createObjectURL(blob), download: `${stem}.${withFiles ? "zip" : "yaml"}` });
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
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

  ui.run.disabled = ui.download.disabled = ui.open.disabled = false;
  status(`ready (${boot})`);
  ui.run.click();  // the first example right away
  window.freedyn2Demo = { py, web, fd };  // for the console and tests
}

boot().catch((e) => {
  status(`cannot start: ${e.message || e}`, "error");
  console.error(e);
});
