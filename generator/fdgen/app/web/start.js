// The start page of the FreeDyn 2 app (fdgen.app.server): open a model, the recent models,
// the examples, the documentation; the update notice and the errors of the app.
"use strict";

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
let status = null;

async function get(url) {
  const r = await fetch(url, { cache: "no-store" });
  return r.json();
}
async function post(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
  const data = await r.json().catch(() => ({ error: `${r.status} ${r.statusText}` }));
  if (!r.ok || data.error) throw new Error(data.error || `${r.status}`);
  return data;
}

function busy(text) { $("busy").textContent = text || ""; $("busy").hidden = !text; }
function showError(text) { $("errors-text").textContent = text; $("errors").hidden = !text; }

// A model is open: the app window opened a window for it; in the browser this page goes there.
function opened(r) {
  busy("");
  if (r.window) { loadRecent(); return; }
  location.href = r.url;
}
async function openPath(path) {
  if (!path) return;
  busy("opening " + path.split(/[\\/]/).pop() + " …");
  try { opened(await post("/api/app/open", { path })); } catch (e) { busy(""); showError(e.message); loadRecent(); }
}

function ago(t) {
  const s = Date.now() / 1000 - t;
  if (s < 90) return "just now";
  if (s < 3600) return `${Math.round(s / 60)} min ago`;
  if (s < 86400) return `${Math.round(s / 3600)} h ago`;
  if (s < 86400 * 14) return `${Math.round(s / 86400)} days ago`;
  return new Date(t * 1000).toLocaleDateString();
}
const kind = (name) => (/\.fds$/i.test(name) ? "FDS" : "YAML");

async function loadRecent() {
  const items = await get("/api/app/recent");
  const box = $("recent");
  $("recent-clear").hidden = !items.length;
  if (!items.length) { box.innerHTML = '<div class="empty">No models yet: open one, or start with an example.</div>'; return; }
  box.innerHTML = "";
  for (const it of items) {
    const d = document.createElement("div");
    d.className = "item" + (it.exists ? "" : " missing");
    d.title = it.exists ? it.path : it.path + " (not found)";
    d.innerHTML = `<span class="kind">${kind(it.name)}</span><div class="main"><div class="name">${esc(it.name)}</div>` +
      `<div class="sub">${esc(it.folder)}</div></div><span class="when">${it.exists ? ago(it.opened) : "not found"}</span>` +
      `<button class="del" title="remove from the list">×</button>`;
    d.addEventListener("click", (ev) => {
      if (ev.target.classList.contains("del")) { post("/api/app/recent", { remove: it.path }).then(loadRecent); return; }
      if (it.exists) openPath(it.path);
    });
    box.appendChild(d);
  }
}

let examples = [];
function drawExamples() {
  const f = $("filter").value.trim().toLowerCase();
  const box = $("examples");
  const shown = examples.filter((e) => !f || (e.name + " " + e.title).toLowerCase().includes(f));
  box.innerHTML = shown.length ? "" : '<div class="empty">no example matches</div>';
  for (const e of shown) {
    const d = document.createElement("div");
    d.className = "item";
    d.title = `copy ${e.file} to the folder above and open it`;
    d.innerHTML = `<div class="main"><div class="name">${esc(e.name)}</div><div class="sub">${esc(e.title)}</div></div>`;
    d.addEventListener("click", async () => {
      busy(`copying ${e.name} …`);
      try { opened(await post("/api/app/example", { name: e.name, folder: $("folder").value.trim() })); }
      catch (err) { busy(""); showError(err.message); }
    });
    box.appendChild(d);
  }
}

const DOC_PAGES = [["Getting started", "getting-started/"], ["Tutorial – tuning a quarter car", "tutorial/"],
  ["The viewer", "viewer/"], ["The model file", "reference/model-file/"], ["Examples", "examples/"],
  ["Working with an assistant", "assistants/"], ["Command line", "cli/"]];

function drawStatus(s) {
  const first = status === null;
  status = s;
  if (first) {
    document.title = s.title;
    $("version").textContent = "version " + s.version;
    $("open-native").hidden = !s.native;
    $("open-browser").hidden = s.native;
    $("folder-browse").hidden = !s.native;
    $("quit").hidden = s.native;
    $("no-runtime").hidden = s.runtime;
    $("folder").value = s.examples_dir;
    $("docs-link").href = s.docs;
    $("doc-links").innerHTML = DOC_PAGES.map(([t, p]) => `<a href="${esc(s.docs + p)}" target="_blank" rel="noopener">${esc(t)}</a>`).join("");
    $("footer").textContent = `freedyn2 · version ${s.version} · ` + (s.update_check ? "checks once a day for a new release (update_check)" : "the check for new releases is off");
  }
  const u = s.update;
  let hidden = true;
  try { hidden = !u || !u.newer || sessionStorage.getItem("fd.update.hide") === u.latest; } catch (e) { hidden = !u || !u.newer; }
  if (!hidden) {
    $("update-text").textContent = `freedyn2 version ${u.latest} is available (you have ${u.current}).`;
    $("update-link").href = u.url || s.releases;
  }
  $("update").hidden = hidden;
  if (s.errors.length) showError(s.errors.map((e) => `${e.time}  ${e.message}`).join("\n"));
}

async function poll() {
  try { drawStatus(await get("/api/app/status")); } catch (e) { /* the app ended */ }
  setTimeout(poll, 4000);
}

$("update-x").addEventListener("click", () => {
  try { sessionStorage.setItem("fd.update.hide", status.update.latest); } catch (e) { /* no storage */ }
  $("update").hidden = true;
});
$("errors-x").addEventListener("click", () => { post("/api/app/errors", { clear: true }); showError(""); });
$("recent-clear").addEventListener("click", () => post("/api/app/recent", { clear: true }).then(loadRecent));
$("filter").addEventListener("input", drawExamples);
$("browse").addEventListener("click", async () => {
  try {
    const r = await post("/api/app/browse", { kind: "model" });
    for (const p of r.paths) await openPath(p);
  } catch (e) { showError(e.message); }
});
$("folder-browse").addEventListener("click", async () => {
  try {
    const r = await post("/api/app/browse", { kind: "folder", start: $("folder").value });
    if (r.paths.length) $("folder").value = r.paths[0];
  } catch (e) { showError(e.message); }
});
$("open-path").addEventListener("click", () => openPath($("path").value.trim().replace(/^"|"$/g, "")));
$("path").addEventListener("keydown", (e) => { if (e.key === "Enter") $("open-path").click(); });
$("quit").addEventListener("click", async () => {
  try { await post("/api/app/quit"); } catch (e) { /* gone */ }
  document.body.innerHTML = '<header><h1 style="line-height:1.5">freedyn2</h1></header><main><section>The app has ended; this tab can be closed.</section></main>';
});

async function upload(file) {
  if (!file) return;
  busy(`opening ${file.name} …`);
  try {
    const text = await file.text();
    opened(await post("/api/app/upload", { name: file.name, text }));
  } catch (e) { busy(""); showError(e.message); }
}
$("upload").addEventListener("change", (e) => upload(e.target.files[0]));
const drop = $("drop");
drop.addEventListener("dragover", (e) => { e.preventDefault(); drop.classList.add("over"); });
drop.addEventListener("dragleave", () => drop.classList.remove("over"));
drop.addEventListener("drop", (e) => { e.preventDefault(); drop.classList.remove("over"); upload(e.dataTransfer.files[0]); });

window.addEventListener("pageshow", () => { busy(""); loadRecent(); });  // back from a viewer
get("/api/app/examples").then((e) => { examples = e; drawExamples(); });
loadRecent();
poll();
