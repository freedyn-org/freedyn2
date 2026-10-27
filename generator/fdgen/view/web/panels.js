// Panels of the viewer that work with the server (freedyn2 view; hidden in an exported view):
// - Tune: numbers of the model file (parameters, and those picked in the inspector) with
//   sliders; the server simulates with them (POST api/overrides) without writing the file.
//   Changed values are pending until computed: the compute state (header) is current, pending
//   (amber: results of the values before, faded), computing (blue, progress, the curves grow)
//   or failed (red). When to compute: adaptive (by itself when the last run took < 1.5 s, else
//   on Enter / the button), always by itself, or only on demand. A new computation stops the
//   one still running (server).
// - Eigenmodes: fdgen.linearize.modes about the current state (GET api/modes), animated; live:
//   they follow the sliders at once (POST api/modes with the values set, about the state of the
//   current run), with the change against the computed values. ⌁ at a slider: the root locus
//   over its range (POST api/locus), a plot pane.
// - Notes to the assistant: text with its context (time, selection, plots, tuned values, mode)
//   and a picture of the view (POST api/notes, fdgen.view.notes); the answers appear here.
// And the report: one HTML file with the checks, tuned values, eigenmodes, notes and pictures.

const $ = (id) => document.getElementById(id);
const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const fmt = (v) => (typeof v === "number" ? String(+v.toPrecision(5)) : String(v));

async function post(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const out = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(out.error || `${url}: ${r.status}`);
  return out;
}

export function tuneLabel(path) {
  // parameters.k -> k; bodies.rod.position.2 -> rod.position.z; bodies.rod.inertia.0 -> rod.inertia.I1
  if (path.startsWith("parameters.")) return path.slice(11);
  const k = path.split(".").slice(1), i = k[k.length - 1];
  if (/^\d$/.test(i)) k[k.length - 1] = k[k.length - 2] === "inertia" ? `I${+i + 1}` : "xyzw"[+i];
  return k.join(".");
}

const sorted = (o) => JSON.stringify(Object.entries(o).sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0)));
const same = (a, b) => sorted(a) === sorted(b);
function diff(a, b) {
  // the paths where the values a and b differ
  const out = {};
  for (const k of new Set([...Object.keys(a), ...Object.keys(b)])) if (a[k] !== b[k]) out[k] = a[k];
  return out;
}

function image(src) {
  return new Promise((ok, fail) => { const i = new Image(); i.onload = () => ok(i); i.onerror = fail; i.src = src; });
}

export class Panels {
  // v: {static, area, state, viewImage, showMode, stopMode, select, setTime, render, model(), results()}
  constructor(v) {
    this.v = v;
    this.overrides = {};  // as the server has them (the values of the current run)
    this.pending = null;  // values changed here and not computed yet (null: none)
    this.server = null;  // the last api/state
    this.timer = null;
    try { $("tune-policy").value = localStorage.getItem("fdview.policy") || "adaptive"; } catch (e) { /* default */ }
    $("tune-policy").addEventListener("change", () => {
      try { localStorage.setItem("fdview.policy", $("tune-policy").value); } catch (e) { /* no storage */ }
      this.maybeCompute();
    });
    window.addEventListener("keydown", (e) => {  // Enter computes pending values (not while typing a note)
      if (e.key !== "Enter" || !this.dirty() || e.target.tagName === "TEXTAREA" || (e.target.type === "text")) return;
      e.preventDefault();
      this.compute();
    });
    this.pinned = [];  // paths picked in the inspector (kept per model in the browser)
    this.notes = [];
    this.notesStamp = null;
    this.modes = null;  // {time, modes}
    this.activeMode = -1;
    if (v.static) for (const id of ["tune-section", "modes-section", "notes-section"]) $(id).style.display = "none";
    $("tune-reset").addEventListener("click", () => this.edit({}));
    $("tune-note").addEventListener("click", () => this.noteWithValues());
    $("modes-run").addEventListener("click", () => this.computeModes());
    $("modes-stop").addEventListener("click", () => v.stopMode());
    $("modes-amp").addEventListener("input", () => { if (v.state.mode) v.state.mode.amplitude = +$("modes-amp").value; });
    $("modes-live").addEventListener("change", () => { this.modesBase = null; if ($("modes-live").checked) this.previewModes(); });
    $("note-send").addEventListener("click", () => this.sendNote());
    $("note-text").addEventListener("keydown", (e) => { if (e.key === "Enter" && (e.ctrlKey || e.metaKey)) this.sendNote(); });
    $("note-text").addEventListener("focus", () => this.showContext());
    $("report").addEventListener("click", () => this.report().catch((e) => alert(`report: ${e.message}`)));
  }

  // ---------- called by the viewer ----------
  update(scene) {
    // a new scene (model or results changed)
    const key = `fdview.tune.${scene.model}`;
    if (this.key !== key) {
      this.key = key;
      try { this.pinned = JSON.parse(localStorage.getItem(key) || "[]"); } catch (e) { this.pinned = []; }
      this.modes = null;
    }
    if (this.modes && this.modes.version !== this.version) this.modes = null;  // other results
    this.modesBase = null;
    if ($("modes-live").checked && !this.v.static) this.previewModes();
    this.renderTune();
    this.renderModes();
  }

  serverState(s) {
    // api/state on every poll
    this.version = s.version;
    this.server = s;
    if (!same(s.overrides || {}, this.overrides)) {
      this.overrides = s.overrides || {};
      if (this.pending && same(this.pending, this.overrides)) this.pending = null;
      this.renderTune();
    }
    this.renderState();
    if (s.notes !== this.notesStamp) {
      this.notesStamp = s.notes;
      this.loadNotes();
    }
  }

  modeStopped() {
    if (this.activeMode < 0) return;
    this.activeMode = -1;
    this.renderModes();
  }

  pin(path) {
    if (!this.pinned.includes(path)) this.pinned.push(path);
    this.savePins();
    $("tune-details").open = true;
    this.renderTune();
    const row = document.querySelector(`.tune-row[data-path="${CSS.escape(path)}"] input[type=number]`);
    if (row) row.focus();
  }

  // ---------- tune ----------
  values() {
    // path -> value of the model file: the parameters and the numbers of the elements
    const s = this.v.model(), out = new Map();
    if (!s) return out;
    for (const x of s.parameters || []) if (x.value !== undefined) out.set(x.path, x.value);
    for (const e of [...s.bodies, ...s.joints, ...s.forces, ...(s.points || [])]) for (const x of e.tunable || []) if (x.value !== undefined) out.set(x.path, x.value);
    return out;
  }

  savePins() { try { localStorage.setItem(this.key, JSON.stringify(this.pinned)); } catch (e) { /* no storage */ } }

  renderTune() {
    const box = $("tune"), s = this.v.model();
    if (!box || !s) return;
    const base = this.values();
    // parameters given as formulas of others are derived: no slider of their own
    const shown = this.shown();
    const paths = [...new Set([...(s.parameters || []).filter((x) => x.value !== undefined).map((x) => x.path),
      ...this.pinned, ...Object.keys(shown), ...Object.keys(this.overrides)])];
    const n = Object.keys(shown).length;
    const unused = new Set(((s.checks && s.checks.findings) || []).filter((f) => f.code === "unused_parameters").flatMap((f) => f.elements));
    $("tune-badge").hidden = !n;
    $("tune-badge").textContent = n;
    if (n) $("tune-details").open = true;
    box.innerHTML = paths.length ? "" : '<div class="muted small">click an element in the 3D view, then one of its numbers ' +
      "(“tune”) to change it here; the parameters of the model appear by themselves. The model is simulated again " +
      "with the new values; the model file stays as it is.</div>";
    for (const path of paths) {
      const b = base.has(path) ? base.get(path) : 0, cur = path in shown ? shown[path] : b;
      const computed = path in this.overrides ? this.overrides[path] : b;
      const span = Math.max(Math.abs(b), Math.abs(cur)) || 1;
      const [lo, hi] = b > 0 && cur >= 0 ? [0, 2 * span] : b < 0 && cur <= 0 ? [-2 * span, 0] : [-span * 2, span * 2];
      const label = tuneLabel(path);
      const row = document.createElement("div");
      const param = path.startsWith("parameters."), off = param && unused.has(label);
      row.className = "tune-row" + (cur !== b ? " changed" : "") + (cur !== computed ? " pending" : "") + (off ? " unused" : "");
      row.dataset.path = path;
      row.innerHTML = `<span class="name" title="${esc(path)} (model file: ${fmt(b)})${off ? ": not used in any expression" : ""}">${esc(label)}</span>` +
        `<input type="range" min="${lo}" max="${hi}" step="${(hi - lo) / 400}" value="${cur}"><input type="number" step="any" value="${cur}">` +
        `<button class="icon reset" title="value of the model file (${fmt(b)})">↺</button>` +
        `<button class="icon locus" title="root locus: the eigenvalues over the range of this slider">⌁</button>` +
        `<button class="icon sweep" title="fan: simulate 7 values over the range of this slider at once">≋</button>` +
        (param ? "" : '<button class="icon unpin" title="remove from the list">×</button>');
      const num = row.querySelector("input[type=number]"), range = row.querySelector("input[type=range]");
      range.addEventListener("input", () => {  // while dragging: pending at once, computed on release
        num.value = +(+range.value).toPrecision(4);
        row.classList.toggle("pending", +num.value !== computed);
        this.set(path, +num.value, b, false);
      });
      range.addEventListener("change", () => this.set(path, +num.value, b));
      num.addEventListener("change", () => this.set(path, +num.value, b));
      row.querySelector(".reset").addEventListener("click", () => this.set(path, b, b));
      row.querySelector(".locus").addEventListener("click", () => this.rootLocus(path, lo, hi, label, computed));
      row.querySelector(".sweep").addEventListener("click", () => this.startSweep(path, lo, hi));
      if (!param) row.querySelector(".unpin").addEventListener("click", () => {
        this.pinned = this.pinned.filter((p) => p !== path);
        this.savePins();
        if (path in shown) this.set(path, b, b); else this.renderTune();
      });
      box.appendChild(row);
    }
  }

  shown() { return this.pending || this.overrides; }  // the values the sliders show

  dirty() { return this.pending !== null && !same(this.pending, this.overrides); }

  set(path, value, base, render = true) {
    if (!Number.isFinite(value)) return;
    const o = { ...this.shown() };
    if (value === base) delete o[path]; else o[path] = value;
    this.edit(o, render);
  }

  edit(values, render = true) {
    // new values: pending until computed
    this.pending = same(values, this.overrides) ? null : values;
    if (render) this.renderTune();
    this.renderState();
    if (render) this.maybeCompute();
    if ($("modes-live").checked) this.previewModes();
  }

  restore(values) {
    // the values of an earlier run (run strip)
    $("tune-details").open = true;
    for (const path of Object.keys(values)) if (!this.pinned.includes(path) && !path.startsWith("parameters.")) this.pinned.push(path);
    this.savePins();
    this.edit({ ...values });
  }

  lastSeconds() {
    const s = this.server, run = s && (s.runs || []).find((r) => r.id === s.current);
    return run ? run.seconds : Infinity;
  }

  maybeCompute() {
    // by itself, after a short pause (the next change cancels it), as the policy says
    clearTimeout(this.timer);
    if (!this.dirty()) return;
    const policy = $("tune-policy").value;
    if (policy === "auto" || (policy === "adaptive" && this.lastSeconds() < 1.5)) this.timer = setTimeout(() => this.compute(), 300);
  }

  async compute() {
    clearTimeout(this.timer);
    const values = this.shown();
    try {
      const s = await post("api/overrides", values);
      this.overrides = s.overrides || {};
      if (this.pending && same(this.pending, this.overrides)) this.pending = null;
      this.server = { ...(this.server || {}), ...s };
    } catch (e) { alert(e.message); }
    this.renderTune();
    this.renderState();
  }

  renderState() {
    // the compute state in the header, and the page faded while its results are not current
    const box = $("state"), s = this.server;
    if (!box || !s || this.v.static) return;
    box.hidden = false;
    const dirty = this.dirty(), running = s.status === "simulating", failed = s.status === "error";
    const kind = dirty ? "pending" : running ? "running" : failed ? "failed" : "current";
    const was = document.body.className;
    document.body.classList.toggle("stale", dirty || running);
    document.body.classList.toggle("running", running && !dirty);
    if (document.body.className !== was) this.v.render();  // the 3D view with its new filter
    $("stale-note").textContent = dirty ? "results of the values before · Enter computes"
      : "computing … the curves grow";
    const n = dirty ? Object.keys(diff(this.pending, this.overrides)).length : 0;
    const p = s.progress, pct = running && p && p.end > 0 ? Math.min(100, Math.round((100 * p.t) / p.end)) : 0;
    const key = `${kind}|${n}|${pct}|${!!p}`;
    if (this.stateKey === key) return;
    this.stateKey = key;
    box.className = kind;
    box.innerHTML = '<span class="dot"></span>' + (
      dirty ? `<span>${n} change${n > 1 ? "s" : ""} not computed</span><button class="go" title="Enter">Compute ⏎</button>` +
        '<button class="discard" title="back to the values of the current run">discard</button>'
      : running ? (p ? `<span>computing ${pct} %</span><span class="bar" style="width:${pct}%"></span>` : "<span>preparing model …</span>")
      : failed ? '<span>error</span>' : "<span>up to date</span>");
    if (dirty) {
      box.querySelector(".go").addEventListener("click", () => this.compute());
      box.querySelector(".discard").addEventListener("click", () => this.edit({ ...this.overrides }));
    }
    this.v.area.draw();  // faded or not
  }

  noteWithValues() {
    const o = Object.entries(this.shown());
    if (!o.length) return;
    $("notes-details").open = true;
    $("note-text").value = "Please take these values into the model file:\n" + o.map(([k, v]) => `${k}: ${fmt(v)}`).join("\n");
    $("note-text").focus();
  }

  // ---------- eigenmodes ----------
  async previewModes() {
    // the eigenmodes for the values set, about the state of the current run; one request at a
    // time, the latest values win
    if (this.previewBusy) { this.previewAgain = true; return; }
    this.previewBusy = true;
    try {
      do {
        this.previewAgain = false;
        const t = this.v.state.time, shown = { ...this.shown() };
        if (!this.modesBase) this.modesBase = await post("api/modes", { t, overrides: this.overrides });
        const data = same(shown, this.overrides) ? this.modesBase : await post("api/modes", { t, overrides: shown });
        this.modes = { ...data, version: this.version, preview: !same(shown, this.overrides) };
        this.renderModes();
      } while (this.previewAgain && $("modes-live").checked);
    } catch (e) {
      $("modes-info").textContent = `no eigenmodes: ${e.message}`;
    }
    this.previewBusy = false;
  }

  async startSweep(path, lo, hi, n = 7) {
    // the fan: n values over the slider's range, simulated at once (the other values as computed)
    if (lo === 0 && hi > 0) lo = hi / 40;
    const values = Array.from({ length: n }, (_, k) => +(lo + ((hi - lo) * k) / (n - 1)).toPrecision(6));
    try {
      await post("api/sweep", { path, values });
      if (!this.v.area.panes.some((p) => p.mode === "kpi")) this.v.area.addKpi();
    } catch (e) { alert(`sweep: ${e.message}`); }
  }

  async rootLocus(path, lo, hi, label, current) {
    // the eigenvalues for 11 values over the slider's range and the current one: a plot pane
    if (lo === 0 && hi > 0) lo = hi / 40;  // a stiffness of 0 has no oscillation to show
    const values = [...new Set([...Array.from({ length: 11 }, (_, k) => lo + ((hi - lo) * k) / 10), current])].sort((a, b) => a - b);
    $("modes-details").open = true;
    $("modes-info").textContent = `root locus of ${label}: ${values.length} linearizations …`;
    try {
      const data = await post("api/locus", { path, values, t: this.v.state.time });
      this.v.area.addLocus({ ...data, label, current });
      $("modes-info").textContent = `root locus of ${label} below the plots (about the state of the current run)`;
    } catch (e) {
      $("modes-info").textContent = `no root locus: ${e.message}`;
    }
  }

  async computeModes() {
    const t = this.v.state.time;
    $("modes-info").textContent = "computing …";
    try {
      const r = await fetch(`api/modes?t=${t}`, { cache: "no-store" });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || r.status);
      this.modes = { ...data, version: this.version };
      this.activeMode = -1;
    } catch (e) {
      this.modes = null;
      $("modes-info").textContent = `no eigenmodes: ${e.message}`;
      $("modes-list").innerHTML = "";
      return;
    }
    this.renderModes();
  }

  renderModes() {
    const box = $("modes-list");
    if (!box) return;
    if (!this.modes) { box.innerHTML = ""; return; }
    const m = this.modes, base = m.preview && this.modesBase ? this.modesBase.modes : null;
    $("modes-info").classList.toggle("preview", !!m.preview);
    $("modes-info").textContent = m.preview
      ? `preview for the values set, about the state of the current run at t = ${fmt(m.time)} s (compute for the exact one)`
      : m.modes.length
      ? `about t = ${fmt(m.time)} s · click: animate (not to time scale) · meaningful in an equilibrium or a steady motion`
      : `about t = ${fmt(m.time)} s: no eigenvalues other than 0 (no stiffness in this state?)`;
    box.innerHTML = "";
    m.modes.forEach((x, k) => {
      const d = document.createElement("div");
      if (k === this.activeMode) d.className = "on";
      const osc = x.eigenvalue[1] > 1e-9;
      const was = base && base[k];  // the same place in the list for the computed values: a hint, not a tracking
      const change = was && osc && Math.abs(x.frequency - was.frequency) > 1e-3 * Math.max(was.frequency, 1e-9)
        ? `<span class="was ${x.frequency > was.frequency ? "up" : "down"}">${x.frequency > was.frequency ? "▲" : "▼"} ${fmt(was.frequency)}</span>` : "";
      d.innerHTML = `<span>${k + 1}</span><span>${osc ? fmt(x.frequency) + " Hz" : "λ = " + fmt(x.eigenvalue[0]) + " 1/s"}</span>` +
        `<span>${osc ? "ζ = " + (100 * x.damping).toFixed(1) + " %" : x.eigenvalue[0] > 0 ? "unstable" : "non-oscillating"}</span>` + change;
      d.title = `eigenvalue ${fmt(x.eigenvalue[0])} ${x.eigenvalue[1] >= 0 ? "+" : "-"} ${fmt(Math.abs(x.eigenvalue[1]))} i`;
      d.addEventListener("click", () => {
        this.v.showMode(x, m.time, +$("modes-amp").value);
        this.activeMode = k;
        this.renderModes();
      });
      box.appendChild(d);
    });
  }

  // ---------- notes ----------
  context() {
    const v = this.v, c = { time: +v.state.time.toFixed(6) };
    if (v.state.selection) c.selection = v.state.selection.name;
    const ch = v.area.selected();
    if (ch.length) c.channels = [...new Set(ch)];
    if (v.area.measure && v.area.cursors.every((x) => x !== null)) c.cursors = [...v.area.cursors];
    if (Object.keys(this.overrides).length) c.overrides = { ...this.overrides };
    if (v.state.mode) c.mode = { frequency: v.state.mode.mode.frequency, damping: v.state.mode.mode.damping };
    return c;
  }

  showContext() {
    const c = this.context(), parts = [`t = ${fmt(c.time)} s`];
    if (c.selection) parts.push(`selected ${c.selection}`);
    if (c.channels) parts.push(`${c.channels.length} plotted channels`);
    if (c.overrides) parts.push(`${Object.keys(c.overrides).length} tuned values`);
    if (c.mode) parts.push(`mode ${fmt(c.mode.frequency)} Hz`);
    $("note-context").textContent = "goes along: " + parts.join(" · ");
  }

  async snapshot() {
    // the 3D view with the plots below it, as one PNG data URL
    const parts = [await image(this.v.viewImage())];
    const p = this.v.area.png();
    if (p) parts.push(await image(p));
    const W = Math.max(...parts.map((i) => i.width)), H = parts.reduce((s, i) => s + i.height, 0);
    const c = document.createElement("canvas");
    c.width = W; c.height = H;
    const g = c.getContext("2d");
    g.fillStyle = "#fff"; g.fillRect(0, 0, W, H);
    let y = 0;
    for (const i of parts) { g.drawImage(i, 0, y); y += i.height; }
    return c.toDataURL("image/png");
  }

  async sendNote() {
    const text = $("note-text").value.trim();
    if (!text) { $("note-text").focus(); return; }
    $("note-send").disabled = true;
    try {
      const body = { text, context: this.context() };
      if ($("note-image").checked) body.image = await this.snapshot();
      await post("api/notes", body);
      $("note-text").value = "";
      $("note-context").textContent = "sent: the assistant reads it with the MCP tool notes (or the file next to the model)";
      await this.loadNotes();
    } catch (e) { alert(e.message); }
    $("note-send").disabled = false;
  }

  async loadNotes() {
    try { this.notes = await (await fetch("api/notes", { cache: "no-store" })).json(); } catch (e) { return; }
    const open = this.notes.filter((n) => n.status === "open").length;
    $("notes-badge").hidden = !open;
    $("notes-badge").textContent = open;
    const box = $("notes-list");
    box.innerHTML = this.notes.length ? "" : '<div class="muted small">no notes yet</div>';
    for (const n of [...this.notes].reverse()) {
      const d = document.createElement("div");
      d.className = "note " + n.status;
      const c = n.context || {};
      d.innerHTML = `<div class="meta"><span class="st">${n.status === "open" ? "OPEN" : "DONE"}</span><span>#${n.id} · ${esc(n.created)}` +
        `${c.time !== undefined ? " · t = " + fmt(c.time) + " s" : ""}${c.selection ? " · " + esc(c.selection) : ""}</span>` +
        `<span style="margin-left:auto"></span><button class="icon toggle" title="${n.status === "open" ? "mark as done" : "open again"}">${n.status === "open" ? "✓" : "↺"}</button>` +
        `<button class="icon del" title="delete">×</button></div><div class="text">${esc(n.text).replace(/\n/g, "<br>")}</div>` +
        (n.reply ? `<div class="reply">${esc(n.reply)}</div>` : "") +
        (n.image ? `<img loading="lazy" src="api/notes/${esc(n.image)}" alt="view of note ${n.id}">` : "");
      d.querySelector(".toggle").addEventListener("click", () => post(`api/notes/${n.id}`, { done: n.status === "open" }).then(() => this.loadNotes()));
      d.querySelector(".del").addEventListener("click", () => { if (confirm(`delete note ${n.id}?`)) post(`api/notes/${n.id}`, { delete: true }).then(() => this.loadNotes()); });
      const img = d.querySelector("img");
      if (img) img.addEventListener("click", () => window.open(img.src, "_blank"));
      const where = d.querySelector(".meta span:nth-child(2)");  // back to the time and element of the note
      where.className = "link";
      where.addEventListener("click", () => {
        if (c.time !== undefined && this.v.results()) this.v.setTime(c.time);
        if (c.selection) this.v.select(c.selection);
      });
      box.appendChild(d);
    }
  }

  // ---------- report ----------
  async report() {
    const s = this.v.model();
    if (!s) return;
    const results = this.v.results(), c = s.checks || { findings: [], references: [] };
    const sum = c.summary || {};
    const view = await image(this.v.viewImage()), plots = this.v.area.png();
    const rows = (items) => items.map((r) => `<tr>${r.map((x) => `<td>${x}</td>`).join("")}</tr>`).join("");
    const sev = (k, label) => `<span class="sev ${k}">${label}</span>`;
    let html = `<!doctype html><html lang="en"><head><meta charset="utf-8"><title>${esc(s.model)} – FreeDyn 2 report</title>
<style>body{font:14px/1.45 system-ui,sans-serif;max-width:960px;margin:24px auto;padding:0 16px;color:#1d2330}
h1{font-size:22px;margin:0}h2{font-size:15px;margin:22px 0 6px;border-bottom:1px solid #dde1e8;padding-bottom:3px}
.muted{color:#677085}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:3px 8px;border-bottom:1px solid #eef0f4;text-align:left;vertical-align:top}
th{color:#677085;font-weight:600}img{max-width:100%;border:1px solid #dde1e8;border-radius:4px}
.sev{font-size:10px;font-weight:700;color:#fff;border-radius:3px;padding:0 4px}.error,.failed{background:#d9480f}.warning{background:#e67700}
.info{background:#868e96}.passed{background:#2b8a3e}code{font:12px ui-monospace,Consolas,monospace}
@media print{button{display:none}}</style></head><body>
<button onclick="print()" style="float:right">print / PDF</button>
<h1>${esc(s.model)}</h1><div class="muted">FreeDyn 2 · ${new Date().toLocaleString()} · ${sum.bodies ?? "?"} bodies · ${sum.joints ?? "?"} joints ·
${sum.forces ?? "?"} forces · ${sum.degrees_of_freedom ?? "?"} degrees of freedom · total mass ${fmt(sum.total_mass ?? "?")} kg` +
      (results ? ` · results t = ${fmt(results.t[0])} … ${fmt(results.t[results.t.length - 1])} s (${results.n} outputs)` : "") + "</div>";
    html += `<h2>View at t = ${fmt(this.v.state.time)} s</h2><img src="${view.src}" alt="3D view">`;
    if (plots) html += `<h2>Plots</h2><img src="${plots}" alt="plots">`;
    html += "<h2>Checks</h2><table>" + rows([
      ...c.findings.map((f) => [sev(f.severity, f.severity.toUpperCase()), esc(f.message)]),
      ...(c.references || []).map((r) => [sev(r.passed ? "passed" : "failed", r.passed ? "OK" : "FAIL"), esc(r.name) + ": " +
        esc(r.error ? `not checked: ${r.error}` : r.kind === "curve" ? `max deviation ${fmt(r.max_deviation)} (tolerance ${fmt(r.tolerance)})`
          : `${fmt(r.value)} against ${fmt(r.reference)} (tolerance ${fmt(r.tolerance)})`)]),
    ]) + "</table>" + (c.findings.length || (c.references || []).length ? "" : '<div class="muted">no findings</div>');
    const o = Object.entries(this.overrides), base = this.values();
    if (o.length) html += "<h2>Tuned values (not in the model file)</h2><table><tr><th>value</th><th>model file</th><th>tuned</th></tr>" +
      rows(o.map(([k, v]) => [`<code>${esc(k)}</code>`, fmt(base.get(k) ?? "–"), `<b>${fmt(v)}</b>`])) + "</table>";
    if (this.modes && this.modes.modes.length) html += `<h2>Eigenmodes about t = ${fmt(this.modes.time)} s</h2><table><tr><th>#</th><th>frequency</th><th>damping ratio</th><th>eigenvalue</th></tr>` +
      rows(this.modes.modes.map((x, k) => [k + 1, x.eigenvalue[1] > 1e-9 ? fmt(x.frequency) + " Hz" : "–",
        x.eigenvalue[1] > 1e-9 ? (100 * x.damping).toFixed(1) + " %" : "–", `${fmt(x.eigenvalue[0])} ± ${fmt(Math.abs(x.eigenvalue[1]))} i`])) + "</table>";
    const notes = this.notes.filter((n) => n.status === "open" || n.reply);
    if (notes.length) html += "<h2>Notes</h2><table>" + rows(notes.map((n) => [`#${n.id}`, esc(n.created),
      esc(n.text).replace(/\n/g, "<br>") + (n.reply ? `<div class="muted">answer: ${esc(n.reply)}</div>` : ""), n.status])) + "</table>";
    html += "</body></html>";
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([html], { type: "text/html" }));
    a.download = `${s.model}_report.html`;
    a.click();
  }
}
