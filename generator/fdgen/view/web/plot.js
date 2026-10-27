// Plots of result channels: panes below each other with a shared time axis (zoom, pan, hover
// values, cursors A/B with statistics), x-y panes (a channel over another, e.g. a K&C curve) and
// spectra (FFT of the visible time range). Channels come from a provider (results, formulas,
// other runs to compare, references); the area draws, the viewer only feeds it. Other runs are
// drawn in the channel's color with their own dash; in difference mode (provider.delta()) the
// time panes show every run minus the reference run. While a simulation runs, its curve so far
// (provider.live) is drawn over the faded results (provider.stale()). A root locus pane shows
// eigenvalues over the values of one tuned number (addLocus): real part over frequency, the
// values as a color scale, lines of equal damping ratio. A parameter sweep (provider.fan) draws
// a fan of curves in the same color scale; a key figure pane shows a key figure of its channels
// (max, rms, ... over the visible time range) over the swept value.
//
// Mouse: wheel zooms the time axis (shift: the value axis of the pane), drag zooms to a range,
// shift+drag pans, double click resets, click sets the time (with cursors on: moves the nearer
// cursor). Channels are added by click (to the active pane) or dragged onto a pane.

import { unit } from "fdview/channels.js";

const COLORS = ["#2f6fdf", "#e8590c", "#2b8a3e", "#c2255c", "#0c8599", "#a07c00", "#7048e8", "#5c636a",
  "#d9480f", "#1098ad", "#5f3dc4", "#74b816"];
const MAX_FFT = 1 << 16;

export function index(t, time) {
  let a = 0, b = t.length - 1;
  if (!(b >= 0)) return 0;
  if (time <= t[0]) return 0;
  if (time >= t[b]) return b;
  while (b - a > 1) { const m = (a + b) >> 1; if (t[m] <= time) a = m; else b = m; }
  return time - t[a] < t[b] - time ? a : b;
}
// y of the run (t, y) at the times tt, linear (constant outside)
function resample(t, y, tt) {
  const out = new Float64Array(tt.length);
  let j = 0;
  for (let k = 0; k < tt.length; k++) {
    const x = tt[k];
    while (j < t.length - 2 && t[j + 1] < x) j++;
    const a = Math.min(1, Math.max(0, (x - t[j]) / ((t[j + 1] - t[j]) || 1)));
    out[k] = t.length > 1 ? y[j] + a * (y[j + 1] - y[j]) : y[0];
  }
  return out;
}
const minus = (a, b) => { const out = new Float64Array(a.length); for (let k = 0; k < a.length; k++) out[k] = a[k] - b[k]; return out; };
// the color of value i of n (blue: first, red: last), as in the root locus and the fan
export function scaleColor(i, n) { return `hsl(${220 - (n > 1 ? i / (n - 1) : 0) * 220}, 75%, 45%)`; }
export const KPIS = {
  "max": (y, a, b) => { let m = -Infinity; for (let k = a; k < b; k++) m = Math.max(m, y[k]); return m; },
  "min": (y, a, b) => { let m = Infinity; for (let k = a; k < b; k++) m = Math.min(m, y[k]); return m; },
  "max |x|": (y, a, b) => { let m = 0; for (let k = a; k < b; k++) m = Math.max(m, Math.abs(y[k])); return m; },
  "peak-peak": (y, a, b) => KPIS.max(y, a, b) - KPIS.min(y, a, b),
  "mean": (y, a, b) => { let s = 0; for (let k = a; k < b; k++) s += y[k]; return s / (b - a); },
  "rms": (y, a, b) => { let s = 0; for (let k = a; k < b; k++) s += y[k] * y[k]; return Math.sqrt(s / (b - a)); },
  "final": (y, a, b) => y[b - 1],
};
const lower = (t, x) => { let a = 0, b = t.length; while (a < b) { const m = (a + b) >> 1; if (t[m] < x) a = m + 1; else b = m; } return a; };

function niceTicks(lo, hi, n) {
  const raw = (hi - lo) / Math.max(n, 1), mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 5, 10].map((f) => f * mag).find((s) => s >= raw) || raw;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-9; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return out;
}
export function fmt(v) {
  if (!Number.isFinite(v)) return String(v);
  const a = Math.abs(v);
  return a !== 0 && (a >= 1e5 || a < 1e-3) ? v.toExponential(3) : String(+v.toPrecision(5));
}
function range(arrays, pad = 0.06) {
  let lo = Infinity, hi = -Infinity;
  for (const [y, i0, i1] of arrays) for (let k = i0; k < i1; k++) { const v = y[k]; if (v < lo) lo = v; if (v > hi) hi = v; }
  if (!Number.isFinite(lo)) return [-1, 1];
  if (!(hi > lo)) { const m = Math.abs(lo) * 0.1 || 1; lo -= m; hi += m; }
  const p = (hi - lo) * pad;
  return [lo - p, hi + p];
}
function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

// amplitude spectrum of y(t) on [t0, t1]: Hann window, mean removed, single sided
export function spectrum(t, y, t0, t1) {
  const i0 = lower(t, t0), i1 = Math.min(t.length, lower(t, t1) + 1);
  if (i1 - i0 < 8) return null;
  let n = 1; while (n < i1 - i0 && n < MAX_FFT) n <<= 1;
  const dt = (t[i1 - 1] - t[i0]) / (n - 1), re = new Float64Array(n), im = new Float64Array(n);
  let mean = 0;
  for (let k = 0, j = i0; k < n; k++) {  // linear resampling on a uniform grid
    const tk = t[i0] + k * dt;
    while (j < i1 - 2 && t[j + 1] < tk) j++;
    const a = (tk - t[j]) / ((t[j + 1] - t[j]) || 1);
    re[k] = y[j] + Math.min(1, Math.max(0, a)) * (y[j + 1] - y[j]);
    mean += re[k] / n;
  }
  for (let k = 0; k < n; k++) re[k] = (re[k] - mean) * (0.5 - 0.5 * Math.cos((2 * Math.PI * k) / (n - 1)));
  for (let i = 1, j = 0; i < n; i++) {  // radix-2 FFT
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len, wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const ar = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci, ai = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
        re[i + k + len / 2] = re[i + k] - ar; im[i + k + len / 2] = im[i + k] - ai;
        re[i + k] += ar; im[i + k] += ai;
        [cr, ci] = [cr * wr - ci * wi, cr * wi + ci * wr];
      }
    }
  }
  const m = n / 2, f = new Float64Array(m), a = new Float64Array(m);
  for (let k = 0; k < m; k++) { f[k] = k / (n * dt); a[k] = (4 / n) * Math.hypot(re[k], im[k]); }  // 2/n single sided, /0.5 Hann
  return { f, a };
}

export class PlotArea {
  constructor(root, provider, { onseek, onchange }) {
    this.root = root;
    this.p = provider;
    this.onseek = onseek;
    this.onchange = onchange || (() => {});
    this.panes = [];
    this.active = 0;
    this.xRange = null;  // [t0, t1] of the time panes, null: all
    this.time = 0;
    this.hover = null;   // time (or x) under the mouse
    this.measure = false;
    this.cursors = [null, null];
    this.colors = new Map();
    this.box = root.querySelector(".panes");
    this.table = root.querySelector(".measure");
  }

  color(name) {
    if (!this.colors.has(name)) this.colors.set(name, COLORS[this.colors.size % COLORS.length]);
    return this.colors.get(name);
  }

  // ---------- layout ----------
  addPane(mode = "time", xName = null) {
    const pane = { mode, xName, series: [], view: null, el: null };
    this.panes.push(pane);
    this.active = this.panes.length - 1;
    this.build();
    return pane;
  }

  removePane(k) {
    this.panes.splice(k, 1);
    this.active = Math.min(this.active, this.panes.length - 1);
    this.build();
  }

  addChannel(name, k = null) {
    if (!this.panes.length) this.addPane("time");
    const pane = this.panes[k ?? this.active] || this.panes[0];
    if (!pane.series.some((s) => s.name === name)) pane.series.push({ name, hidden: false });
    this.build();
  }

  removeChannel(pane, name) {
    pane.series = pane.series.filter((s) => s.name !== name);
    this.build();
  }

  selected() { return this.panes.flatMap((p) => p.series.map((s) => s.name)); }

  showReference(r) {
    // a curve reference: over its channel (x-y) or over time, in a pane of its own
    const pane = r.x_channel && r.x_channel !== "t" ? this.addPane("xy", r.x_channel) : this.addPane("time");
    pane.series.push({ name: r.channel, hidden: false });
    this.build();
  }

  layout() {
    return { panes: this.panes.filter((p) => p.mode !== "locus").map((p) => ({ mode: p.mode, xName: p.xName, kpi: p.kpi, series: p.series.map((s) => s.name) })) };
  }

  addKpi() {
    // key figures over the swept value, of the channels of the active time plot
    const src = this.panes[this.active];
    const pane = this.addPane("kpi");
    pane.kpi = "max";
    if (src && src.series.length) pane.series = src.series.map((s) => ({ ...s }));
    this.build();
    return pane;
  }

  addLocus(locus) {
    // locus: {label, path, values, eigenvalues: [[[re, im], ...] per value], current, time}; replaces one of the same number
    const old = this.panes.findIndex((p) => p.mode === "locus" && p.locus.path === locus.path);
    if (old >= 0) this.panes.splice(old, 1);
    const pane = { mode: "locus", xName: null, series: [], view: null, el: null, locus };
    this.panes.push(pane);
    this.active = this.panes.length - 1;
    this.build();
    return pane;
  }

  restore(layout, has) {
    this.panes = [];
    for (const p of (layout && layout.panes) || []) {
      const series = p.series.filter(has).map((name) => ({ name, hidden: false }));
      if (series.length || p.mode !== "time") this.panes.push({ mode: p.mode, xName: p.xName, kpi: p.kpi, series, view: null, el: null });
    }
    this.active = 0;
    this.build();
  }

  build() {
    this.box.innerHTML = "";
    this.panes.forEach((pane, k) => {
      const el = document.createElement("div");
      el.className = "pane" + (k === this.active ? " active" : "");
      const L = pane.locus, scale = "linear-gradient(90deg, hsl(220,75%,45%), hsl(165,75%,45%), hsl(110,75%,45%), hsl(55,75%,45%), hsl(0,75%,45%))";
      const title = pane.mode === "locus"
        ? `root locus of ${L.label}: ${fmt(L.values[0])} <i style="display:inline-block;width:70px;height:6px;background:${scale};vertical-align:middle"></i> ` +
          `${fmt(L.values[L.values.length - 1])}${L.current !== null ? ` · ○ ${fmt(L.current)} (now)` : ""} · about t = ${fmt(L.time)} s`
        : pane.mode === "kpi" ? `<select class="kpi" title="key figure over the visible time range">${Object.keys(KPIS).map((k) =>
          `<option${k === pane.kpi ? " selected" : ""}>${k}</option>`).join("")}</select> over the swept value (visible time range)`
        : { time: "", xy: `over ${pane.xName}`, fft: "spectrum" }[pane.mode];
      el.innerHTML = `<div class="pane-head"><span class="chips"></span><span class="mode">${title}</span>` +
        `<button class="close" title="remove this plot">×</button></div><canvas></canvas>` +
        (pane.series.length || pane.mode === "locus" ? "" : `<div class="pane-hint">click or drag channels here</div>`);
      const chips = el.querySelector(".chips");
      for (const s of pane.series) {
        const c = document.createElement("span");
        c.className = "chip" + (s.hidden ? " off" : "");
        const u = unit(s.name);
        c.innerHTML = `<i style="background:${this.color(s.name)}"></i>${s.name}${u ? ` <small>[${u}]</small>` : ""}<b title="remove">×</b>`;
        c.title = "click: hide/show";
        c.addEventListener("click", (e) => {
          if (e.target.tagName === "B") this.removeChannel(pane, s.name);
          else { s.hidden = !s.hidden; this.build(); }
        });
        chips.appendChild(c);
      }
      el.querySelector(".close").addEventListener("click", () => this.removePane(k));
      const kpiSelect = el.querySelector("select.kpi");
      if (kpiSelect) kpiSelect.addEventListener("change", () => { pane.kpi = kpiSelect.value; this.onchange(); this.draw(); });
      el.addEventListener("pointerdown", () => { if (this.active !== k) { this.active = k; this.markActive(); } });
      el.addEventListener("dragover", (e) => { e.preventDefault(); el.classList.add("drop"); });
      el.addEventListener("dragleave", () => el.classList.remove("drop"));
      el.addEventListener("drop", (e) => {
        e.preventDefault();
        const name = e.dataTransfer.getData("text/plain");
        if (name) { this.active = k; this.addChannel(name, k); }
      });
      pane.el = el;
      pane.canvas = el.querySelector("canvas");
      this.wire(pane);
      this.box.appendChild(el);
      new ResizeObserver(() => this.drawPane(pane)).observe(pane.canvas);
    });
    this.root.classList.toggle("empty", !this.panes.length);
    this.onchange();
    this.draw();
  }

  markActive() { this.panes.forEach((p, k) => p.el && p.el.classList.toggle("active", k === this.active)); }

  // ---------- interaction ----------
  wire(pane) {
    const c = pane.canvas;
    let down = null;
    const at = (e) => { const r = c.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
    c.addEventListener("pointerdown", (e) => {
      if (!pane.frame) return;
      const [x, y] = at(e);
      down = { x, y, shift: e.shiftKey, range: this.xRange || pane.frame.full, cursor: this.grabCursor(pane, x) };
      c.setPointerCapture(e.pointerId);
    });
    c.addEventListener("pointermove", (e) => {
      if (!pane.frame) return;
      const [x, y] = at(e), f = pane.frame;
      this.hover = { pane, x: f.xv(x), y: f.yv ? f.yv(y) : null, px: x, py: y };
      if (down) {
        if (down.cursor !== null) { this.cursors[down.cursor] = f.xv(x); this.updateTable(); }
        else if (down.shift && pane.mode === "time") {  // pan
          const dx = f.xv(down.x) - f.xv(x), [a, b] = down.range;
          this.xRange = [a + dx, b + dx];
        } else if (Math.abs(x - down.x) > 3) this.selection = { pane, box: (down.box = [down.x, down.y, x, y]) };
      }
      this.draw();
    });
    c.addEventListener("pointerleave", () => { if (!down) { this.hover = null; this.draw(); } });
    c.addEventListener("pointerup", (e) => {
      if (!down || !pane.frame) return;
      const [x] = at(e), f = pane.frame;
      if (down.box) {
        const [x0, y0, x1, y1] = down.box;
        if (pane.mode === "time" || pane.mode === "fft") {
          const a = f.xv(Math.min(x0, x1)), b = f.xv(Math.max(x0, x1));
          if (pane.mode === "time") this.xRange = [a, b]; else pane.view = { x: [a, b] };
        } else if (f.yv) pane.view = { x: [f.xv(Math.min(x0, x1)), f.xv(Math.max(x0, x1))], y: [f.yv(Math.max(y0, y1)), f.yv(Math.min(y0, y1))] };
      } else if (down.cursor === null && !down.shift && Math.abs(x - down.x) < 4 && pane.mode === "time") {
        const t = f.xv(x);
        if (this.measure) {
          const k = this.cursors[0] === null ? 0 : this.cursors[1] === null ? 1
            : Math.abs(this.cursors[0] - t) < Math.abs(this.cursors[1] - t) ? 0 : 1;
          this.cursors[k] = t;
          this.updateTable();
        } else this.onseek(t);
      }
      down = null;
      this.selection = null;
      this.draw();
    });
    c.addEventListener("dblclick", () => { this.xRange = null; for (const p of this.panes) p.view = null; this.draw(); });
    c.addEventListener("wheel", (e) => {
      if (!pane.frame) return;
      e.preventDefault();
      const [x, y] = at(e), f = pane.frame, k = e.deltaY > 0 ? 1.25 : 0.8;
      if (e.shiftKey && f.yv) {  // value axis of this pane
        const v = f.yv(y), [lo, hi] = (pane.view && pane.view.y) || f.yRange;
        pane.view = { ...(pane.view || {}), y: [v - (v - lo) * k, v + (hi - v) * k] };
      } else if (pane.mode === "time") {
        const v = f.xv(x), [a, b] = this.xRange || f.full;
        this.xRange = [v - (v - a) * k, v + (b - v) * k];
      } else {
        const v = f.xv(x), [a, b] = (pane.view && pane.view.x) || f.xRange;
        pane.view = { ...(pane.view || {}), x: [v - (v - a) * k, v + (b - v) * k] };
      }
      this.draw();
    }, { passive: false });
  }

  grabCursor(pane, x) {
    if (!this.measure || pane.mode !== "time") return null;
    for (const k of [0, 1]) if (this.cursors[k] !== null && Math.abs(pane.frame.xp(this.cursors[k]) - x) < 6) return k;
    return null;
  }

  setMeasure(on) {
    this.measure = on;
    if (!on) this.cursors = [null, null];
    this.updateTable();
    this.draw();
  }

  setTime(time) { this.time = time; this.draw(); }

  // ---------- drawing ----------
  draw() {
    if (this.raf) return;
    this.raf = requestAnimationFrame(() => { this.raf = null; for (const p of this.panes) this.drawPane(p); });
  }

  drawPane(pane) {
    const c = pane.canvas, dpr = window.devicePixelRatio || 1;
    if (!c) return;
    const W = c.clientWidth, H = c.clientHeight;
    if (!W || !H) return;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    pane.frame = null;
    const t = this.p.time();
    if (pane.mode === "locus") this.drawLocus(pane, g, W, H);
    else if (!t || !pane.series.length) return;
    else if (pane.mode === "kpi") this.drawKpi(pane, g, W, H, t);
    else if (pane.mode === "time") this.drawTime(pane, g, W, H, t);
    else if (pane.mode === "xy") this.drawXY(pane, g, W, H, t);
    else this.drawFFT(pane, g, W, H, t);
    const sel = this.selection, f = pane.frame;
    if (sel && sel.pane === pane && f) {  // the range being zoomed to
      const [x0, y0, x1, y1] = sel.box, full = pane.mode !== "xy";
      g.fillStyle = css("--accent"); g.globalAlpha = 0.12;
      g.fillRect(Math.min(x0, x1), full ? f.y0 : Math.min(y0, y1), Math.abs(x1 - x0), full ? f.h : Math.abs(y1 - y0));
      g.globalAlpha = 1;
    }
  }

  axes(g, W, H, xr, lanes, xLabel) {
    // frame, grid, ticks; lanes: [[lo, hi, color]], returns pixel <-> value maps
    const text = css("--muted"), line = css("--line");
    g.font = "11px system-ui, sans-serif";
    const ticks = lanes.map(([lo, hi]) => niceTicks(lo, hi, Math.max(2, Math.floor(H / lanes.length / 34))));
    const labelW = Math.max(30, ...ticks.flat().map((v) => g.measureText(fmt(v)).width)) + 10;
    const x0 = labelW + 4, w = W - x0 - 10, y0 = 6, h = H - y0 - (xLabel ? 32 : 20), hl = h / lanes.length;
    const xp = (v) => x0 + ((v - xr[0]) / (xr[1] - xr[0])) * w, xv = (px) => xr[0] + ((px - x0) / w) * (xr[1] - xr[0]);
    const yps = lanes.map(([lo, hi], a) => (v) => y0 + a * hl + ((hi - v) / (hi - lo)) * (hl - (lanes.length > 1 ? 6 : 0)));
    g.lineWidth = 1; g.textAlign = "right"; g.textBaseline = "middle";
    lanes.forEach(([, , color], a) => {
      for (const v of ticks[a]) {
        const y = yps[a](v);
        if (y < y0 - 1 || y > y0 + h + 1) continue;
        g.strokeStyle = line; g.beginPath(); g.moveTo(x0, y); g.lineTo(x0 + w, y); g.stroke();
        g.fillStyle = color || text; g.fillText(fmt(v), x0 - 6, y);
      }
    });
    g.fillStyle = text; g.textAlign = "center"; g.textBaseline = "top";
    for (const v of niceTicks(xr[0], xr[1], Math.max(2, Math.floor(w / 90)))) {
      const x = xp(v);
      g.strokeStyle = line; g.beginPath(); g.moveTo(x, y0); g.lineTo(x, y0 + h); g.stroke();
      g.fillText(fmt(v), x, y0 + h + 4);
    }
    if (xLabel) g.fillText(xLabel, x0 + w / 2, y0 + h + 18);
    return { x0, y0, w, h, xp, xv, yps };
  }

  trace(g, f, x, y, i0, i1, yp, dash) {
    // polyline, or per pixel column first / min / max / last: no peak is lost when thinned;
    // dash: true (the default dash) or a dash pattern
    g.setLineDash(dash ? (Array.isArray(dash) ? dash : [5, 4]) : []);
    g.beginPath();
    if (i1 - i0 < 2 * f.w) {
      for (let k = i0; k < i1; k++) (k === i0 ? g.moveTo : g.lineTo).call(g, f.xp(x[k]), yp(y[k]));
    } else {
      let col = null, lo = 0, hi = 0, last = 0, first = true;
      const flush = () => { g.lineTo(col + 0.5, yp(lo)); g.lineTo(col + 0.5, yp(hi)); g.lineTo(col + 0.5, yp(last)); };
      for (let k = i0; k < i1; k++) {
        const px = Math.floor(f.xp(x[k])), v = y[k];
        if (px !== col) {
          if (col !== null) flush();
          if (first) { g.moveTo(px + 0.5, yp(v)); first = false; } else g.lineTo(px + 0.5, yp(v));
          col = px; lo = hi = last = v;
        } else { if (v < lo) lo = v; if (v > hi) hi = v; last = v; }
      }
      if (col !== null) flush();
    }
    g.stroke();
    g.setLineDash([]);
  }

  drawTime(pane, g, W, H, t) {
    const full = [t[0], t[t.length - 1] > t[0] ? t[t.length - 1] : t[0] + 1];
    const xr = this.xRange || full;
    const delta = this.p.delta(), stale = this.p.stale();
    const items = pane.series.filter((s) => !s.hidden).map((s) => {
      let y = this.p.get(s.name);
      if (!y) return null;
      let others = this.p.others(s.name), live = this.p.live(s.name), refs = this.p.references(s.name);
      const ref = delta ? this.p.reference(s.name) : null;
      if (ref) {  // everything minus the reference run, on its own times
        const d = (r) => ({ ...r, y: minus(r.y, resample(ref.t, ref.y, r.t)) });
        y = minus(y, resample(ref.t, ref.y, t));
        others = others.filter((o) => !o.reference).map(d);
        live = live && d(live);
        refs = [];
      }
      let fan = this.p.fan(s.name);
      if (ref) fan = fan.map((r) => ({ ...r, y: minus(r.y, resample(ref.t, ref.y, r.t)) }));
      return { s, y, others, live, refs, fan };
    }).filter(Boolean);
    const i0 = Math.max(0, lower(t, xr[0]) - 1), i1 = Math.min(t.length, lower(t, xr[1]) + 2);
    const span = (r) => [r.y, Math.max(0, lower(r.t, xr[0]) - 1), Math.min(r.t.length, lower(r.t, xr[1]) + 2)];
    const ranges = items.map((it) => range([[it.y, i0, i1], ...it.others.map(span), ...it.fan.map(span), ...(it.live ? [span(it.live)] : [])]));
    // channels of very different size get a lane each (unless the value axis was zoomed)
    const spans = ranges.map(([a, b]) => b - a);
    const apart = !(pane.view && pane.view.y) && items.length > 1 && Math.max(...spans) > 20 * Math.min(...spans);
    const lanes = pane.view && pane.view.y ? [[...pane.view.y, null]]
      : apart ? ranges.map((r, k) => [...r, this.color(items[k].s.name)])
      : [[Math.min(...ranges.map((r) => r[0])), Math.max(...ranges.map((r) => r[1])), null]];
    if (!items.length) lanes.splice(0, lanes.length, [-1, 1, null]);
    const f = this.axes(g, W, H, xr, lanes, null);
    const laneOf = (k) => (apart ? k : 0);
    f.full = full;
    f.yRange = lanes[0];
    f.yv = lanes.length === 1 ? (py) => lanes[0][1] - ((py - f.y0) / f.h) * (lanes[0][1] - lanes[0][0]) : null;
    pane.frame = f;
    g.save(); g.beginPath(); g.rect(f.x0, f.y0, f.w, f.h); g.clip();
    g.lineJoin = "round";
    if (delta && items.length) {  // the reference itself: zero
      g.strokeStyle = css("--muted"); g.lineWidth = 1; g.setLineDash([7, 4]);
      for (let a = 0; a < lanes.length; a++) { const y = f.yps[a](0); g.beginPath(); g.moveTo(f.x0, y); g.lineTo(f.x0 + f.w, y); g.stroke(); }
      g.setLineDash([]);
    }
    items.forEach((it, k) => {
      const yp = f.yps[laneOf(k)], color = this.color(it.s.name);
      for (const r of it.fan) {  // the sweep: thin, in the value's color
        g.strokeStyle = r.color; g.lineWidth = 1; g.globalAlpha = 0.85;
        const [, j0, j1] = span(r);
        this.trace(g, f, r.t, r.y, j0, j1, yp, false);
        g.globalAlpha = 1;
      }
      for (const o of it.others) {
        g.strokeStyle = color; g.lineWidth = 1.3; g.globalAlpha = 0.75;
        const [, j0, j1] = span(o);
        this.trace(g, f, o.t, o.y, j0, j1, yp, o.dash);
        g.globalAlpha = 1;
      }
      g.strokeStyle = color; g.lineWidth = 1.6;
      if (stale) g.globalAlpha = 0.3;  // results of values that are no longer the current ones
      this.trace(g, f, t, it.y, i0, i1, yp, false);
      g.globalAlpha = 1;
      if (it.live) {
        const [, j0, j1] = span(it.live);
        g.strokeStyle = color; g.lineWidth = 2.2;
        this.trace(g, f, it.live.t, it.live.y, j0, j1, yp, false);
        const e = it.live.t.length - 1;  // the head of the running simulation
        if (e >= 0) { g.fillStyle = color; g.beginPath(); g.arc(f.xp(it.live.t[e]), yp(it.live.y[e]), 3.5, 0, 2 * Math.PI); g.fill(); }
      }
      g.fillStyle = color;
      for (const r of it.refs) {
        const step = Math.max(1, Math.floor(r.x.length / 600));
        for (let j = 0; j < r.x.length; j += step) { g.beginPath(); g.arc(f.xp(r.x[j]), yp(r.y[j]), 2.3, 0, 2 * Math.PI); g.fill(); }
      }
    });
    // cursors: current time, A / B
    const vline = (v, color, dash, label) => {
      const x = f.xp(v);
      if (x < f.x0 || x > f.x0 + f.w) return;
      g.strokeStyle = color; g.lineWidth = 1; g.setLineDash(dash);
      g.beginPath(); g.moveTo(x, f.y0); g.lineTo(x, f.y0 + f.h); g.stroke(); g.setLineDash([]);
      if (label) { g.fillStyle = color; g.font = "bold 11px system-ui"; g.textAlign = "left"; g.textBaseline = "top"; g.fillText(label, x + 3, f.y0 + 2); }
    };
    if (this.measure) {
      const [a, b] = this.cursors;
      if (a !== null && b !== null) {
        g.fillStyle = css("--accent"); g.globalAlpha = 0.08;
        g.fillRect(f.xp(Math.min(a, b)), f.y0, Math.abs(f.xp(b) - f.xp(a)), f.h); g.globalAlpha = 1;
      }
      if (a !== null) vline(a, css("--accent"), [], "A");
      if (b !== null) vline(b, css("--accent"), [], "B");
    }
    vline(this.time, css("--text"), [3, 3], null);
    g.restore();
    // hover: crosshair through all time panes, values of this pane
    const hv = this.hover;
    if (hv && hv.pane.mode === "time") {
      const x = f.xp(hv.x);
      if (x >= f.x0 && x <= f.x0 + f.w) {
        g.strokeStyle = css("--muted"); g.lineWidth = 1; g.setLineDash([2, 3]);
        g.beginPath(); g.moveTo(x, f.y0); g.lineTo(x, f.y0 + f.h); g.stroke(); g.setLineDash([]);
        const k = index(t, hv.x), rows = [`t = ${fmt(t[k])} s`];
        const colors = [css("--text")];
        items.forEach((it) => { rows.push(`${delta ? "Δ " : ""}${it.s.name} = ${fmt(it.y[k])} ${unit(it.s.name)}`); colors.push(this.color(it.s.name)); });
        items.forEach((it, j) => { g.fillStyle = this.color(it.s.name); g.beginPath(); g.arc(f.xp(t[k]), f.yps[laneOf(j)](it.y[k]), 3.5, 0, 2 * Math.PI); g.fill(); });
        if (hv.pane === pane) this.tooltip(g, f, x, hv.py, rows, colors);
      }
    }
  }

  tooltip(g, f, x, y, rows, colors) {
    g.font = "12px system-ui, sans-serif";
    const w = Math.max(...rows.map((r) => g.measureText(r).width)) + 16, h = rows.length * 16 + 8;
    let bx = x + 12, by = Math.max(f.y0, Math.min(y - h / 2, f.y0 + f.h - h));
    if (bx + w > f.x0 + f.w) bx = x - 12 - w;
    g.fillStyle = css("--panel"); g.strokeStyle = css("--line"); g.globalAlpha = 0.95;
    g.beginPath(); g.roundRect(bx, by, w, h, 5); g.fill(); g.stroke(); g.globalAlpha = 1;
    g.textAlign = "left"; g.textBaseline = "top";
    rows.forEach((r, i) => { g.fillStyle = colors[i]; g.fillText(r, bx + 8, by + 5 + i * 16); });
  }

  drawXY(pane, g, W, H, t) {
    const x = this.p.get(pane.xName);
    const xs = this.p.others(pane.xName);
    const items = pane.series.filter((s) => !s.hidden).map((s) => {
      const y = this.p.get(s.name);
      const others = this.p.others(s.name).map((o) => {
        const ox = xs.find((r) => r.id === o.id);
        return ox ? { x: ox.y, y: o.y, dash: o.dash } : null;
      }).filter(Boolean);
      return y ? { s, y, others, refs: this.p.xyReferences(pane.xName, s.name) } : null;
    }).filter(Boolean);
    if (!x || !items.length) return;
    const whole = (a) => [a, 0, a.length];  // ranges over the results, the other runs and the references
    const xParts = [whole(x), ...items.flatMap((it) => [...it.others.map((o) => whole(o.x)), ...it.refs.map((r) => whole(r.x))])];
    const yParts = items.flatMap((it) => [whole(it.y), ...it.others.map((o) => whole(o.y)), ...it.refs.map((r) => whole(r.y))]);
    const xr = (pane.view && pane.view.x) || range(xParts, 0.04), yr = (pane.view && pane.view.y) || range(yParts);
    const f = this.axes(g, W, H, xr, [[...yr, null]], `${pane.xName}${unit(pane.xName) ? ` [${unit(pane.xName)}]` : ""}`);
    f.xRange = xr; f.yRange = yr;
    f.yv = (py) => yr[1] - ((py - f.y0) / f.h) * (yr[1] - yr[0]);
    pane.frame = f;
    const yp = f.yps[0];
    g.save(); g.beginPath(); g.rect(f.x0, f.y0, f.w, f.h); g.clip();
    const k = index(t, this.time);
    for (const it of items) {
      const color = this.color(it.s.name);
      g.strokeStyle = color;
      const line = (X, Y, dash) => {
        g.setLineDash(dash ? (Array.isArray(dash) ? dash : [5, 4]) : []); g.lineWidth = dash ? 1.2 : 1.6; g.beginPath();
        const step = Math.max(1, Math.floor(X.length / 4000));
        for (let j = 0; j < X.length; j += step) (j ? g.lineTo : g.moveTo).call(g, f.xp(X[j]), yp(Y[j]));
        g.stroke(); g.setLineDash([]);
      };
      for (const o of it.others) line(o.x, o.y, o.dash || true);
      if (this.p.stale()) g.globalAlpha = 0.3;
      line(x, it.y, false);
      g.globalAlpha = 1;
      g.fillStyle = css("--text");
      for (const r of it.refs) for (let j = 0; j < r.x.length; j++) { g.beginPath(); g.arc(f.xp(r.x[j]), yp(r.y[j]), 2.3, 0, 2 * Math.PI); g.fill(); }
      g.fillStyle = color; g.strokeStyle = css("--panel"); g.lineWidth = 1.5;
      g.beginPath(); g.arc(f.xp(x[k]), yp(it.y[k]), 5, 0, 2 * Math.PI); g.fill(); g.stroke();
    }
    g.restore();
    const hv = this.hover;
    if (hv && hv.pane === pane) {
      this.tooltip(g, f, hv.px, hv.py, [`${pane.xName} = ${fmt(hv.x)}`, `value = ${fmt(hv.y)}`], [css("--text"), css("--muted")]);
    }
  }

  drawKpi(pane, g, W, H, t) {
    // a key figure of each channel over the swept value, from the runs of the sweep in the
    // visible time range of the time plots
    const sw = this.p.sweep(), name = pane.kpi || "max", fn = KPIS[name];
    if (!sw || !sw.runs.some(Boolean)) {
      g.fillStyle = css("--muted"); g.font = "12px system-ui"; g.textAlign = "center"; g.textBaseline = "middle";
      g.fillText("no sweep: ≋ at a slider (Tune) simulates it over its range", W / 2, H / 2);
      return;
    }
    const [t0, t1] = this.xRange || [t[0], t[t.length - 1]];
    const items = pane.series.filter((s) => !s.hidden).map((s) => {
      const pts = sw.runs.map((run, i) => {
        const y = run && sw.channel(run, s.name);
        if (!y) return null;
        const a = lower(run.t, t0), b = Math.max(a + 1, Math.min(run.t.length, lower(run.t, t1) + 1));
        return { x: sw.values[i], y: fn(y, a, b), i };
      }).filter(Boolean);
      return { s, pts };
    }).filter((it) => it.pts.length);
    const xs = sw.values, xr = (pane.view && pane.view.x) || range([[xs, 0, xs.length]], 0.05);
    // channels of very different size get a lane each (as in the time plots)
    const ranges = items.map((it) => { const ys = it.pts.map((q) => q.y); return range([[ys, 0, ys.length]]); });
    const spans = ranges.map(([a, b]) => b - a);
    const apart = !(pane.view && pane.view.y) && items.length > 1 && Math.max(...spans) > 20 * Math.min(...spans);
    const all = items.flatMap((it) => it.pts.map((q) => q.y));
    const lanes = pane.view && pane.view.y ? [[...pane.view.y, null]]
      : apart ? ranges.map((r, k) => [...r, this.color(items[k].s.name)]) : [[...range([[all, 0, all.length]]), null]];
    const yr = lanes[0];
    const f = this.axes(g, W, H, xr, lanes, sw.label);
    f.xRange = xr; f.yRange = yr;
    f.yv = lanes.length === 1 ? (py) => yr[1] - ((py - f.y0) / f.h) * (yr[1] - yr[0]) : null;
    pane.frame = f;
    const laneOf = (k) => (apart ? k : 0);
    g.save(); g.beginPath(); g.rect(f.x0, f.y0, f.w, f.h); g.clip();
    if (sw.current !== null && sw.current !== undefined) {  // the value of the current run
      g.strokeStyle = css("--text"); g.setLineDash([3, 3]); g.lineWidth = 1;
      g.beginPath(); g.moveTo(f.xp(sw.current), f.y0); g.lineTo(f.xp(sw.current), f.y0 + f.h); g.stroke(); g.setLineDash([]);
    }
    items.forEach((it, k) => {
      const yp = f.yps[laneOf(k)];
      it.yp = yp;
      g.strokeStyle = this.color(it.s.name); g.lineWidth = 1.6; g.beginPath();
      it.pts.forEach((q, j) => (j ? g.lineTo : g.moveTo).call(g, f.xp(q.x), yp(q.y)));
      g.stroke();
      for (const q of it.pts) {
        g.fillStyle = scaleColor(q.i, xs.length); g.beginPath(); g.arc(f.xp(q.x), yp(q.y), 4, 0, 2 * Math.PI); g.fill();
      }
    });
    g.restore();
    const hv = this.hover;
    if (hv && hv.pane === pane) {
      let best = null, bd = 144;
      for (const it of items) for (const q of it.pts) {
        const d = (f.xp(q.x) - hv.px) ** 2 + (it.yp(q.y) - hv.py) ** 2;
        if (d < bd) { bd = d; best = { ...q, name: it.s.name }; }
      }
      if (best) this.tooltip(g, f, hv.px, hv.py, [`${sw.label} = ${fmt(best.x)}`, `${name} ${best.name} = ${fmt(best.y)} ${unit(best.name)}`,
        `t = ${fmt(t0)} … ${fmt(t1)} s`], [scaleColor(best.i, xs.length), this.color(best.name), css("--muted")]);
    }
  }

  drawLocus(pane, g, W, H) {
    // eigenvalues: x real part [1/s], y frequency [Hz] = imaginary part / 2 pi (upper half)
    const L = pane.locus, n = L.values.length;
    const pts = [];
    L.eigenvalues.forEach((ev, i) => { for (const [re, im] of ev) if (im >= 0) pts.push({ re, f: im / (2 * Math.PI), i }); });
    if (!pts.length) return;
    // the view starts on the eigenvalues that move with the value (those at the same place for
    // every value are drawn, but do not set the scale); double click: back to it
    const key = (q) => `${q.re.toPrecision(5)},${q.f.toPrecision(5)}`, count = new Map();
    for (const q of pts) count.set(key(q), (count.get(key(q)) || 0) + 1);
    const moving = pts.filter((q) => count.get(key(q)) < n);
    const fit = moving.length ? moving : pts;
    const xr = (pane.view && pane.view.x) || range([[fit.map((q) => q.re).concat([0]), 0, fit.length + 1]], 0.08);
    const yr = (pane.view && pane.view.y) || range([[fit.map((q) => q.f).concat([0]), 0, fit.length + 1]], 0.08);
    const f = this.axes(g, W, H, xr, [[...yr, null]], "real part [1/s]   (frequency [Hz] upwards)");
    f.xRange = xr; f.yRange = yr;
    f.yv = (py) => yr[1] - ((py - f.y0) / f.h) * (yr[1] - yr[0]);
    pane.frame = f;
    const yp = f.yps[0];
    g.save(); g.beginPath(); g.rect(f.x0, f.y0, f.w, f.h); g.clip();
    // lines of equal damping ratio: re = -zeta wn, f = wn sqrt(1 - zeta^2) / 2 pi
    g.font = "10px system-ui"; g.textAlign = "left"; g.textBaseline = "bottom";
    for (const z of [0.05, 0.1, 0.2, 0.3, 0.5, 0.7]) {
      const wn = 4 * Math.max(Math.abs(xr[0]), Math.abs(xr[1]), 2 * Math.PI * Math.abs(yr[1]));
      const x1 = -z * wn, y1 = (wn * Math.sqrt(1 - z * z)) / (2 * Math.PI);
      g.strokeStyle = css("--line"); g.lineWidth = 1; g.setLineDash([3, 3]);
      g.beginPath(); g.moveTo(f.xp(0), yp(0)); g.lineTo(f.xp(x1), yp(y1)); g.stroke(); g.setLineDash([]);
      // label where the line leaves the plot at the top
      const s = Math.min(1, (yr[1] - 0) / (y1 || 1), xr[0] < 0 ? xr[0] / x1 : 1) * 0.92;
      g.fillStyle = css("--muted"); g.fillText(`ζ ${z}`, f.xp(x1 * s) + 2, yp(y1 * s));
    }
    g.strokeStyle = css("--muted"); g.beginPath(); g.moveTo(f.xp(0), f.y0); g.lineTo(f.xp(0), f.y0 + f.h); g.stroke();
    const color = (i) => scaleColor(i, n);  // blue (first value) to red (last)
    for (const q of pts) {
      g.fillStyle = color(q.i);
      g.beginPath(); g.arc(f.xp(q.re), yp(q.f), 3.2, 0, 2 * Math.PI); g.fill();
      if (L.current !== null && L.values[q.i] === L.current) {
        g.strokeStyle = css("--text"); g.lineWidth = 1.5; g.beginPath(); g.arc(f.xp(q.re), yp(q.f), 6, 0, 2 * Math.PI); g.stroke();
      }
    }
    g.restore();
    const hv = this.hover;
    if (hv && hv.pane === pane) {  // the nearest eigenvalue
      let best = null, bd = 144;
      for (const q of pts) { const d = (f.xp(q.re) - hv.px) ** 2 + (yp(q.f) - hv.py) ** 2; if (d < bd) { bd = d; best = q; } }
      if (best) {
        const wn = Math.hypot(best.re, 2 * Math.PI * best.f);
        this.tooltip(g, f, hv.px, hv.py, [`${L.label} = ${fmt(L.values[best.i])}`, `f = ${fmt(best.f)} Hz`,
          `ζ = ${wn ? fmt(-best.re / wn) : "–"}`, `λ = ${fmt(best.re)} ± ${fmt(2 * Math.PI * best.f)} i`],
          [color(best.i), css("--text"), css("--text"), css("--muted")]);
      }
    }
  }

  drawFFT(pane, g, W, H, t) {
    const full = [t[0], t[t.length - 1]];
    const [t0, t1] = this.xRange || full;
    const items = pane.series.filter((s) => !s.hidden).map((s) => {
      const y = this.p.get(s.name);
      const sp = y && spectrum(t, y, t0, t1);
      return sp ? { s, sp } : null;
    }).filter(Boolean);
    if (!items.length) return;
    const fmax = items[0].sp.f[items[0].sp.f.length - 1];
    const xr = (pane.view && pane.view.x) || [0, fmax];
    const parts = items.map((it) => [it.sp.a, lower(it.sp.f, xr[0]), Math.min(it.sp.f.length, lower(it.sp.f, xr[1]) + 1)]);
    const [, hi] = range(parts, 0.08);
    const yr = (pane.view && pane.view.y) || [0, hi];
    const f = this.axes(g, W, H, xr, [[...yr, null]], `frequency [Hz] · amplitude spectrum of t = ${fmt(t0)} .. ${fmt(t1)} s`);
    f.xRange = xr; f.yRange = yr;
    f.yv = (py) => yr[1] - ((py - f.y0) / f.h) * (yr[1] - yr[0]);
    pane.frame = f;
    g.save(); g.beginPath(); g.rect(f.x0, f.y0, f.w, f.h); g.clip();
    const peaks = [];
    items.forEach((it, k) => {
      g.strokeStyle = this.color(it.s.name); g.lineWidth = 1.5;
      const [j0, j1] = [parts[k][1], parts[k][2]];
      this.trace(g, f, it.sp.f, it.sp.a, j0, j1, f.yps[0], false);
      let best = j0 + 1;
      for (let j = j0 + 1; j < j1; j++) if (it.sp.a[j] > it.sp.a[best]) best = j;
      peaks.push([it, best]);
    });
    g.restore();
    g.font = "11px system-ui"; g.textAlign = "right"; g.textBaseline = "top";
    peaks.forEach(([it, j], k) => {
      g.fillStyle = this.color(it.s.name);
      g.fillText(`peak ${fmt(it.sp.f[j])} Hz, amplitude ${fmt(it.sp.a[j])}`, f.x0 + f.w - 4, f.y0 + 4 + 14 * k);
    });
    const hv = this.hover;
    if (hv && hv.pane === pane) this.tooltip(g, f, hv.px, hv.py, [`f = ${fmt(hv.x)} Hz`], [css("--text")]);
  }

  // ---------- cursors A / B: statistics ----------
  updateTable() {
    const box = this.table, t = this.p.time();
    const [a, b] = this.cursors;
    if (!this.measure || !t) { box.style.display = "none"; return; }
    box.style.display = "block";
    if (a === null || b === null) { box.innerHTML = `<div class="hint">click in a time plot to set cursor ${a === null ? "A" : "B"}; drag a cursor to move it</div>`; return; }
    const [lo, hi] = a < b ? [a, b] : [b, a], i0 = lower(t, lo), i1 = Math.max(i0 + 1, Math.min(t.length, lower(t, hi) + 1));
    const ka = index(t, a), kb = index(t, b);
    let html = `<table><tr><th>A ${fmt(t[ka])} s · B ${fmt(t[kb])} s · Δt ${fmt(t[kb] - t[ka])} s · 1/Δt ${fmt(1 / Math.abs(t[kb] - t[ka]))} Hz</th>` +
      `<th>A</th><th>B</th><th>B − A</th><th>min</th><th>max</th><th>mean</th><th>rms</th></tr>`;
    const names = [...new Set(this.panes.filter((p) => p.mode === "time").flatMap((p) => p.series.map((s) => s.name)))];
    for (const n of names) {
      const y = this.p.get(n);
      if (!y) continue;
      let mn = Infinity, mx = -Infinity, sum = 0, sq = 0;
      for (let k = i0; k < i1; k++) { const v = y[k]; if (v < mn) mn = v; if (v > mx) mx = v; sum += v; sq += v * v; }
      const m = i1 - i0;
      html += `<tr><td><i style="background:${this.color(n)}"></i>${n} <small>${unit(n)}</small></td>` +
        [y[ka], y[kb], y[kb] - y[ka], mn, mx, sum / m, Math.sqrt(sq / m)].map((v) => `<td>${fmt(v)}</td>`).join("") + "</tr>";
    }
    box.innerHTML = html + "</table>";
  }

  // ---------- export ----------
  png() {
    const panes = this.panes.filter((p) => p.canvas && p.canvas.width);
    if (!panes.length) return null;
    const dpr = window.devicePixelRatio || 1, W = Math.max(...panes.map((p) => p.canvas.width));
    const head = 22 * dpr, H = panes.reduce((s, p) => s + p.canvas.height + head, 0);
    const out = document.createElement("canvas");
    out.width = W; out.height = H;
    const g = out.getContext("2d");
    g.fillStyle = css("--panel"); g.fillRect(0, 0, W, H);
    let y = 0;
    for (const p of panes) {
      g.fillStyle = css("--text"); g.font = `${12 * dpr}px system-ui`; g.textBaseline = "top";
      let x = 8 * dpr;
      for (const s of p.series.filter((s) => !s.hidden)) {
        g.fillStyle = this.color(s.name); g.fillRect(x, y + 9 * dpr, 10 * dpr, 3 * dpr); x += 14 * dpr;
        g.fillStyle = css("--text"); const label = s.name + (p.mode === "xy" ? ` over ${p.xName}` : "");
        g.fillText(label, x, y + 4 * dpr); x += g.measureText(label).width + 16 * dpr;
      }
      g.drawImage(p.canvas, 0, y + head);
      y += p.canvas.height + head;
    }
    return out.toDataURL("image/png");
  }

  csv() {
    // the channels of the time plots in the visible time range
    const t = this.p.time();
    if (!t) return "";
    const names = [...new Set(this.panes.filter((p) => p.mode === "time").flatMap((p) => p.series.map((s) => s.name)))];
    const [a, b] = this.xRange || [t[0], t[t.length - 1]];
    const cols = names.map((n) => this.p.get(n));
    const lines = ["t," + names.join(",")];
    for (let k = lower(t, a); k < t.length && t[k] <= b; k++) lines.push([t[k], ...cols.map((c) => (c ? c[k] : ""))].join(","));
    return lines.join("\n") + "\n";
  }
}
