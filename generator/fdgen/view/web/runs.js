// The run strip above the plots: every simulation of this viewer session as a card, labelled
// with its name or what differs from the model file (tuned values) and which version of the file
// it ran with. A click on a card shows that run in the plots (dashed, in the channel's color, its
// dash on the card); ★ makes it the reference (grey bodies in the 3D view, the base of the
// difference plots), ⟲ takes its values back into Tune, × removes it. A parameter sweep is one
// card: its fan of curves shown or not, its progress, × removes it.
//
// With many runs the strip shows only the current run, the reference, the runs shown, kept or
// named, and the newest; "≡ all runs" opens the run table: every run with its tuned values as
// columns (sortable), a name, "keep" (not dropped as one of the oldest; saved next to the model
// and there again after a restart, as are named runs), show, ★, ⟲, ×, and
// "remove the others" for all runs neither current, reference, kept nor shown. The data lives in
// the viewer (fdview/viewer.js); strip and table only show it and report the clicks.

import { tuneLabel } from "fdview/panels.js";

const STRIP = 6;  // cards in the strip at most (besides the fan)
const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const short = (v) => {
  const a = Math.abs(v);
  return a >= 1e4 && a < 1e7 ? `${+(v / 1e3).toPrecision(4)}k` : String(+v.toPrecision(4));
};

// when a run was made: the time for today's runs, else the date (runs kept from earlier sessions)
function when(created) {
  const d = new Date(), pad = (n) => String(n).padStart(2, "0");
  const today = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}`;
  return created.length > 10 && created.slice(0, 10) === today ? created.slice(11) : created.slice(0, 10);
}

export function runLabel(run, runs, named = true) {
  // the run's name, or what it is: its tuned values, and the version of the model file if there were several
  if (named && run.label) return run.label;
  const o = Object.entries(run.overrides || {}).map(([k, v]) => `${tuneLabel(k)} ${short(v)}`);
  const edits = new Set(runs.map((r) => r.edit));
  return [edits.size > 1 ? `file v${run.edit}` : null, ...(o.length ? o : ["model file"])].filter(Boolean).join(" · ");
}

function swatch(dash) {
  // a line sample with the run's dash ([] solid)
  const d = dash && dash.length ? ` stroke-dasharray="${dash.join(" ")}"` : "";
  return `<svg width="22" height="8" aria-hidden="true"><line x1="1" y1="4" x2="21" y2="4" stroke="currentColor" stroke-width="2"${d}/></svg>`;
}

export class RunStrip {
  // a: {toggle(id), show(ids, on), reference(id | null), restore(run), remove(id), removeMany(ids), keep(id, on),
  //     label(id, text), dash(id), sweepShown(), sweepStop()}
  constructor(box, table, a) {
    this.box = box;
    this.tableBox = table;
    this.a = a;
    this.open = false;
    this.sort = { key: "id", dir: -1 };
    this.last = null;
    document.addEventListener("keydown", (e) => { if (e.key === "Escape" && this.open) this.setOpen(false); });
  }

  setOpen(open) {
    this.open = open;
    if (this.last) this.render(...this.last);
  }

  render(runs, current, reference, shown, sweep = null) {
    this.last = [runs, current, reference, shown, sweep];
    const box = this.box;
    box.hidden = runs.length < 2 && reference === null && !sweep;
    if (box.hidden) this.open = false;
    box.innerHTML = '<span class="title" title="every simulation of this session; click: compare in the plots">runs</span>';
    if (sweep) {
      const n = sweep.values.length, ok = sweep.results.filter((r) => r === "ok").length;
      const failed = sweep.results.filter((r) => r && r !== "ok");
      const card = document.createElement("span");
      card.className = "run fan" + (sweep.shown ? " on" : " off");
      card.title = `sweep of ${sweep.path}: ${sweep.values.map(short).join(", ")}` + (failed.length ? `\n${failed.join("\n")}` : "") +
        (sweep.error ? `\n${sweep.error}` : "") + "\nclick: show / hide the fan of curves";
      const state = sweep.status === "running" ? ` ${ok}/${n} …` : failed.length ? ` ${ok}/${n} ⚠` : ` ×${n}`;
      card.innerHTML = `<span class="scale"></span><b>fan</b> <span class="lbl">${esc(tuneLabel(sweep.path))} ${short(sweep.values[0])}…${short(sweep.values[n - 1])}${state}</span>` +
        '<button class="icon del" title="stop / remove the sweep">×</button>';
      card.addEventListener("click", (e) => { if (!e.target.closest("button")) this.a.sweepShown(); });
      card.querySelector(".del").addEventListener("click", () => this.a.sweepStop());
      box.appendChild(card);
    }
    // the cards: current, reference, shown, kept, named first, then the newest, at most STRIP
    const ranked = [...runs].reverse().map((r, age) => {
      const rank = r.id === current ? 0 : r.id === reference ? 1 : shown.has(r.id) ? 2 : r.keep || r.label ? 3 : 4;
      return { r, key: rank * 1e6 + age };
    }).sort((x, y) => x.key - y.key);
    const strip = new Set(ranked.slice(0, STRIP).map((x) => x.r.id));
    const latestEdit = Math.max(...runs.map((r) => r.edit));
    for (const r of [...runs].reverse()) {
      if (!strip.has(r.id)) continue;
      const isCur = r.id === current, isRef = r.id === reference, on = isCur || isRef || shown.has(r.id);
      const card = document.createElement("span");
      card.className = "run" + (isCur ? " current" : "") + (isRef ? " ref" : "") + (on ? " on" : "");
      const label = runLabel(r, runs);
      card.title = `run ${r.id}${r.label ? ` "${r.label}": ${runLabel(r, runs, false)}` : ""} at ${r.created}, ${r.seconds.toFixed(2)} s simulation` +
        (r.edit < latestEdit ? " (an older version of the model file)" : "") + (r.keep ? "\nkept" : "") +
        (isCur ? "\nthe current results" : "\nclick: show / hide in the plots");
      card.innerHTML = `${on ? swatch(isCur ? [] : this.a.dash(r.id)) : ""}<b>#${r.id}</b>${r.keep ? '<span class="kept">⚑</span>' : ""} <span class="lbl">${esc(label)}</span>` +
        `<button class="icon star" title="${isRef ? "no reference" : "reference: grey in 3D, base of the Δ plots"}">${isRef ? "★" : "☆"}</button>` +
        (isCur ? "" : '<button class="icon back" title="these values back into Tune">⟲</button><button class="icon del" title="remove">×</button>');
      card.addEventListener("click", (e) => {
        if (e.target.closest("button") || isCur) return;
        this.a.toggle(r.id);
      });
      card.querySelector(".star").addEventListener("click", () => this.a.reference(isRef ? null : r.id));
      if (!isCur) {
        card.querySelector(".back").addEventListener("click", () => this.a.restore(r));
        card.querySelector(".del").addEventListener("click", () => this.a.remove(r.id));
      }
      box.appendChild(card);
    }
    if (runs.length > 1) {
      const more = document.createElement("button");
      const hidden = runs.length - strip.size;
      more.className = "all" + (this.open ? " on" : "");
      more.title = "every run in a table: tuned values, names, keep, show, remove";
      more.textContent = `≡ all ${runs.length} runs` + (hidden ? ` (${hidden} more)` : "");
      more.addEventListener("click", () => this.setOpen(!this.open));
      box.appendChild(more);
    }
    this.renderTable(runs, current, reference, shown);
  }

  renderTable(runs, current, reference, shown) {
    const box = this.tableBox;
    box.hidden = !this.open;
    if (!this.open) { box.innerHTML = ""; return; }
    if (box.contains(document.activeElement) && document.activeElement.tagName === "INPUT" && document.activeElement.type === "text") return;  // typing a name
    const keys = [...new Set(runs.flatMap((r) => Object.keys(r.overrides || {})))].sort();
    const edits = new Set(runs.map((r) => r.edit)).size > 1;
    const cols = [
      { key: "show", head: "show", title: "compare in the plots", get: (r) => (r.id === current || r.id === reference || shown.has(r.id) ? 1 : 0) },
      { key: "id", head: "#", get: (r) => r.id },
      { key: "label", head: "name", get: (r) => r.label || "" },
      ...keys.map((k) => ({ key: `o:${k}`, head: tuneLabel(k), title: k, num: true, get: (r) => (r.overrides && k in r.overrides ? r.overrides[k] : null) })),
      ...(edits ? [{ key: "edit", head: "file", title: "version of the model file", get: (r) => r.edit }] : []),
      { key: "created", head: "at", get: (r) => r.created, show: when },
      { key: "seconds", head: "sim s", title: "simulated time", num: true, get: (r) => r.seconds },
      { key: "keep", head: "keep", title: "kept: not dropped as one of the oldest runs, saved next to the model (<model>.runs/) and there again after a restart", get: (r) => (r.keep ? 1 : 0) },
    ];
    const col = cols.find((c) => c.key === this.sort.key) || cols[1];
    const rows = [...runs].sort((x, y) => {
      const a = col.get(x), b = col.get(y);
      if (a === b) return y.id - x.id;
      if (a === null) return 1;  // the value of the model file: last
      if (b === null) return -1;
      return (a < b ? -1 : 1) * this.sort.dir;
    });
    const others = runs.filter((r) => r.id !== current && r.id !== reference && !r.keep && !shown.has(r.id));
    box.innerHTML = `<div class="bar"><b>${runs.length} runs</b><span class="hint">tuned values as columns, empty: as in the model file · click a column to sort</span>` +
      `<button class="none" title="compare no other run">show none</button>` +
      `<button class="others" title="remove the runs that are neither current, reference, kept nor shown"${others.length ? "" : " disabled"}>remove the others (${others.length})</button>` +
      `<button class="icon close" title="close (Esc)">×</button></div>`;
    const table = document.createElement("table");
    const head = document.createElement("tr");
    for (const c of cols) {
      const th = document.createElement("th");
      th.textContent = c.head + (c.key === this.sort.key ? (this.sort.dir > 0 ? " ▲" : " ▼") : "");
      if (c.title) th.title = c.title;
      if (c.num) th.className = "num";
      th.addEventListener("click", () => {
        this.sort = c.key === this.sort.key ? { key: c.key, dir: -this.sort.dir } : { key: c.key, dir: c.num || c.key === "id" ? -1 : 1 };
        this.renderTable(runs, current, reference, shown);
      });
      head.appendChild(th);
    }
    head.insertAdjacentHTML("beforeend", "<th></th>");
    table.appendChild(head);
    for (const r of rows) {
      const isCur = r.id === current, isRef = r.id === reference, on = isCur || isRef || shown.has(r.id);
      const tr = document.createElement("tr");
      tr.className = (isCur ? "current" : "") + (isRef ? " ref" : "");
      tr.title = isCur ? "the current results" : "";
      const cells = cols.map((c) => {
        if (c.key === "show") return `<td><input type="checkbox" class="show"${on ? " checked" : ""}${isCur || isRef ? " disabled" : ""}> ${on && !isCur ? swatch(this.a.dash(r.id)) : ""}</td>`;
        if (c.key === "label") return `<td><input type="text" class="name" spellcheck="false" maxlength="80" placeholder="${esc(runLabel(r, runs, false))}" value="${esc(r.label || "")}"></td>`;
        if (c.key === "keep") return `<td><input type="checkbox" class="keep"${r.keep ? " checked" : ""}></td>`;
        const v = c.get(r);
        return `<td${c.num ? ' class="num"' : ""}>${v === null ? "" : c.show ? esc(c.show(v)) : c.key === "seconds" ? v.toFixed(2) : c.num ? short(v) : c.key === "edit" ? `v${v}` : esc(v)}</td>`;
      });
      tr.innerHTML = cells.join("") + `<td class="acts"><button class="icon star" title="${isRef ? "no reference" : "reference"}">${isRef ? "★" : "☆"}</button>` +
        (isCur ? "" : '<button class="icon back" title="these values back into Tune">⟲</button><button class="icon del" title="remove">×</button>') + "</td>";
      tr.querySelector(".show").addEventListener("change", () => this.a.toggle(r.id));
      tr.querySelector(".keep").addEventListener("change", (e) => this.a.keep(r.id, e.target.checked));
      const name = tr.querySelector(".name");
      const save = () => { if (name.value.trim() !== (r.label || "")) this.a.label(r.id, name.value.trim()); };
      name.addEventListener("keydown", (e) => {
        e.stopPropagation();
        if (e.key === "Enter") name.blur();
        else if (e.key === "Escape") { name.value = r.label || ""; name.blur(); }
      });
      name.addEventListener("blur", save);
      tr.querySelector(".star").addEventListener("click", () => this.a.reference(isRef ? null : r.id));
      if (!isCur) {
        tr.querySelector(".back").addEventListener("click", () => this.a.restore(r));
        tr.querySelector(".del").addEventListener("click", () => this.a.remove(r.id));
      }
      table.appendChild(tr);
    }
    const wrap = document.createElement("div");
    wrap.className = "scroll";
    wrap.appendChild(table);
    box.appendChild(wrap);
    box.querySelector(".close").addEventListener("click", () => this.setOpen(false));
    box.querySelector(".none").addEventListener("click", () => this.a.show([...shown], false));
    box.querySelector(".others").addEventListener("click", () => {
      if (confirm(`Remove ${others.length} run${others.length > 1 ? "s" : ""}?`)) this.a.removeMany(others.map((r) => r.id));
    });
  }
}
