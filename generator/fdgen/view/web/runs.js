// The run strip above the plots: every simulation of this viewer session as a card, labelled
// with what differs from the model file (tuned values) and which version of the file it ran
// with. A click on a card shows that run in the plots (dashed, in the channel's color, its dash
// on the card); ★ makes it the reference (grey bodies in the 3D view, the base of the
// difference plots), ⟲ takes its values back into Tune, × removes it. A parameter sweep is one
// card: its fan of curves shown or not, its progress, × removes it. The data lives in the
// viewer (fdview/viewer.js); the strip only shows it and reports the clicks.

import { tuneLabel } from "fdview/panels.js";

const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const short = (v) => {
  const a = Math.abs(v);
  return a >= 1e4 && a < 1e7 ? `${+(v / 1e3).toPrecision(4)}k` : String(+v.toPrecision(4));
};

export function runLabel(run, runs) {
  // what this run is: its tuned values, and the version of the model file if there were several
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
  // a: {toggle(id), reference(id | null), restore(run), remove(id), dash(id), sweepShown(), sweepStop()}
  constructor(box, a) {
    this.box = box;
    this.a = a;
  }

  render(runs, current, reference, shown, sweep = null) {
    const box = this.box;
    box.hidden = runs.length < 2 && reference === null && !sweep;
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
    const latestEdit = Math.max(...runs.map((r) => r.edit));
    for (const r of [...runs].reverse()) {
      const isCur = r.id === current, isRef = r.id === reference, on = isCur || isRef || shown.has(r.id);
      const card = document.createElement("span");
      card.className = "run" + (isCur ? " current" : "") + (isRef ? " ref" : "") + (on ? " on" : "");
      const label = runLabel(r, runs);
      card.title = `run ${r.id} at ${r.created}, ${r.seconds.toFixed(2)} s simulation` +
        (r.edit < latestEdit ? " (an older version of the model file)" : "") +
        (isCur ? "\nthe current results" : "\nclick: show / hide in the plots");
      card.innerHTML = `${on ? swatch(isCur ? [] : this.a.dash(r.id)) : ""}<b>#${r.id}</b> <span class="lbl">${esc(label)}</span>` +
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
  }
}
