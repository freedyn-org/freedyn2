"""What the viewer keeps next to the model file, so it is there again after a restart.

  <model dir>/<model stem>.view.json         the plot layout: pages, plots, formulas (small, worth
                                             committing: the same plots for everyone)
  <model dir>/<model stem>.runs/index.json   the runs kept (keep in the run table, or a name):
                                             [{id, label, overrides, source, seconds, created}],
                                             and which of them is the reference
  <model dir>/<model stem>.runs/run_<id>.npz the results of a kept run: one float64 array per
                                             channel, in the order of the CSV (numpy.load)
  <model dir>/<model stem>.runs/.gitignore   "*": results stay out of a repository

A run's `source` is a hash of the model file it ran with, so runs of the same file version are
recognized after a restart. Files that cannot be written (a read-only folder) are logged, not an
error: the viewer then keeps the runs for the session only.
"""

import hashlib
import json
import logging
import os
from pathlib import Path

import numpy as np

from .scene import read_results

log = logging.getLogger(__name__)
META = ("id", "label", "overrides", "source", "seconds", "created")


def layout_path(model_path):
    p = Path(model_path).resolve()
    return p.parent / f"{p.stem}.view.json"


def runs_dir(model_path):
    p = Path(model_path).resolve()
    return p.parent / f"{p.stem}.runs"


def source_hash(model_path):
    """A short hash of the model file's text (None if it cannot be read)."""
    try:
        return hashlib.sha1(Path(model_path).read_bytes()).hexdigest()[:16]
    except OSError:
        return None


def _write(path, text):
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def read_layout(model_path):
    try:
        data = json.loads(layout_path(model_path).read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def write_layout(model_path, layout):
    """The layout as JSON; unchanged text is not written again. False if it could not be written."""
    path = layout_path(model_path)
    text = json.dumps(layout, indent=1, ensure_ascii=False) + "\n"
    try:
        if path.is_file() and path.read_text(encoding="utf-8") == text:
            return True
        _write(path, text)
        return True
    except OSError as e:
        log.warning("layout not saved: %s", e)
        return False


def read_runs(model_path):
    """(runs, reference) kept next to the model: runs [{id, label, overrides, source, seconds,
    created, npz}] whose data file exists, ordered by id."""
    d = runs_dir(model_path)
    try:
        data = json.loads((d / "index.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return [], None
    runs = []
    for r in data.get("runs") or []:
        try:
            run = {k: r.get(k) for k in META}
            run["id"], run["seconds"] = int(run["id"]), float(run["seconds"] or 0.0)
            run["label"], run["overrides"] = str(run["label"] or ""), {str(k): float(v) for k, v in (run["overrides"] or {}).items()}
            run["created"] = str(run["created"] or "")
        except (TypeError, ValueError):
            continue
        run["npz"] = d / f"run_{run['id']}.npz"
        if run["npz"].is_file():
            runs.append(run)
    runs.sort(key=lambda r: r["id"])
    ref = data.get("reference")
    return runs, ref if any(r["id"] == ref for r in runs) else None


def write_index(model_path, runs, reference):
    """index.json of the kept runs (those with keep); data files of runs no longer kept go."""
    d = runs_dir(model_path)
    kept = [r for r in runs if r.get("keep")]
    try:
        if not kept and not d.exists():
            return True
        d.mkdir(exist_ok=True)
        gitignore = d / ".gitignore"
        if not gitignore.exists():
            gitignore.write_text("# results of the viewer's kept runs (freedyn2)\n*\n", encoding="utf-8")
        ids = {r["id"] for r in kept}
        _write(d / "index.json", json.dumps({
            "reference": reference if reference in ids else None,
            "runs": [{k: r.get(k) for k in META} for r in kept]}, indent=1, ensure_ascii=False) + "\n")
        for f in d.glob("run_*.npz"):
            stem = f.stem[len("run_"):]
            if not (stem.isdigit() and int(stem) in ids):
                f.unlink(missing_ok=True)
        return True
    except OSError as e:
        log.warning("kept runs not saved: %s", e)
        return False


def save_run(model_path, run):
    """The results of a run (its CSV) as run_<id>.npz; its path, or None if it could not be written."""
    d = runs_dir(model_path)
    path = d / f"run_{run['id']}.npz"
    if path.is_file():
        return path
    try:
        cols = read_results(run["csv"])
        d.mkdir(exist_ok=True)
        tmp = d / f"run_{run['id']}.tmp.npz"
        np.savez_compressed(tmp, **cols)
        os.replace(tmp, path)
        return path
    except (OSError, ValueError) as e:
        log.warning("run %s not saved: %s", run["id"], e)
        return None


def npz_to_csv(npz, csv_path):
    """A kept run's results back as a CSV (for the page), with the full precision."""
    with np.load(npz, allow_pickle=False) as z:
        names = list(z.files)
        data = np.column_stack([z[n] for n in names]) if names else np.zeros((0, 0))
    with open(csv_path, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(names) + "\n")
        np.savetxt(f, data, delimiter=",", fmt="%.17g")
    return csv_path
