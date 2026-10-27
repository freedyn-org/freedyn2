"""Run legacy FreeDyn and FreeDyn 2.0 on the same .fds models and compare results and speed.

Usage:
  python legacy/compare.py examples/compare/*.fds [--level exact|physics]
         [--legacy build/legacy/build/FreeDyn] [--v2 build/freedyn2_run] [--repeat 3]
         [--json report.json] [--markdown report.md]

Two levels:
- exact: FreeDyn 2.0 with the "legacy" profile (legacy algorithm) against legacy FreeDyn,
  to round-off. Verifies the generator, assembly and solver implementation.
- physics: FreeDyn 2.0 with its own defaults ("v2" profile) and legacy FreeDyn, each against
  an independent reference (legacy/references). Both must solve the same physics; FreeDyn 2.0
  is free to use other methods and is expected to be at least as accurate.

For every model both programs run in a fresh directory, single-threaded
(OMP_NUM_THREADS = MKL_NUM_THREADS = 1), writing results at the model's output step.
Compared: output times, the accepted step sequence (legacy DebugLevel 1 log), and per body
R, A (rotation matrix), Rdot and omega (body frame). Times are wall-clock times of the whole
program run (best of --repeat); FreeDyn 2.0 additionally reports its time per step.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "generator"))

from references import reference  # noqa: E402
from results import read_rigid_body_results, read_step_log, read_v2_results  # noqa: E402

ENV = dict(os.environ, OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")


LEGACY_SUCCESS = 2  # SIM_SUCCESS, the exit code of a successful legacy FreeDyn run


def timed(cmd, cwd, repeat, ok=0):
    best, out = float("inf"), None
    for _ in range(repeat):
        t0 = time.perf_counter()
        out = subprocess.run(cmd, cwd=cwd, env=ENV, capture_output=True, text=True)
        best = min(best, time.perf_counter() - t0)
        if out.returncode != ok:
            raise RuntimeError(f"{cmd[0]} failed:\n{out.stdout}\n{out.stderr}")
    return best, out.stdout


def absolute_data_paths(text, src_dir):
    """Model text with relative PathToSplineData made absolute (copies live elsewhere)."""
    def fix(m):
        p = Path(m.group(2).strip().replace("\\", "/"))
        return m.group(1) + str(p if p.is_absolute() else (Path(src_dir) / p).resolve())
    return re.sub(r"(PathToSplineData\s*=\s*)(.*)", fix, text)


def copy_model(fds, dest):
    dest.write_text(absolute_data_paths(fds.read_text(encoding="latin-1"), fds.parent), encoding="latin-1")
    return dest


def truncated(fds, work, fraction=0.1):
    """Copy of the model with a shorter end time, for the fixed cost of a run."""
    text = absolute_data_paths(fds.read_text(encoding="latin-1"), fds.parent)
    end = float(re.search(r"SimulationTimeEnd = (\S+)", text).group(1))
    out = work / f"{fds.stem}_short.fds"
    out.write_text(re.sub(r"SimulationTimeEnd = \S+", f"SimulationTimeEnd = {end * fraction!r}", text),
                   encoding="latin-1")
    return out


def legacy_steps(model):
    return int(read_step_log(model.with_suffix(".db1"))[2].sum())


def run_legacy(exe, fds, work, repeat):
    model = copy_model(fds, work / fds.name)
    seconds, _ = timed([str(exe), str(model), "no"], work, repeat, ok=LEGACY_SUCCESS)
    res = read_rigid_body_results(model.with_suffix(""))
    # Second run with DebugLevel 1 for the step log (not timed).
    dbg = work / "debug"
    dbg.mkdir()
    text = re.sub(r"DebugLevel = \S+", "DebugLevel = 1", absolute_data_paths(fds.read_text(encoding="latin-1"), fds.parent))
    (dbg / fds.name).write_text(text, encoding="latin-1")
    timed([str(exe), str(dbg / fds.name), "no"], dbg, 1, ok=LEGACY_SUCCESS)
    res["steps"] = read_step_log((dbg / fds.name).with_suffix(".db1"))
    # Short run for the fixed cost (input, setup, files); its step count from a debug run.
    short = truncated(dbg / fds.name, work)
    short_seconds, _ = timed([str(exe), str(short), "no"], work, repeat, ok=LEGACY_SUCCESS)
    n_full, n_short = int(res["steps"][2].sum()), legacy_steps(short)
    res["us_per_step"] = 1e6 * (seconds - short_seconds) / (n_full - n_short)
    return seconds, res


def run_v2(exe, fds, work, repeat, profile="legacy"):
    from fdgen.fds import load_fds
    from fdgen.model import write_fdm

    model = load_fds(fds, profile)
    fdm = work / f"{fds.stem}.fdm"
    write_fdm(model, fdm)
    csv_path, log = work / "v2.csv", work / "v2_steps.txt"
    seconds, stdout = timed([str(exe), str(fdm), "-o", str(csv_path)], work, repeat)
    per_step = float(re.search(r"time per step: ([\d.]+) us", stdout).group(1))
    subprocess.run([str(exe), str(fdm), "-o", str(csv_path), "--step-log", str(log)], env=ENV, check=True,
                   capture_output=True)
    short = truncated(fds, work)
    load_short = load_fds(short, profile)
    write_fdm(load_short, work / "short.fdm")
    short_seconds, short_out = timed([str(exe), str(work / "short.fdm"), "-o", str(work / "short.csv")], work, repeat)
    n_short = int(re.search(r": (\d+) steps", short_out).group(1))
    n_full = int(re.search(r": (\d+) steps", stdout).group(1))
    marginal = 1e6 * (seconds - short_seconds) / (n_full - n_short)
    res = read_v2_results(csv_path, list(model["bodies"]))
    steps = np.loadtxt(log, skiprows=1, ndmin=2)
    res["steps"] = (steps[:, 0], steps[:, 1], steps[:, 2] > 0, steps[:, 3])
    return seconds, (per_step, marginal), res


def compare(legacy_res, v2_res):
    d = {"frames": (len(legacy_res["t"]), len(v2_res["t"]))}
    if d["frames"][0] != d["frames"][1]:
        return d
    d["t"] = float(np.max(np.abs(legacy_res["t"] - v2_res["t"])))
    (lt, lh, lok, lit), (vt, vh, vok, vit) = legacy_res["steps"], v2_res["steps"]
    d["steps"] = (int(lok.sum()), int(vok.sum()))
    d["rejected"] = (int((~lok).sum()), int((~vok).sum()))
    if len(lt) == len(vt):
        # The legacy log prints t and h with 5 significant digits.
        d["step_rel"] = float(max(np.max(np.abs(lt - vt) / np.maximum(np.abs(vt), 1e-300)),
                                  np.max(np.abs(lh - vh) / vh)))
        d["same_accept"] = bool(np.all(lok == vok))
        d["same_iterations"] = bool(np.all(lit == vit))
        d["iterations"] = (int(lit.sum()), int(vit.sum()))
    for key in ("R", "A", "Rdot", "omega_body"):
        err = scale = 0.0
        for lb, vb in zip(legacy_res["bodies"], v2_res["bodies"]):
            err = max(err, float(np.max(np.abs(lb[key] - vb[key]))))
            scale = max(scale, float(np.max(np.abs(lb[key]))))
        d[key] = err
        d[key + "_rel"] = err / max(scale, 1e-300)
    return d


# Time window of the physical comparison for models whose solution is sensitive to small
# perturbations: near the unstable axis differences grow by e^(lambda t), so after the first flip
# any two accurate solutions drift apart and a maximum over the whole run measures nothing.
# double_pendulum is chaotic; spring_chain starts to tumble after ~2 s (second-order convergence
# of FreeDyn 2.0 to the reference up to there: 1.9e-5, 4.7e-6, 1.2e-6 for h = 1e-3, 5e-4, 2.5e-4).
HORIZON = {"dzhanibekov": 5.0, "double_pendulum": 3.0, "spring_chain": 2.0}


def physics(fds, legacy_res, v2_res):
    """Errors of both programs against the independent reference, at their own output times."""
    from fdgen.fds import load_fds

    model = load_fds(fds)
    t_max = HORIZON.get(fds.stem, np.inf)
    d = {"horizon": None if np.isinf(t_max) else t_max}
    cache = {}
    for who, res in (("legacy", legacy_res), ("v2", v2_res)):
        err_w = err_a = err_r = 0.0
        m = res["t"] <= t_max + 1e-9
        key = res["t"][m].tobytes()
        if key not in cache:  # same output times: one reference run
            cache[key] = reference(model, res["t"][m])
        for ref, b in zip(cache[key], res["bodies"]):
            b = {k: v[m] for k, v in b.items() if k != "id"}
            w, A, R = ref
            err_w = max(err_w, float(np.max(np.abs(b["omega_body"] - w)) / max(np.max(np.abs(w)), 1e-300)))
            err_a = max(err_a, float(np.max(np.abs(b["A"] - A))))
            err_r = max(err_r, float(np.max(np.abs(b["R"] - R)) / max(1.0, np.max(np.abs(R)))))
        d[who] = {"omega_rel": err_w, "A": err_a, "R_rel": err_r}
    (_, _, lok, _), (_, _, vok, _) = legacy_res["steps"], v2_res["steps"]
    d["steps"] = (int(lok.sum()), int(vok.sum()))
    return d


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="+", type=Path)
    ap.add_argument("--legacy", type=Path, default=ROOT / "build/legacy/build/FreeDyn")
    ap.add_argument("--v2", type=Path, default=ROOT / "build/freedyn2_run")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--markdown", type=Path)
    ap.add_argument("--level", choices=("exact", "physics"), default="exact")
    args = ap.parse_args(argv)
    profile = "legacy" if args.level == "exact" else "v2"

    report = []
    for fds in args.models:
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            (work / "legacy").mkdir()
            (work / "v2").mkdir()
            t_legacy, legacy_res = run_legacy(args.legacy.resolve(), fds.resolve(), work / "legacy", args.repeat)
            t_v2, per_step, v2_res = run_v2(args.v2.resolve(), fds.resolve(), work / "v2", args.repeat, profile)
        d = compare(legacy_res, v2_res) if args.level == "exact" else physics(fds, legacy_res, v2_res)
        d.update(model=fds.stem, legacy_seconds=t_legacy, v2_seconds=t_v2, v2_us_per_step_solver=per_step[0],
                 v2_us_per_step=per_step[1], legacy_us_per_step=legacy_res["us_per_step"])
        report.append(d)

    if args.level == "physics":
        hdr = ("| model | steps legacy / v2 | omega error legacy / v2 | A error legacy / v2 | R error legacy / v2 "
               "| per step legacy / v2 [us] | time to solution legacy / v2 [s] |")
        lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
        for d in report:
            L, V = d["legacy"], d["v2"]
            name = d["model"] + (f" (t <= {d['horizon']:g} s)" if d["horizon"] else "")
            lines.append(
                f"| {name} | {d['steps'][0]} / {d['steps'][1]} | {L['omega_rel']:.1e} / {V['omega_rel']:.1e} "
                f"| {L['A']:.1e} / {V['A']:.1e} | {L['R_rel']:.1e} / {V['R_rel']:.1e} "
                f"| {d['legacy_us_per_step']:.1f} / {d['v2_us_per_step']:.1f} "
                f"| {d['legacy_seconds']:.3f} / {d['v2_seconds']:.3f} |"
            )
        return _finish(report, lines, args)

    hdr = ("| model | steps legacy / v2 | rejected | Newton iterations | max rel. step diff | R rel | A rel "
           "| Rdot rel | omega rel | run legacy / v2 [s] | per step legacy / v2 [us] | speed-up per step |")
    lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
    for d in report:
        steps = d.get("steps", ("-", "-"))
        lines.append(
            f"| {d['model']} | {steps[0]} / {steps[1]} | {d['rejected'][0]} / {d['rejected'][1]} "
            f"| {'%d / %d' % d['iterations'] if 'iterations' in d else '-'} "
            f"| {d.get('step_rel', float('nan')):.1e} "
            f"| {d.get('R_rel', float('nan')):.1e} | {d.get('A_rel', float('nan')):.1e} "
            f"| {d.get('Rdot_rel', float('nan')):.1e} | {d.get('omega_body_rel', float('nan')):.1e} "
            f"| {d['legacy_seconds']:.3f} / {d['v2_seconds']:.3f} "
            f"| {d['legacy_us_per_step']:.1f} / {d['v2_us_per_step']:.1f} "
            f"| {d['legacy_us_per_step'] / d['v2_us_per_step']:.1f}x |"
        )
    return _finish(report, lines, args)


def _finish(report, lines, args):
    table = "\n".join(lines)
    print(table)
    if args.json:
        args.json.write_text(json.dumps(report, indent=2))
    if args.markdown:
        args.markdown.write_text(table + "\n")
    return report


if __name__ == "__main__":
    main()
