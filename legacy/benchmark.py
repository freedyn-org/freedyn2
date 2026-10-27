"""freedyn2 compare: legacy FreeDyn and FreeDyn 2.0 on the same .fds file, accuracy and speed.

Usage:
  freedyn2 compare model.fds [...] --legacy build/legacy/build/FreeDyn [--runtime build/freedyn2_run]
                   [--refine 16] [--repeat 3] [--markdown report.md] [--json report.json]

Both programs read the same .fds and run it as given (its step size, tolerance and end time);
FreeDyn 2.0 with its own methods (profile v2). Single-threaded, best of --repeat runs.

Accuracy: against a converged solution, computed by both programs themselves, so it needs no
model-specific reference code. Each program runs the model again with the maximum step size
divided by --refine and the tolerance by --refine^2. The FreeDyn 2.0 fine run is the
reference; a second one with half that step size estimates its uncertainty, and the legacy
fine run cross-checks it (an independent program converging to the same solution: its error
drops with the step size). Errors are the largest deviations over all output times and bodies:
  position   |R - R_ref| / max(1, |R_ref|)              (relative to the model size, at least 1 m)
  rotation   |A - A_ref| (rotation matrix entries)       (~ angle in rad)
  velocity   max of |omega - omega_ref| / max |omega_ref| and |Rdot - Rdot_ref| / max |Rdot_ref|
For chaotic or unstable motion (a body flipping about its intermediate axis, a double pendulum)
any two accurate solutions drift apart after a while, and a maximum over the whole run measures
nothing. The comparison window therefore ends where the two FreeDyn 2.0 reference runs start
to disagree by more than HORIZON_TOL (the "trust horizon", reported); for regular motion it is
the whole run. Within the window, "accurate until" is the time at which a program's deviation
first exceeds VALID_TOL: for chaotic motion every finite step size loses the solution after a
while, and the later the better. FreeDyn 2.0 counts as equally accurate when it stays accurate
at least as long and its errors are within 10 % or the reference uncertainty of the legacy ones.

Timing: legacy FreeDyn as one program run (input, setup, simulation, result files).
FreeDyn 2.0 split into build (import, model file, model plugin; once per model, independent of
the simulated time; cold: without the plugin cache, cached: with it) and simulation (runtime: load, setup,
simulation, result file).
"""

import argparse
import json
import os
import re
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "generator"))

from compare import LEGACY_SUCCESS, absolute_data_paths  # noqa: E402
from results import read_rigid_body_results, read_v2_results  # noqa: E402

ENV = dict(os.environ, OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
HORIZON_TOL = 1e-2
VALID_TOL = 1e-2


def _set(text, key, value):
    text, n = re.subn(rf"({key}\s*=\s*)\S+", lambda m: m.group(1) + repr(value), text)
    if n != 1:
        raise ValueError(f"{key} not found once in the .fds")
    return text


def run_legacy(exe, fds, work, repeat, refine=1):
    """Results and best wall-clock time of legacy FreeDyn; refine > 1: finer step and tolerance."""
    import subprocess

    from fdgen.fds import load_fds

    text = absolute_data_paths(fds.read_text(encoding="latin-1"), fds.parent)
    if refine > 1:
        s = load_fds(fds, "legacy")["solver"]
        h, eps = s["step"], s["eps"]
        text = _set(_set(text, "SolverMaxTimeStep", h / refine), "GlobalErrorTolerance", eps / refine**2)
    work.mkdir(parents=True, exist_ok=True)
    model = work / fds.name
    model.write_text(text, encoding="latin-1")
    best = np.inf
    for _ in range(repeat):
        t0 = time.perf_counter()
        r = subprocess.run([str(exe), str(model), "no"], cwd=work, env=ENV, capture_output=True, text=True)
        best = min(best, time.perf_counter() - t0)
        if r.returncode != LEGACY_SUCCESS and refine > 1:
            # Observed with the car model of examples/scaling: the refined run, started after the
            # other runs of a comparison, ends at once with SIM_ERROR in output mode "no", but
            # runs through in mode "screen" (and in mode "no" when started alone). Not timed.
            r = subprocess.run([str(exe), str(model), "screen"], cwd=work, env=ENV, capture_output=True, text=True)
        if r.returncode != LEGACY_SUCCESS:
            raise RuntimeError(f"legacy FreeDyn failed on {fds.name} (exit code {r.returncode}, refine {refine}):"
                               f"\n{r.stdout}\n{r.stderr}")
    return best, read_rigid_body_results(model.with_suffix(""))


def run_v2(runtime, fds, work, repeat, refine=1, cold=False):
    """Results, build time, best simulation time and step count of FreeDyn 2.0 (profile v2).
    cold: build without the plugin cache."""
    from fdgen import cli

    work.mkdir(parents=True, exist_ok=True)
    saved = os.environ.get("FREEDYN2_PLUGIN_CACHE")
    if cold:
        os.environ["FREEDYN2_PLUGIN_CACHE"] = "off"
    try:
        fdm, model, build_seconds = cli.build(fds, work, "v2")
    finally:
        if cold:
            os.environ.pop("FREEDYN2_PLUGIN_CACHE")
            if saved is not None:
                os.environ["FREEDYN2_PLUGIN_CACHE"] = saved
    s = model["solver"]
    options = ["--step", repr(s["step"] / refine), "--tolerance", repr(s["eps"] / refine**2)] if refine > 1 else []
    best = np.inf
    for _ in range(repeat):
        seconds, stdout = cli.simulate(runtime, fdm, work / "v2.csv", options, env=ENV)
        best = min(best, seconds)
    res = read_v2_results(work / "v2.csv", list(model["bodies"]))
    res["n_steps"] = int(re.search(r": (\d+) steps", stdout).group(1))
    return build_seconds, best, res


def _scales(ref):
    """Normalization per body from the reference: (position, omega, Rdot)."""
    return [(max(1.0, np.max(np.abs(b["R"]))), max(np.max(np.abs(b["omega_body"])), 1e-300),
             max(np.max(np.abs(b["Rdot"])), 1e-300)) for b in ref["bodies"]]


def deviations(res, ref, scales):
    """Per output time: position, rotation and velocity deviation (max over bodies), shape (n, 3)."""
    if len(res["t"]) != len(ref["t"]) or np.max(np.abs(res["t"] - ref["t"])) > 1e-9:
        raise ValueError("output times differ")
    rows = []
    for b, r, (sr, sw, sv) in zip(res["bodies"], ref["bodies"], scales):
        pos = np.max(np.abs(b["R"] - r["R"]), axis=1) / sr
        rot = np.max(np.abs(b["A"] - r["A"]), axis=(1, 2))
        vel = np.maximum(np.max(np.abs(b["omega_body"] - r["omega_body"]), axis=1) / sw,
                         np.max(np.abs(b["Rdot"] - r["Rdot"]), axis=1) / sv)
        rows.append(np.stack([pos, rot, vel], axis=1))
    return np.max(rows, axis=0)


def horizon(dev, t):
    """Last output time up to which the deviations stay below HORIZON_TOL (None: whole run)."""
    bad = np.flatnonzero(np.max(dev, axis=1) > HORIZON_TOL)
    return None if bad.size == 0 else float(t[max(bad[0] - 1, 0)])


def benchmark(fds, legacy_exe, runtime, repeat, refine):
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        t_legacy, legacy = run_legacy(legacy_exe, fds, work / "legacy", repeat)
        build, t_v2, v2 = run_v2(runtime, fds, work / "v2", repeat, cold=True)
        build_cached, _, _ = run_v2(runtime, fds, work / "v2_cached", 1)
        _, legacy_fine = run_legacy(legacy_exe, fds, work / "legacy_fine", 1, refine)
        _, _, ref = run_v2(runtime, fds, work / "v2_fine", 1, refine)
        _, _, check = run_v2(runtime, fds, work / "v2_finer", 1, 2 * refine)
    scales = _scales(ref)
    unc = deviations(check, ref, scales)
    t_end = horizon(unc, ref["t"])
    window = ref["t"] <= (np.inf if t_end is None else t_end + 1e-12)
    dev = lambda res: deviations(res, ref, scales)[window]
    err = lambda res: [float(v) for v in np.max(dev(res), axis=0)]
    t_window = ref["t"][window]
    until = lambda res: horizon(dev(res), t_window) if np.max(dev(res)) > VALID_TOL else None
    return {
        "model": fds.stem, "end_time": float(ref["t"][-1]), "horizon": t_end, "refine": refine,
        "error": {"legacy": err(legacy), "v2": err(v2), "legacy_fine": err(legacy_fine)},
        "accurate_until": {"legacy": until(legacy), "v2": until(v2)},
        "reference_uncertainty": [float(v) for v in np.max(unc[window], axis=0)],
        "steps_v2": v2["n_steps"],
        "seconds": {"legacy": t_legacy, "v2_build": build, "v2_build_cached": build_cached, "v2_simulation": t_v2},
    }


def verdict(d):
    # v2 counts as equally accurate within 10 % or the uncertainty of the reference.
    unc = d["reference_uncertainty"]
    worse = [k for k, a, b, c in zip(("position", "rotation", "velocity"), d["error"]["legacy"], d["error"]["v2"], unc)
             if b > 1.1 * a + c]
    u = d["accurate_until"]
    if u["v2"] is not None and (u["legacy"] is None or u["v2"] < u["legacy"]):
        worse.append("time accurate")
    accurate = "v2 equal or more accurate" if not worse else "v2 less accurate in " + ", ".join(worse)
    s = d["seconds"]
    return f"{accurate}, {s['legacy'] / s['v2_simulation']:.1f}x faster"


def markdown(report):
    hdr = ("| model | window [s] | error legacy (pos / rot / vel) | error v2 (pos / rot / vel) "
           "| accurate until [s] legacy / v2 | reference: uncertainty, legacy fine | legacy [s] "
           "| v2 build cold (cached) + simulation [s] | result |")
    lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
    e3 = lambda v: " / ".join(f"{x:.1e}" for x in v)
    for d in report:
        s, e = d["seconds"], d["error"]
        window = f"{d['horizon']:g} of {d['end_time']:g}" if d["horizon"] is not None else f"{d['end_time']:g}"
        u = " / ".join("end" if x is None else f"{x:g}" for x in d["accurate_until"].values())
        lines.append(f"| {d['model']} | {window} | {e3(e['legacy'])} | {e3(e['v2'])} | {u} "
                     f"| {max(d['reference_uncertainty']):.0e}, {max(e['legacy_fine']):.0e} | {s['legacy']:.3f} "
                     f"| {s['v2_build']:.3f} ({s['v2_build_cached']:.3f}) + {s['v2_simulation']:.3f} | {verdict(d)} |")
    return "\n".join(lines)


def main(argv=None):
    from fdgen import cli

    ap = argparse.ArgumentParser(prog="freedyn2 compare", description=__doc__.split("\n\n")[0])
    ap.add_argument("models", nargs="+", type=Path, help="legacy FreeDyn .fds files")
    ap.add_argument("--legacy", type=Path, required=True, help="legacy FreeDyn executable")
    ap.add_argument("--runtime", help="path of freedyn2_run")
    ap.add_argument("--refine", type=int, default=16, help="step size divisor of the reference runs")
    ap.add_argument("--repeat", type=int, default=3, help="timed runs per program (best counts)")
    ap.add_argument("--markdown", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args(argv)
    runtime = cli.find_runtime(args.runtime)
    report = []
    for fds in args.models:
        print(f"{fds.stem} ...", file=sys.stderr, flush=True)
        report.append(benchmark(fds.resolve(), args.legacy.resolve(), runtime, args.repeat, args.refine))
    table = markdown(report)
    print(table)
    if args.markdown:
        args.markdown.write_text(table + "\n")
    if args.json:
        args.json.write_text(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
