"""Parameter sweeps: one number of a model file over several values, simulated in parallel.

  run_sweep(model_path, path, values, runtime, workdir, overrides={}, ...) -> [csv path or error]
  kpi(t, y, name, t0=None, t1=None) -> float      key figure of a channel over a time range

The models are built one after the other (the elements of the first are reused by the others,
fdgen.custom_element), the simulations run in parallel, one freedyn2_run per value. The path
of the number is that of the viewer's tuned values (fdgen.view.server.apply_overrides), e.g.
parameters.k_susp or forces.spring.stiffness; the other tuned values (overrides) hold for all.
Used by the viewer (a fan of curves, key figures over the value) and the MCP tool sweep.
"""

import os
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

KPIS = {
    "max": lambda y: float(np.max(y)),
    "min": lambda y: float(np.min(y)),
    "max |x|": lambda y: float(np.max(np.abs(y))),
    "peak-peak": lambda y: float(np.max(y) - np.min(y)),
    "mean": lambda y: float(np.mean(y)),
    "rms": lambda y: float(np.sqrt(np.mean(np.square(y)))),
    "final": lambda y: float(y[-1]),
}


def kpi(t, y, name, t0=None, t1=None):
    """Key figure name (KPIS) of the values y(t) with t0 <= t <= t1 (default: all)."""
    t, y = np.asarray(t), np.asarray(y)
    keep = np.ones(len(t), dtype=bool)
    if t0 is not None:
        keep &= t >= t0
    if t1 is not None:
        keep &= t <= t1
    if not keep.any():
        raise ValueError(f"no output in [{t0}, {t1}]")
    return KPIS[name](y[keep])


class Cancelled(Exception):
    pass


def run_sweep(model_path, path, values, runtime, workdir, overrides=None, options=(), jobs=None,
              cancelled=lambda: False, done=lambda k, result: None, lock=None):
    """Simulate the model with path set to each of values. Returns per value the result CSV
    (Path) or the error message (str). done(k, result) is called as each run ends; cancelled()
    is polled (True: stop all, raise Cancelled); lock: held while building (CasADi)."""
    from .model import load_yaml, validate, write_fdm
    from .view.server import apply_overrides

    model_path, workdir = Path(model_path), Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    fdms = []
    for k, v in enumerate(values):
        if cancelled():
            raise Cancelled()
        d = workdir / f"v{k}"
        d.mkdir(exist_ok=True)
        try:
            if lock:
                lock.acquire()
            try:
                model = validate(apply_overrides(load_yaml(model_path), {**(overrides or {}), path: float(v)}),
                                 model_path.parent)
                fdm = d / f"{model['model']}.fdm"
                write_fdm(model, fdm)
            finally:
                if lock:
                    lock.release()
            fdms.append(fdm)
        except Exception as e:  # noqa: BLE001  (this value fails, the others run)
            fdms.append(f"{path} = {v:g}: {e}")

    def simulate(k):
        fdm = fdms[k]
        if isinstance(fdm, str):
            done(k, fdm)
            return fdm
        csv = fdm.parent / "result.csv"
        proc = subprocess.Popen([str(runtime), str(fdm), "-o", str(csv), *options], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
        while True:
            try:
                out, _ = proc.communicate(timeout=0.1)
                break
            except subprocess.TimeoutExpired:
                if cancelled():
                    proc.kill()
                    proc.communicate()
                    return None
        result = csv if proc.returncode == 0 else f"{path} = {values[k]:g}: freedyn2_run failed: {out.strip()[-400:]}"
        done(k, result)
        return result

    jobs = jobs or max(1, min(len(values), os.cpu_count() or 1))
    with ThreadPoolExecutor(jobs) as pool:
        results = list(pool.map(simulate, range(len(values))))
    if cancelled():
        raise Cancelled()
    return results
