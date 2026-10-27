"""Parameter sweeps (fdgen.sweep): key figures, parallel runs, a failing value, stopping."""

import math

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.sweep import KPIS, Cancelled, kpi, run_sweep
from fdgen.view.scene import read_results

C = math.sqrt(0.5)


def test_key_figures():
    t = np.linspace(0, 1, 11)
    y = np.sin(2 * np.pi * t) + 1
    assert kpi(t, y, "max") == pytest.approx(y.max()) and kpi(t, y, "min") == pytest.approx(y.min())
    assert kpi(t, y, "peak-peak") == pytest.approx(y.max() - y.min())
    assert kpi(t, y, "final") == y[-1] and kpi(t, y, "mean") == pytest.approx(y.mean())
    assert kpi(t, -y, "max |x|") == pytest.approx(y.max())
    assert kpi(t, y, "rms") == pytest.approx(math.sqrt(np.mean(y * y)))
    assert kpi(t, y, "max", 0.5, 1.0) == pytest.approx(y[t >= 0.5].max())  # a time window
    with pytest.raises(ValueError):
        kpi(t, y, "max", 2.0, 3.0)
    assert set(KPIS) == {"max", "min", "max |x|", "peak-peak", "mean", "rms", "final"}


def pendulum(path):
    m = {"model": "p", "gravity": [0, 0, -9.81], "solver": {"type": "HHT", "end_time": 0.5, "step": 1e-3, "output_step": 1e-2},
         "parameters": {"m": 1.0},
         "bodies": {"rod": {"mass": "=m", "inertia": [0.01, 0.1, 0.1], "position": [0.5, 0, 0], "orientation": [C, 0, 0, C]}},
         "joints": {"pin": {"type": "revolute", "body1": "rod", "location": [0, 0, 0], "orientation": [C, -C, 0, 0]}}}
    path.write_text(yaml.safe_dump(m))
    return path


@pytest.fixture
def runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def test_sweep_runs_each_value(tmp_path, runtime):
    model = pendulum(tmp_path / "p.yaml")
    seen = []
    results = run_sweep(model, "parameters.m", [0.5, 1.0, 2.0, -1.0], runtime, tmp_path / "s",
                        done=lambda k, r: seen.append(k))
    assert sorted(seen) == [0, 1, 2, 3]
    ok = [read_results(r)["rod.R_z"] for r in results[:3]]
    assert all(r.is_file() for r in results[:3])
    assert np.max(np.abs(ok[0] - ok[2])) > 1e-3  # the mass changes the swing (the inertia is fixed)
    assert isinstance(results[3], str) and results[3].startswith("parameters.m = -1:")  # an invalid mass


def test_sweep_stops(tmp_path, runtime):
    model = pendulum(tmp_path / "p.yaml")
    with pytest.raises(Cancelled):
        run_sweep(model, "parameters.m", [1.0, 2.0], runtime, tmp_path / "s", cancelled=lambda: True)
