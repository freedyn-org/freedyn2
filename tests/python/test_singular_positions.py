"""A mechanism in a singular position: a parallelogram whose ground pivots lie above each other
hangs stretched in its static equilibrium (all links on one vertical line). There one more
constraint is redundant than in any other position. The run must find it after the static
equilibrium and then stay at rest; before, the integrator went unstable there (the position
1e-8 off the line grew by a factor of 30 every 10 ms and the run broke down)."""

import csv

import numpy as np
import pytest
import yaml

from fdgen import cli

Q_Y = [0.7071067811865476, -0.7071067811865476, 0, 0]  # joint z along global y


def stacked_parallelogram():
    rod = [0.01, 0.0833333333333333, 0.0833333333333333]
    return {"model": "stacked", "gravity": [0, 0, -9.81],
            "bodies": {"crank_a": {"mass": 1.0, "inertia": rod, "position": [0.5, 0, 0]},
                       "crank_b": {"mass": 1.0, "inertia": rod, "position": [0.5, 0, -0.5]},
                       "coupler": {"mass": 1.0, "inertia": [0.0208333333333333, 0.0208333333333333, 0.005],
                                   "position": [1, 0, -0.25]}},
            "joints": {"pivot_a": {"type": "revolute", "body1": "crank_a", "location": [0, 0, 0], "orientation": Q_Y},
                       "pivot_b": {"type": "revolute", "body1": "crank_b", "location": [0, 0, -0.5], "orientation": Q_Y},
                       "pin_a": {"type": "revolute", "body1": "coupler", "body2": "crank_a", "location": [1, 0, 0],
                                 "orientation": Q_Y},
                       "pin_b": {"type": "revolute", "body1": "coupler", "body2": "crank_b", "location": [1, 0, -0.5],
                                 "orientation": Q_Y}},
            "solver": {"type": "HHT", "alpha": -0.1, "statics": True, "end_time": 2.0, "step": 1e-3,
                       "output_step": 1e-2}}


def test_stretched_equilibrium_stays_at_rest(tmp_path):
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    path = tmp_path / "stacked.yaml"
    path.write_text(yaml.safe_dump(stacked_parallelogram()))
    fdm, _, _ = cli.build(path, tmp_path)
    _, stdout = cli.simulate(runtime, fdm, tmp_path / "r.csv")
    assert "redundant in this position only" in stdout
    rows = list(csv.DictReader(open(tmp_path / "r.csv")))
    x = np.array([float(r["coupler.R_x"]) for r in rows])
    vx = np.array([float(r["coupler.Rdot_x"]) for r in rows])
    z = np.array([float(r["coupler.R_z"]) for r in rows])
    assert z[0] == pytest.approx(-1.25, abs=1e-9)  # hanging stretched
    assert np.abs(x).max() < 1e-6 and np.abs(vx).max() < 1e-6
