"""Linearization and eigenvalues (fdgen.linearize): pendulum, free spinning body."""

import math

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.linearize import analyze


@pytest.fixture(autouse=True)
def runtime():
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def eigenvalues(m, work):
    """Eigenvalues of model m (dict) about its consistent initial state."""
    path = work / f"{m['model']}.yaml"
    path.write_text(yaml.safe_dump(m))
    fdm, _, _ = cli.build(path, work, "v2")
    cli.simulate(cli.find_runtime(), fdm, work / "r.csv")
    return analyze(path, fdm, work / "r.csv", 0.0)[0]


def solver():
    return {"type": "HHT", "alpha": -0.1, "end_time": 0.002, "step": 1.0e-3, "output_step": 1.0e-3}


def test_pendulum(tmp_path):
    m, I, l = 2.0, 0.03, 0.5
    model = {"model": "pendulum", "gravity": [0.0, 0.0, -9.81],
             "bodies": {"bob": {"mass": m, "inertia": [I, I, I], "position": [0.0, 0.0, -l]}},
             "joints": {"pivot": {"type": "revolute", "body1": "bob", "body2": "ground", "location": [0.0, 0.0, 0.0],
                                  "orientation": [math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0]}},
             "solver": solver()}
    lam = eigenvalues(model, tmp_path)
    w = max(abs(lam.imag))
    assert w == pytest.approx(math.sqrt(m * 9.81 * l / (I + m * l * l)), rel=1e-9)
    assert np.max(np.abs(lam.real)) < 1e-9


def test_free_spinning_body(tmp_path):
    J1, J2, J3, w3 = 1.0, 2.0, 3.0, 5.0  # spin about the major axis: stable, Euler frequency
    model = {"model": "spinner", "gravity": [0.0, 0.0, 0.0],
             "bodies": {"b": {"mass": 1.0, "inertia": [J1, J2, J3], "angular_velocity": [0.0, 0.0, w3]}},
             "solver": solver()}
    lam = eigenvalues(model, tmp_path)
    w = max(abs(lam.imag))
    assert w == pytest.approx(w3 * math.sqrt((J3 - J1) * (J3 - J2) / (J1 * J2)), rel=1e-9)
