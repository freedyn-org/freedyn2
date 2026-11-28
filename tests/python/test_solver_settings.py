"""Solver settings from YAML to the model file: generalized-alpha (rho_inf), the step size
controller, and the legacy controller of imported .fds models."""

import warnings
from pathlib import Path

import pytest

from fdgen.errors import ModelError
from fdgen.model import _solver, validate, write_fdm
from fdgen import cli

ROOT = Path(__file__).resolve().parents[2]
BASE = {"type": "HHT", "end_time": 1.0, "step": 1e-3}


def solver_line(tmp_path, solver, joints=False):
    m = {"model": "m", "bodies": {"b": {"mass": 1.0, "inertia": [1, 1, 1]}}, "solver": {**BASE, **solver}}
    if joints:
        m["joints"] = {"pin": {"type": "revolute", "body1": "b"}}
    model = validate(m, tmp_path)
    write_fdm(model, tmp_path / "m.fdm")
    return next(line for line in (tmp_path / "m.fdm").read_text().splitlines() if line.startswith("solver "))


def test_rho_inf_replaces_alpha(tmp_path):
    line = solver_line(tmp_path, {"rho_inf": 0.8})
    assert "rho_inf 0.8" in line and "alpha" not in line.replace("rho_inf", "")
    assert "alpha -0.1" in solver_line(tmp_path, {})  # HHT, the default
    with pytest.raises(ModelError, match="exclude each other"):
        _solver({**BASE, "alpha": -0.1, "rho_inf": 0.8})


def test_rho_inf_one_with_joints_warns(tmp_path):
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        solver_line(tmp_path, {"rho_inf": 1.0}, joints=True)
    assert any("rho_inf = 1" in str(x.message) for x in w)


def test_adaptive_takes_the_pi_controller(tmp_path):
    assert "controller pi" in solver_line(tmp_path, {"adaptive": True, "eps": 1e-6})
    assert "controller legacy" in solver_line(tmp_path, {"adaptive": True, "controller": "legacy"})
    assert "controller" not in solver_line(tmp_path, {})  # fixed step: no controller


@pytest.mark.parametrize("profile", ["legacy", "v2"])
def test_fds_models_keep_the_legacy_controller(profile):
    model = cli.load(ROOT / "examples" / "compare" / "pendulum.fds", profile)
    assert model["solver"]["adaptive"] and model["solver"]["controller"] == "legacy"
