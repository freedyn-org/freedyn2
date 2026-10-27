"""Checks of models and results (fdgen.checks, freedyn2 check): findings for impossible inertia,
units, topology, joints far from their bodies, springs, tires at the start; result findings;
references (mass, values, curves over time and over another channel)."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from conftest import ROUGH_ROAD, car
from fdgen import cli
from fdgen.checks import (check_model, check_references, check_results, degrees_of_freedom, report_text,
                          unused_parameters)
from fdgen.model import ModelError, validate

ROOT = Path(__file__).resolve().parents[2]

C = math.sqrt(0.5)
SOLVER = {"type": "HHT", "end_time": 0.5, "step": 1e-3, "output_step": 1e-2}


def pendulum(**extra):
    m = {"model": "p", "gravity": [0, 0, -9.81], "solver": SOLVER,
         "bodies": {"rod": {"mass": 1.0, "inertia": [0.01, 0.1, 0.1], "position": [0.5, 0, 0]}},
         "joints": {"pin": {"type": "revolute", "body1": "rod", "location": [0, 0, 0], "orientation": [C, -C, 0, 0]}}}
    m.update(extra)
    return m


def codes(report):
    return {f["code"] for f in report["findings"]}


def test_clean_models_have_no_warnings():
    for path in ("examples/double_pendulum.yaml", "examples/compare/pendulum.fds"):
        r = check_model(cli.load(ROOT / path))
        assert not [f for f in r["findings"] if f["severity"] != "info"], path
    r = check_model(validate(car(ROUGH_ROAD), ROOT / "examples"))
    assert r["findings"] == [] and r["summary"]["degrees_of_freedom"] == 14


def test_degrees_of_freedom():
    m = validate(pendulum())
    assert degrees_of_freedom(m) == 1
    m = validate(pendulum(joints={"a": {"type": "revolute", "body1": "rod", "location": [0, 0, 0]},
                                  "b": {"type": "fixed", "body1": "rod", "location": [0, 0, 0]}}))
    assert degrees_of_freedom(m) == -5 and "overconstrained" in codes(check_model(m))


def test_impossible_inertia_and_units():
    m = pendulum(gravity=[0, 0, -9810])
    m["bodies"]["rod"].update(inertia=[0.01, 0.01, 0.5], mass=1e7)
    m["bodies"]["far"] = {"mass": 1.0, "inertia": [1, 1, 1], "position": [5000.0, 0, 0]}
    r = check_model(validate(m))
    assert {"inertia_impossible", "units_gravity", "units_length", "units_mass", "unconnected"} <= codes(r)
    impossible = next(f for f in r["findings"] if f["code"] == "inertia_impossible")
    assert impossible["severity"] == "error" and impossible["elements"] == ["rod"]


def test_topology_and_joint_far():
    m = pendulum()
    m["bodies"]["b2"] = {"mass": 1.0, "inertia": [0.01, 0.01, 0.01], "position": [3, 0, 0]}
    m["bodies"]["b3"] = {"mass": 1.0, "inertia": [0.01, 0.01, 0.01], "position": [3, 1, 0]}
    m["joints"]["link"] = {"type": "spherical", "body1": "b2", "body2": "b3", "location": [3, 0.5, 0]}
    m["joints"]["typo"] = {"type": "revolute", "body1": "rod", "location": [0, 0, 50]}  # 50 instead of 0.5
    r = check_model(validate(m))
    nog = next(f for f in r["findings"] if f["code"] == "not_grounded")
    assert set(nog["elements"]) == {"b2", "b3"}
    far = next(f for f in r["findings"] if f["code"] == "joint_far")
    assert "typo" in far["elements"]


def test_springs_and_tires_at_the_start():
    m = pendulum(forces={"s0": {"type": "spring_damper", "body1": "rod", "point1": [0, 0, 0], "point2": [0, 0, 0],
                                "stiffness": 1.0},
                         "s1": {"type": "spring_damper", "body1": "rod", "point1": [1, 0, 0], "point2": [1, 0, 1],
                                "stiffness": 1.0, "length": 3.0}})
    r = check_model(validate(m))
    assert {"spring_zero_length", "spring_preload"} <= codes(r)
    m = car(ROUGH_ROAD)
    m["bodies"]["wheel_fl"]["position"] = [2.6, 0.8, 0.5]
    m["bodies"]["wheel_fr"]["position"] = [2.6, -0.8, 0.15]
    r = check_model(validate(m, ROOT / "examples"))
    assert {"tire_in_air", "tire_deep"} <= codes(r)


def results_of(**cols):
    t = np.linspace(0, 1, 101)
    out = {"t": t}
    out.update({k: v(t) if callable(v) else v for k, v in cols.items()})
    return out


def test_result_findings():
    m = validate(pendulum())
    r = results_of(**{"rod.theta_norm": lambda t: 1 + 1e-4 * t, "rod.R_x": lambda t: np.where(t > 0.5, np.nan, 0.0),
                      "rod.Rdot_x": lambda t: 0 * t, "rod.Rdot_y": lambda t: 0 * t, "rod.Rdot_z": lambda t: -9.81 * t})
    found = codes(check_results(m, r))
    assert {"not_finite", "rotation_drift", "start_transient"} <= found


def test_references(tmp_path):
    (tmp_path / "curve.csv").write_text("# measured\ntime,z\n0,0\n0.5,0.51\n1,1\n")
    (tmp_path / "sweep.csv").write_text("0.0,0.0\n0.5,0.25\n1.0,1.0\n")
    m = pendulum(checks={
        "mass": {"total": 1.0, "tolerance": 0.01},
        "values": [{"channel": "a", "time": 0.5, "value": 0.5, "tolerance": 1e-9},
                   {"channel": "missing", "value": 0.0, "tolerance": 1.0}],
        "curves": [{"channel": "a", "file": "curve.csv", "x": "time", "y": "z", "tolerance": 0.02},
                   {"name": "b over a", "channel": "b", "x_channel": "a", "file": "sweep.csv", "tolerance": 1e-3}]})
    model = validate(m, tmp_path)
    assert Path(model["checks"]["curves"][0]["file"]).is_absolute()
    refs = {r["name"]: r for r in check_references(model, results_of(a=lambda t: t, b=lambda t: t * t))}
    assert refs["total mass"]["passed"]
    assert refs["a at t = 0.5"]["passed"] and not refs["missing at t = 0"]["passed"]
    assert refs["a"]["passed"] and np.isclose(refs["a"]["max_deviation"], 0.01) and refs["a"]["at_x"] == 0.5
    assert refs["b over a"]["passed"] and "note" not in refs["b over a"]
    # a channel that goes back and forth is no sweep
    back = check_references(model, results_of(a=lambda t: np.sin(6 * t) ** 2, b=lambda t: t))
    assert "note" in next(r for r in back if r["name"] == "b over a")
    with pytest.raises(ModelError, match="not found"):
        validate(pendulum(checks={"curves": [{"channel": "a", "file": "nope.csv", "tolerance": 1}]}), tmp_path)
    text = report_text(check_model(model), check_results(model, results_of(a=lambda t: t, b=lambda t: t * t)))
    assert "REFERENCE missing at t = 0: not checked" in text


def test_check_command(tmp_path, capsys):
    path = tmp_path / "p.yaml"
    path.write_text(yaml.safe_dump(pendulum(checks={"mass": {"total": 2.0, "tolerance": 0.1}})))
    assert cli.main(["check", str(path)]) == 1  # the mass reference fails
    assert "total mass: FAILED" in capsys.readouterr().out
    path.write_text(yaml.safe_dump(pendulum()))
    assert cli.main(["check", str(path)]) == 0


def test_unused_parameters():
    m = {"model": "m", "parameters": {"k": 1.0, "w": 2.0, "spare": 3.0},
         "solver": {"type": "HHT", "end_time": 1.0, "step": 1e-3},
         "bodies": {"b": {"mass": 1.0, "inertia": [1, 1, 1]}},
         "forces": {"f": {"type": "expression_force", "body1": "b", "force": "k * sin(w * t)", "direction": [0, 0, 1]}}}
    model = validate(m)
    assert unused_parameters(model) == ["spare"]
    (f,) = [f for f in check_model(model)["findings"] if f["code"] == "unused_parameters"]
    assert f["severity"] == "info" and f["elements"] == ["spare"]
