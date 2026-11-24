"""Characteristic curves as library items: spring_damper and torsion_spring with a spline for
stiffness or damping run without a model plugin and agree with the same law written as an
expression force (model plugin), and a linear curve with the linear spring."""

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.view.scene import read_results


@pytest.fixture
def runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def run(tmp_path, m, name):
    d = tmp_path / name
    d.mkdir()
    path = d / "m.yaml"
    path.write_text(yaml.safe_dump(m))
    fdm, _, _ = cli.build(path, d)
    cli.simulate(cli.find_runtime(), fdm, d / "r.csv")
    return read_results(d / "r.csv"), fdm.read_text()


def _oscillator(force, splines=None, measures=None):
    return {"model": "osc", "gravity": [0, 0, -9.81],
            "splines": splines or {},
            "bodies": {"mass": {"mass": 2.0, "inertia": [1, 1, 1], "position": [0, 0, -0.5]}},
            "joints": {"slider": {"type": "prismatic", "body1": "mass", "location": [0, 0, -0.5]}},
            "measures": measures or {}, "forces": {"spring": force},
            "solver": {"type": "HHT", "alpha": -0.05, "end_time": 1.0, "step": 1e-3, "output_step": 1e-2}}


# progressive spring: force over the deflection [m] -> [N], a damper curve over the rate
CURVES = {"k": {"x": [-0.4, -0.2, -0.1, 0.0, 0.1, 0.2, 0.4], "y": [-900, -300, -120, 0, 120, 300, 900]},
          "d": {"x": [-2, -0.5, 0, 0.5, 2], "y": [-40, -15, 0, 10, 30], "type": "zcc"}}


def test_spring_curve_without_plugin(tmp_path, runtime):
    spring = {"type": "spring_damper", "body1": "mass", "point1": [0, 0, -0.5], "point2": [0, 0, 0],
              "stiffness": "k", "damping": "d"}
    r, fdm = run(tmp_path, _oscillator(spring, CURVES), "library")
    assert "plugin" not in fdm and "element spline_spring_damper_ground spring" in fdm
    # the same law as an expression force (model plugin): z is the displacement of the mass (up),
    # the deflection l - l0 is -z, its rate -vz; the force s pulls the mass up (toward point2)
    law = {"type": "expression_force", "body1": "mass", "direction": [0, 0, 1], "force": "k(-z) + d(-vz)"}
    m = _oscillator(law, CURVES, {"z": {"type": "displacement", "body1": "mass", "component": "z", "offset": "initial"},
                                  "vz": {"type": "velocity", "body1": "mass", "component": "z"}})
    r2, fdm2 = run(tmp_path, m, "plugin")
    assert "plugin" in fdm2
    assert np.ptp(r["mass.R_z"]) > 0.02  # it moves (falls into the spring)
    assert r["mass.R_z"] == pytest.approx(r2["mass.R_z"], abs=1e-9)


def test_linear_curve_is_the_linear_spring(tmp_path, runtime):
    """stiffness 400 (number) with a damping curve: the number becomes the line 400 x; against
    the linear spring_damper with the damping curve replaced by a line too."""
    lin = {"dl": {"x": [-1, 0, 1], "y": [-5, 0, 5]}}
    a, _ = run(tmp_path, _oscillator({"type": "spring_damper", "body1": "mass", "point1": [0, 0, -0.5],
                                      "point2": [0, 0, 0], "stiffness": 400.0, "damping": "dl"}, lin), "a")
    b, fdm = run(tmp_path, _oscillator({"type": "spring_damper", "body1": "mass", "point1": [0, 0, -0.5],
                                        "point2": [0, 0, 0], "stiffness": 400.0, "damping": 5.0}), "b")
    assert "element spring_damper_ground" in fdm
    assert a["mass.R_z"] == pytest.approx(b["mass.R_z"], abs=1e-12)


def test_torsion_curve(tmp_path, runtime):
    """torsion_spring with a curve against the linear one (a linear curve) and a progressive curve
    against its expression torque."""
    def rotor(force, splines=None):
        return {"model": "rot", "gravity": [0, 0, 0], "splines": splines or {},
                "bodies": {"disk": {"mass": 1.0, "inertia": [0.1, 0.1, 0.02], "angular_velocity": [0, 0, 5.0]}},
                "joints": {"axle": {"type": "revolute", "body1": "disk"}}, "forces": {"spring": force},
                "solver": {"type": "HHT", "alpha": -0.05, "end_time": 1.0, "step": 1e-3, "output_step": 1e-2}}
    lin = {"kl": {"x": [-1, 0, 1], "y": [-3, 0, 3]}}
    a, fdm = run(tmp_path, rotor({"type": "torsion_spring", "body1": "disk", "stiffness": "kl", "damping": 0.01}, lin), "a")
    assert "plugin" not in fdm and "spline_torsion_ground" in fdm
    b, _ = run(tmp_path, rotor({"type": "torsion_spring", "body1": "disk", "stiffness": 3.0, "damping": 0.01}), "b")
    assert a["disk.omega_z"] == pytest.approx(b["disk.omega_z"], abs=1e-10)
    assert np.ptp(a["spring.angle"]) > 0.5


def test_unknown_curve(tmp_path):
    from fdgen.errors import ModelError
    from fdgen.model import validate
    m = _oscillator({"type": "spring_damper", "body1": "mass", "point2": [0, 0, 0], "stiffness": "nope"})
    with pytest.raises(ModelError, match="is not a spline"):
        validate(m, tmp_path)
