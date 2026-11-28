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


# --- curve forces -------------------------------------------------------------------------------

CURVE = {"c": {"x": [-3, -1, -0.2, 0, 0.3, 1, 3], "y": [-60, -12, -2, 0, 3, 15, 70]}}
MEASURES = {
    "displacement": {"type": "displacement", "body1": "a", "body2": "b", "component": "z", "frame": "a", "offset": "initial"},
    "velocity": {"type": "velocity", "body1": "a", "body2": "b", "component": "x"},
    "distance": {"type": "distance", "body1": "a", "body2": "b", "point1": [0.1, 0, 0], "offset": "initial"},
    "distance_rate": {"type": "distance_rate", "body1": "b", "body2": "a"},  # the other way round
    "angle": {"type": "angle", "body1": "a", "body2": "b", "vector": [1, 0, 0], "plane": [[1, 0, 0], [0, 1, 0]],
              "offset": "initial"},
    "angular_rate": {"type": "angular_rate", "body1": "a", "body2": "b", "vector": [1, 0, 0],
                     "plane": [[1, 0, 0], [0, 1, 0]]},
    "angular_velocity": {"type": "angular_velocity", "body1": "a", "body2": "b", "component": "z", "frame": "b"},
}
APPS = {"los": {"point1": [0.1, 0.05, 0], "point2": [0.9, 0.1, 0.2]},
        "dir": {"direction": [0.2, 0, 1], "frame": "body", "point1": [0.1, 0, 0]},
        "torque": {"axis": [0, 0.3, 1], "frame": "global"}}


def _pair(force, measure, ground=False):
    m = {"model": "pair", "gravity": [0, 0, 0], "splines": CURVE,
         "bodies": {"a": {"mass": 1.0, "inertia": [0.1, 0.12, 0.08], "velocity": [0.3, 0.2, -0.4],
                          "angular_velocity": [0.5, -1.0, 2.0]},
                    "b": {"mass": 2.0, "inertia": [0.2, 0.1, 0.15], "position": [1, 0.1, 0.2],
                          "angular_velocity": [0, 0.4, -0.6]}},
         "measures": {"m": measure}, "forces": {"f": force},  # short: some laws make it chaotic
         "solver": {"type": "HHT", "alpha": -0.05, "end_time": 0.25, "step": 1e-3, "output_step": 1e-2}}
    if ground:
        del m["bodies"]["b"]
        m["measures"]["m"] = {k: v for k, v in measure.items() if k not in ("body2",) and v != "b"}
    return m


@pytest.mark.parametrize("app", APPS)
@pytest.mark.parametrize("kind", MEASURES)
def test_curve_force_library_equals_expression(tmp_path, runtime, kind, app):
    """curve_force / curve_torque without a plugin against the same law as an expression force /
    torque."""
    from fdgen.curve_forces import as_expression
    force = {"type": "curve_torque" if app == "torque" else "curve_force", "body1": "a", "body2": "b", "measure": "m", "curve": "c", "scale": -0.5,
             "argument_scale": 2.0, "argument_offset": 0.05, **APPS[app]}
    m = _pair(force, MEASURES[kind])
    r, fdm = run(tmp_path, m, "library")
    assert "plugin" not in fdm and f"element curve_{kind}_{app}_bb f" in fdm
    m["forces"]["f"] = {k: v for k, v in as_expression(dict(force, name="f")).items() if k != "name"}
    r2, fdm2 = run(tmp_path, m, "plugin")
    assert "plugin" in fdm2
    assert np.abs(r["f.value"]).max() > 1e-3  # the law acts
    for ch in ("a.R_x", "a.R_z", "b.R_y", "a.omega_z", "f.value"):
        assert r[ch] == pytest.approx(r2[ch], abs=1e-9 * max(1.0, np.abs(r2[ch]).max())), ch


@pytest.mark.parametrize("kind", ["displacement", "angle", "angular_velocity"])
def test_curve_force_to_ground(tmp_path, runtime, kind):
    from fdgen.curve_forces import as_expression
    force = {"type": "curve_force", "body1": "a", "measure": "m", "curve": "c", "direction": [0, 0, 1]}
    m = _pair(force, MEASURES[kind], ground=True)
    r, fdm = run(tmp_path, m, "library")
    assert f"element curve_{kind}_dir_ground f" in fdm
    m["forces"]["f"] = {k: v for k, v in as_expression(dict(force, name="f")).items() if k != "name"}
    r2, _ = run(tmp_path, m, "plugin")
    assert r["a.R_z"] == pytest.approx(r2["a.R_z"], abs=1e-9)


def test_curve_force_and_torque_keys(tmp_path):
    """A force has a direction, a torque an axis: the other key is an error, not another type."""
    from fdgen.errors import ModelError
    from fdgen.model import validate
    m = _pair({"type": "curve_force", "body1": "a", "measure": "m", "curve": "c", "axis": [0, 0, 1]},
              MEASURES["angle"], ground=True)
    with pytest.raises(ModelError, match="no axis"):
        validate(m, tmp_path)
    m["forces"]["f"] = {"type": "curve_torque", "body1": "a", "measure": "m", "curve": "c", "direction": [0, 0, 1],
                        "axis": [0, 0, 1]}
    with pytest.raises(ModelError, match="no direction"):
        validate(m, tmp_path)
    m["forces"]["f"] = {"type": "curve_torque", "body1": "a", "measure": "m", "curve": "c"}
    with pytest.raises(ModelError, match="needs axis"):
        validate(m, tmp_path)


def test_curve_force_not_covered_is_compiled(tmp_path, runtime):
    """A measure of other bodies: the same law, compiled (expression force)."""
    force = {"type": "curve_force", "body1": "a", "measure": "m", "curve": "c", "direction": [0, 0, 1]}
    m = _pair(force, MEASURES["velocity"])  # the measure is between a and b, the force on a and the ground
    r, fdm = run(tmp_path, m, "fallback")
    assert "plugin" in fdm and "curve_velocity" not in fdm
    assert np.abs(r["f.value"]).max() > 0


def test_legacy_import_makes_curve_forces(tmp_path, runtime):
    """The .fds importer maps a law a * SPL[MEAVAL[..], ..] to a curve_force (no plugin); a sum
    stays an expression force; both give the same motion."""
    import shutil

    from conftest import ROOT

    from fdgen.fds import load_fds
    from fdgen.fds.loads import curve_law
    meas = {"z": "m_z"}
    used = {"s_k_akima": {}}
    assert curve_law("s_k_akima(m_z)", meas, used) == (1.0, "s_k_akima", "m_z")
    assert curve_law("-(2.5*s_k_akima(m_z))", meas, used) == (-2.5, "s_k_akima", "m_z")
    assert curve_law("-(s_k_akima(m_z))+1", meas, used) is None
    assert curve_law("s_k_akima(m_z)-p_d*m_vz", meas, used) is None
    assert curve_law("s_k_akima(2*m_z)", meas, used) is None
    text = (ROOT / "examples" / "compare" / "spline_spring.fds").read_text()
    law = "SPL[MEAVAL[z],stiff]-PARVAL[d]*MEAVAL[vz]+ZCCSPL[SIMTIME[],drive]+0.5*AKISPL[SIMTIME[],drive]"
    assert law in text
    results = {}
    for name, new in (("curve", "2*SPL[MEAVAL[z],stiff]"), ("expression", "2*SPL[MEAVAL[z],stiff]+0*SIMTIME[]")):
        d = tmp_path / name
        d.mkdir()
        (d / "m.fds").write_text(text.replace(law, new))
        shutil.copytree(ROOT / "examples" / "compare" / "data", d / "data")
        m = load_fds(d / "m.fds")
        (force,) = [f for f in m["forces"] if f["name"] == "spring"]
        assert force["type"] == ("curve_force" if name == "curve" else "expression_force")
        fdm, _, _ = cli.build(d / "m.fds", d)
        assert ("plugin" in fdm.read_text()) == (name == "expression")
        cli.simulate(runtime, fdm, d / "r.csv")
        results[name] = read_results(d / "r.csv")
    a, b = results["curve"], results["expression"]
    key = next(k for k in a if k.endswith(".R_z"))
    assert np.ptp(a[key]) > 0 and a[key] == pytest.approx(b[key], abs=1e-9)
