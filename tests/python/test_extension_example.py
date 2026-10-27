"""The example extension of docs/extensions.md (examples/extension, force type aero_drag): found
through FREEDYN2_EXTENSIONS, adds its keys to the schema, checks its forces, its element type is
compiled into the model plugin and gives the right physics (terminal velocity), exact
derivatives, and the core alone rejects its type. Keeps the documented example working."""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from fdgen import cli, extensions, model as fdmodel
from fdgen.model import ModelError, load_schema, validate

ROOT = Path(__file__).resolve().parents[2]
G = 9.81


@pytest.fixture
def drag(monkeypatch):
    """The example extension, active for one test (the caches of the extensions and the schema
    are cleared before and after)."""
    monkeypatch.syspath_prepend(str(ROOT / "examples" / "extension"))
    monkeypatch.setenv("FREEDYN2_EXTENSIONS", "freedyn2_drag")
    for f in (extensions.extensions, fdmodel._validator):
        f.cache_clear()
    yield sys.modules.get("freedyn2_drag") or __import__("freedyn2_drag")
    monkeypatch.delenv("FREEDYN2_EXTENSIONS")
    for f in (extensions.extensions, fdmodel._validator):
        f.cache_clear()


def falling_ball(**force):
    return {"model": "ball", "gravity": [0, 0, -G],
            "bodies": {"ball": {"mass": 2.0, "inertia": [0.01, 0.01, 0.01], "position": [0, 0, 100]}},
            "forces": {"drag": dict({"type": "aero_drag", "body1": "ball", "coefficient": 0.05}, **force)},
            "solver": {"type": "HHT", "end_time": 8.0, "step": 1e-3, "output_step": 0.1}}


def test_the_extension_adds_its_type(drag):
    assert any(e.name == "drag" for e in extensions.extensions())
    force = load_schema()["$defs"]["force"]["properties"]
    assert "aero_drag" in force["type"]["enum"] and "coefficient" in force
    with pytest.raises(ModelError, match="aero_drag needs coefficient"):
        validate({**falling_ball(), "forces": {"drag": {"type": "aero_drag", "body1": "ball"}}})


def test_terminal_velocity(drag, tmp_path):
    """A ball falling with drag c v^2 reaches v_t = sqrt(m g / c); v(t) = v_t tanh(g t / v_t)."""
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    path = tmp_path / "ball.yaml"
    path.write_text(yaml.safe_dump(falling_ball()))
    fdm, _, _ = cli.build(path, tmp_path, "v2")
    assert "element aero_drag drag" in fdm.read_text() and "plugin ball_plugin" in fdm.read_text()
    cli.simulate(cli.find_runtime(), fdm, tmp_path / "r.csv")
    rows = list(csv.DictReader(open(tmp_path / "r.csv")))
    t = np.array([float(r["t"]) for r in rows])
    vz = np.array([float(r["ball.Rdot_z"]) for r in rows])
    vt = math.sqrt(2.0 * G / 0.05)
    np.testing.assert_allclose(vz, -vt * np.tanh(G * t / vt), atol=2e-3 * vt)
    np.testing.assert_allclose(float(rows[-1]["drag.airspeed"]), abs(vz[-1]), rtol=1e-6)
    assert float(rows[-1]["drag.force_z"]) == pytest.approx(2.0 * G, rel=1e-2)  # nearly balances gravity


def test_wind_and_derivatives(drag, rng):
    """The force against the wind, and the iteration matrix against finite differences."""
    from fdgen import rigid_body

    e = drag.extension.element_types["aero_drag"]("lie")
    lie = rigid_body.build()
    p = np.r_[0.2, 0.1, -0.3, 0.4, 3.0, -1.0, 0.5]  # point, c, wind
    q_n = np.r_[rng.normal(size=3), [1.0, 0.1, -0.2, 0.3] / np.linalg.norm([1.0, 0.1, -0.2, 0.3])]
    v0, beta, gamma, h = rng.normal(size=6), 0.2722, 0.54, 0.01
    coef = np.array([0.0, beta, gamma, h])
    x0, incr0 = rng.normal(size=6), 0.01 * rng.normal(size=6)

    def st(x):
        incr = incr0 + beta * h * h * x
        return np.array(lie.update(q_n, incr)).ravel(), v0 + gamma * h * x, incr

    def r(x):
        q, v, incr = st(x)
        return np.array(e.res(q, v, x, np.zeros(0), p, coef, incr, 0.0)[0]).ravel()

    q, v, incr = st(x0)
    J = np.array(e.iter(q, v, x0, np.zeros(0), p, coef, incr, 0.0))
    J_fd = np.column_stack([(r(x0 + 1e-6 * d) - r(x0 - 1e-6 * d)) / 2e-6 for d in np.eye(6)])
    assert np.max(np.abs(J - J_fd)) / max(1, np.max(np.abs(J_fd))) < 1e-6
    # at rest in a wind w: F = c |w| w along the wind
    q0, rest = np.r_[0, 0, 0, 1.0, 0, 0, 0], np.zeros(6)
    F = np.array(e.out(q0, rest, rest, p, 0.0)[0]).ravel()
    w = p[4:7]
    np.testing.assert_allclose(F, p[3] * np.linalg.norm(w) * w, rtol=1e-9)


def test_the_core_alone_rejects_the_type():
    for f in (extensions.extensions, fdmodel._validator):
        f.cache_clear()
    if any(e.name == "drag" for e in extensions.extensions()):
        pytest.skip("the example extension is installed")
    with pytest.raises(ModelError, match="aero_drag|coefficient"):
        validate(falling_ball())
