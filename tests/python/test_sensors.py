"""Accelerations: body channels Rddot / omegadot from the accelerations HHT solves for, and
sensors (specific force, angular velocity and acceleration at a point, in a frame carried by
the body), against analytic motions. The Euler-parameter forms (legacy .fds models) are
compared with the Lie group ones in test_forces (test_euler_and_lie_versions_agree)."""

import csv
import math

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.model import ModelError, validate

G = 9.81


@pytest.fixture(autouse=True)
def runtime():
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def run(model, tmp_path):
    path, out = tmp_path / f"{model['model']}.yaml", tmp_path / f"{model['model']}.csv"
    path.write_text(yaml.safe_dump(model))
    assert cli.main(["run", str(path), "-o", str(out)]) == 0
    rows = list(csv.DictReader(open(out)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def vec(res, name, k=None):
    return np.array([res[f"{name}_{c}"] if k is None else res[f"{name}_{c}"][k] for c in "xyz"])


def test_rotor_centripetal_in_the_turning_frame(tmp_path):
    """A disk spinning at w about the vertical: at radius r a sensor turning with it measures
    -w^2 r radially and +g upwards, at every time; a frame turned by 90 deg about z sees the
    same vector in other components."""
    w, r = 10.0, 0.5
    c, s = math.cos(math.pi / 4), math.sin(math.pi / 4)
    res = run({"model": "rotor", "gravity": [0, 0, -G],
               "bodies": {"disk": {"mass": 2.0, "inertia": [0.1, 0.1, 0.2], "angular_velocity": [0, 0, w]}},
               "joints": {"axle": {"type": "revolute", "body1": "disk", "location": [0, 0, 0]}},
               "sensors": {"rim": {"body1": "disk", "point": [r, 0, 0]},
                           "turned": {"body1": "disk", "point": [r, 0, 0], "orientation": [c, 0, 0, s]}},
               "solver": {"type": "HHT", "alpha": -0.05, "end_time": 1.0, "step": 1e-3, "output_step": 0.05}},
              tmp_path)
    n = res["t"].size
    np.testing.assert_allclose(vec(res, "rim.specific_force"), np.tile([[-w * w * r], [0], [G]], n), atol=1e-6)
    np.testing.assert_allclose(vec(res, "rim.acceleration"), np.tile([[-w * w * r], [0], [0]], n), atol=1e-6)
    np.testing.assert_allclose(vec(res, "turned.specific_force"), np.tile([[0], [w * w * r], [G]], n), atol=1e-6)
    np.testing.assert_allclose(vec(res, "rim.angular_velocity"), np.tile([[0], [0], [w]], n), atol=1e-8)
    np.testing.assert_allclose(vec(res, "rim.angular_acceleration"), 0, atol=1e-6)
    np.testing.assert_allclose(vec(res, "disk.omegadot"), 0, atol=1e-6)
    # the position is global: on the circle of radius r
    np.testing.assert_allclose(np.hypot(res["rim.position_x"], res["rim.position_y"]), r, atol=1e-9)


def test_pendulum_released_at_rest(tmp_path):
    """Physical pendulum released horizontally: at t = 0 the angular acceleration is
    m g l / (I + m l^2), the center of mass accelerates downward by l times that; the pivot
    does not move, a sensor there measures |g|. Over time Rddot is the derivative of Rdot."""
    m, I, l = 2.0, 0.03, 0.5
    alpha0 = m * G * l / (I + m * l * l)
    c = math.cos(math.pi / 4)
    res = run({"model": "pendulum", "gravity": [0, 0, -G],
               "bodies": {"bob": {"mass": m, "inertia": [I, I, I], "position": [l, 0, 0]}},
               "joints": {"pivot": {"type": "revolute", "body1": "bob", "location": [0, 0, 0],
                                    "orientation": [c, -c, 0, 0]}},  # axis along global y
               "sensors": {"at_pivot": {"body1": "bob", "point": [0, 0, 0]},
                           "at_cm": {"body1": "bob"}},
               "solver": {"type": "HHT", "alpha": -0.05, "end_time": 1.0, "step": 1e-4, "output_step": 1e-3}},
              tmp_path)
    assert res["bob.Rddot_z"][0] == pytest.approx(-alpha0 * l, rel=1e-9)
    assert abs(res["bob.omegadot_y"][0]) == pytest.approx(alpha0, rel=1e-9)
    np.testing.assert_allclose(vec(res, "at_cm.acceleration", 0), [0, 0, -alpha0 * l], atol=1e-9)
    # the pivot stands still: no acceleration, the specific force is -g (in the turning frame);
    # HHT keeps the constraints on positions, on accelerations they hold to ~1e-5 m/s^2 here
    np.testing.assert_allclose(vec(res, "at_pivot.acceleration"), 0, atol=1e-6 * G * 10)
    np.testing.assert_allclose(np.linalg.norm(vec(res, "at_pivot.specific_force"), axis=0), G, atol=1e-5 * G)
    # sensor at the center of mass, frame = global axes at t = 0 turning with the bob
    # (about y), vs the body channels
    rddot = vec(res, "bob.Rddot")
    th = np.arctan2(-res["bob.R_z"], res["bob.R_x"])  # rotation of the bob about global y
    ca, sa = np.cos(th), np.sin(th)
    local_x, local_z = ca * rddot[0] - sa * rddot[2], sa * rddot[0] + ca * rddot[2]
    np.testing.assert_allclose(res["at_cm.acceleration_x"], local_x, atol=1e-6)
    np.testing.assert_allclose(res["at_cm.acceleration_z"], local_z, atol=1e-6)
    # Rddot is the derivative of Rdot (central differences, second order in the output step)
    t, rdot = res["t"], vec(res, "bob.Rdot")
    der = (rdot[:, 2:] - rdot[:, :-2]) / (t[2:] - t[:-2])
    np.testing.assert_allclose(der, rddot[:, 1:-1], atol=2e-3 * alpha0 * l)


def test_free_fall_feels_nothing(tmp_path):
    """A free body falls with g; an accelerometer on it measures zero."""
    res = run({"model": "fall", "gravity": [0, 0, -G],
               "bodies": {"box": {"mass": 1.0, "inertia": [0.1, 0.2, 0.3], "position": [0, 0, 10]}},
               "sensors": {"acc": {"body1": "box", "point": [0.2, 0.1, 10.3]}},
               "solver": {"type": "HHT", "alpha": -0.05, "end_time": 0.5, "step": 1e-3, "output_step": 0.1}},
              tmp_path)
    np.testing.assert_allclose(vec(res, "box.Rddot"), np.tile([[0], [0], [-G]], res["t"].size), atol=1e-9)
    np.testing.assert_allclose(vec(res, "acc.specific_force"), 0, atol=1e-9)


def test_sensor_needs_a_body():
    model = {"model": "m", "bodies": {"b": {"mass": 1.0, "inertia": [1, 1, 1]}},
             "sensors": {"s": {"body1": "nothing"}}, "solver": {"type": "HHT", "end_time": 1.0, "step": 0.01}}
    with pytest.raises(ModelError, match="sensor s: unknown body nothing"):
        validate(model)
