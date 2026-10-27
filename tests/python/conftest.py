from pathlib import Path

import casadi as ca
import numpy as np
import pytest

from fdgen import rigid_body

import yaml

ROOT = Path(__file__).resolve().parents[2]

# A rough analytic road with a friction patch (split mu) under the left wheels
ROUGH_ROAD = {"harmonics": [{"amplitude": 0.01, "wavelength": 20.0, "heading": 0.0},
                            {"amplitude": 0.003, "wavelength": 2.5, "heading": 0.4, "phase": 1.2}],
              "slope": [0.0, 0.005], "friction": 1.0,
              "patch": {"friction": 0.3, "x": [60.0, 100.0], "y": [0.0, 5.0], "edge": 0.5}}


def road_height(x, y, road=ROUGH_ROAD):
    gx, gy = road.get("slope", [0.0, 0.0])
    return gx * x + gy * y + sum(w["amplitude"] * np.sin(2 * np.pi * (x * np.cos(w.get("heading", 0.0)) + y * np.sin(
        w.get("heading", 0.0))) / w["wavelength"] + w.get("phase", 0.0)) for w in road["harmonics"])


def car(road=None, end_time=1.0, shapes=True):
    """The car of examples/car_on_crg.yaml (built-in tires, 14 DOF) on a road (default: the
    OpenCRG test track, as an absolute path; a dict: that road), for end_time seconds."""
    m = yaml.safe_load((ROOT / "examples" / "car_on_crg.yaml").read_text())
    m["road"] = dict(road) if road is not None else {"crg": str(ROOT / "examples" / "data" / "test_track.crg")}
    m["solver"]["end_time"] = end_time
    if not shapes:
        for b in m["bodies"].values():
            b.pop("shapes", None)
    return m


@pytest.fixture(scope="session")
def elem():
    """Euler-parameter rigid body (legacy profile)."""
    return rigid_body.build_euler()


@pytest.fixture(scope="session")
def lie():
    """Lie group rigid body (FreeDyn 2.0 default)."""
    return rigid_body.build()


@pytest.fixture
def rng():
    return np.random.default_rng(1234)


def unit_quat(rng):
    """Random unit quaternion."""
    t = rng.normal(size=4)
    return t / np.linalg.norm(t)


def num(f, *args):
    """Numeric value of a CasADi function at numpy arguments."""
    return np.array(ca.DM(f(*[ca.DM(a) for a in args])))


def random_state(rng, unit=True):
    """Random local state of a rigid body; theta normalized if unit."""
    q = rng.normal(size=7)
    if unit:
        q[3:] /= np.linalg.norm(q[3:])
    return q, rng.normal(size=7), rng.normal(size=7), rng.normal(size=1)


P = np.array([2.0, 0.1, 0.2, 0.3, 0.3, -0.5, -9.81])  # m, J1..J3, gravity
COEF = np.array([-0.1, 0.3025, 0.6, 0.1])  # alpha, beta, gamma, h (large h: all blocks matter)
