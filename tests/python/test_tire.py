"""Tire element (fdgen.tire): contact, Magic Formula limits, road surface."""

import numpy as np
import pytest

from fdgen import forces, tire

E = forces.build("tire")
FLAT = {"friction": 1.0}


def out(q, v, road=FLAT, **kw):
    p = tire.parameters(kw, road)
    names = [E.out.name_out(i) for i in range(E.out.n_out())]
    vals = E.out(q, v, 0 * v, p, 0.0)
    return {n: np.array(x).ravel() for n, x in zip(names, vals)}


def wheel(z=0.29, vx=10.0, omega=None, yaw=0.0):
    """Wheel center at height z, axle along y turned by yaw about z, rolling forward at vx."""
    q = np.r_[0.0, 0.0, z, np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    w = vx / z if omega is None else omega  # body-frame spin about the axle
    return q, np.r_[vx, 0.0, 0.0, 0.0, w, 0.0]


def test_static_load_is_vertical_spring():
    o = out(*wheel(z=0.29, vx=0.0, omega=0.0), kz=250000.0, cz=0.0)
    assert o["penetration"][0] == pytest.approx(0.01)
    assert o["force"][2] == pytest.approx(2500.0, rel=1e-4)  # softplus: eps_z^2 / (4 f) off
    np.testing.assert_allclose(o["force"][:2], 0.0, atol=1e-9)


def test_lift_off():
    o = out(*wheel(z=0.35))
    assert 0 < o["force"][2] < 1.0 and abs(o["force"][0]) < 1e-3


def test_longitudinal_slip_stiffness_and_peak():
    p = dict(tire.TIRE_DEFAULTS)
    z = p["r0"] - p["Fz0"] / p["kz"]  # nominal load: dfz = 0
    kappa = 1e-4
    o = out(*wheel(z=z, omega=10.0 * (1 + kappa) / z), cz=0.0)
    assert o["slip"][0] == pytest.approx(kappa, rel=3e-3)  # v_ref = sqrt(vx^2 + v_eps^2): -0.125 %
    Fz = o["force"][2]
    assert o["force"][0] == pytest.approx(p["pKx1"] * Fz * o["slip"][0], rel=1e-3)  # Kx kappa
    peak = max(out(*wheel(z=z, omega=10.0 * (1 + k) / z), cz=0.0)["force"][0] for k in np.linspace(0, 0.5, 51))
    assert peak <= p["pDx1"] * Fz * (1 + 1e-9) and peak > 0.95 * p["pDx1"] * Fz


def test_lateral_force_follows_the_heading_and_friction_patch_scales_it():
    p = dict(tire.TIRE_DEFAULTS)
    z = p["r0"] - p["Fz0"] / p["kz"]
    q, v = wheel(z=z, yaw=0.05)  # axle turned: the wheel heads 0.05 rad left of its velocity
    o = out(q, v, cz=0.0)
    # the lateral force pulls toward the heading (to the left): a steered wheel turns the car
    assert o["slip"][1] == pytest.approx(0.05, rel=3e-3) and o["force"][1] > 0
    patch = {"friction": 1.0, "patch": {"friction": 0.3, "x": [-1, 1], "y": [-1, 1], "edge": 0.01}}
    assert out(q, v, patch)["friction"][0] == pytest.approx(0.3)
    # the peak lateral force (over the slip angle) is mu Dy: it scales with the friction
    peak = lambda road: max(out(*wheel(z=z, yaw=a), road, cz=0.0)["force"][1] for a in np.linspace(0, 0.6, 121))
    assert peak(patch) / peak(FLAT) == pytest.approx(0.3, rel=1e-2)  # peak on a grid of 0.005 rad


def test_road_height_and_normal():
    road = {"harmonics": [{"amplitude": 0.02, "wavelength": 4.0, "phase": 0.5}], "slope": [0.0, 0.01]}
    q = np.r_[1.0, 0.5, 0.3, 1.0, 0, 0, 0]
    o = out(q, np.zeros(6), road)
    assert o["road_height"][0] == pytest.approx(0.02 * np.sin(2 * np.pi / 4.0 + 0.5) + 0.005)
    # the contact point lies on the road surface
    c = o["contact"]
    h = 0.02 * np.sin(2 * np.pi * c[0] / 4.0 + 0.5) + 0.01 * c[1]
    assert c[2] == pytest.approx(h, abs=2e-5)
