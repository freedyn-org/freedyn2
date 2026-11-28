"""Contacts (force type contact): the law against independent numpy code, and runs against
closed-form and independently integrated references: the onset of a falling ball's first
contact, its Hertz penetration and energy, the coefficient of restitution of a damped impact,
a sliding sphere that starts to roll (5/7 of its speed), the velocity exchange of two equal
spheres. The runtime locates the onset and the end of each contact (events)."""

import csv
import math

import numpy as np
import pytest
import yaml

from conftest import unit_quat

from fdgen import cli, forces, rigid_body
from fdgen.errors import ModelError
from fdgen.model import validate
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

G = 9.81


def A(t):
    return np.array(rigid_body.rotation_matrix(t))


def test_sphere_plane_law_against_numpy(rng):
    """Penetration, normal force (Hunt-Crossley), friction (Coulomb with Stribeck, regularized) and
    the generalized force F at P, (P - R) x F, written independently."""
    e = forces.build("contact_sphere_plane_ground")
    for _ in range(5):
        q = np.r_[rng.normal(size=3), unit_quat(rng)]
        v = rng.normal(size=6)
        n2 = rng.normal(size=3)
        n2 /= np.linalg.norm(n2)
        k, ex, d, mu, mus, vs, veps = 1e4, 1.5, 30.0, 0.4, 0.6, 0.3, 0.05
        u1, x2 = rng.normal(size=3) * 0.1, rng.normal(size=3)
        R, Am = q[0:3], A(q[3:7])
        c = R + Am @ u1
        r = n2 @ (c - x2) + 0.02  # 2 cm in contact
        p = np.r_[u1, r, x2, n2, k, ex, d, mu, mus, vs, veps]
        omega = Am @ v[3:6]
        delta = r - n2 @ (c - x2)
        ddot = -n2 @ (v[0:3] + np.cross(omega, c - R))
        Fn = max(0.0, k * delta**ex + d * delta**ex * ddot)
        P = c - (r - delta / 2) * n2
        vP = v[0:3] + np.cross(omega, P - R)
        vt = vP - (vP @ n2) * n2
        mu_v = mu + (mus - mu) * math.exp(-(vt @ vt) / vs**2)
        F = Fn * n2 - mu_v * Fn * vt / math.sqrt(vt @ vt + veps**2)
        Q = np.r_[F, Am.T @ np.cross(P - R, F)]  # rotational part in the body frame (omega_bar)
        out = dict(zip(e.out.name_out(), e.out.call([q, v, np.zeros(6), p, 0.0])))
        assert float(out["penetration"]) == pytest.approx(delta, rel=1e-12)
        assert float(out["penetration_rate"]) == pytest.approx(ddot, rel=1e-12, abs=1e-12)
        assert float(out["normal_force"]) == pytest.approx(Fn, rel=1e-12)
        np.testing.assert_allclose(np.array(out["force"]).ravel(), F, rtol=1e-12, atol=1e-9)
        f = np.array(e.res(q, v, np.zeros(6), np.zeros(0), p, np.r_[0, .25, .5, .01], 0 * v, 0.0)[1]).ravel()
        np.testing.assert_allclose(f, -Q, rtol=1e-12, atol=1e-9)  # f = h - Q, h = 0 for a force
        assert np.array(e.event(q, v, p, 0.0)).item() == pytest.approx(delta, rel=1e-12)


def test_contact_parameters_are_checked(tmp_path):
    base = {"model": "m", "bodies": {"b": {"mass": 1.0, "inertia": [1, 1, 1]}},
            "solver": {"type": "HHT", "end_time": 1.0, "step": 1e-3}}
    from fdgen.model import write_fdm

    for force, msg in (({"stiffness": "1.0e7", "radius": 0.1}, "must be a number"),
                       ({"stiffness": 1e7, "radius": 0.1, "geometry": "sphere_sphere"}, "radius2"),
                       ({"stiffness": 1e7}, "needs radius")):
        with pytest.raises(ModelError, match=msg):
            m = validate({**base, "forces": {"c": {"type": "contact", "body1": "b", **force}}}, tmp_path)
            write_fdm(m, tmp_path / "m.fdm")


# --- runs ---------------------------------------------------------------------------------------

def run(tmp_path, model, options=()):
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    path = tmp_path / f"{model['model']}.yaml"
    path.write_text(yaml.safe_dump(model))
    fdm, _, _ = cli.build(path, tmp_path)
    out = tmp_path / "r.csv"
    log = tmp_path / "events.txt"
    _, stdout = cli.simulate(runtime, fdm, out, ["--event-log", str(log), *options])
    rows = list(csv.DictReader(open(out)))
    y = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    events = [line.split() for line in log.read_text().splitlines()[1:]]
    return y, [(float(t), name, int(d)) for t, name, d in events], stdout


def ball(h0=1.0, r=0.05, m=0.5, k=1e7, d=0.0, **solver):
    return {"model": "ball", "gravity": [0, 0, -G],
            "bodies": {"ball": {"mass": m, "inertia": [0.4 * m * r * r] * 3, "position": [0, 0, h0 + r]}},
            "forces": {"floor": {"type": "contact", "body1": "ball", "radius": r, "stiffness": k, "damping": d}},
            "solver": {"type": "HHT", "rho_inf": 0.9, "end_time": 3.0, "step": 1e-4, "output_step": 1e-3, **solver}}


@pytest.mark.parametrize("adaptive", [False, True])
def test_bouncing_ball(tmp_path, adaptive):
    """Onset of the first contact at sqrt(2 h0 / g), to the event tolerance; Hertz penetration
    (5/4 m v^2 / k)^(2/5); kinetic + potential + elastic energy kept over three bounces. With the
    fixed step and with the PI step size control. With a fixed step the trapezoidal-like scheme is
    time-reversible and the energy errors of loading and unloading cancel over an impact; with
    variable steps they do not, and the energy error is that of a second-order method at eps."""
    m, k, h0 = 0.5, 1e7, 1.0
    solver = {"adaptive": True, "eps": 1e-8, "step": 1e-3} if adaptive else {}
    y, events, _ = run(tmp_path, ball(h0, m=m, k=k, **solver))
    assert [d for _, _, d in events] == [1, -1] * 3 and all(name == "floor" for _, name, _ in events)
    assert events[0][0] == pytest.approx(math.sqrt(2 * h0 / G), abs=2e-7)
    v = math.sqrt(2 * G * h0)
    assert y["floor.penetration"].max() == pytest.approx((1.25 * m * v * v / k) ** 0.4, rel=2e-2)  # sampled
    E = y["ball.T"] + m * G * y["ball.R_z"] + k / 2.5 * np.maximum(y["floor.penetration"], 0) ** 2.5
    assert abs(E[-1] - E[0]) / E[0] < (5e-4 if adaptive else 1e-4)
    # the bounces are alike: the second flight as long as twice the first fall
    assert events[2][0] - events[1][0] == pytest.approx(2 * events[0][0], rel=1e-3)


def restitution_rk4(m, k, ex, d, v0, g=G):
    """Exit / impact velocity of the 1-dof impact m z'' = -m g + max(0, k pen^e + d pen^e pen'),
    pen = -z, integrated with RK4 at a fine step from the onset (z = 0, z' = -v0)."""
    def acc(z, zd):
        pen, pend = -z, -zd
        F = max(0.0, k * max(pen, 0) ** ex + d * max(pen, 0) ** ex * pend) if pen > 0 else 0.0
        return -g + F / m

    z, zd, h = 0.0, -v0, 1e-8
    for _ in range(10_000_000):
        k1 = (zd, acc(z, zd))
        k2 = (zd + h / 2 * k1[1], acc(z + h / 2 * k1[0], zd + h / 2 * k1[1]))
        k3 = (zd + h / 2 * k2[1], acc(z + h / 2 * k2[0], zd + h / 2 * k2[1]))
        k4 = (zd + h * k3[1], acc(z + h * k3[0], zd + h * k3[1]))
        z += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        zd += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        if z > 0:
            return zd / v0
    raise RuntimeError("no exit")


def test_coefficient_of_restitution(tmp_path):
    """A damped impact (Hunt-Crossley): the rebound speed over the impact speed, from the time
    between the first two contacts, against an independent RK4 integration of the impact."""
    m, k, d, h0 = 0.5, 1e6, 1.5e5, 0.5
    y, events, _ = run(tmp_path, ball(h0, m=m, k=k, d=d, end_time=1.5, step=2e-5, output_step=1e-3))
    v0 = math.sqrt(2 * G * h0)
    e_sim = G * (events[2][0] - events[1][0]) / 2 / v0  # flight time 2 v1 / g
    e_ref = restitution_rk4(m, k, 1.5, d, v0)
    assert 0.5 < e_ref < 0.8  # a clearly damped impact
    assert e_sim == pytest.approx(e_ref, rel=2e-3)


def test_sliding_sphere_starts_to_roll(tmp_path):
    """A solid sphere sliding without spin at v0 on a plane with friction mu: the friction slows it
    until it rolls, at t = 2 v0 / (7 mu g), with 5/7 v0."""
    m, r, mu, v0 = 1.0, 0.1, 0.3, 2.0
    model = {"model": "roll", "gravity": [0, 0, -G],
             "bodies": {"s": {"mass": m, "inertia": [0.4 * m * r * r] * 3, "position": [0, 0, r - 1e-5],
                              "velocity": [v0, 0, 0]}},
             "forces": {"floor": {"type": "contact", "body1": "s", "radius": r, "stiffness": 1e7, "damping": 1e4,
                                  "friction": mu}},
             "solver": {"type": "HHT", "rho_inf": 0.8, "statics": False, "end_time": 1.0, "step": 1e-4,
                        "output_step": 1e-3}}
    y, _, _ = run(tmp_path, model)
    t = y["t"]
    assert y["s.Rdot_x"][-1] == pytest.approx(5 / 7 * v0, rel=1e-3)
    rolling = t[np.argmax(y["floor.slip_velocity"] < 1e-3)]
    assert rolling == pytest.approx(2 * v0 / (7 * mu * G), rel=1e-2)
    # rolling: v = omega r (no slip), omega about y
    assert y["s.omega_y"][-1] * r == pytest.approx(y["s.Rdot_x"][-1], rel=1e-3)


def test_equal_spheres_exchange_their_velocities(tmp_path):
    """Two equal spheres, no damping, central impact: the moving one stops, the other takes its
    velocity; momentum is kept exactly (the forces are opposite)."""
    m, r, v = 1.0, 0.05, 1.0
    I = [0.4 * m * r * r] * 3
    model = {"model": "cradle", "gravity": [0, 0, 0],
             "bodies": {"a": {"mass": m, "inertia": I, "position": [0, 0, 0], "velocity": [v, 0, 0]},
                        "b": {"mass": m, "inertia": I, "position": [0.2, 0, 0]}},
             "forces": {"hit": {"type": "contact", "body1": "a", "body2": "b", "geometry": "sphere_sphere",
                                "radius": r, "radius2": r, "stiffness": 1e8}},
             "solver": {"type": "HHT", "rho_inf": 0.9, "end_time": 0.3, "step": 1e-5, "output_step": 1e-3}}
    y, events, _ = run(tmp_path, model)
    assert [d for _, _, d in events] == [1, -1]
    assert events[0][0] == pytest.approx(0.1 / v, abs=1e-7)
    assert abs(y["a.Rdot_x"][-1]) < 2e-3 * v and y["b.Rdot_x"][-1] == pytest.approx(v, rel=2e-3)
    np.testing.assert_allclose(y["a.Rdot_x"] + y["b.Rdot_x"], v, atol=1e-10)


def spinning_impact_rk4(m, r, I, k, ex, d, mu, veps, vx, vz, w, g=G):
    """Planar impact of a sphere with spin w (about y) on the plane z = 0, the same contact law,
    integrated with RK4 at a fine step from the touch: (vx, w) after it."""
    def rhs(s):
        _, z, vx, vz, w = s
        pen = r - z
        Fn = max(0.0, k * pen**ex + d * pen**ex * -vz) if pen > 0 else 0.0
        arm = r - pen / 2  # contact point P - R = (0, 0, -arm)
        slip = vx - arm * w
        Ft = -mu * Fn * slip / math.sqrt(slip * slip + veps * veps)
        return np.array([vx, vz, Ft / m, Fn / m - g, -arm * Ft / I])

    s, h = np.array([0.0, r, vx, vz, w]), 1e-8
    while True:
        k1 = rhs(s)
        k2 = rhs(s + h / 2 * k1)
        k3 = rhs(s + h / 2 * k2)
        k4 = rhs(s + h * k3)
        s = s + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if s[1] > r and s[3] > 0:
            return s[2], s[4]


def test_spinning_ball_impact(tmp_path):
    """examples/bouncing_ball.yaml: thrown with backspin, the friction of the first impact reverses
    the spin and the ball leaves rolling; vx and the spin after it against an independent RK4
    integration of the planar impact with the same law."""
    model = yaml.safe_load(open(ROOT / "examples" / "bouncing_ball.yaml"))
    model["solver"]["end_time"] = 0.6
    y, events, _ = run(tmp_path, model)
    b, f = model["bodies"]["ball"], model["forces"]["floor"]
    r, m = f["radius"], b["mass"]
    vz = -math.sqrt(2 * G * (b["position"][2] - r))
    vx_ref, w_ref = spinning_impact_rk4(m, r, b["inertia"][1], f["stiffness"], 1.5, f["damping"], f["friction"], 1e-3,
                                        b["velocity"][0], vz, b["angular_velocity"][1])
    i = np.searchsorted(y["t"], events[1][0] + 0.01)  # after the first impact
    assert y["ball.Rdot_x"][i] == pytest.approx(vx_ref, rel=2e-3)
    assert y["ball.omega_y"][i] == pytest.approx(w_ref, rel=2e-3)
    assert w_ref > 0 > b["angular_velocity"][1]  # the spin reversed


def test_check_warns_about_a_deep_start(tmp_path):
    from fdgen.checks import check_model

    m = ball(0.0)
    m["bodies"]["ball"]["position"] = [0, 0, 0.03]  # 2 cm into the floor, 40 % of the radius
    codes = [f["code"] for f in check_model(validate(m, tmp_path))["findings"]]
    assert "contact_deep" in codes
    assert "contact_deep" not in [f["code"] for f in check_model(validate(ball(1.0), tmp_path))["findings"]]
