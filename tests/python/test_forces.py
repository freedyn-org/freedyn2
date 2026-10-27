"""Force elements of the library: generalized forces, Jacobians, Lie group vs Euler parameters."""

import numpy as np
import pytest

from conftest import unit_quat

from fdgen import forces, rigid_body, tire
from fdgen.rigid_body import NQ

LIE = rigid_body.build()


def params(kind, rng):
    n_p = forces.KINDS[kind][1]
    p = rng.normal(size=n_p)
    if kind.startswith("torsion"):  # orthonormal x2, y2 on body 2 / the ground
        x2, y2 = np.linalg.qr(rng.normal(size=(3, 2)))[0].T
        p[3:6], p[6:9] = x2, y2
    if kind.startswith("spring_damper"):
        p[8] = abs(p[8])  # free length
    if kind == "tire":  # a passenger car tire on a rough road with a friction patch
        return np.array(tire.parameters({}, ROAD))
    if kind.startswith("bushing"):  # orthonormal frames, positive stiffness and damping
        p[3:12] = np.linalg.qr(rng.normal(size=(3, 3)))[0].T.ravel()
        p[15:24] = np.linalg.qr(rng.normal(size=(3, 3)))[0].T.ravel()
        p[24:36] = np.abs(p[24:36])
    return p


ROAD = {"harmonics": [{"amplitude": 0.01, "wavelength": 3.0, "heading": 0.3, "phase": 1.0},
                      {"amplitude": 0.002, "wavelength": 0.7, "heading": -1.2, "phase": 0.2}],
        "slope": [0.01, -0.02], "friction": 1.0, "patch": {"friction": 0.4, "x": [-0.5, 0.5], "y": [-1, 1]}}


def state(kind, rng):
    n = forces.KINDS[kind][0]
    q = np.concatenate([np.r_[rng.normal(size=3), unit_quat(rng)] for _ in range(n)])
    return n, q, rng.normal(size=6 * n)


def wheel_near_road(rng):
    """Wheel center 2 cm into the tire radius, axle near the y axis, rolling at about 10 m/s."""
    tilt = 0.1 * rng.normal(size=3)
    q = np.r_[0.3 * rng.normal(size=2), 0.28, unit_quat(rng) * 0 + np.r_[1.0, tilt] / np.linalg.norm(np.r_[1.0, tilt])]
    v = np.r_[10.0, 0.5 * rng.normal(size=2), 0.3 * rng.normal(size=1), 10.0 / 0.3 * (1 + 0.05 * rng.normal()),
              0.3 * rng.normal(size=1)]
    return q, v


def f_out(e, q, v, p, t=0.0):
    return np.array(e.res(q, v, np.zeros(v.size), np.zeros(0), p, np.r_[0, 0.25, 0.5, 1e-2], 0 * v, t)[1]).ravel()


@pytest.mark.parametrize("seed", range(3))
@pytest.mark.parametrize("kind", list(forces.KINDS))
def test_iteration_matrix_vs_fd_through_body_updates(kind, seed):
    """J vs FD of the residual with q_k = q_k,n o exp(incr_k), incr = incr0 + beta h^2 x,
    v = v0 + gamma h x (as for the joints; forces depend on q and v)."""
    rng = np.random.default_rng(seed)
    e = forces.build(kind)
    n, _, v0 = state(kind, rng)
    p = params(kind, rng)
    beta, gamma, h = 0.2722, 0.54, 0.05
    coef = np.array([0.0, beta, gamma, h])
    q_n = [np.r_[rng.normal(size=3), unit_quat(rng)] for _ in range(n)]
    x0, incr0 = rng.normal(size=6 * n), 0.3 * rng.normal(size=6 * n)
    if kind == "tire":  # in contact, rolling, small steps (smooth but strongly nonlinear law)
        q_w, v0 = wheel_near_road(rng)
        q_n, h = [q_w], 1e-3
        coef[3] = h
        incr0 *= 1e-3

    def st(x):
        incr = incr0 + beta * h * h * x
        q = np.concatenate([np.array(LIE.update(q_n[k], incr[6 * k:6 * k + 6])).ravel() for k in range(n)])
        return q, v0 + gamma * h * x, incr

    def r(x):
        q, v, incr = st(x)
        return np.array(e.res(q, v, x, np.zeros(0), p, coef, incr, 0.3)[0]).ravel()

    q, v, incr = st(x0)
    J = np.array(e.iter(q, v, x0, np.zeros(0), p, coef, incr, 0.3))
    eps = 1e-2 if kind == "tire" else 1e-5  # tire: h = 1e-3, beta h^2 eps = 3e-9 m; smaller eps is round-off
    J_fd = np.column_stack([(r(x0 + eps * d) - r(x0 - eps * d)) / (2 * eps) for d in np.eye(len(x0))])
    assert np.max(np.abs(J - J_fd)) / max(1, np.max(np.abs(J_fd))) < 1e-8


def _potential(kind, q, p):
    """Potential energy of the conservative force laws (d = 0, f0 = tau0 = 0)."""
    def A(t):
        return np.array(rigid_body.rotation_matrix(t))

    if kind.startswith("spring_damper"):
        P1 = q[0:3] + A(q[3:7]) @ p[0:3]
        P2 = p[3:6] if kind.endswith("ground") else q[7:10] + A(q[10:14]) @ p[3:6]
        return 0.5 * p[6] * (np.linalg.norm(P2 - P1) - p[8]) ** 2
    if kind.startswith("bushing"):
        F1 = A(q[3:7]) @ p[3:12].reshape(3, 3).T
        B = np.eye(3) if kind.endswith("ground") else A(q[10:14])
        P2 = p[12:15] if kind.endswith("ground") else q[7:10] + B @ p[12:15]
        F2 = B @ p[15:24].reshape(3, 3).T
        d = F2.T @ (q[0:3] + A(q[3:7]) @ p[0:3] - P2)
        Rr = F2.T @ F1
        psi = 0.5 * np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]])
        return 0.5 * p[24:27] @ d**2 + 0.5 * p[30:33] @ psi**2
    x1 = A(q[3:7]) @ p[0:3]
    B = np.eye(3) if kind.endswith("ground") else A(q[10:14])
    phi = np.arctan2(x1 @ (B @ p[6:9]), x1 @ (B @ p[3:6]))
    return 0.5 * p[9] * (phi - p[11]) ** 2


@pytest.mark.parametrize("kind", ["spring_damper_ground", "spring_damper_bb", "torsion_ground", "torsion_bb",
                                  "bushing_ground", "bushing_bb"])
def test_conservative_force_is_minus_potential_gradient(kind, rng):
    """f = -Q = dV/ddelta, the derivative along the configuration perturbation q o exp(delta)."""
    e = forces.build(kind)
    n, q, _ = state(kind, rng)
    p = params(kind, rng)
    if kind.startswith("spring"):
        p[7] = p[9] = 0.0  # damping, preload
    elif kind.startswith("bushing"):
        p[27:30] = p[33:36] = 0.0  # damping
    else:
        p[10] = p[12] = 0.0
    f = f_out(e, q, np.zeros(6 * n), p)
    eps = 1e-6
    grad = []
    for d in np.eye(6 * n):
        def V(s):
            qq = np.concatenate([np.array(LIE.update(q[7 * k:7 * k + 7], s * d[6 * k:6 * k + 6])).ravel()
                                 for k in range(n)])
            return _potential(kind, qq, p)
        grad.append((V(eps) - V(-eps)) / (2 * eps))
    np.testing.assert_allclose(f, grad, atol=1e-7 * max(1, np.max(np.abs(grad))))


@pytest.mark.parametrize("kind", list(forces.KINDS))
def test_euler_and_lie_versions_agree(kind, rng):
    """Same virtual power: Q_euler . qd = Q_lie . v for v = [Rdot; 2 Gbar thetadot]; same outputs."""
    n, q, v = state(kind, rng)
    p = params(kind, rng)
    qd = np.concatenate([np.r_[v[6 * k:6 * k + 3],
                               0.5 * np.array(rigid_body.g_body(q[7 * k + 3:7 * k + 7]).T @ v[6 * k + 3:6 * k + 6]).ravel()]
                         for k in range(n)])
    lie, eul = forces.build(kind), forces.build_euler(kind)
    f_l = np.array(lie.res(q, v, np.zeros(6 * n), np.zeros(0), p, np.r_[0, .25, .5, .01], 0 * v, 0.2)[1]).ravel()
    f_e = np.array(eul.res(q, qd, np.zeros(NQ * n), np.zeros(0), p, np.r_[0, .25, .5, .01], 0 * qd, 0.2)[1]).ravel()
    w = rng.normal(size=6 * n)  # arbitrary virtual velocity
    wd = np.concatenate([np.r_[w[6 * k:6 * k + 3],
                               0.5 * np.array(rigid_body.g_body(q[7 * k + 3:7 * k + 7]).T @ w[6 * k + 3:6 * k + 6]).ravel()]
                         for k in range(n)])
    assert f_l @ w == pytest.approx(f_e @ wd, rel=1e-12, abs=1e-12)
    # accelerations: Lie [Rddot; omegadot_bar], Euler thetaddot = d/dt (1/2 Gbar^T omega_bar)
    a = rng.normal(size=6 * n)
    gb = lambda x: np.array(rigid_body.g_body(x))
    qdd = np.concatenate([np.r_[a[6 * k:6 * k + 3],
                                0.5 * gb(qd[7 * k + 3:7 * k + 7]).T @ v[6 * k + 3:6 * k + 6]
                                + 0.5 * gb(q[7 * k + 3:7 * k + 7]).T @ a[6 * k + 3:6 * k + 6]]
                          for k in range(n)])
    outs = lambda e, vel, acc: e.out.call([q, vel, acc, p, 0.2])
    for a, b in zip(outs(lie, v, a), outs(eul, qd, qdd)):
        np.testing.assert_allclose(np.array(a), np.array(b), atol=1e-12)


@pytest.mark.parametrize("kind", ["spring_damper_bb", "torsion_bb", "bushing_bb"])
def test_body_body_forces_conserve_momentum(kind, rng):
    """Internal forces (stiffness and damping): zero resultant force and moment about the origin."""
    e = forces.build(kind)
    _, q, v = state(kind, rng)
    p = params(kind, rng)
    Q = -f_out(e, q, v, p)
    A = lambda t: np.array(rigid_body.rotation_matrix(t))
    F = Q[0:3] + Q[6:9]
    M = sum(np.cross(q[7 * k:7 * k + 3], Q[6 * k:6 * k + 3]) + A(q[7 * k + 3:7 * k + 7]) @ Q[6 * k + 3:6 * k + 6]
            for k in range(2))
    np.testing.assert_allclose(F, 0, atol=1e-12 * max(1, np.abs(Q).max()))
    np.testing.assert_allclose(M, 0, atol=1e-12 * max(1, np.abs(Q).max()))
