"""Kinematics and CasADi derivatives of the rigid body element vs references and FD."""

import numpy as np
import pytest

from conftest import COEF, P, num, random_state
from fdgen import rigid_body as rb


def shabana_A(t):
    """Shabana 2005 Eq. 2.13, written out with numpy."""
    t0, t1, t2, t3 = t
    return np.array(
        [
            [1 - 2 * (t2**2 + t3**2), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)],
            [2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1**2 + t3**2), 2 * (t2 * t3 - t0 * t1)],
            [2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1**2 + t2**2)],
        ]
    )


def test_rotation_matrix_is_shabana_2_13(rng):
    for _ in range(5):
        t = rng.normal(size=4)  # also off |theta| = 1: the exact form matters for derivatives
        assert np.array_equal(num(rb.rotation_matrix, t), shabana_A(t))


def test_rotation_matrix_properties(rng):
    t = rng.normal(size=4)
    t /= np.linalg.norm(t)
    A = shabana_A(t)
    G, Gb = num(rb.g_global, t), num(rb.g_body, t)
    np.testing.assert_allclose(A.T @ A, np.eye(3), atol=1e-14)
    assert np.linalg.det(A) == pytest.approx(1.0, abs=1e-14)
    np.testing.assert_allclose(G @ Gb.T, A, atol=1e-14)
    np.testing.assert_allclose(G @ t, 0, atol=1e-15)
    np.testing.assert_allclose(Gb @ t, 0, atol=1e-15)


def test_angular_velocity_frames(elem, rng):
    q, qd, _, _ = random_state(rng)
    qd[3:] -= q[3:] * (q[3:] @ qd[3:])  # satisfy theta . thetadot = 0
    out = elem.out(q, qd, 0 * qd, P, 0)
    omega, omega_b = np.array(out[3]).ravel(), np.array(out[4]).ravel()
    np.testing.assert_allclose(omega, shabana_A(q[3:]) @ omega_b, atol=1e-14)
    np.testing.assert_allclose(omega, 2 * num(rb.g_global, q[3:]) @ qd[3:], atol=1e-14)
    J = np.diag(P[1:4])
    np.testing.assert_allclose(float(out[5]), 0.5 * P[0] * qd[:3] @ qd[:3] + 0.5 * omega_b @ J @ omega_b)
    np.testing.assert_allclose(np.array(out[6]).ravel(), shabana_A(q[3:]) @ J @ omega_b, atol=1e-14)


def test_init_velocity(elem, rng):
    q, _, _, _ = random_state(rng)
    u = rng.normal(size=6)
    qd = np.array(elem.init(q, u, P)).ravel()
    assert q[3:] @ qd[3:] == pytest.approx(0, abs=1e-15)
    np.testing.assert_allclose(qd[:3], u[:3])
    np.testing.assert_allclose(2 * num(rb.g_global, q[3:]) @ qd[3:], u[3:], atol=1e-14)


def test_mass_matrix_analytic(elem, rng):
    q, qd, _, _ = random_state(rng, unit=False)
    A = np.array(elem.ic(q, qd, 0, 0, P, 0, 0, 0)[0])
    Gb = num(rb.g_body, q[3:])
    M = np.zeros((7, 7))
    M[:3, :3] = P[0] * np.eye(3)
    M[3:, 3:] = 4 * Gb.T @ np.diag(P[1:4]) @ Gb
    np.testing.assert_allclose(A[:7, :7], M, atol=1e-14)
    np.testing.assert_allclose(A[7, :7], np.r_[0, 0, 0, 2 * q[3:]], atol=1e-15)


def fd_jacobian(fun, x, eps=1e-6):
    cols = []
    for j in range(len(x)):
        e = np.zeros_like(x)
        e[j] = eps
        cols.append((fun(x + e) - fun(x - e)) / (2 * eps))
    return np.column_stack(cols)


def test_iteration_matrix_vs_fd(elem, rng):
    """J = dr/dqdd + gamma h dr/dqd + beta h^2 dr/dq | dr/dlam, by central FD of the residual."""
    q, qd, qdd, lam = random_state(rng, unit=False)
    alpha, beta, gamma, h = COEF
    r = lambda q_, qd_, qdd_, lam_: np.array(elem.res(q_, qd_, qdd_, lam_, P, COEF, 0, 0)[0]).ravel()
    Dq = fd_jacobian(lambda v: r(v, qd, qdd, lam), q)
    Dqd = fd_jacobian(lambda v: r(q, v, qdd, lam), qd)
    Dqdd = fd_jacobian(lambda v: r(q, qd, v, lam), qdd)
    Dlam = fd_jacobian(lambda v: r(q, qd, qdd, v), lam)
    J_fd = np.hstack([Dqdd + gamma * h * Dqd + beta * h**2 * Dq, Dlam])
    J = np.array(elem.iter(q, qd, qdd, lam, P, COEF, 0, 0))
    assert np.max(np.abs(J - J_fd)) / max(1, np.max(np.abs(J_fd))) < 1e-7


def test_residual_structure(elem, rng):
    """r_dyn = M qdd / (1 + alpha) + h + Phi_q^T lam, r_con = Phi / (beta h^2)."""
    q, qd, qdd, lam = random_state(rng, unit=False)
    alpha, beta, _, h = COEF
    A, b, phi, _ = (np.array(v) for v in elem.ic(q, qd, 0, 0, P, 0, 0, 0))
    r, f = (np.array(v).ravel() for v in elem.res(q, qd, qdd, lam, P, COEF, 0, 0))
    np.testing.assert_allclose(f[:7], -b[:7].ravel() + A[:7, 7] * lam[0], atol=1e-12)
    np.testing.assert_allclose(r[:7], A[:7, :7] @ qdd / (1 + alpha) + f[:7], atol=1e-12)
    assert r[7] == pytest.approx(phi[7, 0] / (beta * h**2))


def test_acceleration_constraint_rhs(elem, rng):
    """b_con = -(Phi_q qd)_q qd, by FD of Phi along q + s qd."""
    q, qd, _, _ = random_state(rng, unit=False)
    phi = lambda s: float(np.array(elem.ic(q + s * qd, qd, 0, 0, P, 0, 0, 0)[2])[7, 0])
    eps = 1e-4
    d2 = (phi(eps) - 2 * phi(0) + phi(-eps)) / eps**2  # d2/ds2 Phi(q + s qd) = (Phi_q qd)_q qd
    b = np.array(elem.ic(q, qd, 0, 0, P, 0, 0, 0)[1]).ravel()
    assert b[7] == pytest.approx(-d2, rel=1e-6)


def test_structural_sparsity(elem):
    sp = elem.iter.sparsity_out(0)
    assert (sp.size1(), sp.size2()) == (8, 8)
    assert sp.nnz() == 27  # 3 (R) + 16 (theta) + 2 x 4 (constraint)
    assert elem.ic.sparsity_out(0).nnz() == 27
    assert elem.res.sparsity_out(1).nnz() == 7  # f = h + Phi_q^T lam - Q (gravity in the R rows)
