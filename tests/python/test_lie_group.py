"""Lie group rigid body: exponential map, tangent operator, update and iteration matrix vs FD."""

import numpy as np
import pytest

from conftest import P, num, unit_quat
from fdgen import rigid_body as rb


def rodrigues(psi):
    phi = np.linalg.norm(psi)
    S = num(rb.skew, psi)
    if phi == 0:
        return np.eye(3)
    return np.eye(3) + np.sin(phi) / phi * S + (1 - np.cos(phi)) / phi**2 * S @ S


def test_quaternion_product_composes_rotations(rng):
    a, b = unit_quat(rng), unit_quat(rng)
    A = lambda t: num(rb.rotation_matrix, t)
    np.testing.assert_allclose(A(num(rb.quat_mul, a, b).ravel()), A(a) @ A(b), atol=1e-15)


@pytest.mark.parametrize("scale", [1.0, 1e-3, 1e-5, 1e-9, 0.0])
def test_quat_exp(rng, scale):
    """Both branches (exact and Taylor below 1e-4 rad) give exp(skew(psi))."""
    psi = scale * rng.normal(size=3)
    t = num(rb.quat_exp, psi).ravel()
    assert np.linalg.norm(t) == pytest.approx(1.0, abs=2e-16)
    np.testing.assert_allclose(num(rb.rotation_matrix, t), rodrigues(psi), atol=1e-15)


@pytest.mark.parametrize("scale", [1.0, 2.5, 1e-3, 1e-5, 0.0])
def test_tangent_operator(rng, scale):
    """exp(psi + e d) = exp(psi) exp(e T(psi) d) + O(e^2), checked by central differences."""
    psi, d = scale * rng.normal(size=3), rng.normal(size=3)
    T = num(rb.tangent_so3, psi)
    e = 1e-6
    # body-frame derivative: skew(T d) = exp(psi)^T d/de exp(psi + e d)
    dR = (rodrigues(psi + e * d) - rodrigues(psi - e * d)) / (2 * e)
    W = rodrigues(psi).T @ dR
    w = np.array([W[2, 1], W[0, 2], W[1, 0]])
    np.testing.assert_allclose(T @ d, w, atol=1e-8)


def test_update_keeps_unit_quaternion(lie, rng):
    q = np.r_[rng.normal(size=3), unit_quat(rng)]
    incr = rng.normal(size=6)
    qn = np.array(lie.update(q, incr)).ravel()
    np.testing.assert_allclose(qn[:3], q[:3] + incr[:3], atol=1e-15)
    assert np.linalg.norm(qn[3:]) == pytest.approx(1.0, abs=1e-15)
    A = lambda t: num(rb.rotation_matrix, t)
    np.testing.assert_allclose(A(qn[3:]), A(q[3:]) @ rodrigues(incr[3:]), atol=1e-14)


def test_init_and_outputs(lie, rng):
    q = np.r_[rng.normal(size=3), unit_quat(rng)]
    u = rng.normal(size=6)
    v = np.array(lie.init(q, u, P)).ravel()
    A = num(rb.rotation_matrix, q[3:])
    np.testing.assert_allclose(v, np.r_[u[:3], A.T @ u[3:]], atol=1e-15)
    out = lie.out(q, v, 0 * v, P, 0)
    np.testing.assert_allclose(np.array(out[3]).ravel(), u[3:], atol=1e-14)  # global omega


def test_mass_matrix_and_gyroscopic_term(lie, rng):
    q = np.r_[rng.normal(size=3), unit_quat(rng)]
    v = rng.normal(size=6)
    A, b, phi, _ = (np.array(x) for x in lie.ic(q, v, 0, 0, P, 0, 0, 0))
    np.testing.assert_allclose(A, np.diag([P[0]] * 3 + list(P[1:4])), atol=0)
    J = np.diag(P[1:4])
    Q = np.r_[P[0] * P[4:7], 0, 0, 0]  # gravity at the center of mass
    np.testing.assert_allclose(b.ravel(), Q - np.r_[0, 0, 0, np.cross(v[3:], J @ v[3:])], atol=1e-14)
    assert phi.size == 0 or np.all(phi == 0)


@pytest.mark.parametrize("rot", [0.3, 1e-6])
def test_iteration_matrix_vs_fd_through_update(lie, rng, rot):
    """J = d r(q(x), v(x), x)/dx with q = q_n o exp(incr0 + beta h^2 x), v = v0 + gamma h x."""
    alpha, beta, gamma, h = 0.0, 0.3025 * 0.9, 0.6 * 0.9, 0.05  # effective Arnold-Bruls coefficients
    coef = np.array([alpha, beta, gamma, h])
    q_n = np.r_[rng.normal(size=3), unit_quat(rng)]
    v0, x0 = rng.normal(size=6), rng.normal(size=6)
    incr0 = np.r_[rng.normal(size=3), rot * rng.normal(size=3)]

    def state(x):
        incr = incr0 + beta * h * h * x
        return np.array(lie.update(q_n, incr)).ravel(), v0 + gamma * h * x, incr

    def r(x):
        q, v, incr = state(x)
        return np.array(lie.res(q, v, x, np.zeros(0), P, coef, incr, 0)[0]).ravel()

    q, v, incr = state(x0)
    J = np.array(lie.iter(q, v, x0, np.zeros(0), P, coef, incr, 0))
    eps = 1e-6
    J_fd = np.column_stack([(r(x0 + eps * e) - r(x0 - eps * e)) / (2 * eps) for e in np.eye(6)])
    assert np.max(np.abs(J - J_fd)) / max(1, np.max(np.abs(J_fd))) < 1e-8


def test_modified_newmark_matches_legacy_formula(elem, rng):
    """mn_qd reproduces SOL_DynSolverHHT::modifyQdRotInNewmark and its Jacobian is exact."""
    beta, gamma, h = 0.3025, 0.6, 1e-2
    coef = np.array([-0.1, beta, gamma, h])
    q_n = np.r_[rng.normal(size=3), unit_quat(rng)]
    qd_n, qdd_n, qdd = rng.normal(size=7), rng.normal(size=7), rng.normal(size=7)
    q_pred = q_n + h * qd_n + 0.5 * h * h * (1 - 2 * beta) * qdd_n

    def mn(x):
        q = q_pred + beta * h * h * x
        return [np.array(o) for o in elem.mn_qd(q, q_n, qd_n, qdd_n, x, coef)]

    qd, D = mn(qdd)
    # Legacy formula written out (theta = theta_{n+1}).
    t, tn = (q_pred + beta * h * h * qdd)[3:], q_n[3:]
    E = lambda s: np.array([[-s[1], s[0], -s[3], s[2]], [-s[2], s[3], s[0], -s[1]], [-s[3], -s[2], s[1], s[0]]])
    b = 2 * E(tn) @ (qd_n[3:] + h * (1 - gamma) * qdd_n[3:])
    rot = 0.5 * E(t).T @ b + h * gamma * (np.eye(4) - np.outer(t, t)) @ qdd[3:]
    trans = qd_n[:3] + (h - h * gamma) * qdd_n[:3] + h * gamma * qdd[:3]
    np.testing.assert_allclose(qd.ravel(), np.r_[trans, rot], atol=1e-14)
    eps = 1e-6
    D_fd = np.column_stack([(mn(qdd + eps * e)[0] - mn(qdd - eps * e)[0]).ravel() / (2 * eps) for e in np.eye(7)])
    np.testing.assert_allclose(D, D_fd, atol=1e-9)
