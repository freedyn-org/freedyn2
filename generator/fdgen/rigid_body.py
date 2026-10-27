"""Rigid body element types.

- rigid_body (FreeDyn 2.0 default): Lie group R^3 x SO(3). Configuration q = [R (3); theta (4)]
  with theta a unit quaternion (Euler parameters) used only to store the rotation, velocity
  v = [Rdot (3, global); omega_bar (3, body frame)]. Updated by the exponential map; no
  normalization constraint.
- rigid_body_euler (legacy profile): absolute coordinates q = [R; theta] with qd = dq/dt and
  the normalization constraint theta.theta = 1, as in the legacy FreeDyn kernel
  (MBS_RigidBody_EulerP_PAI). Also provides the legacy modified Newmark formulas.

Conventions (see docs/architecture.md):
- R: global position of the center of mass; theta = [theta0..theta3], theta0 scalar part;
  rotation matrix A(theta) of Shabana 2005, Eq. 2.13; Hamilton quaternion product, so that
  A(p o q) = A(p) A(q).
- Parameters p = [m, J1, J2, J3, gx, gy, gz]: mass, principal moments in the body frame and the
  gravity acceleration (global); gravity acts at the center of mass, Q = [m g; 0].
- User velocity input u = [v (3, global); omega (3, global)].
"""

import casadi as ca

from .lagrange import ElementFunctions, group_element, lagrange_element

NQ = 7   # stored coordinates of both types
NV = 6   # velocity dofs of rigid_body
PARAM_NAMES = ("mass", "J1", "J2", "J3", "gx", "gy", "gz")  # g: gravity acceleration (global)
OUT_NAMES = ["R", "theta", "Rdot", "omega", "omega_body", "T", "H", "theta_norm", "Rddot", "omegadot",
             "omegadot_body"]


def rotation_matrix(t):
    """Rotation matrix, Shabana 2005, Eq. 2.13 (form used by the legacy FreeDyn kernel).

    Other forms agree on |theta| = 1 but have different derivatives; keep this one.
    """
    t0, t1, t2, t3 = t[0], t[1], t[2], t[3]
    return ca.vertcat(
        ca.horzcat(1 - 2 * (t2**2 + t3**2), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)),
        ca.horzcat(2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1**2 + t3**2), 2 * (t2 * t3 - t0 * t1)),
        ca.horzcat(2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1**2 + t2**2)),
    )


def g_global(t):
    """G with omega = 2 G thetadot (global angular velocity)."""
    t0, t1, t2, t3 = t[0], t[1], t[2], t[3]
    return ca.vertcat(
        ca.horzcat(-t1, t0, -t3, t2),
        ca.horzcat(-t2, t3, t0, -t1),
        ca.horzcat(-t3, -t2, t1, t0),
    )


def g_body(t):
    """Gbar with omega_bar = 2 Gbar thetadot (body-frame angular velocity)."""
    t0, t1, t2, t3 = t[0], t[1], t[2], t[3]
    return ca.vertcat(
        ca.horzcat(-t1, t0, t3, -t2),
        ca.horzcat(-t2, -t3, t0, t1),
        ca.horzcat(-t3, t2, -t1, t0),
    )


def quat_mul(a, b):
    """Hamilton product a o b of quaternions [scalar; vector]."""
    a0, av, b0, bv = a[0], a[1:4], b[0], b[1:4]
    return ca.vertcat(a0 * b0 - ca.dot(av, bv), a0 * bv + b0 * av + ca.cross(av, bv))


def skew(w):
    return ca.vertcat(ca.horzcat(0, -w[2], w[1]), ca.horzcat(w[2], 0, -w[0]), ca.horzcat(-w[1], w[0], 0))


def perturb(q, d):
    """q o exp(d) of a rigid body configuration q = [R; theta] to first order in d (its
    derivative at d = 0 is all the Lie group derivatives use)."""
    return ca.vertcat(q[0:3] + d[0:3], quat_mul(q[3:7], ca.vertcat(1, d[3:6] / 2)))


_SMALL = 1e-4  # below this rotation angle the Taylor expansions are exact to round-off


def quat_exp(psi):
    """Unit quaternion of the rotation vector psi (A(quat_exp(psi)) = exp(skew(psi)))."""
    phi2 = ca.dot(psi, psi)
    phi = ca.sqrt(phi2)
    small = phi < _SMALL
    c = ca.if_else(small, 1 - phi2 / 8 + phi2**2 / 384, ca.cos(phi / 2))
    s = ca.if_else(small, 0.5 - phi2 / 48 + phi2**2 / 3840, ca.sin(phi / 2) / phi)
    return ca.vertcat(c, s * psi)


def tangent_so3(psi):
    """T(psi) with exp(psi + d) = exp(psi) exp(T(psi) d) to first order (body increments)."""
    phi2 = ca.dot(psi, psi)
    phi = ca.sqrt(phi2)
    small = phi < _SMALL
    a = ca.if_else(small, 0.5 - phi2 / 24 + phi2**2 / 720, (1 - ca.cos(phi)) / phi2)
    b = ca.if_else(small, 1.0 / 6 - phi2 / 120 + phi2**2 / 5040, (phi - ca.sin(phi)) / (phi2 * phi))
    S = skew(psi)
    return ca.SX.eye(3) - a * S + b * ca.mtimes(S, S)


def _outputs(name, q, vel, acc, p, omega_b, omegadot_b, Rd):
    """Common output function: q, vel (element velocity input), acc (its rate), p, t. The
    accelerations are those HHT solves for (no difference quotients); d/dt (A omega_bar) =
    A omegadot_bar."""
    R, th = q[0:3], q[3:7]
    J = ca.diag(p[1:4])
    A = rotation_matrix(th)
    T = 0.5 * p[0] * ca.dot(Rd, Rd) + 0.5 * ca.mtimes([omega_b.T, J, omega_b])
    return ca.Function(
        f"fd_{name}_out",
        [q, vel, acc, p, ca.SX.sym("t")],
        [R, th, Rd, ca.mtimes(A, omega_b), omega_b, T, ca.mtimes([A, J, omega_b]), ca.norm_2(th),
         acc[0:3], ca.mtimes(A, omegadot_b), omegadot_b],
        ["q", "qd", "qdd", "p", "t"],
        OUT_NAMES,
    )


def build() -> ElementFunctions:
    """Lie group rigid body, R^3 x SO(3)."""
    name = "rigid_body"
    q = ca.SX.sym("q", NQ)
    v = ca.SX.sym("qd", NV)
    p = ca.SX.sym("p", len(PARAM_NAMES))
    th = q[3:7]
    Rd, w = v[0:3], v[3:6]
    J = ca.diag(p[1:4])
    M = ca.diagcat(p[0] * ca.SX.eye(3), J)
    h = ca.vertcat(ca.SX(3, 1), ca.cross(w, ca.mtimes(J, w)))  # Euler's equations
    Q = ca.vertcat(p[0] * p[4:7], ca.SX(3, 1))  # gravity

    def exp_update(qq, incr):
        t = quat_mul(qq[3:7], quat_exp(incr[3:6]))
        return ca.vertcat(qq[0:3] + incr[0:3], t / ca.norm_2(t))

    def tangent(incr):
        return ca.diagcat(ca.SX.eye(3), tangent_so3(incr[3:6]))

    def scale(qq):
        # Magnitudes for the relative part of the error tolerance: |R| for the translations;
        # a rotation increment has no magnitude to be relative to (absolute tolerance only).
        return ca.vertcat(ca.fabs(qq[0:3]), ca.DM.zeros(3, 1))

    funs = group_element(name, q, v, p, M, h, perturb, exp_update, tangent, scale, Q=Q)
    u = ca.SX.sym("u", 6)
    funs["init"] = ca.Function(f"fd_{name}_init", [q, u, p],
                               [ca.vertcat(u[0:3], ca.mtimes(rotation_matrix(th).T, u[3:6]))],
                               ["q", "u", "p"], ["qd"])
    a = ca.SX.sym("qdd", NV)
    funs["out"] = _outputs(name, q, v, a, p, w, a[3:6], Rd)
    return ElementFunctions(name, **funs)


def build_euler() -> ElementFunctions:
    """Rigid body in absolute coordinates with Euler parameters, as the legacy kernel."""
    name = "rigid_body_euler"
    q = ca.SX.sym("q", NQ)
    qd = ca.SX.sym("qd", NQ)
    p = ca.SX.sym("p", len(PARAM_NAMES))
    th = q[3:7]
    Rd, thd = qd[0:3], qd[3:7]
    J = ca.diag(p[1:4])

    omega_b = 2 * ca.mtimes(g_body(th), thd)
    T = 0.5 * p[0] * ca.dot(Rd, Rd) + 0.5 * ca.mtimes([omega_b.T, J, omega_b])
    Phi = ca.dot(th, th) - 1

    # Quadratic velocity vector in the form of the legacy FreeDyn kernel
    # (MBS_RigidBody_EulerP_PAI: Qvv_rot = -G^T omega~ J omega with G = 2 Gbar):
    # h_theta = 2 Gbar^T (omega_bar x J omega_bar). The Lagrange form 4 Gbardot^T J omega_bar
    # differs from it only along theta when theta.theta = 1 and theta.thetadot = 0
    # (absorbed by lambda), but not off that manifold, where HHT index 3 lives numerically.
    h = ca.vertcat(ca.SX(3, 1), 2 * ca.mtimes(g_body(th).T, ca.cross(omega_b, ca.mtimes(J, omega_b))))
    Q = ca.vertcat(p[0] * p[4:7], ca.SX(4, 1))  # gravity (legacy MBS_GravityForce: m g at the COM)
    funs = lagrange_element(name, q, qd, p, T, Q=Q, Phi=Phi, h=h)

    # Initial velocities: thetadot = 1/2 G^T omega satisfies omega = 2 G thetadot
    # and theta . thetadot = 0 for |theta| = 1.
    u = ca.SX.sym("u", 6)
    funs["init"] = ca.Function(f"fd_{name}_init", [q, u, p],
                               [ca.vertcat(u[0:3], 0.5 * ca.mtimes(g_global(th).T, u[3:6]))],
                               ["q", "u", "p"], ["qd"])
    qdd = ca.SX.sym("qdd", NQ)
    omegadot_b = ca.jtimes(omega_b, q, qd) + ca.jtimes(omega_b, qd, qdd)
    funs["out"] = _outputs(name, q, qd, qdd, p, omega_b, omegadot_b, Rd)
    funs["mn_qd"] = modified_newmark(name)
    return ElementFunctions(name, **funs)


def modified_newmark(name):
    """Legacy modified Newmark formulas (SOL_DynSolverHHT::modifyQdRotInNewmark).

    Newmark on the global angular velocity instead of thetadot:
        omega_{n+1} = omega_n + h (1-gamma) omegad_n + h gamma omegad_{n+1},  omega = 2 E thetadot,
        thetadot_{n+1} = 1/2 E(theta_{n+1})^T (omega_n + h (1-gamma) omegad_n) + h gamma Z thetadd_{n+1},
    with omegad = 2 E thetadd and Z = I - theta theta^T = E^T E (theta = theta_{n+1}).
    Translations keep the classical formula. Outputs qd and D = dqd/dqdd, where
    q = q_pred + beta h^2 qdd (effective Negrut coefficients).
    """
    q = ca.SX.sym("q", NQ)
    q_n = ca.SX.sym("q_n", NQ)
    qd_n = ca.SX.sym("qd_n", NQ)
    qdd_n = ca.SX.sym("qdd_n", NQ)
    qdd = ca.SX.sym("qdd", NQ)
    coef = ca.SX.sym("coef", 4)
    _, beta, gamma, hs = ca.vertsplit(coef)
    th, th_n = q[3:7], q_n[3:7]
    hg = hs * gamma
    qd_trans = qd_n[0:3] + (hs - hg) * qdd_n[0:3] + hg * qdd[0:3]
    b = 2 * ca.mtimes(g_global(th_n), qd_n[3:7] + hs * (1 - gamma) * qdd_n[3:7])
    Z = ca.SX.eye(4) - ca.mtimes(th, th.T)
    qd_rot = 0.5 * ca.mtimes(g_global(th).T, b) + hg * ca.mtimes(Z, qdd[3:7])
    qd = ca.vertcat(qd_trans, qd_rot)
    D = ca.jacobian(qd, qdd) + (0.5 * hs * hs * beta * 2.0) * ca.jacobian(qd, q)
    return ca.Function(f"fd_{name}_mn_qd", [q, q_n, qd_n, qdd_n, qdd, coef], [qd, D],
                       ["q", "q_n", "qd_n", "qdd_n", "qdd", "coef"], ["qd", "D"])
