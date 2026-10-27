"""Revolute joint chains with a common axis under gravity: minimal coordinates (one angle per
body), equations of motion from the Lagrangian, RK4."""

import math

import casadi as ca
import numpy as np

from .common import rotation_about, rotation_matrix

HINGE = {"dx", "dy", "dz", "ax", "ay"}  # revolute joint: free rotation about the joint z axis


def hinge_tree_reference(model, times, h=1e-4):
    """[(omega_body, A, R)] per body at `times` for a tree of revolute joints with one common axis.

    `model` is the dict of fdgen.fds.load_fds: every body is body1 of exactly one revolute
    joint whose body2 is the ground or another body. Each body turns by phi_k about the common
    axis a through its joint point; positions follow from the parent's motion. Initial angular
    rates are a . omega(0); bodies must start with velocities consistent with that.
    """
    bodies, joints, g = model["bodies"], model["joints"], np.array(model["gravity"], dtype=float)
    names = list(bodies)
    parent, axis = {}, None
    for j in joints:
        if set(j["fix"]) != HINGE or j["body1"] in parent:
            raise ValueError("hinge_tree_reference: one revolute joint per body (as body1) expected")
        a = np.array(j["frame"])[:, 2]
        if axis is not None and abs(abs(axis @ a) - 1) > 1e-12:
            raise ValueError("hinge_tree_reference: all joint axes must be parallel")
        axis = a if axis is None else axis
        parent[j["body1"]] = (j["body2"], np.array(j["location"], dtype=float))
    if set(parent) != set(names):
        raise ValueError("hinge_tree_reference: every body needs a joint")

    n = len(names)
    phi, phid = ca.SX.sym("phi", n), ca.SX.sym("phid", n)
    c0 = {b: np.array(bodies[b]["position"], dtype=float) for b in names}
    A0 = {b: np.array(rotation_matrix(bodies[b]["orientation"])) for b in names}
    Rot, R, Rd = {}, {}, {}

    def place(b, depth=0):
        if b in R:
            return
        if depth > n:
            raise ValueError("hinge_tree_reference: joint loop")
        k = names.index(b)
        par, p0 = parent[b]
        Rot[b] = rotation_about(axis, phi[k])
        if par == "ground":
            P = ca.DM(p0)
        else:
            place(par, depth + 1)
            P = R[par] + Rot[par] @ ca.DM(p0 - c0[par])
        R[b] = P + Rot[b] @ ca.DM(c0[b] - p0)
        Rd[b] = ca.jtimes(R[b], phi, phid)

    T = V = 0
    for k, b in enumerate(names):
        place(b)
        m, Jb = bodies[b]["mass"], np.diag(bodies[b]["inertia"])
        ab = A0[b].T @ axis  # axis in the body frame (constant)
        T += 0.5 * m * ca.dot(Rd[b], Rd[b]) + 0.5 * float(ab @ Jb @ ab) * phid[k] ** 2
        V -= m * ca.dot(ca.DM(g), R[b])
    L = T - V
    M = ca.hessian(T, phid)[0]
    rhs = ca.gradient(L, phi) - ca.jacobian(ca.gradient(L, phid), phi) @ phid
    f = ca.Function("f", [phi, phid], [ca.vertcat(phid, ca.solve(M, rhs))])
    pos = ca.Function("pos", [phi], [ca.horzcat(*[R[b] for b in names])])

    y = np.r_[np.zeros(n), [axis @ np.array(bodies[b]["angular_velocity"]) for b in names]]
    rhs_np = lambda y: np.array(f(y[:n], y[n:])).ravel()
    t, out = 0.0, []
    for target in times:
        steps = max(1, math.ceil((target - t) / h - 1e-9)) if target > t else 0
        dt = (target - t) / steps if steps else 0.0
        for _ in range(steps):
            k1 = rhs_np(y)
            k2 = rhs_np(y + 0.5 * dt * k1)
            k3 = rhs_np(y + 0.5 * dt * k2)
            k4 = rhs_np(y + dt * k3)
            y = y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        t = target
        out.append(y.copy())
    out = np.array(out)
    P = np.array([np.array(pos(row[:n])) for row in out])  # (nt, 3, n)
    result = []
    for k, b in enumerate(names):
        ab = A0[b].T @ axis
        w = np.outer(out[:, n + k], ab)
        A = np.array([np.array(rotation_about(axis, p)) @ A0[b] for p in out[:, k]])
        result.append((w, A, P[:, :, k]))
    return result
