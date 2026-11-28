"""Flexible body element types (fdgen.flex_body): the decomposed instances add up to the
equations of the whole body, and those conserve energy, momentum and angular momentum."""

import casadi as ca
import numpy as np
import pytest

from fdgen import flex_body as fb
from fdgen import rigid_body as rb
from fdgen.flex.modal import ModalData
from fdgen.lagrange import group_element


def random_modal(nm, seed=0, coupled=True):
    """Modal data with the symmetries of real invariants (I5_j[:, k] = -I5_k[:, j],
    I9_jk = I9_kj^T, symmetric I7, I6, K)."""
    rng = np.random.default_rng(seed)
    A = rng.normal(size=(nm, nm))
    I6 = np.eye(nm) + 0.1 * (A + A.T) / 2 * (nm > 1)
    B = rng.normal(size=(nm, nm))
    K = np.diag(rng.uniform(10, 100, nm)) + 0.5 * (B + B.T)
    I7 = np.diag([3.0, 4.0, 5.0]) + 0.1 * np.ones((3, 3))
    I5 = rng.normal(size=(nm, 3, nm)) * coupled
    I5 = (I5 - I5.transpose(2, 1, 0)) / 2
    I9 = rng.normal(size=(nm, nm, 3, 3)) * coupled
    I9 = (I9 + I9.transpose(1, 0, 3, 2)) / 2
    return ModalData(
        mass=2.0, I2=rng.normal(size=3) * 0.1, I3=rng.normal(size=(3, nm)) * 0.1, I4=rng.normal(size=(3, nm)),
        I5=I5, I6=I6, I7=I7, I8=rng.normal(size=(nm, 3, 3)), I9=I9, K=K, frequencies=np.sqrt(np.diag(K)) / 2 / np.pi,
        node_labels=np.array([1]), node_positions=np.zeros((1, 3)), Phi=np.zeros((1, 3, nm)), Psi=np.zeros((1, 3, nm)))


def whole_body_terms(q, v, data, g, A):
    """T, V of the whole body written directly from the invariants (module docstring)."""
    nm = data.nm
    R, eta = q[0:3], q[7:7 + nm]
    Rd, w, ed = v[0:3], v[3:6], v[6:6 + nm]
    S = data.I2 + ca.mtimes(ca.DM(data.I3), eta)
    J = ca.DM(data.I7)
    Irf = ca.DM(data.I4)
    for j in range(nm):
        J = J - eta[j] * ca.DM(data.I8[j] + data.I8[j].T)
        Irf = Irf + eta[j] * ca.DM(data.I5[j])
        for k in range(nm):
            J = J - eta[j] * eta[k] * ca.DM(data.I9[j, k])
    T = (0.5 * data.mass * ca.dot(Rd, Rd) + ca.dot(Rd, ca.mtimes(A, -ca.mtimes(rb.skew(S), w) + ca.mtimes(ca.DM(data.I3), ed)))
         + 0.5 * ca.mtimes([w.T, J, w]) + ca.mtimes([w.T, Irf, ed]) + 0.5 * ca.mtimes([ed.T, ca.DM(data.I6), ed]))
    V = 0.5 * ca.mtimes([eta.T, ca.DM(data.K), eta]) - ca.dot(ca.DM(g), data.mass * R + ca.mtimes(A, S))
    return T, V


def state(nm, seed=1):
    rng = np.random.default_rng(seed)
    th = rng.normal(size=4)
    q = np.concatenate([rng.normal(size=3), th / np.linalg.norm(th), 0.1 * rng.normal(size=nm)])
    v = rng.normal(size=6 + nm)
    return q, v


def assembled(data, g, q, v, damping=0.0):
    """M, f = h - Q of the body from its instances (as the runtime assembles them)."""
    nm = data.nm
    funs = {k: b() for k, b in fb.BUILDERS.items()}
    frame, modes, pairs = fb.instance_params(data, g, damping)
    n = 6 + nm
    M, f = np.zeros((n, n)), np.zeros(n)

    def add(kind, qi, vi, p):
        res = funs[kind].res
        nvl = len(vi)
        args = dict(q=q[qi], qd=v[vi], lam=np.zeros(0), p=p, coef=[0, 1, 1, 1], incr=np.zeros(nvl), t=0)
        f0 = np.array(res(qdd=np.zeros(nvl), **args)["f"]).ravel()
        f[vi] += f0
        for c in range(nvl):
            e = np.zeros(nvl)
            e[c] = 1
            r = np.array(res(qdd=e, **args)["r"]).ravel()
            M[np.ix_(vi, [vi[c]])] += (r - f0)[:, None]

    add("flex_frame", list(range(7)), list(range(6)), frame)
    for i, p in enumerate(modes):
        add("flex_mode", list(range(7)) + [7 + i], list(range(6)) + [6 + i], p)
    for (i, j), p in pairs:
        add("flex_pair", [7 + i, 7 + j], [3, 4, 5, 6 + i, 6 + j], p)
    return M, f


@pytest.mark.parametrize("nm", [1, 3])
def test_instances_add_up_to_the_whole_body(nm):
    data = random_modal(nm)
    g = np.array([0.0, -9.81, 1.0])
    q, v = state(nm)
    M, f = assembled(data, g, q, v)

    qs, vs = ca.SX.sym("q", 7 + nm), ca.SX.sym("v", 6 + nm)
    fr = fb._Frame(qs, 6 + nm)
    T, V = whole_body_terms(qs, vs, data, g, fr.A)
    Mw, hw = fb.hamel(qs, vs, T, V, fb._frame_perturb(nm), slice(3, 6), fr.intermediates)
    funs = group_element("whole", qs, vs, ca.SX.sym("p", 0), Mw, hw, fb._frame_perturb(nm), None,
                         fb._frame_tangent(nm), None, intermediates=fr.intermediates)
    args = dict(q=q, qd=v, lam=np.zeros(0), p=np.zeros(0), coef=[0, 1, 1, 1], incr=np.zeros(6 + nm), t=0)
    fw = np.array(funs["res"](qdd=np.zeros(6 + nm), **args)["f"]).ravel()
    Mref = np.column_stack([np.array(funs["res"](qdd=e, **args)["r"]).ravel() - fw for e in np.eye(6 + nm)])
    assert np.allclose(M, Mref, atol=1e-12, rtol=1e-12)
    assert np.allclose(f, fw, atol=1e-11, rtol=1e-12)


@pytest.mark.parametrize("nm", [1, 3])
def test_conservation(nm):
    """dE/dt = 0 without damping (E = T + V), and without gravity dP/dt = 0, dL/dt = 0 with
    P = dT/dRdot (global), L = R x P + A dT/domega_bar (about the global origin), along the
    accelerations of the assembled equations M vdot = -f."""
    data = random_modal(nm, seed=3)
    q, v = state(nm, seed=4)
    qs, vs = ca.SX.sym("q", 7 + nm), ca.SX.sym("v", 6 + nm)
    A = rb.rotation_matrix(qs[3:7])
    delta = ca.SX.sym("d", 6 + nm)
    pert = ca.vertcat(rb.perturb(qs[0:7], delta[0:6]), qs[7:] + delta[6:])

    def rate(expr, vd):
        """d/dt of expr(q, v) along v and vdot."""
        dd = ca.substitute(ca.jacobian(ca.substitute(expr, qs, pert), delta), delta, ca.DM.zeros(6 + nm))
        return ca.mtimes(dd, vs) + ca.mtimes(ca.jacobian(expr, vs), vd)

    for g in ([0.0, -9.81, 1.0], [0.0, 0.0, 0.0]):
        M, f = assembled(data, g, q, v)
        vd = np.linalg.solve(M, -f)
        T, V = whole_body_terms(qs, vs, data, g, A)
        checks = {"energy": T + V}
        if not any(g):
            P = ca.gradient(T, vs[0:3])
            checks["momentum"] = P
            checks["angular momentum"] = ca.cross(qs[0:3], P) + ca.mtimes(A, ca.gradient(T, vs[3:6]))
        for what, e in checks.items():
            fn = ca.Function("r", [qs, vs], [rate(e, ca.DM(vd)), e])
            r, val = (np.array(x).ravel() for x in fn(q, v))
            assert np.max(np.abs(r)) < 1e-10 * max(1.0, np.max(np.abs(val))), (what, r)


def test_damping_dissipates():
    data = random_modal(2, seed=5)
    q, v = state(2, seed=6)
    M, f0 = assembled(data, [0, 0, 0], q, v)
    _, f1 = assembled(data, [0, 0, 0], q, v, damping=0.05)
    D = data.damping(0.05)
    assert np.allclose(f1[6:] - f0[6:], D @ v[6:])
    assert np.allclose(f1[:6], f0[:6])
