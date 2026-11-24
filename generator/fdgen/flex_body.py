"""Flexible body element types (floating frame of reference, modal coordinates).

A flexible body with nm modes has the coordinates q = [R; theta; eta] and the velocities
v = [Rdot (global); omega_bar (body frame); etadot] on R^3 x SO(3) x R^nm (Lie group form, as
rigid_body). Its kinetic energy with the inertia invariants of fdgen.flex.modal,

    T = 1/2 m Rdot.Rdot + Rdot.A (-S~ omega_bar + I3 etadot) + 1/2 omega_bar.J omega_bar
        + omega_bar.(I4 + sum_j eta_j I5_j) etadot + 1/2 etadot.I6 etadot,
    S = I2 + I3 eta,  J = I7 - sum_j eta_j (I8_j + I8_j^T) - sum_jk eta_j eta_k I9_jk,

the elastic energy 1/2 eta.K eta, the potential of gravity -g.(m R + A S) and the modal damping
force -D etadot are polynomials of degree two at most in eta and etadot. So they split into
terms that depend on the rigid part only, on one mode i, or on one pair of modes i < j, and each
term becomes an element type of fixed size, independent of nm:

    flex_frame   R, theta              (owns them, as a rigid body)      invariants 1, 2, 7
    flex_coord   eta_i                 (owns it, no terms of its own)
    flex_mode    R, theta, eta_i       (connects frame and mode i)        terms in mode i only
    flex_pair    eta_i, eta_j, omega   (connects modes i and j, i < j)    terms in both

A body is one flex_frame, nm flex_coord, nm flex_mode and up to nm (nm - 1) / 2 flex_pair
instances (the latter only where the pair invariants are not zero). The runtime adds the
contributions of all instances (scatter assembly), which gives exactly the equations of the
whole body: the equations of motion are linear in T, V and Q. No code depends on nm, so a model
with flexible bodies needs no compiler. All derivatives are taken by CasADi.

Equations of motion of each term (Hamel / Euler-Poincare form for the left-trivialized
velocities; R^3 and R^nm are abelian, so only the rotation has an ad term):

    p = dT/dv,  M = dp/dv,
    h = (dp/ddelta) v + [0; omega_bar x p_omega; 0] - (dT/ddelta)^T + (dV/ddelta)^T,

with d/ddelta the derivative along the perturbation q o exp(delta) at fixed v (as rigid_body,
fdgen.lagrange.group_element). For a rigid body (T = 1/2 m Rdot^2 + 1/2 omega.J omega) this is
Newton-Euler.
"""

import casadi as ca
import numpy as np

from . import rigid_body as rb
from .lagrange import Chain, ElementFunctions, group_element, lagrange_element

FRAME_PARAMS = 13   # m, I2 (3), I7 (xx yy zz xy xz yz), g (3)
MODE_PARAMS = 33    # I3_i (3), I4_i (3), I8_i (9), I9_ii (9), I5_i[:, i] (3), I6_ii, K_ii, D_ii, g (3)
PAIR_PARAMS = 18    # I9_ij + I9_ji (9), I5_i[:, j] (3), I5_j[:, i] (3), I6_ij, K_ij, D_ij
FRAME_OUT = ["R", "theta", "Rdot", "omega", "omega_body", "Rddot", "omegadot", "omegadot_body"]
COORD_OUT = ["eta", "etadot", "etaddot"]


def _sym3(v):
    """Symmetric 3x3 from (xx, yy, zz, xy, xz, yz)."""
    return ca.vertcat(ca.horzcat(v[0], v[3], v[4]), ca.horzcat(v[3], v[1], v[5]), ca.horzcat(v[4], v[5], v[2]))


def _mat3(v):
    """3x3 from 9 values row by row."""
    return ca.vertcat(*[ca.horzcat(v[3 * i], v[3 * i + 1], v[3 * i + 2]) for i in range(3)])


class _Frame:
    """Rotation matrix of the body as an intermediate symbol A (known derivative
    dA/ddelta_rot,i = A skew(e_i), as fdgen.kinematics.Connector) for q = [R; theta; ...] with
    the rotation velocity at v[3:6]."""

    def __init__(self, q, nv):
        self.A = ca.SX.sym("A", 3, 3)
        dA = ca.SX(9, nv)
        for i in range(3):
            dA[:, 3 + i] = ca.vec(ca.mtimes(self.A, rb.skew(ca.DM.eye(3)[:, i])))
        self.intermediates = [(ca.vec(self.A), ca.vec(rb.rotation_matrix(q[3:7])),
                               {"delta": dA, "v": ca.SX(9, nv), "t": ca.SX(9, 1)})]


def hamel(q, v, T, V, perturb, rot, intermediates=()):
    """M and h of a term with kinetic energy T(q, v) and potential V(q) (see the module
    docstring); rot: slice of the body angular velocity in v (None: no rotation dofs)."""
    nv = v.numel()
    delta = ca.SX.sym("delta", nv)
    zero = ca.DM.zeros(nv, 1)

    def partial_delta(expr):
        pert = ca.substitute(expr, q, perturb(q, delta))
        return ca.substitute(ca.jacobian(pert, delta), delta, zero)

    chain = Chain(intermediates, {"delta": partial_delta})
    p = ca.gradient(T, v)
    M = ca.jacobian(p, v)
    h = ca.mtimes(chain.d(p, "delta"), v) - chain.d(T, "delta").T + chain.d(V, "delta").T
    if rot is not None:
        w = v[rot]
        h[rot] = h[rot] + ca.cross(w, p[rot])
    return ca.simplify(M), h


def _frame_perturb(nrest):
    def perturb(q, d):
        return ca.vertcat(rb.perturb(q[0:7], d[0:6]), q[7:7 + nrest] + d[6:6 + nrest])
    return perturb


def _frame_tangent(nrest):
    def tangent(incr):
        return ca.diagcat(ca.SX.eye(3), rb.tangent_so3(incr[3:6]), ca.SX.eye(nrest))
    return tangent


def frame_terms(q, v, p, A):
    """T, V of the rigid part: q = [R; theta], v = [Rdot; omega_bar]."""
    m, I2, I7, g = p[0], p[1:4], _sym3(p[4:10]), p[10:13]
    Rd, w = v[0:3], v[3:6]
    T = 0.5 * m * ca.dot(Rd, Rd) - ca.dot(Rd, ca.mtimes([A, rb.skew(I2), w])) + 0.5 * ca.mtimes([w.T, I7, w])
    V = -ca.dot(g, m * q[0:3] + ca.mtimes(A, I2))
    return T, V


def mode_terms(q, v, p, A):
    """T, V, Q of the terms in mode i only: q = [R; theta; eta_i], v = [Rdot; omega_bar; etadot_i]."""
    I3, I4, I8, I9, I5, I6, K, D, g = (p[0:3], p[3:6], _mat3(p[6:15]), _mat3(p[15:24]), p[24:27], p[27], p[28],
                                       p[29], p[30:33])
    e, ed = q[7], v[6]
    Rd, w = v[0:3], v[3:6]
    J = -e * (I8 + I8.T) - e * e * I9
    T = (ca.dot(Rd, ca.mtimes(A, -ca.mtimes(rb.skew(I3 * e), w) + I3 * ed)) + 0.5 * ca.mtimes([w.T, J, w])
         + ca.dot(w, I4 + e * I5) * ed + 0.5 * I6 * ed * ed)
    V = 0.5 * K * e * e - ca.dot(g, ca.mtimes(A, I3 * e))
    Q = ca.vertcat(ca.SX(6, 1), -D * ed)
    return T, V, Q


def pair_terms(q, v, p):
    """T, V, Q of the terms in modes i and j (i < j): q = [eta_i; eta_j],
    v = [omega_bar; etadot_i; etadot_j]."""
    I9s, I5ij, I5ji, I6, K, D = _mat3(p[0:9]), p[9:12], p[12:15], p[15], p[16], p[17]
    ei, ej = q[0], q[1]
    w, edi, edj = v[0:3], v[3], v[4]
    T = (-0.5 * ei * ej * ca.mtimes([w.T, I9s, w]) + ca.dot(w, I5ij) * ei * edj + ca.dot(w, I5ji) * ej * edi
         + I6 * edi * edj)
    V = K * ei * ej
    Q = ca.vertcat(ca.SX(3, 1), -D * edj, -D * edi)
    return T, V, Q


def build_frame() -> ElementFunctions:
    name = "flex_frame"
    q, v, p = ca.SX.sym("q", 7), ca.SX.sym("qd", 6), ca.SX.sym("p", FRAME_PARAMS)
    fr = _Frame(q, 6)
    T, V = frame_terms(q, v, p, fr.A)
    M, h = hamel(q, v, T, V, _frame_perturb(0), slice(3, 6), fr.intermediates)

    def exp_update(qq, incr):
        t = rb.quat_mul(qq[3:7], rb.quat_exp(incr[3:6]))
        return ca.vertcat(qq[0:3] + incr[0:3], t / ca.norm_2(t))

    def scale(qq):
        return ca.vertcat(ca.fabs(qq[0:3]), ca.DM.zeros(3, 1))

    funs = group_element(name, q, v, p, M, h, _frame_perturb(0), exp_update, _frame_tangent(0), scale,
                         intermediates=fr.intermediates)
    u = ca.SX.sym("u", 6)
    A = rb.rotation_matrix(q[3:7])
    funs["init"] = ca.Function(f"fd_{name}_init", [q, u, p], [ca.vertcat(u[0:3], ca.mtimes(A.T, u[3:6]))],
                               ["q", "u", "p"], ["qd"])
    a = ca.SX.sym("qdd", 6)
    funs["out"] = ca.Function(
        f"fd_{name}_out", [q, v, a, p, ca.SX.sym("t")],
        [q[0:3], q[3:7], v[0:3], ca.mtimes(A, v[3:6]), v[3:6], a[0:3], ca.mtimes(A, a[3:6]), a[3:6]],
        ["q", "qd", "qdd", "p", "t"], FRAME_OUT)
    return ElementFunctions(name, **funs)


def build_coord() -> ElementFunctions:
    """Owner of one modal coordinate: no terms of its own (they are in flex_mode / flex_pair)."""
    name = "flex_coord"
    q, qd, p = ca.SX.sym("q", 1), ca.SX.sym("qd", 1), ca.SX.sym("p", 0)
    funs = lagrange_element(name, q, qd, p, T=ca.SX(1, 1))
    funs.pop("iter_noqv"), funs.pop("jac_qd")
    funs["scale"] = ca.Function(f"fd_{name}_scale", [q], [ca.DM.zeros(1, 1)], ["q"], ["s"])  # absolute tolerance
    u = ca.SX.sym("u", 1)
    funs["init"] = ca.Function(f"fd_{name}_init", [q, u, p], [u], ["q", "u", "p"], ["qd"])
    a = ca.SX.sym("qdd", 1)
    funs["out"] = ca.Function(f"fd_{name}_out", [q, qd, a, p, ca.SX.sym("t")], [q, qd, a],
                              ["q", "qd", "qdd", "p", "t"], COORD_OUT)
    return ElementFunctions(name, **funs)


def build_mode() -> ElementFunctions:
    name = "flex_mode"
    q, v, p = ca.SX.sym("q", 8), ca.SX.sym("qd", 7), ca.SX.sym("p", MODE_PARAMS)
    fr = _Frame(q, 7)
    T, V, Q = mode_terms(q, v, p, fr.A)
    M, h = hamel(q, v, T, V, _frame_perturb(1), slice(3, 6), fr.intermediates)
    funs = group_element(name, q, v, p, M, h, _frame_perturb(1), None, _frame_tangent(1), None, Q=Q,
                         intermediates=fr.intermediates)
    return ElementFunctions(name, **funs)


def build_pair() -> ElementFunctions:
    name = "flex_pair"
    q, v, p = ca.SX.sym("q", 2), ca.SX.sym("qd", 5), ca.SX.sym("p", PAIR_PARAMS)
    T, V, Q = pair_terms(q, v, p)

    def perturb(qq, d):
        return qq + d[3:5]

    def tangent(incr):
        return ca.diagcat(rb.tangent_so3(incr[0:3]), ca.SX.eye(2))

    M, h = hamel(q, v, T, V, perturb, slice(0, 3))
    funs = group_element(name, q, v, p, M, h, perturb, None, tangent, None, Q=Q)
    return ElementFunctions(name, **funs)


BUILDERS = {"flex_frame": build_frame, "flex_coord": build_coord, "flex_mode": build_mode, "flex_pair": build_pair}


# ---------------------------------------------------------------------------------------------
# Parameters of the instances from modal data (fdgen.flex.ModalData)


def coupling_invariants(data, rf=True, ff=True):
    """Invariants with the legacy coupling options (MBS_FlexBody_Full: Is_Itf_active,
    Is_QuadraticTermOF_Itt_active): rf = False drops the rotation-deformation coupling
    I4 + sum eta_j I5_j, ff = False the quadratic term I9 of the inertia tensor."""
    I4 = data.I4 if rf else np.zeros_like(data.I4)
    I5 = data.I5 if rf else np.zeros_like(data.I5)
    I9 = data.I9 if ff else np.zeros_like(data.I9)
    return I4, I5, I9


def instance_params(data, gravity, damping=0.0, rf=True, ff=True, tol=0.0):
    """Parameters of the instances of one flexible body:
    (frame p, [mode p per mode], [((i, j), pair p) for the pairs with a nonzero term]).
    tol: pairs whose parameters are all below tol times the largest one are left out."""
    g = np.asarray(gravity, float)
    I4, I5, I9 = coupling_invariants(data, rf, ff)
    I7 = data.I7
    frame = [data.mass, *data.I2, I7[0, 0], I7[1, 1], I7[2, 2], I7[0, 1], I7[0, 2], I7[1, 2], *g]
    D = data.damping(damping) if np.any(np.asarray(damping) != 0) else np.zeros((data.nm, data.nm))
    modes = []
    for i in range(data.nm):
        modes.append([*data.I3[:, i], *I4[:, i], *data.I8[i].ravel(), *I9[i, i].ravel(), *I5[i][:, i],
                      data.I6[i, i], data.K[i, i], D[i, i], *g])
    pairs, plist = [], []
    for i in range(data.nm):
        for j in range(i + 1, data.nm):
            pp = [*(I9[i, j] + I9[j, i]).ravel(), *I5[i][:, j], *I5[j][:, i], data.I6[i, j], data.K[i, j], D[i, j]]
            plist.append(((i, j), pp))
    big = max([abs(x) for _, pp in plist for x in pp] + [0.0])
    for ij, pp in plist:
        if big > 0 and max(abs(x) for x in pp) > tol * big:
            pairs.append((ij, pp))
    return frame, modes, pairs


def fdm_elements(name, body, gravity, nq, nv):
    """.fdm lines of a flexible body whose coordinates start at nq and velocity dofs at nv:
    (lines, frame coords, frame dofs, nq, nv after the body). Instances: <name> (flex_frame),
    <name>.eta<i> (flex_coord), <name>.mode<i> (flex_mode), <name>.pair<i>_<j> (flex_pair),
    modes numbered from 1."""
    from .flex import load_modal

    fmt = lambda v: " ".join(repr(float(x)) for x in v)
    flex = body["flex"]
    data = load_modal(flex["mtx"])
    frame, modes, pairs = instance_params(data, gravity, flex.get("damping", 0.0),
                                          flex.get("rotation_coupling", True), flex.get("quadratic_inertia", True))
    fc, fd = list(range(nq, nq + 7)), list(range(nv, nv + 6))
    ec, ed = list(range(nq + 7, nq + 7 + data.nm)), list(range(nv + 6, nv + 6 + data.nm))
    ints = lambda v: " ".join(map(str, v))
    lines = [f"element flex_frame {name}", f"coords {ints(fc)}", f"dofs {ints(fd)}", "cons ", f"p {fmt(frame)}",
             f"q {fmt([*body['position'], *body['orientation']])}",
             f"u {fmt([*body['velocity'], *body['angular_velocity']])}", "end"]
    for i in range(data.nm):
        lines += [f"element flex_coord {name}.eta{i + 1}", f"coords {ec[i]}", f"dofs {ed[i]}", "cons ", "p ",
                  "q 0.0", "u 0.0", "end"]
    for i, p in enumerate(modes):
        lines += [f"element flex_mode {name}.mode{i + 1}", f"coords {ints(fc + [ec[i]])}", f"dofs {ints(fd + [ed[i]])}",
                  "cons ", f"p {fmt(p)}", "end"]
    for (i, j), p in pairs:
        lines += [f"element flex_pair {name}.pair{i + 1}_{j + 1}", f"coords {ec[i]} {ec[j]}",
                  f"dofs {ints(fd[3:6] + [ed[i], ed[j]])}", "cons ", f"p {fmt(p)}", "end"]
    return lines, fc, fd, nq + 7 + data.nm, nv + 6 + data.nm


# ---------------------------------------------------------------------------------------------
# Joints at a node of a flexible body
#
# A joint primitive (fdgen.joints) whose flexible body is attached at a node with the mode
# shapes S (translation, 3 x nm) and Psi (small rotation, 3 x nm) sees the node point
# u + S eta and the node axes (I + skew(Psi eta)) b in the body frame (linear in eta, as the
# legacy MBS_RiBo_FleBo_FixJoint). The primitive is then its rigid form at the node position
# (the library types transfix_bb, rotfix_bb, transfix_ground, rotfix_ground with the body frame
# of the flexible body) plus one term per mode, linear in eta_i:
#
#   transfix_bb_node      (A2 s eta) . (A1 n1)        p = [n1, s]        flexible body = body 2
#   rotfix_bb_node        (A1 b1) . (A2 w) eta        p = [b1, w]        w = psi x b2
#   transfix_ground_node  n . (A s eta)               p = [n, s]
#   rotfix_ground_node    (A w) . g eta               p = [w, g]         w = psi x b
#
# The mode terms add to the constraint row of the rigid form (same constraint index).

NODE_KINDS = {"transfix_bb_node": 2, "rotfix_bb_node": 2, "transfix_ground_node": 1, "rotfix_ground_node": 1}


def node_term(kind, A, eta, p):
    if kind == "transfix_bb_node":
        return ca.dot(ca.mtimes(A[1], p[3:6]) * eta, ca.mtimes(A[0], p[0:3]))
    if kind == "rotfix_bb_node":
        return ca.dot(ca.mtimes(A[0], p[0:3]), ca.mtimes(A[1], p[3:6])) * eta
    if kind == "transfix_ground_node":
        return ca.dot(p[0:3], ca.mtimes(A[0], p[3:6])) * eta
    if kind == "rotfix_ground_node":
        return ca.dot(ca.mtimes(A[0], p[0:3]), p[3:6]) * eta
    raise ValueError(kind)


def build_node(kind) -> ElementFunctions:
    """Mode term of a joint primitive at a node: bodies (Lie group, the flexible body last)
    and the modal coordinate eta_i, one constraint row (shared with the rigid form)."""
    nb = NODE_KINDS[kind]
    nv = 6 * nb + 1
    q, v, p = ca.SX.sym("q", 7 * nb + 1), ca.SX.sym("qd", nv), ca.SX.sym("p", 6)
    As, inter = [], []
    for k in range(nb):
        A = ca.SX.sym(f"A{k}", 3, 3)
        dA = ca.SX(9, nv)
        for i in range(3):
            dA[:, 6 * k + 3 + i] = ca.vec(ca.mtimes(A, rb.skew(ca.DM.eye(3)[:, i])))
        inter.append((ca.vec(A), ca.vec(rb.rotation_matrix(q[7 * k + 3:7 * k + 7])),
                      {"delta": dA, "v": ca.SX(9, nv), "t": ca.SX(9, 1)}))
        As.append(A)
    Phi = node_term(kind, As, q[7 * nb], p)

    def perturb(qq, d):
        return ca.vertcat(*[rb.perturb(qq[7 * k:7 * k + 7], d[6 * k:6 * k + 6]) for k in range(nb)],
                          qq[7 * nb] + d[6 * nb])

    def tangent(incr):
        return ca.diagcat(*[ca.diagcat(ca.SX.eye(3), rb.tangent_so3(incr[6 * k + 3:6 * k + 6])) for k in range(nb)],
                          ca.SX.eye(1))

    funs = group_element(kind, q, v, p, ca.SX(nv, nv), ca.SX(nv, 1), perturb, None, tangent, None, Phi=Phi,
                         intermediates=inter)
    # no output channels (the rigid form outputs the multiplier)
    funs["out"] = ca.Function(f"fd_{kind}_out", [q, v, ca.SX.sym("qdd", nv), p, ca.SX.sym("t")], [],
                              ["q", "qd", "qdd", "p", "t"], [])
    return ElementFunctions(kind, **funs)


BUILDERS.update({k: (lambda k=k: build_node(k)) for k in NODE_KINDS})


def node_terms(kind, p, S, Psi):
    """Parameters of the mode terms [p per mode] of the joint primitive kind (fdgen.joints) with
    parameters p (body 2, or the body of a ground joint, flexible), node shapes S, Psi (3 x nm)."""
    p = np.asarray(p, float)
    out = []
    for i in range(S.shape[1]):
        s, psi = S[:, i], Psi[:, i]
        if kind == "transfix_bb":       # p = [n1, u1, u2, c]
            out.append(("transfix_bb_node", [*p[0:3], *s]))
        elif kind == "rotfix_bb":       # p = [b1, b2, c]
            out.append(("rotfix_bb_node", [*p[0:3], *np.cross(psi, p[3:6])]))
        elif kind == "transfix_ground":  # p = [n, u, c]
            out.append(("transfix_ground_node", [*p[0:3], *s]))
        elif kind == "rotfix_ground":   # p = [b, g, c]
            out.append(("rotfix_ground_node", [*np.cross(psi, p[0:3]), *p[3:6]]))
        else:
            raise ValueError(kind)
    return out
