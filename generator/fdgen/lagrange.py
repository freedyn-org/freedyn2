"""Generic element builders: derive all runtime functions of one element type.

Two kinds of element types:

- vector-space elements (`lagrange_element`): coordinates q in R^n, velocities qd = dq/dt.
  Given T, V, Q and holonomic constraints Phi(q), CasADi derives the mass matrix, the
  quadratic velocity vector, the constraint Jacobian, the HHT iteration matrix and the
  initial-condition system.
- Lie group elements (`group_element`): configuration q on a group (stored with nq numbers),
  velocities v in the Lie algebra (nv numbers), configuration update q_{n+1} = q_n o exp(incr).
  Given M, h(q, v), Q and Phi(q) in terms of v, CasADi derives the same functions with
  derivatives with respect to configuration perturbations q o exp(delta) and the tangent
  operator of the exponential map.

No derivative is written by hand.

Local unknown vector of an element: x = [vd (nv); lam (nc)], local residual rows
[dynamics (nv); constraints (nc)]; rows and columns share one local-to-global index map in
the runtime. All res / iter / ic functions have the input signature STD_INPUTS.
"""

import casadi as ca

# Order of the integration coefficients in the `coef` input.
COEF_NAMES = ("alpha", "beta", "gamma", "h")
# Uniform input signature of res / iter / ic: incr is the configuration increment of the
# step (q = q_n o exp(incr)), vector-space elements do not use it; t is the time of the state
# (time-dependent forces).
STD_INPUTS = ("q", "qd", "qdd", "lam", "p", "coef", "incr", "t")
# Function roles in registry order. Roles an element type does not provide are empty.
# post(q, qd, p, t) -> p: new element parameters after each accepted step (element state,
# e.g. the reference value that keeps an angle measure continuous).
ROLES = ("res", "iter", "iter_noqv", "ic", "init", "out", "update", "scale", "mn_qd", "jac_qd", "post")


class ElementFunctions:
    """CasADi functions of one element type, keyed by role."""

    def __init__(self, name, **funs):
        unknown = set(funs) - set(ROLES)
        if unknown:
            raise ValueError(f"unknown roles {unknown}")
        self.name = name
        self.funs = funs

    def __getattr__(self, role):
        try:
            return self.__dict__["funs"][role]
        except KeyError:
            raise AttributeError(role) from None

    def all(self):
        return [self.funs[r] for r in ROLES if r in self.funs]


def lagrange_terms(q, qd, T, V=0, Q=None, Phi=None, h=None):
    """Return M, h, Q, Phi, Phi_q with h = (d2L/dqd dq) qd - dL/dq.

    An element type may pass its own quadratic velocity vector `h` (for example a form
    that must match the legacy FreeDyn kernel); M is always d2T/dqd2.
    """
    nq = q.numel()
    L = T - V
    dL_dqd = ca.gradient(L, qd)
    M = ca.jacobian(dL_dqd, qd)
    if h is None:
        h = ca.jtimes(dL_dqd, q, qd) - ca.gradient(L, q)
    if Q is None:
        Q = ca.SX(nq, 1)
    if Phi is None:
        Phi = ca.SX(0, 1)
    Phi_q = ca.jacobian(Phi, q)
    return M, h, Q, Phi, Phi_q


def _symbols(nv, nc):
    qdd = ca.SX.sym("qdd", nv)
    lam = ca.SX.sym("lam", nc)
    coef = ca.SX.sym("coef", len(COEF_NAMES))
    incr = ca.SX.sym("incr", nv)
    return qdd, lam, coef, incr


def _time(t):
    return ca.SX.sym("t") if t is None else t


def _std_functions(name, ins, r, f_full, J, ic):
    """res, iter and ic with the input signature STD_INPUTS; ic = (A, b, phi, phid)."""
    pre, names = f"fd_{name}_", list(STD_INPUTS)
    return dict(
        res=ca.Function(pre + "res", ins, [r, f_full], names, ["r", "f"]),
        iter=ca.Function(pre + "iter", ins, [J], names, ["J"]),
        ic=ca.Function(pre + "ic", ins, list(ic), names, ["A", "b", "phi", "phid"]),
    )


def lagrange_element(name, q, qd, p, T, V=0, Q=None, Phi=None, h=None, owns_coordinates=True, t=None):
    """Build res, iter, iter_noqv, ic, update and scale for a vector-space element type.

    Elements that only constrain the coordinates of other elements (joints) set
    owns_coordinates=False and get no update / scale / jac_qd functions. `t` is the time
    symbol T, V, Q may depend on (a fresh one if None).

    HHT residual with effective coefficients (alpha, beta, gamma, h) chosen by the runtime
    for the Negrut form, or (0, (1+alpha) beta, (1+alpha) gamma, h) for the Arnold-Bruls form:

        r_dyn = M qdd / (1+alpha) + f(q, qd, lam)    [- alpha/(1+alpha) f_n, added by the runtime]
        r_con = Phi(q) / (beta h^2)
        f     = h(q, qd) + Phi_q^T lam - Q

    With q = q_pred + beta h^2 qdd and qd = qd_pred + gamma h qdd the Newton iteration matrix
    with respect to x = [qdd; lam] is J = [dr/dqdd + gamma h dr/dqd + beta h^2 dr/dq, dr/dlam].
    """
    nq = q.numel()
    M, h, Q, Phi, Phi_q = lagrange_terms(q, qd, T, V, Q, Phi, h)
    nc = Phi.numel()
    qdd, lam, coef, incr = _symbols(nq, nc)
    alpha, beta, gamma, hs = ca.vertsplit(coef)

    def residual(hv):
        f = hv + ca.mtimes(Phi_q.T, lam) - Q
        return f, ca.vertcat(ca.mtimes(M, qdd) / (1 + alpha) + f, Phi / (beta * hs**2))

    def iteration_matrix(r):
        return ca.horzcat(
            ca.jacobian(r, qdd) + gamma * hs * ca.jacobian(r, qd) + beta * hs**2 * ca.jacobian(r, q),
            ca.jacobian(r, lam),
        )

    f, r = residual(h)
    J = iteration_matrix(r)
    # Variant without the derivatives of h, like the legacy kernel with
    # RegardVelocityInertiaForcesInsideJacobian = no: h enters as a constant. h is additive
    # in r, so the matrix does not contain that constant.
    h_const = ca.SX.sym("h_const", nq)
    J_noqv = iteration_matrix(residual(h_const)[1])
    assert not ca.depends_on(J_noqv, h_const)
    # f padded with structural zeros so every vector output lives in the x row space.
    f_full = ca.vertcat(f, ca.SX(nc, 1))

    # Consistent accelerations: [M Phi_q^T; Phi_q 0] [qdd; lam] = [Q - h; gamma] with
    # gamma = -(Phi_q qd)_q qd - 2 (Phi_q qd)_t - Phi_tt (rheonomic constraints: drivers).
    tt = _time(t)
    A = ca.blockcat([[M, Phi_q.T], [Phi_q, ca.SX(nc, nc)]])
    Phid = ca.mtimes(Phi_q, qd)
    Phi_t = ca.jacobian(Phi, tt)
    gam = -ca.jtimes(Phid, q, qd) - 2 * ca.jacobian(Phid, tt) - ca.jacobian(Phi_t, tt)
    b = ca.vertcat(Q - h, gam)
    phi_full = ca.vertcat(ca.SX(nq, 1), Phi)
    phid_full = ca.vertcat(ca.SX(nq, 1), Phid + Phi_t)
    # Jacobian of r with respect to qd (used by the modified Newmark formulas).
    jac_qd = ca.jacobian(r, qd)

    ins = [q, qd, qdd, lam, p, coef, incr, tt]
    pre = f"fd_{name}_"
    funs = _std_functions(name, ins, r, f_full, J, (A, b, phi_full, phid_full))
    funs["iter_noqv"] = ca.Function(pre + "iter_noqv", ins, [J_noqv], list(STD_INPUTS), ["J"])
    if owns_coordinates:
        funs.update(
            jac_qd=ca.Function(pre + "jac_qd", ins, [jac_qd], list(STD_INPUTS), ["J_qd"]),
            update=ca.Function(pre + "update", [q, incr], [q + incr], ["q_n", "incr"], ["q"]),
            scale=ca.Function(pre + "scale", [q], [ca.fabs(q)], ["q"], ["s"]),
        )
    return funs


class Chain:
    """Total derivatives through intermediate symbols [(S_k, s_k[, known_k])] (see
    group_element): d(e)/dw = partial(e)/dw + sum_k de/dS_k d(S_k)/dw, where d(S_k)/dw is
    known_k[w] if given (an expression that may use S_k itself, e.g. dA/ddelta = A skew(e_i)
    for a rotation matrix A) and otherwise follows from s_k the same way. partial: {w: function
    of an expression} with the S_k held fixed."""

    def __init__(self, intermediates, partial):
        self.inter, self.partial, self.memo = [tuple(x) for x in intermediates], partial, {}

    def d(self, e, wrt):
        out = self.partial[wrt](e)
        for k, entry in enumerate(self.inter):
            if ca.depends_on(e, entry[0]):
                out = out + ca.mtimes(ca.jacobian(e, entry[0]), self._dS(k, wrt))
        return out

    def _dS(self, k, wrt):
        if (k, wrt) not in self.memo:
            entry = self.inter[k]
            known = entry[2] if len(entry) > 2 else {}
            self.memo[(k, wrt)] = known[wrt] if wrt in known else self.d(entry[1], wrt)
        return self.memo[(k, wrt)]

    def expand(self, exprs):
        """exprs with every S_k replaced by s_k (last first: s_k may use earlier S_j)."""
        return [expand(e, self.inter) for e in exprs]


def expand(expr, intermediates):
    """expr with the intermediate symbols S_k replaced by their expressions s_k."""
    for entry in reversed(list(intermediates)):
        expr = ca.substitute(expr, entry[0], entry[1])
    return expr


def group_element(name, q, v, p, M, h, perturb, exp_update, tangent, scale, Q=None, Phi=None, t=None,
                  intermediates=()):
    """Build res, iter, ic, update and scale for a Lie group element type.

    Elements that only constrain the coordinates of other elements (joints) pass
    exp_update = scale = None and get no update / scale functions.

    perturb(q, delta): q o exp(delta) to first order in delta (only its derivative at
        delta = 0 is used).
    exp_update(q_n, incr): q_n o exp(incr), exact.
    t: time symbol M, h, Q may depend on (a fresh one if None).
    tangent(incr): T(incr) with q_n o exp(incr + d) = q_n o exp(incr) o exp(T(incr) d + O(d^2)).
    scale(q): magnitude per velocity dof, for the weighted norms of the step size control.

    Lie group HHT (Arnold-Bruls form, effective coefficients (0, beta', gamma', h)): with
    q = q_n o exp(incr), incr = h Delta_pred + beta' h^2 vd and v = v_pred + gamma' h vd, a
    change d vd moves the configuration by delta = T(incr) beta' h^2 d vd, so

        J = [dr/dvd + gamma' h dr/dv + beta' h^2 (dr/ddelta) T(incr),  dr/dlam].

    intermediates: [(S_k, s_k[, known_k])] symbols S_k that Q and Phi may use in place of the
    expressions s_k(q, v, t, S_1..S_k-1): the rotation matrices of the connected bodies (with
    their known derivative), measures, the value of a force law. Derivatives follow the chain
    rule through them (dr/dS_k dS_k/dx, see Chain): each s_k is differentiated once, instead of
    once per row of r and never through the quaternion-to-matrix map, which keeps the generated
    code small. All outputs are expressed in q, v, t at the end.
    """
    nv = v.numel()
    if Q is None:
        Q = ca.SX(nv, 1)
    if Phi is None:
        Phi = ca.SX(0, 1)
    nc = Phi.numel()
    vd, lam, coef, incr = _symbols(nv, nc)
    alpha, beta, gamma, hs = ca.vertsplit(coef)
    delta = ca.SX.sym("delta", nv)
    zero = ca.DM.zeros(nv, 1)

    tt = _time(t)

    def partial_delta(expr):
        """Derivative of expr(q) with respect to the perturbation q o exp(delta) at delta = 0."""
        pert = ca.substitute(expr, q, perturb(q, delta))
        return ca.substitute(ca.jacobian(pert, delta), delta, zero)

    partial = {"delta": partial_delta, "v": lambda e: ca.jacobian(e, v), "t": lambda e: ca.jacobian(e, tt)}
    chain = Chain(intermediates, partial)
    d_delta, d_v, d_t = (lambda e, w=w: chain.d(e, w) for w in ("delta", "v", "t"))

    B = d_delta(Phi)  # constraint Jacobian with respect to delta
    f = h + ca.mtimes(B.T, lam) - Q
    r = ca.vertcat(ca.mtimes(M, vd) / (1 + alpha) + f, Phi / (beta * hs**2))
    J = ca.horzcat(
        ca.jacobian(r, vd) + gamma * hs * d_v(r) + beta * hs**2 * ca.mtimes(d_delta(r), tangent(incr)),
        ca.jacobian(r, lam),
    )
    f_full = ca.vertcat(f, ca.SX(nc, 1))
    # Consistent accelerations: [M B^T; B 0] [vd; lam] = [Q - h; gamma] with
    # gamma = -d/dt(B) v - 2 (B v)_t - Phi_tt (rheonomic constraints: drivers).
    A = ca.blockcat([[M, B.T], [B, ca.SX(nc, nc)]])
    Bv = ca.mtimes(B, v)
    Phi_t = d_t(Phi)
    b = ca.vertcat(Q - h, -ca.mtimes(d_delta(Bv), v) - 2 * d_t(Bv) - d_t(Phi_t))
    phi_full = ca.vertcat(ca.SX(nv, 1), Phi)
    phid_full = ca.vertcat(ca.SX(nv, 1), Bv + Phi_t)
    r, f_full, J, A, b, phi_full, phid_full = chain.expand([r, f_full, J, A, b, phi_full, phid_full])

    ins = [q, v, vd, lam, p, coef, incr, tt]
    pre = f"fd_{name}_"
    q_n = ca.SX.sym("q_n", q.numel())
    funs = _std_functions(name, ins, r, f_full, J, (A, b, phi_full, phid_full))
    if exp_update is not None:
        funs.update(
            update=ca.Function(pre + "update", [q_n, incr], [exp_update(q_n, incr)], ["q_n", "incr"], ["q"]),
            scale=ca.Function(pre + "scale", [q], [scale(q)], ["q"], ["s"]),
        )
    return funs
