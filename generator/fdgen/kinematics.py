"""Rigid body kinematics in terms of the local symbols of an element, for both body types.

Joints, forces and measures connect bodies and are written once in terms of this interface:
point positions and velocities, rotation matrices and angular velocities of each connected
body. The body type decides how they depend on the element's coordinates and velocities:

- "lie" (rigid_body): q_k = [R; theta] (7), v_k = [Rdot; omega_bar] (6),
- "euler" (rigid_body_euler, legacy profile): q_k = [R; theta] (7), qd_k = dq_k/dt (7).

Accelerations (outputs only: sensors, body channels) use a_k, the accelerations of the velocity
dofs that HHT solves for: [Rddot; omegadot_bar] on the Lie group, d(qd_k)/dt for Euler parameters.
"""

import casadi as ca

from . import rigid_body as rb
from .lagrange import Chain, expand, group_element, lagrange_element

NV = {"lie": 6, "euler": 7}
OUT_INPUTS = ["q", "qd", "qdd", "p", "t"]  # inputs of every output function (runtime: System::outputs)


class Body:
    """Kinematics of body k of an element (local symbols q_k, v_k). A: symbol of the rotation
    matrix (Lie group bodies, an intermediate of the element), else A(theta)."""

    def __init__(self, rep, q, v, A=None, a=None):
        self.rep, self.q, self.v, self.a = rep, q, v, a
        self.R, self.theta = q[0:3], q[3:7]
        self.A = rb.rotation_matrix(self.theta) if A is None else A
        self.Rdot = v[0:3]
        if rep == "lie":
            self.omega_body = v[3:6]
            self.omega = ca.mtimes(self.A, self.omega_body)  # global
        else:
            # Legacy forms, exact also off |theta| = 1: omega = 2 G thetadot, omega_bar = 2 Gbar thetadot.
            self.omega_body = 2 * ca.mtimes(rb.g_body(self.theta), v[3:7])
            self.omega = 2 * ca.mtimes(rb.g_global(self.theta), v[3:7])

    def point(self, u):
        """Global position of the body-fixed point u (body coordinates)."""
        return self.R + ca.mtimes(self.A, u)

    def point_velocity(self, u):
        if self.rep == "lie":
            return self.Rdot + ca.mtimes(self.A, ca.cross(self.omega_body, u))
        return ca.jtimes(self.point(u), self.q, self.v)  # Rdot + d(A u)/dtheta thetadot

    def axis(self, b):
        """Global direction of the body-fixed vector b."""
        return ca.mtimes(self.A, b)

    def _rate2(self, expr):
        """Time derivative of an expression of q and v (Euler parameters): along v and a."""
        return ca.jtimes(expr, self.q, self.v) + ca.jtimes(expr, self.v, self.a)

    @property
    def omegadot_body(self):
        return self.a[3:6] if self.rep == "lie" else self._rate2(self.omega_body)

    @property
    def omegadot(self):
        """Global angular acceleration: d/dt (A omega_bar) = A omegadot_bar."""
        return ca.mtimes(self.A, self.a[3:6]) if self.rep == "lie" else self._rate2(self.omega)

    def point_acceleration(self, u):
        """Global acceleration of the body-fixed point u."""
        if self.rep == "lie":
            w = self.omega_body
            return self.a[0:3] + ca.mtimes(self.A, ca.cross(self.a[3:6], u) + ca.cross(w, ca.cross(w, u)))
        return self._rate2(self.point_velocity(u))


class Ground:
    """The ground as a connected 'body': fixed points and axes, no motion."""

    def point(self, r):
        return r

    def point_velocity(self, r):
        return ca.SX(3, 1)

    def axis(self, b):
        return b

    omega = ca.SX(3, 1)


class Connector:
    """Symbols of an element that connects n_bodies bodies of one representation."""

    def __init__(self, rep, n_bodies, n_p):
        self.rep, self.n = rep, n_bodies
        nv = NV[rep]
        self.q = ca.SX.sym("q", 7 * n_bodies)
        self.v = ca.SX.sym("qd", nv * n_bodies)
        self.a = ca.SX.sym("qdd", nv * n_bodies)  # accelerations (outputs only)
        self.p = ca.SX.sym("p", n_p)
        self.t = ca.SX.sym("t")
        # Lie group: the rotation matrix of each body is a symbol A_k (intermediate) with the
        # known derivative dA/ddelta_rot,i = A skew(e_i) of A(theta o exp(delta)), so no derivative
        # is ever taken through the quaternion-to-matrix map.
        self.intermediates = []
        As = [None] * n_bodies
        if rep == "lie":
            for k in range(n_bodies):
                A = As[k] = ca.SX.sym(f"A{k}", 3, 3)
                dA = ca.SX(9, nv * n_bodies)
                for i in range(3):
                    dA[:, nv * k + 3 + i] = ca.vec(ca.mtimes(A, rb.skew(ca.DM.eye(3)[:, i])))
                self.intermediates.append((ca.vec(A), ca.vec(rb.rotation_matrix(self.q[7 * k + 3:7 * k + 7])),
                                           {"delta": dA, "v": ca.SX(9, nv * n_bodies), "t": ca.SX(9, 1)}))
        self.bodies = [Body(rep, self.q[7 * k:7 * k + 7], self.v[nv * k:nv * k + nv], As[k], self.a[nv * k:nv * k + nv])
                       for k in range(n_bodies)]
        self._chain = Chain(self.intermediates, {"delta": self._partial_delta})

    def _perturb(self, q, d):
        """q o exp(d) of all connected bodies (first order)."""
        return ca.vertcat(*[rb.perturb(q[7 * i:7 * i + 7], d[6 * i:6 * i + 6]) for i in range(self.n)])

    def _tangent(self, incr):
        return ca.diagcat(*[ca.diagcat(ca.SX.eye(3), rb.tangent_so3(incr[6 * i + 3:6 * i + 6])) for i in range(self.n)])

    def _partial_delta(self, expr):
        delta = ca.SX.sym("delta", self.v.numel())
        pert = self._perturb(self.q, delta)
        return ca.substitute(ca.jacobian(ca.substitute(expr, self.q, pert), delta), delta, ca.DM.zeros(self.v.numel()))

    def rate(self, expr):
        """Exact time derivative of a configuration-dependent expression (linear in v): along
        thetadot for Euler parameters, along the configuration perturbation q o exp(delta) on
        the Lie group."""
        if self.rep == "euler":
            return ca.jtimes(expr, self.q, self.v)
        return ca.mtimes(self._chain.d(expr, "delta"), self.v)

    def expand(self, expr):
        """expr in q, v, t only (rotation matrix symbols replaced)."""
        return expand(expr, self.intermediates)

    def output_function(self, name, out, expand=None):
        """Output function of the element: (q, qd, qdd, p, t) -> the named expressions of out."""
        expand = expand or self.expand
        return ca.Function(f"fd_{name}_out", [self.q, self.v, self.a, self.p, self.t], [expand(v) for v in out.values()],
                           OUT_INPUTS, list(out))

    def generalized_force(self, loads):
        """Q of the element from its loads, by virtual power Q . v:
        (body index, force F at body point u | None, torque M | None), global F and M:
            F . v_P + M . omega;
        (None, s, rate): s conjugate to a scalar measure m with rate mdot (linear in v): s mdot."""
        Q = ca.SX(self.v.numel(), 1)
        for load in loads:
            if load[0] is None:
                _, s, rate = load
                Q += s * ca.gradient(rate, self.v)
                continue
            k, F, u, M = load
            b = self.bodies[k]
            if F is not None:
                Q += ca.mtimes(ca.jacobian(b.point_velocity(u), self.v).T, F)
            if M is not None:
                Q += ca.mtimes(ca.jacobian(b.omega, self.v).T, M)
        return Q

    def functions(self, name, Q=None, Phi=None, intermediates=()):
        """res, iter (, iter_noqv), ic of an element without coordinates of its own.
        intermediates: [(S_k, s_k)] scalars Q and Phi are written in (see group_element)."""
        nv = self.v.numel()
        if self.rep == "euler":
            Q, Phi = (None if e is None else expand(e, intermediates) for e in (Q, Phi))
            return lagrange_element(name, self.q, self.v, self.p, ca.SX(0), Q=Q, Phi=Phi,
                                    owns_coordinates=False, t=self.t)
        return group_element(name, self.q, self.v, self.p, ca.SX(nv, nv), ca.SX(nv, 1), self._perturb, None,
                             self._tangent, None, Q=Q, Phi=Phi, t=self.t,
                             intermediates=self.intermediates + list(intermediates))
