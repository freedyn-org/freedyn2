"""Measures: scalars of the current state used in force laws and drivers.

Types (used by name in expressions):

  displacement  component of P1 - P2 in `frame` (global or a body's frame)
  velocity      component of V1 - V2 (velocities of the points) in `frame`; not the derivative
                of the displacement in a rotating frame
  distance      |P1 - P2|;  distance_rate: d/dt |P1 - P2|
  angle         angle of the vector a (body1) in the plane (b, c) of body2 / the ground,
                atan2(a.c, a.b); a = x, (b, c) = (x, y) of `orientation`, or `vector` and
                `plane` (global at t = 0). range: continuous (default; counts revolutions: the
                last accepted value is element state, updated by the post function),
                minus_pi_pi or zero_2pi.
  angular_rate  its exact rate (x ydot - y xdot) / (x^2 + y^2) with x = a.b, y = a.c;
                form: legacy gives the numerator only (legacy AngleVel measures)
  angular_velocity  component of omega1 - omega2 in `frame` (global or a body)

P1 = body1(point1), P2 = body2(point2) or the ground point point2 (global, at t = 0).
offset: initial subtracts the initial value (displacement, distance, angle), a number subtracts
that number.
"""

import casadi as ca
import numpy as np

from .geometry import local_axis, local_point, rotation_matrix

MEASURE_TYPES = ("displacement", "velocity", "distance", "distance_rate", "angle", "angular_rate", "direction",
                 "angular_velocity")
POSITION_MEASURES = ("displacement", "distance", "angle")  # usable by drivers


class Consts:
    """Element parameters p collected while building expressions."""

    def __init__(self):
        self.syms, self.values = [], []
        self.updates = {}  # state symbol -> its new value after an accepted step (post)

    def add(self, value, name="c"):
        v = np.atleast_1d(np.asarray(value, dtype=float))
        s = ca.SX.sym(f"{name}_{len(self.values)}", v.size)
        self.syms.append(s)
        self.values += list(v)
        return s if v.size > 1 else s[0]

    def add_state(self, value, name="state"):
        """Scalar element state kept in p, updated by the post function."""
        s = ca.SX.sym(f"{name}_{len(self.values)}")
        self.syms.append(s)
        self.values.append(float(value))
        return s

    def post(self):
        """p after an accepted step: state entries replaced by their updates."""
        return ca.vertcat(*[self.updates.get(id(s), s) if s.numel() == 1 else s for s in self.syms]) \
            if self.syms else ca.SX(0, 1)


_TWO_PI = 2 * np.pi
ANGLE_TOL_PI = 1e-10  # initial angles closer than this to -pi count as +pi (as MBS_ANGLE_TOL_PI)


def _wrap(v, lo_open):
    """v into (-pi, pi] (lo_open) or [0, 2 pi)."""
    if lo_open:
        return ca.if_else(v <= -np.pi, v + _TWO_PI, ca.if_else(v > np.pi, v - _TWO_PI, v))
    return ca.if_else(v < 0, v + _TWO_PI, ca.if_else(v >= _TWO_PI, v - _TWO_PI, v))


def _component(vec, spec):
    return vec["xyz".index(spec["component"])]


def _in_frame(vec, spec, kin):
    """vec in the frame of spec (global or a body)."""
    frame = spec.get("frame", "global")
    return vec if frame == "global" else ca.mtimes(kin(frame).A.T, vec)


def _direction(spec, b1, b2, kin, consts, model):
    # component of a global direction in the frame of a body (e.g. the vertical in the frame
    # of a vehicle: its lean and pitch)
    g = consts.add(local_axis(model, "ground", spec["vector"]), "g")
    return _component(ca.mtimes(kin(spec.get("frame", b1)).A.T, g), spec)


def _angular_velocity(spec, b1, b2, kin):
    if spec.get("frame", "global") == b1 and b2 == "ground":
        return _component(kin(b1).omega_body, spec)  # also the legacy form for Euler parameters (2 Gbar thetadot)
    return _component(_in_frame(kin(b1).omega - kin(b2).omega, spec, kin), spec)


def _angle(spec, b1, b2, kin, consts, model, q0_value, rate):
    """Angle of a (body1) in the plane (b, c) of body2 / the ground, atan2(a.c, a.b), or its rate."""
    t = spec["type"]
    norm = spec.get("normalize", True)
    if "vector" in spec:
        a0, (b0, c0) = spec["vector"], spec["plane"]
    else:
        o = np.asarray(spec.get("orientation", [1.0, 0, 0, 0]), dtype=float)
        K = rotation_matrix(o / np.linalg.norm(o))
        a0, b0, c0 = K[:, 0], K[:, 0], K[:, 1]
    a = kin(b1).axis(consts.add(local_axis(model, b1, a0, norm), "a"))
    bb = kin(b2).axis(consts.add(local_axis(model, b2, b0, norm), "b"))
    cc = kin(b2).axis(consts.add(local_axis(model, b2, c0, norm), "c"))
    x, y = ca.dot(a, bb), ca.dot(a, cc)
    if t == "angular_rate":
        num = x * rate(y) - y * rate(x)
        return num if spec.get("form") == "legacy" else num / (x * x + y * y)
    r = ca.atan2(y, x)
    r0 = q0_value(r)
    rng = spec.get("range", "continuous")
    offset = spec.get("offset", "none") == "initial"
    if rng == "continuous":
        ref0 = r0 + _TWO_PI if r0 < -np.pi + ANGLE_TOL_PI else r0
        ref = consts.add_state(ref0, "angle_ref")  # last accepted value: keeps the angle continuous
        value = r + _TWO_PI * ca.floor((ref - r) / _TWO_PI + 0.5)
        consts.updates[id(ref)] = value
        return value - consts.add(ref0, "offset") if offset else value
    if rng == "minus_pi_pi":
        return _wrap(r - consts.add(r0, "offset"), True) if offset else r
    if rng == "zero_2pi":
        r2 = ca.if_else(r < 0, r + _TWO_PI, r)
        return _wrap(r2 - consts.add(r0, "offset"), False) if offset else r2
    raise ValueError(f"angle range must be continuous, minus_pi_pi or zero_2pi, not {rng!r}")


def _points(spec, b1, b2, kin, consts, model, q0_value):
    """displacement, velocity, distance, distance_rate of the points P1, P2."""
    t = spec["type"]
    u1 = consts.add(local_point(model, b1, spec.get("point1")), "u1")
    u2 = consts.add(local_point(model, b2, spec.get("point2")), "u2")
    k1, k2 = kin(b1), kin(b2)
    d = k1.point(u1) - k2.point(u2)
    w = k1.point_velocity(u1) - k2.point_velocity(u2)
    if t == "distance_rate":
        return ca.dot(d, w) / ca.norm_2(d)
    if t == "velocity":
        return _component(_in_frame(w, spec, kin), spec)
    value = ca.norm_2(d) if t == "distance" else _component(_in_frame(d, spec, kin), spec)
    off = spec.get("offset", "none")
    if off == "initial":
        value = value - consts.add(q0_value(value), "offset")
    elif not isinstance(off, str):
        value = value - consts.add(off, "offset")
    return value


def measure(spec, kin, consts, model, q0_value, rate):
    """CasADi expression of a measure; kin(body name) gives Body/Ground kinematics, rate(expr)
    the exact time derivative of a configuration expression."""
    b1, b2 = spec["body1"], spec.get("body2", "ground")
    t = spec["type"]
    if t == "direction":
        return _direction(spec, b1, b2, kin, consts, model)
    if t == "angular_velocity":
        return _angular_velocity(spec, b1, b2, kin)
    if t in ("angle", "angular_rate"):
        return _angle(spec, b1, b2, kin, consts, model, q0_value, rate)
    return _points(spec, b1, b2, kin, consts, model, q0_value)
