"""Curve forces: a force or torque that is a curve (spline) of a measure,

    s = a * curve(b * m + c)

along a direction (line of sight, or fixed in space or in body1) or about an axis, on body1 and
-s on body2. Library element types (compiled into the runtime, the curve's table is data): the
measure's frame, component and directions are parameters, so a few types cover them all

    curve_<measure>_<application>_<end>   measure: displacement, velocity, distance,
        distance_rate, angle, angular_rate, angular_velocity; application: los (force along
        the line of sight), dir (force along a direction), torque; end: ground or bb.

The measure must act between the force's bodies (either order); a measure of other bodies, an
angle range other than continuous, the legacy form of an angular rate or the Euler-parameter
representation make it an expression force in the model plugin (fdgen.expression_forces),
with the same law.
"""

import casadi as ca
import numpy as np

from .forces import LibraryElement, force_element_type
from .geometry import body_frame, local_axis, local_point, rotation_matrix
from .kinematics import Ground
from .measures import ANGLE_TOL_PI
from .splines import cubic
from .splines import table as spline_table

MEASURES = ("displacement", "velocity", "distance", "distance_rate", "angle", "angular_rate", "angular_velocity")
APPLICATIONS = ("los", "dir", "torque")
# parameters of the measure part, the law [a, b, c, segment (5)] and the application part
_N_MEASURE = {"displacement": 16, "velocity": 16, "distance": 7, "distance_rate": 6, "angle": 11,
              "angular_rate": 9, "angular_velocity": 9}
_N_APP = {"los": 6, "dir": 12, "torque": 6}
_TWO_PI = 2 * np.pi

KINDS = {f"curve_{m}_{a}_{end}": (1 if end == "ground" else 2, _N_MEASURE[m] + 8 + _N_APP[a])
         for m in MEASURES for a in APPLICATIONS for end in ("ground", "bb")}


def _parts(kind):
    _, m, *rest = kind.split("_")
    if m in ("distance", "angular") and rest[0] in ("rate", "velocity"):
        m += "_" + rest.pop(0)
    return m, rest[0], rest[1]


def _measure(c, m, b1, b2, p):
    """(value, rate-free parts) of the measure from p[0:n]; also the state update of an angle."""
    if m in ("displacement", "velocity"):
        u1, u2, eg, e1, e2, m0 = p[0:3], p[3:6], p[6:9], p[9:12], p[12:15], p[15]
        e = eg + b1.axis(e1) + b2.axis(e2)
        if m == "displacement":
            return ca.dot(e, b1.point(u1) - b2.point(u2)) - m0, None
        return ca.dot(e, b1.point_velocity(u1) - b2.point_velocity(u2)), None
    if m in ("distance", "distance_rate"):
        u1, u2 = p[0:3], p[3:6]
        d = b1.point(u1) - b2.point(u2)
        if m == "distance":
            return ca.norm_2(d) - p[6], None
        return ca.dot(d, b1.point_velocity(u1) - b2.point_velocity(u2)) / ca.norm_2(d), None
    if m == "angular_velocity":
        eg, e1, e2 = p[0:3], p[3:6], p[6:9]
        return ca.dot(eg + b1.axis(e1) + b2.axis(e2), b1.omega - b2.omega), None
    a, bb, cc = b1.axis(p[0:3]), b2.axis(p[3:6]), b2.axis(p[6:9])
    x, y = ca.dot(a, bb), ca.dot(a, cc)
    if m == "angular_rate":
        return (x * c.rate(y) - y * c.rate(x)) / (x * x + y * y), None
    r = ca.atan2(y, x)
    ref, m0 = p[9], p[10]
    value = r + _TWO_PI * ca.floor((ref - r) / _TWO_PI + 0.5)
    return value - m0, (9, value)  # the reference: the last accepted value


def law(kind, c):
    """Loads and outputs of a curve force type; sets c.spline_args and c.state_update."""
    m, app, _ = _parts(kind)
    p, b1 = c.p, c.bodies[0]
    b2 = c.bodies[1] if c.n == 2 else Ground()
    n = _N_MEASURE[m]
    value, update = _measure(c, m, b1, b2, p[0:n])
    A, B, C, seg = p[n], p[n + 1], p[n + 2], p[n + 3:n + 8]
    arg = B * value + C
    c.spline_args = [arg]
    c.state_update = update
    s = A * cubic(seg, arg)
    q = p[n + 8:]
    if app == "torque":
        e = q[0:3] + b1.axis(q[3:6])
        loads = [(0, None, None, s * e)] + ([(1, None, None, -s * e)] if c.n == 2 else [])
        return loads, {"value": s, "torque": s * e, "measure": value}
    u1, u2 = q[0:3], q[3:6]
    if app == "los":
        dr = b2.point(u2) - b1.point(u1)
        e = dr / ca.norm_2(dr)
    else:
        e = q[6:9] + b1.axis(q[9:12])
    loads = [(0, s * e, u1, None)] + ([(1, -s * e, u2, None)] if c.n == 2 else [])
    return loads, {"value": s, "force": s * e, "measure": value}


def build(kind):
    """ElementFunctions of a curve force type (Lie group, fdgen.codegen)."""
    holder = {}

    def f(c):
        out = law(kind, c)
        holder["c"] = c
        return out
    elem = force_element_type(kind, *KINDS[kind], f)
    c = holder["c"]
    if c.state_update is not None:  # continuous angle: its reference after each accepted step
        k, value = c.state_update
        new = ca.SX(c.p.numel(), 1)
        new[k] = c.expand(value)
        elem.funs["post"] = ca.Function(f"fd_{kind}_post", [c.q, c.v, c.p, c.t], [new], ["q", "qd", "p", "t"], ["p_new"])
    return elem


# --- elements of a model ------------------------------------------------------------------------

def _ends(force):
    return force["body1"], force["body2"]


def _frame_weights(spec, fb, sigma):
    """(eg, e1, e2) of the component of a measure in its frame, on the force's bodies fb."""
    k = "xyz".index(spec["component"])
    unit = np.eye(3)[k] * sigma
    frame = spec.get("frame", "global")
    z = np.zeros(3)
    if frame == "global":
        return unit, z, z
    if frame == fb[0]:
        return z, unit, z
    if frame == fb[1] and fb[1] != "ground":
        return z, z, unit
    return None


def _angle_axes(model, spec):
    norm = spec.get("normalize", True)
    if "vector" in spec:
        a0, (b0, c0) = spec["vector"], spec["plane"]
    else:
        o = np.asarray(spec.get("orientation", [1.0, 0, 0, 0]), dtype=float)
        K = rotation_matrix(o / np.linalg.norm(o))
        a0, b0, c0 = K[:, 0], K[:, 0], K[:, 1]
    b1, b2 = spec["body1"], spec.get("body2", "ground")
    return (local_axis(model, b1, a0, norm), local_axis(model, b2, b0, norm), local_axis(model, b2, c0, norm))


def _global(model, body, v, point=False):
    if body == "ground":
        return np.asarray(v, float)
    R, A = body_frame(model["bodies"][body])
    return R + A @ v if point else A @ v


def _measure_params(model, spec, fb):
    """p of the measure part, or None if no library type covers it."""
    t = spec["type"]
    if t not in MEASURES:
        return None
    mb = (spec["body1"], spec.get("body2", "ground"))
    if mb == fb:
        sigma = 1.0
    elif mb == fb[::-1] and t in ("displacement", "velocity", "distance", "distance_rate"):
        sigma = -1.0  # the measure between the same bodies the other way round
    else:
        return None
    if t in ("displacement", "velocity", "distance", "distance_rate"):
        u = {mb[0]: local_point(model, mb[0], spec.get("point1")), mb[1]: local_point(model, mb[1], spec.get("point2"))}
        u1, u2 = u[fb[0]], u[fb[1]]
        if t in ("distance", "distance_rate"):
            if t == "distance_rate":
                return [*u1, *u2]
            d = np.linalg.norm(_global(model, fb[0], u1, True) - _global(model, fb[1], u2, True))
            return [*u1, *u2, _offset(spec, d)]
        w = _frame_weights(spec, fb, sigma)
        if w is None:
            return None
        if t == "velocity":
            return [*u1, *u2, *w[0], *w[1], *w[2], 0.0]
        e = w[0] + _global(model, fb[0], w[1]) + (_global(model, fb[1], w[2]) if fb[1] != "ground" else 0)
        d = _global(model, fb[0], u1, True) - _global(model, fb[1], u2, True)
        return [*u1, *u2, *w[0], *w[1], *w[2], _offset(spec, float(e @ d))]
    if t == "angular_velocity":
        w = _frame_weights(spec, fb, 1.0)
        return None if w is None else [*w[0], *w[1], *w[2]]
    if t == "angular_rate" and spec.get("form") == "legacy":
        return None
    a, b, c = _angle_axes(model, spec)
    if t == "angular_rate":
        return [*a, *b, *c]
    if spec.get("range", "continuous") != "continuous":
        return None
    ag, bg, cg = _global(model, mb[0], a), _global(model, mb[1], b), _global(model, mb[1], c)
    r0 = float(np.arctan2(ag @ cg, ag @ bg))
    ref0 = r0 + _TWO_PI if r0 < -np.pi + ANGLE_TOL_PI else r0
    return [*a, *b, *c, ref0, ref0 if spec.get("offset", "none") == "initial" else 0.0]


def _offset(spec, value0):
    off = spec.get("offset", "none")
    if off == "initial":
        return value0
    return 0.0 if isinstance(off, str) else float(off)


def element(force, model):
    """LibraryElement of a curve_force, or None (then it is an expression force, as_expression)."""
    if any(model["bodies"][b].get("element", "rigid_body") != "rigid_body" for b in _ends(force) if b != "ground"):
        return None
    fb = _ends(force)
    spec = model["measures"][force["measure"]]
    mp = _measure_params(model, spec, fb)
    if mp is None:
        return None
    sp = model["splines"][force["curve"]]
    tab = spline_table(sp["x"], sp["y"], sp.get("type", "akima"))
    a, b, c = (float(force.get(k, d)) for k, d in (("scale", 1.0), ("argument_scale", 1.0), ("argument_offset", 0.0)))
    if "axis" in force:
        app = "torque"
        ap = _direction_params(model, force, force["axis"])
    else:
        u1 = local_point(model, fb[0], force.get("point1"))
        u2 = local_point(model, fb[1], force.get("point2"))
        direction = force.get("direction", "line_of_sight")
        if direction == "line_of_sight":
            app, ap = "los", [*u1, *u2]
        else:
            app, ap = "dir", [*u1, *u2, *_direction_params(model, force, direction)]
    m = spec["type"]
    kind = f"curve_{m}_{app}_{'ground' if fb[1] == 'ground' else 'bb'}"
    n = len(mp)
    names = [fb[0]] + ([] if fb[1] == "ground" else [fb[1]])
    return LibraryElement(kind, names, [*mp, a, b, c, *tab[:, 0], *ap], [(n + 3, tab)])


def _direction_params(model, force, vec):
    if force.get("frame", "global") == "global":
        return [*local_axis(model, "ground", vec), 0.0, 0.0, 0.0]
    return [0.0, 0.0, 0.0, *local_axis(model, force["body1"], vec)]


def as_expression(force):
    """The expression force (or torque) with the same law, for the model plugin."""
    a, b, c = (force.get(k, d) for k, d in (("scale", 1.0), ("argument_scale", 1.0), ("argument_offset", 0.0)))
    law = f"{float(a)!r} * {force['curve']}({float(b)!r} * {force['measure']} + {float(c)!r})"
    f = {k: v for k, v in force.items() if k not in ("measure", "curve", "scale", "argument_scale", "argument_offset")}
    if "axis" in force:
        return dict(f, type="expression_torque", torque=law)
    return dict(f, type="expression_force", force=law)

