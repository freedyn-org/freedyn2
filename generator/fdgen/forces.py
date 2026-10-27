"""Force element types of the library: parametric force laws, no model-specific code.

Forces own no coordinates; like joints they act on the coordinates and velocity dofs of the
connected bodies (body 1, and body 2 or the ground). Each type exists for Lie group bodies
(default) and Euler-parameter bodies (suffix _euler, legacy profile). Points u are body
coordinates, ground points r global; axes b are body-fixed, ground axes g global.

  spring_damper_ground / _bb   line-of-sight spring-damper between P1 = body1(u1) and P2:
                               s = k (l - l0) + d ldot + f0 (tension > 0), force s e on body 1
                               at P1 and -s e on body 2 at P2, e = (P2 - P1) / l.
                               p = [u1 (3), r2 | u2 (3), k, d, l0, f0]
  force_global / force_body    force F (global, or body-fixed components) at u1 on body 1.
                               p = [u1 (3), F (3)]
  torque_global / torque_body  torque M (global, or body-fixed components) on body 1. p = [M (3)]
  torsion_ground / _bb         rotational spring-damper about the axis z2 = x2 x y2 fixed on
                               body 2 (or the ground): angle phi of the body-1 axis x1 in the
                               (x2, y2) plane, phi = atan2(x1.y2, x1.x2) (|phi| < pi), phidot its
                               exact rate; tau = -k (phi - phi0) - d phidot + tau0 is the
                               generalized force conjugate to phi (virtual power tau phidot):
                               conservative in any configuration, the torque tau z2 on body 1
                               and -tau z2 on body 2 when x1 is perpendicular to z2 (revolute joint).
                               p = [x1 (3), x2 (3), y2 (3), k, d, phi0, tau0]

  bushing_ground / _bb         linear 6-dof spring-damper between a frame K1 carried by body 1 and
                               K2 carried by body 2 (or the ground), coincident at t = 0:
                               coordinates d = K2^T (P1 - P2) and psi = axial vector of the skew
                               part of K2^T K1 (sin(angle) * axis, a small-angle measure);
                               generalized forces conjugate to them (virtual power), so the
                               stiffness part is conservative and momentum is conserved:
                               s = -(kt d + dt ddot), m = -(kr psi + dr psidot), diagonal in K2.
                               p = [u1 (3), K1 (9, columns), u2 | r2 (3), K2 (9), kt (3), dt (3),
                               kr (3), dr (3)]

  tire                         tire on the road surface (fdgen.tire: Magic Formula, point contact on
                               an analytic road, friction patch). p = fdgen.tire.parameters(...)

  sensor                       no force: the motion of a point u1 of body 1 in a frame K (columns,
                               body coordinates) carried by body 1, for motion cueing and
                               acceleration sensors. p = [u1 (3), K (9, columns), g (3)]

Outputs: sensor: position, velocity, acceleration, specific_force (a - g, what an
accelerometer measures), angular_velocity, angular_acceleration, all in K (position global);
spring_damper: length, velocity (ldot), force (s); torsion: angle, rate, torque (tau);
bushing: displacement (d), rotation (psi), force (s, in K2), torque (m, in K2).
"""

import casadi as ca
import numpy as np

from .errors import ModelError
from .geometry import body_frame, rotation_matrix
from .kinematics import Connector, Ground
from .lagrange import ElementFunctions
from . import tire as tire_model

# kind: (number of bodies, number of parameters)
KINDS = {
    "spring_damper_ground": (1, 10),
    "spring_damper_bb": (2, 10),
    "force_global": (1, 6),
    "force_body": (1, 6),
    "torque_global": (1, 3),
    "torque_body": (1, 3),
    "torsion_ground": (1, 13),
    "torsion_bb": (2, 13),
    "bushing_ground": (1, 36),
    "bushing_bb": (2, 36),
    "tire": (1, tire_model.N_P),
    "tire_local": (1, tire_model.N_P_LOCAL),
    "sensor": (1, 15),
}
# Offset of the runtime road (fdgen.tire.LOCAL_KEYS) in the parameters of the *_local tires.
LOCAL_ROAD_OFFSET = {"tire_local": len(tire_model.TIRE_KEYS)}


def _other(c):
    return c.bodies[1] if c.n == 2 else Ground()


def _spring_damper(c):
    p, b1, b2 = c.p, c.bodies[0], _other(c)
    u1, x2 = p[0:3], p[3:6]
    k, d, l0, f0 = p[6], p[7], p[8], p[9]
    dr = b2.point(x2) - b1.point(u1)
    l = ca.norm_2(dr)
    e = dr / l
    ldot = ca.dot(e, b2.point_velocity(x2) - b1.point_velocity(u1))
    s = k * (l - l0) + d * ldot + f0
    ld = [(0, s * e, u1, None)]
    if c.n == 2:
        ld.append((1, -s * e, x2, None))
    return ld, {"length": l, "velocity": ldot, "force": s}


def _force(c, frame):
    p = c.p
    F = p[3:6] if frame == "global" else c.bodies[0].axis(p[3:6])
    return [(0, F, p[0:3], None)], {"force": F}


def _torque(c, frame):
    p = c.p
    M = p[0:3] if frame == "global" else c.bodies[0].axis(p[0:3])
    return [(0, None, None, M)], {"torque": M}


def _torsion(c):
    p, b1, b2 = c.p, c.bodies[0], _other(c)
    x1, x2, y2 = b1.axis(p[0:3]), b2.axis(p[3:6]), b2.axis(p[6:9])
    k, d, phi0, tau0 = p[9], p[10], p[11], p[12]
    a, b = ca.dot(x1, y2), ca.dot(x1, x2)
    x1d, x2d, y2d = (ca.cross(w, x) for w, x in ((b1.omega, x1), (b2.omega, x2), (b2.omega, y2)))
    ad, bd = ca.dot(x1d, y2) + ca.dot(x1, y2d), ca.dot(x1d, x2) + ca.dot(x1, x2d)
    phi = ca.atan2(a, b)
    phidot = (b * ad - a * bd) / (a * a + b * b)
    tau = -k * (phi - phi0) - d * phidot + tau0
    return [(None, tau, phidot)], {"angle": phi, "rate": phidot, "torque": tau}


def _bushing(c):
    p, b1, b2 = c.p, c.bodies[0], _other(c)
    u1, K1, x2, K2 = p[0:3], ca.reshape(p[3:12], 3, 3), p[12:15], ca.reshape(p[15:24], 3, 3)
    kt, dt, kr, dr = p[24:27], p[27:30], p[30:33], p[33:36]
    F1 = ca.horzcat(*[b1.axis(K1[:, j]) for j in range(3)])
    F2 = ca.horzcat(*[b2.axis(K2[:, j]) for j in range(3)])
    d = ca.mtimes(F2.T, b1.point(u1) - b2.point(x2))
    Rr = ca.mtimes(F2.T, F1)
    psi = 0.5 * ca.vertcat(Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1])
    dd, psid = c.rate(d), c.rate(psi)
    s = -(kt * d + dt * dd)
    m = -(kr * psi + dr * psid)
    ld = [(None, s[i], dd[i]) for i in range(3)] + [(None, m[i], psid[i]) for i in range(3)]
    return ld, {"displacement": d, "rotation": psi, "force": s, "torque": m}


def _sensor(c):
    p, b = c.p, c.bodies[0]
    u, K, g = p[0:3], ca.reshape(p[3:12], 3, 3), p[12:15]
    S = ca.mtimes(b.A, K)  # global axes of the sensor frame
    a = b.point_acceleration(u)
    local = lambda x: ca.mtimes(S.T, x)
    return [], {"position": b.point(u), "velocity": local(b.point_velocity(u)), "acceleration": local(a),
                "specific_force": local(a - g), "angular_velocity": local(b.omega),
                "angular_acceleration": local(b.omegadot)}


_LAWS = {
    "spring_damper_ground": _spring_damper, "spring_damper_bb": _spring_damper,
    "force_global": lambda c: _force(c, "global"), "force_body": lambda c: _force(c, "body"),
    "torque_global": lambda c: _torque(c, "global"), "torque_body": lambda c: _torque(c, "body"),
    "torsion_ground": _torsion, "torsion_bb": _torsion,
    "bushing_ground": _bushing, "bushing_bb": _bushing,
    "tire": tire_model.loads,
    "tire_local": lambda c: tire_model.loads(c, tire_model.LocalRoad),
    "sensor": _sensor,
}


def loads(kind, c):
    """Loads and named outputs {name: expr} of a force element. Loads are (body, F, u, M) or
    (None, s, rate): generalized force s conjugate to a scalar measure with this rate."""
    if kind not in _LAWS:
        raise ValueError(kind)
    return _LAWS[kind](c)


def force_element_type(name, n_bodies, n_p, law, rep="lie") -> ElementFunctions:
    """Element functions of a force element type: law(connector) -> (loads, {output: expr});
    rep "lie" or "euler" (the name gets _euler). Also for the element types of extensions."""
    name = name if rep == "lie" else name + "_euler"
    c = Connector(rep, n_bodies, n_p)
    ld, out = law(c)
    funs = c.functions(name, Q=c.generalized_force(ld))
    funs["out"] = c.output_function(name, out)
    return ElementFunctions(name, **funs)


def _build(kind, rep):
    return force_element_type(kind, *KINDS[kind], lambda c: loads(kind, c), rep)


def build(kind) -> ElementFunctions:
    """Lie group version (acts on rigid_body elements)."""
    return _build(kind, "lie")


def build_euler(kind) -> ElementFunctions:
    """Euler-parameter version (acts on rigid_body_euler elements, legacy profile)."""
    return _build(kind, "euler")


def _tire(force, b1, b2, names):
    tire = dict(force.get("tire", {}))
    road = force.get("road", {})
    local = "crg" in road  # OpenCRG road: evaluated by the runtime, the tire gets a local plane
    suffix = "_local" if local else ""
    try:
        if "radius" in force:
            tire["r0"] = force["radius"]
        return "tire" + suffix, names, tire_model.parameters(tire, road, local)
    except ValueError as e:
        raise ModelError(f"force {force['name']}: {e}") from None


def _spring_damper_element(force, b1, b2, names):
    (R1, A1), ground = b1, b2 is None
    r1 = np.array(force.get("point1", R1), dtype=float)
    r2 = np.array(force.get("point2", [0.0, 0.0, 0.0] if ground else b2[0]), dtype=float)
    l0 = force.get("length", float(np.linalg.norm(r2 - r1)))
    if l0 <= 0:
        raise ModelError(f"force {force['name']}: points coincide, give length > 0")
    x2 = r2 if ground else b2[1].T @ (r2 - b2[0])
    p = [*(A1.T @ (r1 - R1)), *x2, force["stiffness"], force.get("damping", 0.0), l0, force.get("preload", 0.0)]
    return "spring_damper_" + ("ground" if ground else "bb"), names, p


def _force_element(force, b1, b2, names):
    R1, A1 = b1
    u = A1.T @ (np.array(force.get("point", R1), dtype=float) - R1)
    return "force_" + force.get("frame", "global"), names, [*u, *force["vector"]]


def _torque_element(force, b1, b2, names):
    return "torque_" + force.get("frame", "global"), names, list(force["vector"])


def _joint_frame(force):
    o = np.array(force.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=float)
    return rotation_matrix(o / np.linalg.norm(o))


def _bushing_element(force, b1, b2, names):
    (R1, A1), ground = b1, b2 is None
    K = _joint_frame(force)
    r = np.array(force.get("location", R1), dtype=float)
    vec3 = lambda key: list(np.broadcast_to(np.asarray(force.get(key, 0.0), dtype=float), 3))
    K1 = A1.T @ K
    if ground:
        x2, K2 = r, K
    else:
        x2, K2 = b2[1].T @ (r - b2[0]), b2[1].T @ K
    p = [*(A1.T @ (r - R1)), *K1.T.ravel(), *x2, *K2.T.ravel(), *vec3("stiffness"), *vec3("damping"),
         *vec3("rot_stiffness"), *vec3("rot_damping")]
    return "bushing_" + ("ground" if ground else "bb"), names, p


def _torsion_element(force, b1, b2, names):
    A1, ground = b1[1], b2 is None
    K = _joint_frame(force)
    x2, y2 = (K[:, 0], K[:, 1]) if ground else (b2[1].T @ K[:, 0], b2[1].T @ K[:, 1])
    p = [*(A1.T @ K[:, 0]), *x2, *y2, force["stiffness"], force.get("damping", 0.0), force.get("angle", 0.0),
         force.get("preload", 0.0)]
    return "torsion_" + ("ground" if ground else "bb"), names, p


_ELEMENTS = {"tire": _tire, "spring_damper": _spring_damper_element, "force": _force_element,
             "torque": _torque_element, "bushing": _bushing_element, "torsion_spring": _torsion_element}


def element(force, bodies):
    """Library element of a validated YAML force: (kind, [bodies], p) from the geometry at t = 0."""
    ground = force["body2"] == "ground"
    names = [force["body1"]] + ([] if ground else [force["body2"]])
    b1 = body_frame(bodies[force["body1"]])
    b2 = None if ground else body_frame(bodies[force["body2"]])
    return _ELEMENTS[force["type"]](force, b1, b2, names)


def sensor_element(sensor, bodies, gravity):
    """Library element of a validated sensor: (kind, [body], p) from the geometry at t = 0."""
    R1, A1 = body_frame(bodies[sensor["body1"]])
    u = A1.T @ (np.array(sensor.get("point", R1), dtype=float) - R1)
    K = A1.T @ rotation_matrix(np.array(sensor["orientation"]))
    return "sensor", [sensor["body1"]], [*u, *K.T.ravel(), *gravity]
