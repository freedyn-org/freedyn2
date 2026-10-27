"""Joint constraint element types: the primitive constraints of the legacy "Constraint Generic".

A joint is a set of scalar constraints between a body and the ground or between two bodies,
defined in a joint frame K (axes k_0, k_1, k_2, global at t = 0) at a point r (global at t = 0).
Translation along k_i fixed and rotation about k_i fixed give these primitive element types
(as MBS_RiBo_Ground_TransFix, MBS_RiBo_Ground_RotFix, MBS_RiBo_RiBo_TransFix, MBS_RiBo_RiBo_RotFix):

  transfix_ground  Phi = n . (R + A u) - c                  p = [n (3), u (3), c]
  rotfix_ground    Phi = (A b) . g - c                      p = [b (3), g (3), c]
  transfix_bb      Phi = (R2 + A2 u2 - R1) . (A1 n1) - c    p = [n1 (3), u1 (3), u2 (3), c]
  rotfix_bb        Phi = (A1 b1) . (A2 b2) - c              p = [b1 (3), b2 (3), c]

n, g: global vectors; b, b1, b2, n1: body-fixed axes; u, u1, u2: the joint point in body
coordinates. Rotation about k_i fixed means axes k_j and k_l stay perpendicular, with
(j, l) = (1, 2), (0, 2), (0, 1) for i = 0, 1, 2 (j on the body / body 1, l on the ground / body 2).
The constant c includes the initial value of the expression, so Phi(q_0) = 0 (as legacy).

Each type exists for both body representations: Lie group bodies (rigid_body, default) and
Euler-parameter bodies (rigid_body_euler, legacy profile). Joints own no coordinates; their local
coordinates and velocity dofs are those of the connected bodies.
"""

import casadi as ca
import numpy as np

from .geometry import body_frame
from .kinematics import Connector
from .lagrange import ElementFunctions

KINDS = {
    "transfix_ground": (1, 7),
    "rotfix_ground": (1, 7),
    "transfix_bb": (2, 10),
    "rotfix_bb": (2, 7),
}
# Axis pairs of the rotation fixes: (axis on body / body 1, axis on ground / body 2).
ROT_AXES = {0: (1, 2), 1: (0, 2), 2: (0, 1)}

# Fixed directions of the joint presets, in the joint frame (joint axis = z).
PRESETS = {
    "fixed": ["dx", "dy", "dz", "ax", "ay", "az"],
    "spherical": ["dx", "dy", "dz"],
    "revolute": ["dx", "dy", "dz", "ax", "ay"],
    "cylindrical": ["dx", "dy", "ax", "ay"],
    "prismatic": ["dx", "dy", "ax", "ay", "az"],
}


def constraint(kind, bodies, p):
    """Constraint expression of a primitive joint between the kinematics.Body objects bodies."""
    R1, A1 = bodies[0].R, bodies[0].A
    if kind == "transfix_ground":
        return ca.dot(p[0:3], R1 + ca.mtimes(A1, p[3:6])) - p[6]
    if kind == "rotfix_ground":
        return ca.dot(ca.mtimes(A1, p[0:3]), p[3:6]) - p[6]
    R2, A2 = bodies[1].R, bodies[1].A
    if kind == "transfix_bb":
        return ca.dot(R2 + ca.mtimes(A2, p[6:9]) - R1, ca.mtimes(A1, p[0:3])) - p[9]
    if kind == "rotfix_bb":
        return ca.dot(ca.mtimes(A1, p[0:3]), ca.mtimes(A2, p[3:6])) - p[6]
    raise ValueError(kind)


def build(kind) -> ElementFunctions:
    """Lie group version (connects rigid_body elements)."""
    c = Connector("lie", *KINDS[kind])
    return ElementFunctions(kind, **c.functions(kind, Phi=constraint(kind, c.bodies, c.p)))


def build_euler(kind) -> ElementFunctions:
    """Euler-parameter version (connects rigid_body_euler elements, legacy profile)."""
    name = kind + "_euler"
    c = Connector("euler", *KINDS[kind])
    return ElementFunctions(name, **c.functions(name, Phi=constraint(kind, c.bodies, c.p)))


def primitives(joint, bodies):
    """Primitive constraints of a joint as (kind, suffix, [bodies], p), in the legacy order
    (translation i, then rotation i, for i = x, y, z). The constants c make Phi(q_0) = 0."""
    K = np.array(joint["frame"])  # columns: joint axes (global)
    r = np.array(joint["location"], dtype=float)
    R1, A1 = body_frame(bodies[joint["body1"]])
    ground = joint["body2"] == "ground"
    if not ground:
        R2, A2 = body_frame(bodies[joint["body2"]])
    out = []
    for i, axis in enumerate("xyz"):
        if "d" + axis in joint["fix"]:
            if ground:
                n, u = K[:, i], A1.T @ (r - R1)
                out.append(("transfix_ground", "d" + axis, [joint["body1"]], [*n, *u, n @ (R1 + A1 @ u)]))
            else:
                n1, u1, u2 = A1.T @ K[:, i], A1.T @ (r - R1), A2.T @ (r - R2)
                c = (R2 + A2 @ u2 - R1) @ (A1 @ n1)
                out.append(("transfix_bb", "d" + axis, [joint["body1"], joint["body2"]], [*n1, *u1, *u2, c]))
        if "a" + axis in joint["fix"]:
            j, l = ROT_AXES[i]
            if ground:
                b, g = A1.T @ K[:, j], K[:, l]
                out.append(("rotfix_ground", "a" + axis, [joint["body1"]], [*b, *g, (A1 @ b) @ g]))
            else:
                bb1, bb2 = A1.T @ K[:, j], A2.T @ K[:, l]
                out.append(("rotfix_bb", "a" + axis, [joint["body1"], joint["body2"]],
                            [*bb1, *bb2, (A1 @ bb1) @ (A2 @ bb2)]))
    return out
