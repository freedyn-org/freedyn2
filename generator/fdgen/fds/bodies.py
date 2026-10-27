"""Body Rigid_EulerParameter_PAI and Constraint Generic blocks."""

import math

from ..errors import ModelError
from .blocks import flag, numbers


def orthonormalize_columns(A):
    """MBS_Math::performOrthonormalization: Gram-Schmidt on the columns of A (3x3, row lists)."""
    w = [[A[i][j] for i in range(3)] for j in range(3)]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    norm = lambda a: math.sqrt(dot(a, a))
    v1 = [x / norm(w[0]) for x in w[0]]
    v2 = [w[1][i] - dot(v1, w[1]) * v1[i] for i in range(3)]
    v2 = [x / norm(v2) for x in v2]
    v3 = [w[2][i] - dot(v1, w[2]) * v1[i] - dot(v2, w[2]) * v2[i] for i in range(3)]
    v3 = [x / norm(v3) for x in v3]
    return [[v1[i], v2[i], v3[i]] for i in range(3)]


def euler_parameters(R):
    """MBS_EulerParameterFunctions::getEulerParameterFromRotationTensor (Diebel 2006, Eq. 145)."""
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]
    if r11 >= -r22 and r00 >= -r11 and r00 >= -r22:
        d = math.sqrt(1.0 + r00 + r11 + r22)
        return [0.5 * d, 0.5 * (-r12 + r21) / d, 0.5 * (-r20 + r02) / d, 0.5 * (-r01 + r10) / d]
    if r11 <= -r22 and r00 >= r11 and r00 >= r22:
        d = math.sqrt(1.0 + r00 - r11 - r22)
        return [0.5 * (-r12 + r21) / d, 0.5 * d, 0.5 * (r01 + r10) / d, 0.5 * (r20 + r02) / d]
    if r11 >= r22 and r00 <= r11 and r00 <= -r22:
        d = math.sqrt(1.0 - r00 + r11 - r22)
        return [0.5 * (-r20 + r02) / d, 0.5 * (r01 + r10) / d, 0.5 * d, 0.5 * (r12 + r21) / d]
    if r11 <= r22 and r00 <= -r11 and r00 <= r22:
        d = math.sqrt(1.0 - r00 - r11 + r22)
        return [0.5 * (-r01 + r10) / d, 0.5 * (r20 + r02) / d, 0.5 * (r12 + r21) / d, 0.5 * d]
    raise ModelError("OrientationPAI is not a rotation matrix")


def body(block):
    name = block[2].get("BodyName") or block[2].get("Name") or f"body{block[2].get('BodyID', '')}"
    cols = numbers(block, "OrientationPAI", 9)
    A = [[cols[3 * j + i] for j in range(3)] for i in range(3)]  # values are the columns of A
    return name, {
        "mass": numbers(block, "Mass", 1)[0],
        "inertia": numbers(block, "Inertia", 3),
        "position": numbers(block, "LocationCenterMass", 3),
        "orientation": euler_parameters(orthonormalize_columns(A)),
        "velocity": numbers(block, "InitialVelocityVector", 3),
        "angular_velocity": numbers(block, "InitialOmegaVector", 3),
        "jacobian": "full" if flag(block, "RegardVelocityInertiaForcesInsideJacobian", False) else "no_qvv",
    }


def joint(block, body_ids, bodies):
    ids = [x.strip() for x in block[2].get("BodyIDs", "").split(",")]
    if len(ids) != 2:
        raise ModelError(f"Constraint {block[1]}: BodyIDs needs two entries")
    names = []
    for i in ids:
        if i == "0":
            names.append("ground")
        elif i in body_ids:
            names.append(body_ids[i])
        else:
            raise ModelError(f"Constraint {block[1]}: unknown body ID {i}")
    if names[0] == "ground":
        raise ModelError(f"Constraint {block[1]}: the first body must not be the ground")
    cols = numbers(block, "ConstraintOrientation", 9, [1, 0, 0, 0, 1, 0, 0, 0, 1])
    K = orthonormalize_columns([[cols[3 * j + i] for j in range(3)] for i in range(3)])
    fix = [k for k in ("dx", "dy", "dz", "ax", "ay", "az") if flag(block, k + "Fixed", False)]
    name = block[2].get("Name") or block[2].get("ConstraintID") or block[2].get("ID") or "joint"
    name = "".join(c if c.isalnum() or c == "_" else "_" for c in name)
    return {"name": name, "body1": names[0], "body2": names[1],
            "location": numbers(block, "Location", 3, [0.0, 0.0, 0.0]), "frame": K, "fix": fix}
