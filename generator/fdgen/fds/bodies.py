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


def legacy_file(value, base_dir):
    """A file named in a .fds model, often with an absolute Windows path of the machine it was
    made on: the path itself if it exists (also relative to the model directory), else its last
    components relative to the model directory (longest first), else its name next to the model."""
    from pathlib import Path, PureWindowsPath
    p = Path(value)
    if p.is_file():
        return str(p)
    if (Path(base_dir) / p).is_file():  # relative to the model
        return str(Path(base_dir) / p)
    parts = PureWindowsPath(value).parts
    for k in range(len(parts) - 1, 0, -1):
        cand = Path(base_dir).joinpath(*parts[-k:])
        if cand.is_file():
            return str(cand)
    raise ModelError(f"file {value} not found (also not relative to {base_dir})")


def flex_body(block, base_dir):
    """Body MBS_FlexBody_Full: Path2MtxFile, Is_Itf_active, Is_QuadraticTermOF_Itt_active,
    LocationCenterMass and OrientationPAI (the body frame of the modal data: origin and axes),
    InitialVelocityVector (of that origin), InitialOmegaVector, GlobalDampingFactor (modal damping
    ratios, the last for the remaining modes)."""
    name = block[2].get("BodyName") or block[2].get("Name") or f"body{block[2].get('BodyID', '')}"
    cols = numbers(block, "OrientationPAI", 9)
    A = [[cols[3 * j + i] for j in range(3)] for i in range(3)]
    damping = numbers(block, "GlobalDampingFactor", None, [0.0])
    return name, {
        "flex": {"mtx": legacy_file(block[2].get("Path2MtxFile", ""), base_dir),
                 "damping": damping[0] if len(damping) == 1 else damping,
                 "rotation_coupling": flag(block, "Is_Itf_active", False),
                 "quadratic_inertia": flag(block, "Is_QuadraticTermOF_Itt_active", False)},
        "position": numbers(block, "LocationCenterMass", 3),
        "orientation": euler_parameters(orthonormalize_columns(A)),
        "velocity": numbers(block, "InitialVelocityVector", 3, [0.0, 0.0, 0.0]),
        "angular_velocity": numbers(block, "InitialOmegaVector", 3, [0.0, 0.0, 0.0]),
    }


def flex_fixed_joint(block, body_ids, bodies):
    """Constraint FixRiBoFleBo: a fixed joint between a rigid body (BodyIDs first) and an
    InterfaceNode of a flexible body (second), at Location, with the global axes as joint frame
    (MBS_RiBo_FleBo_FixJoint)."""
    j = joint(dict_block(block, ("ConstraintOrientation",)), body_ids, bodies)
    if "flex" not in bodies[j["body2"]]:
        raise ModelError(f"Constraint FixRiBoFleBo {j['name']}: the second body must be flexible")
    j["fix"] = ["dx", "dy", "dz", "ax", "ay", "az"]
    j["node"] = int(numbers(block, "InterfaceNode", 1)[0])
    return j


def dict_block(block, drop):
    return (block[0], block[1], {k: v for k, v in block[2].items() if k not in drop})
