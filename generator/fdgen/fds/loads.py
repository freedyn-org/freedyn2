"""Measures, parameters, splines, expression forces / torques and TimeMotion drivers."""

from pathlib import Path

from ..errors import ModelError
from ..measures import POSITION_MEASURES
from .blocks import flag, ident, numbers


def body_pair(block, body_ids):
    """(body1, body2 or "ground") of the BodyIDs of a measure or force block (body, body or body, 0)."""
    ids = [x.strip() for x in block[2].get("BodyIDs", "").split(",")]
    if len(ids) != 2 or ids[0] == "0" or ids[0] not in body_ids or (ids[1] != "0" and ids[1] not in body_ids):
        raise ModelError(f"{block[0]} {block[1]}: BodyIDs must be body, body or body, 0")
    return body_ids[ids[0]], ("ground" if ids[1] == "0" else body_ids[ids[1]])


def translational_measure(block, body_ids):
    """v2 measure of a legacy Measure Translational block (legacy definitions):

    ground (BodyIDs = b, 0), points r1 (body), r2 (ground):
      disp X/Y/Z  (R + A u1 - r2)_c - offset, offset = initial value if UseInitialValueAsOffset
      vel  X/Y/Z  d/dt (R + A u1)_c
      disp XYZ    |R + A u1 - r2| - offset;  vel XYZ: its rate
    body-body (b1, b2):
      disp X/Y/Z  (A1^T (P2 - P1))_c - (r2 - r1)_c [UseInitialValueAsOffset: initial *global* distance]
      vel  X/Y/Z  (A1 e_c) . (V2 - V1)
      disp XYZ    |P1 - P2| - offset;  vel XYZ: d/dt |P1 - P2|
    """
    b1, b2 = body_pair(block, body_ids)
    mode = block[2].get("MeasureMode", "")
    coord = block[2].get("MeasureCoordinate", "")
    r1, r2 = numbers(block, "MeasureLocation_Body1", 3, [0.0] * 3), numbers(block, "MeasureLocation_Body2", 3, [0.0] * 3)
    offset = flag(block, "UseInitialValueAsOffset", False)
    if mode not in ("disp", "vel") or coord not in ("X", "Y", "Z", "XYZ"):
        raise ModelError(f"Measure Translational: MeasureMode {mode} / MeasureCoordinate {coord} not supported yet")
    if coord == "XYZ":
        m = {"type": "distance" if mode == "disp" else "distance_rate", "body1": b1, "body2": b2,
             "point1": r1, "point2": r2}
        if mode == "disp" and offset:
            m["offset"] = "initial"
        return m
    c = coord.lower()
    if b2 == "ground":
        m = {"type": "displacement" if mode == "disp" else "velocity", "body1": b1, "body2": "ground",
             "point1": r1, "point2": r2 if mode == "disp" else [0.0] * 3, "component": c}
        if mode == "disp" and offset:
            m["offset"] = "initial"
        return m
    # Body-body: relative to body 1 in its frame (body roles swapped for P2 - P1).
    m = {"type": "displacement" if mode == "disp" else "velocity", "body1": b2, "body2": b1,
         "point1": r2, "point2": r1, "component": c, "frame": b1}
    if mode == "disp" and offset:
        m["offset"] = r2["xyz".index(c)] - r1["xyz".index(c)]
    return m


def rotational_measure(block, body_ids):
    """v2 measure of a legacy Measure Rotational block (MBS_RefFrameMeasureRiBo_Angle / _AngleVel,
    MBS_SingleRiBo_LocalAngleVel, MBS_RiBoRiBoMeasure_Diff_Angle / _AngleVel)."""
    b1, b2 = body_pair(block, body_ids)
    coord = block[2].get("MeasureCoordinate", "")
    if coord == "LocalAngleVel":
        if b2 != "ground":
            raise ModelError("Measure Rotational LocalAngleVel: body-ground only")
        comp = int(numbers(block, "Component", 1)[0])
        return {"type": "angular_velocity", "body1": b1, "body2": "ground", "frame": b1, "component": "xyz"[comp]}
    if coord not in ("Angle", "AngleVel"):
        raise ModelError(f"Measure Rotational: MeasureCoordinate {coord!r} not supported")
    m = {"type": "angle" if coord == "Angle" else "angular_rate", "body1": b1, "body2": b2,
         "vector": numbers(block, "MeasureVec", 3),
         "plane": [numbers(block, "MeasurePlaneVec1", 3), numbers(block, "MeasurePlaneVec2", 3)],
         # the ground AngleVel normalizes its vectors, Angle and body-body AngleVel use them as given
         "normalize": coord == "AngleVel" and b2 == "ground"}
    if coord == "AngleVel":
        m["form"] = "legacy"
    elif flag(block, "UseInitialValueAsOffset", False):
        m["offset"] = "initial"
    return m


_STEP_MODES = ("hard", "polynomial", "cosine")


def legacy_expression(text, measures, parameters, splines=None, used_splines=None):
    """Translate a legacy force expression (lowercased like the legacy kernel).

    splines: legacy spline data objects {label: (x, y, default type)}; spl[x, label] and
    akispl[x, label] use the data object's InterpolationType (akispl is an alias of spl in
    MBS_FunctionExpression), zccspl forces the zero clamped cubic. Each (label, type) used
    becomes a v2 spline in used_splines."""
    import re

    s = text.lower()
    lookup = lambda table, kind: lambda m: table.get(m.group(1).strip()) or _unknown(kind, m.group(1), text)
    s = re.sub(r"simtime\[\s*\]", "t", s)
    s = re.sub(r"meaval\[([^\]]*)\]", lookup(measures, "measure"), s)
    s = re.sub(r"parval\[([^\]]*)\]", lookup(parameters, "parameter"), s)

    def step(m):
        args = [a.strip() for a in m.group(1).split(",")]
        if len(args) not in (5, 6):
            raise ModelError(f"step needs 5 or 6 arguments in {text!r}")
        mode = args[5] if len(args) == 6 else "hard"
        if mode not in _STEP_MODES:
            mode = "hard"  # FL_Step: any other mode is a hard step
        return "step(" + ", ".join(args[:5]) + f', "{mode}")'

    s = re.sub(r"\bstep\[([^\[\]]*)\]", step, s)

    def spline(m):
        fn, args = m.group(1), [a.strip() for a in m.group(2).split(",")]
        if len(args) != 2:
            raise ModelError(f"{fn} needs 2 arguments (variable, spline label) in {text!r}")
        label = args[1]
        if not splines or label not in splines:
            raise ModelError(f"unknown spline data {label!r} in {text!r}")
        x, y, default = splines[label]
        kind = {"spl": default, "akispl": default, "zccspl": "zcc"}[fn]
        name = ident("s_", label) + "_" + kind
        if used_splines is not None:
            used_splines[name] = {"x": x, "y": y, "type": kind}
        return f"{name}({args[0]})"

    s = re.sub(r"\b(akispl|zccspl|spl)\[([^\[\]]*)\]", spline, s)
    if "[" in s:
        raise ModelError(f"unsupported function in expression {text!r}")
    return s


def _unknown(kind, name, text):
    raise ModelError(f"unknown {kind} {name!r} in {text!r}")


def expression_force(block, body_ids, measures, parameters, splines=None, used_splines=None):
    b1, b2 = body_pair(block, body_ids)
    f = {"body1": b1, "body2": b2}
    label = block[2].get("Label") or block[2].get("ForceName") or block[2].get("Name") or ("force" + block[2].get("ForceID", ""))
    f["name"] = ident("", label)
    if block[1] == "GenericForce":
        f["type"] = "expression_force"
        f["force"] = legacy_expression(block[2].get("ForceExpression", ""), measures, parameters, splines, used_splines)
        f["point1"] = numbers(block, "PointOfApplication_Body1", 3, [0.0] * 3)
        f["point2"] = numbers(block, "PointOfApplication_Body2", 3, [0.0] * 3)
        dtype = block[2].get("ForceDirectionType", "")
        if dtype == "In-line-of-sight":
            f["direction"] = "line_of_sight"
            if f["body2"] == "ground":  # legacy: from the ground point to the body point
                f["force"] = f"-({f['force']})"
        elif dtype in ("SpaceFixed", "BodyFixed"):
            f["direction"] = numbers(block, "ForceDirection", 3)
            f["frame"] = "global" if dtype == "SpaceFixed" else "body"
        else:
            raise ModelError(f"Force GenericForce: ForceDirectionType {dtype!r} not valid")
    else:
        f["type"] = "expression_torque"
        f["torque"] = legacy_expression(block[2].get("TorqueExpression", ""), measures, parameters, splines, used_splines)
        f["axis"] = numbers(block, "Axis", 3)
        atype = block[2].get("AxisType", "")
        if atype not in ("SpaceFixed", "BodyFixed"):
            raise ModelError(f"Force GenericTorque: AxisType {atype!r} not valid")
        f["frame"] = "global" if atype == "SpaceFixed" else "body"
    return f


def parameter(block):
    """(name, value) of a DataObject Parameter block."""
    label = block[2].get("Label") or block[2].get("Name")
    if not label:
        raise ModelError("DataObject Parameter without Label")
    return ident("p_", label), numbers(block, "InitialValue", 1)[0]


def spline_data(block, model_dir):
    """(label, (x, y, default type)) of a DataObject Spline_1D block (data inline or in a file
    relative to the model directory)."""
    from ..splines import read_table

    b = block
    label = b[2].get("Label") or b[2].get("Name")
    if not label:
        raise ModelError("DataObject Spline_1D without Label")
    if "NumDataPts" in b[2]:
        n = int(numbers(b, "NumDataPts", 1)[0])
        xy = (numbers(b, "XData", n), numbers(b, "YData", n))
    elif "PathToSplineData" in b[2]:
        f = Path(b[2]["PathToSplineData"].replace("\\", "/"))
        f = f if f.is_absolute() else Path(model_dir) / f
        try:
            xy = read_table(f, int(numbers(b, "NumberOfHeaderLines", 1)[0]),
                            int(numbers(b, "ColumnXData", 1)[0]), int(numbers(b, "ColumnYData", 1)[0]))
        except (OSError, ValueError) as e:
            raise ModelError(f"DataObject Spline_1D {label}: {e}") from None
    else:
        raise ModelError("DataObject Spline_1D: either NumDataPts or PathToSplineData")
    interp = b[2].get("InterpolationType", "akima").lower()
    if interp not in ("akima", "akispl", "zccspl", "zcc"):
        raise ModelError(f"DataObject Spline_1D: unknown InterpolationType {interp!r}")
    return label.lower(), (*xy, "akima" if interp in ("akima", "akispl") else "zcc")


def measure_name(block):
    label = block[2].get("MeasureName") or block[2].get("Name") or block[2].get("Label") or ("m" + block[2].get("MeasureID", ""))
    return ident("m_", label)


def time_motion(block, body_ids, measures, parameters, splines, used_splines):
    """(name, driver) of a Constraint TimeMotion block (MBS_RiBo_Time_Motion)."""
    b = block
    label = b[2].get("Name") or b[2].get("ConstraintName") or b[2].get("Label") or ("motion" + b[2].get("ConstraintID", ""))
    mname = ident("m_", b[2].get("Measure", ""))
    if mname not in measures:
        raise ModelError(f"Constraint TimeMotion: unknown Measure {b[2].get('Measure')!r}")
    ids = [x.strip() for x in b[2].get("BodyIDs", "").split(",")]
    mb = [measures[mname]["body1"], measures[mname].get("body2", "ground")]
    want = [body_ids.get(ids[0]), "ground" if len(ids) > 1 and ids[1] == "0" else body_ids.get(ids[1] if len(ids) > 1 else "")]
    if sorted(mb) != sorted(want):
        raise ModelError("Constraint TimeMotion: the Measure must use the constraint's BodyIDs")
    if measures[mname]["type"] not in POSITION_MEASURES:
        raise ModelError("Constraint TimeMotion: Measure must be a displacement, distance or angle")
    return ident("", label), {
        "measure": mname,
        "motion": legacy_expression(b[2].get("MotionExpression", ""), short_names(measures, "m_"),
                                    short_names(parameters, "p_"), splines, used_splines),
        "ramp": numbers(b, "ActivationPeriod", 1, [0.1])[0],  # legacy default 0.1 s
    }


def short_names(table, prefix):
    """Legacy labels (without the prefix) -> FreeDyn 2.0 names."""
    return {k[len(prefix):]: k for k in table}
