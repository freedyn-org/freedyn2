"""Importer for the legacy FreeDyn model format (.fds), for the subset FreeDyn 2.0 supports.

Modules: blocks (block parser, values), bodies (bodies, joints), loads (measures, expressions,
forces, splines, drivers), solver (simulation / solver blocks, profiles).

Supported blocks: Body Rigid_EulerParameter_PAI, Constraint Generic, Constraint TimeMotion,
Force Gravity, Force GenericForce / GenericTorque, Measure Translational / Rotational,
DataObject Parameter / Spline_1D, Simulation Dynamic, Solver HHT / HHT_Negrut.
Header and Globals blocks are ignored; any other block is rejected, so that a model is never
silently simulated with parts missing. Conventions follow the legacy kernel
(MBS_RigidBody_EulerP_PAI::setParameter):

- OrientationPAI lists the rotation matrix A (body to global) column by column, i.e. the
  principal axes in global components. It is orthonormalized (Gram-Schmidt on the columns)
  and converted to Euler parameters as in MBS_EulerParameterFunctions.
- InitialVelocityVector and InitialOmegaVector are global.
- Constraint Generic: BodyIDs = body1, body2 (ID or BodyID of the bodies, 0 = ground; body1 must
  be a body), Location (global), ConstraintOrientation (columns = joint axes, orthonormalized
  like OrientationPAI), dx/dy/dz/ax/ay/azFixed.
- Force Gravity: GravityAccVector; ActivationTime must be 0 (no ramp yet).
- Force GenericForce / GenericTorque: expression forces of the model plugin. ForceExpression /
  TorqueExpression (ExprTk syntax, case-insensitive: simtime[], meaval[m], parval[p],
  step[x, x0, y0, x1, y1, mode], ^) are translated to fdgen.expressions. GenericForce: force
  value * e on body 1 at PointOfApplication_Body1 and the opposite on body 2 at
  PointOfApplication_Body2, e from ForceDirectionType SpaceFixed / BodyFixed (ForceDirection,
  global at t = 0) or In-line-of-sight: body-body from point 1 to point 2
  (MBS_3DUnitVector_BodyBody), but body-ground from the ground point to the body point
  (MBS_3DUnitVector_GroundBody), so the expression is negated for FreeDyn 2.0's line_of_sight
  (always from point 1 to point 2). GenericTorque: Axis, AxisType.
- Measure Translational (MeasureMode disp / vel, MeasureCoordinate X / Y / Z / XYZ) with the
  legacy definitions (MBS_RefFrameMeasureRiBo_*, MBS_RiBoRiBoMeasure_*); see _measure.
- Measure Rotational (MeasureCoordinate Angle / AngleVel / LocalAngleVel): MeasureVec (body 1),
  MeasurePlaneVec1 / 2 (body 2 or the ground), global at t = 0. Angle: atan2(a.c, a.b),
  continuous (revolutions counted), UseInitialValueAsOffset; AngleVel: x ydot - y xdot of that
  angle (vectors normalized for the ground, as given for body-body); LocalAngleVel: component
  (Component = 0, 1, 2) of the body-frame angular velocity.
- Constraint TimeMotion: drivers. Measure (a Translational X/Y/Z or XYZ disp or a Rotational
  Angle of the constraint's bodies) = MotionExpression (time, parameters, splines only), with the
  activation polynomial for t < ActivationPeriod (default 0.1 s), as MBS_RiBo_Time_Motion.
- DataObject Parameter: Label, InitialValue.
- DataObject Spline_1D: Label, InterpolationType (akima default, zccspl), NumDataPts, XData,
  YData or PathToSplineData (relative to the model directory), NumberOfHeaderLines,
  ColumnXData, ColumnYData (from 0; as IO_TableDataReader); used as spl[x, label] / akispl[x, label] (both: the data object's type) /
  zccspl[x, label] (zero clamped cubic), see fdgen.splines.
- SolverMaxTimeStep defaults to 1e3 and SolverMinTimeStep to 1e-14, as in SOL_SolverParameter.

Profiles (how the model is solved; the model itself is the same):
- "legacy": the legacy algorithm (Euler-parameter bodies, Negrut HHT form, legacy Newton rule,
  step size and output time control, Jacobian and ModifyNewmark options). Used to compare with
  legacy FreeDyn to round-off.
- "v2": FreeDyn 2.0 defaults (Lie group bodies, Arnold-Bruls HHT form, velocity-level Newton
  criterion, full iteration matrix); the step size control keeps the model's GlobalErrorTolerance and
  SolverMaxTimeStep. Compared with legacy FreeDyn on the level of the physics, against an
  independent reference.
"""


from pathlib import Path

from ..errors import ModelError
from . import bodies as _bodies
from . import loads as _loads
from . import solver as _solver
from .blocks import IGNORED_BLOCKS, numbers, parse_blocks
from .bodies import euler_parameters, orthonormalize_columns
from .loads import legacy_expression
from .solver import PROFILES

__all__ = ["load_fds", "parse_blocks", "euler_parameters", "orthonormalize_columns", "legacy_expression",
           "PROFILES"]


def _unique_names(items):
    seen = set()
    for it in items:
        base, k = it["name"] or "element", 2
        it["name"] = base
        while it["name"] in seen:
            it["name"], k = f"{base}_{k}", k + 1
        seen.add(it["name"])


def load_fds(path, profile="legacy"):
    """Read a .fds model into the validated model dict used by write_fdm."""
    if profile not in PROFILES:
        raise ModelError(f"profile must be one of {PROFILES}")
    path = Path(path)
    bodies, body_ids, sim, solver = {}, {}, None, None
    joint_blocks, measure_blocks, force_blocks, motion_blocks = [], [], [], []
    parameters, spline_data, gravity = {}, {}, [0.0, 0.0, 0.0]
    for b in parse_blocks(path.read_text(encoding="latin-1")):
        kind = (b[0], b[1])
        if b[0] in IGNORED_BLOCKS:
            continue
        if kind == ("Body", "Rigid_EulerParameter_PAI"):
            name, body = _bodies.body(b)
            if name in bodies:
                raise ModelError(f"duplicate body name {name}")
            bodies[name] = body
            for key in ("ID", "BodyID"):
                if b[2].get(key):
                    body_ids[b[2][key]] = name
        elif kind == ("Constraint", "Generic"):
            joint_blocks.append(b)
        elif kind == ("Constraint", "TimeMotion"):
            motion_blocks.append(b)
        elif kind == ("Force", "Gravity"):
            if numbers(b, "ActivationTime", 1, [0.0])[0] != 0.0:
                raise ModelError("Force Gravity: ActivationTime > 0 is not supported yet")
            gravity = numbers(b, "GravityAccVector", 3)
        elif kind in (("Force", "GenericForce"), ("Force", "GenericTorque")):
            force_blocks.append(b)
        elif kind in (("Measure", "Translational"), ("Measure", "Rotational")):
            measure_blocks.append(b)
        elif kind == ("DataObject", "Spline_1D"):
            label, data = _loads.spline_data(b, path.parent)
            spline_data[label] = data
        elif kind == ("DataObject", "Parameter"):
            name, value = _loads.parameter(b)
            parameters[name] = value
        elif kind == ("Simulation", "Dynamic"):
            if sim is not None:
                raise ModelError("only one Simulation block is supported")
            sim = b
        elif b[0] == "Solver" and b[1] in ("HHT", "HHT_Negrut"):
            solver = b
        else:
            raise ModelError(f"block '{b[0]} {b[1]}' is not supported yet")
    if not bodies or sim is None or solver is None:
        raise ModelError("a model needs bodies, a Simulation Dynamic and a Solver HHT block")

    joints = [_bodies.joint(b, body_ids, bodies) for b in joint_blocks]
    measures = {}
    for b in measure_blocks:
        make = _loads.translational_measure if b[1] == "Translational" else _loads.rotational_measure
        measures[_loads.measure_name(b)] = make(b, body_ids)
    used_splines, drivers = {}, {}
    for b in motion_blocks:
        name, driver = _loads.time_motion(b, body_ids, measures, parameters, spline_data, used_splines)
        drivers[name] = driver
    forces = [_loads.expression_force(b, body_ids, _loads.short_names(measures, "m_"),
                                      _loads.short_names(parameters, "p_"), spline_data, used_splines)
              for b in force_blocks]
    _unique_names(forces)
    _unique_names(joints)

    s = _solver.settings(sim, solver)
    _solver.apply_profile(profile, s, bodies, solver)
    return {"model": path.stem, "bodies": bodies, "joints": joints, "forces": forces, "parameters": parameters,
            "measures": measures, "splines": used_splines, "drivers": drivers, "gravity": gravity, "solver": s}
