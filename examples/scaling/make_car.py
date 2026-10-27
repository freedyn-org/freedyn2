"""Fictitious car on a chassis dynamometer as a legacy FreeDyn .fds, for scaling tests.

  python examples/scaling/make_car.py [--cars N] [--end-time T] [-o car_N.fds]

One car has 13 rigid bodies, 12 joints (60 constraint equations), 16 expression forces and
torques and 30 measures (x forward, z up, gravity -z):

  chassis        sprung mass, free in space (held only by the suspension)
  4 carriers     wheel carrier + unsprung mass, prismatic joint (vertical) to the chassis;
                 suspension: vertical spring-damper with preload carrier - chassis
  4 wheels       revolute joint (axis y) to the carrier; tire: vertical spring-damper wheel
                 center - ground on a road profile (bumps under the left and right wheels,
                 rear wheels later), no longitudinal tire forces (the car stands on rollers)
  driveline      engine (crankshaft, revolute about x to the chassis, torque ramped up from
                 0.5 s), propeller shaft (revolute about x), gearbox as a torsional spring with
                 ratio ig between engine and propeller shaft, open differential with ratio id
                 as a torsional spring between the propeller shaft and the mean of the two
                 half shafts (revolute about y), half shafts - rear wheels torsional springs,
                 roller load torque on the rear wheels proportional to their speed.
                 All couplings with a ratio are written as two torques derived from one
                 potential, so the reactions go into the chassis (it rolls and pitches).

With N cars the copies stand side by side (3 m apart) and do not interact: the model size grows
linearly, the work per car stays the same.
"""

import argparse
from pathlib import Path

G = 9.81
M_CHASSIS, M_CARRIER, M_WHEEL = 1200.0, 30.0, 15.0
M_ENGINE, M_PROP, M_HALF = 20.0, 10.0, 5.0
R_WHEEL, TRACK, WHEELBASE, CM_HEIGHT = 0.3, 1.6, 2.6, 0.55
SPRUNG = M_CHASSIS + M_ENGINE + M_PROP + 2 * M_HALF
PARAMETERS = {
    "k_susp": 30000.0, "c_susp": 2500.0, "f_susp": SPRUNG * G / 4,          # suspension
    "k_tire": 250000.0, "c_tire": 300.0, "f_tire": (SPRUNG / 4 + M_CARRIER + M_WHEEL) * G,
    "t_engine": 100.0, "c_engine": 0.05,                                     # engine
    "i_gear": 3.0, "k_gear": 2000.0, "c_gear": 2.0,                              # gearbox
    "i_diff": 4.0, "k_diff": 8000.0, "c_diff": 8.0,                              # differential
    "k_shaft": 3000.0, "c_shaft": 3.0, "c_roller": 30.0,                     # half shafts, rollers
}
ROAD = {"bump": 0.03, "t_bump": 1.5, "t_rear": 0.26}  # bump height, time under the front / rear wheels
WHEELS = {"fl": (WHEELBASE, TRACK / 2), "fr": (WHEELBASE, -TRACK / 2),
          "rl": (0.0, TRACK / 2), "rr": (0.0, -TRACK / 2)}
EX, EY, EZ = "1.0,0.0,0.0", "0.0,1.0,0.0", "0.0,0.0,1.0"
IDENTITY = "1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0"


def vec(v):
    return ",".join(repr(float(x)) for x in v)


class Fds:
    def __init__(self):
        self.blocks, self.next_id, self.body_ids = [], 1, {}

    def new_id(self):
        self.next_id += 1
        return self.next_id - 1

    def add(self, head, fields, end):
        self.blocks.append("\n".join([head] + [f"\t{k} = {v}" for k, v in fields] + [end]))

    def body(self, name, mass, inertia, pos):
        i = self.body_ids[name] = self.new_id()
        self.add("Body Rigid_EulerParameter_PAI", [
            ("BodyName", name), ("BodyID", i), ("InitialVelocityVector", "0.0,0.0,0.0"),
            ("InitialOmegaVector", "0.0,0.0,0.0"), ("RegardVelocityInertiaForcesInsideJacobian", "yes"),
            ("Mass", mass), ("LocationCenterMass", vec(pos)), ("Inertia", vec(inertia)),
            ("OrientationPAI", IDENTITY)], "EndBody")

    def joint(self, b1, b2, location, free):
        fields = [("ConstraintID", self.new_id()), ("BodyIDs", f"{self.body_ids[b1]}, {self.body_ids[b2]}")]
        fields += [(f"{d}Fixed", "no" if d == free else "yes") for d in ("dx", "dy", "dz", "ax", "ay", "az")]
        fields += [("Location", vec(location)), ("ConstraintOrientation", IDENTITY)]
        self.add("Constraint Generic", fields, "EndConstraint")

    def translational(self, name, b1, b2, p1, p2, mode, offset=False):
        bid2 = 0 if b2 == "ground" else self.body_ids[b2]
        fields = [("MeasureName", name), ("MeasureID", self.new_id()), ("BodyIDs", f"{self.body_ids[b1]},{bid2}"),
                  ("MeasureMode", mode), ("MeasureCoordinate", "Z"), ("MeasureLocation_Body1", vec(p1)),
                  ("MeasureLocation_Body2", vec(p2))]
        if mode == "disp":
            fields.append(("UseInitialValueAsOffset", "yes" if offset else "no"))
        self.add("Measure Translational", fields, "EndMeasure")

    def rotational(self, name, b1, b2, axis, coord):
        # angle of a body-1 vector in the body-2 plane normal to the axis (x: y, z; y: z, x)
        v1, v2 = (EY, EZ) if axis == "x" else (EZ, EX)
        fields = [("MeasureName", name), ("MeasureID", self.new_id()),
                  ("BodyIDs", f"{self.body_ids[b1]},{self.body_ids[b2]}"), ("MeasureCoordinate", coord),
                  ("MeasureVec", v1), ("MeasurePlaneVec1", v1), ("MeasurePlaneVec2", v2)]
        if coord == "Angle":
            fields.append(("UseInitialValueAsOffset", "no"))
        self.add("Measure Rotational", fields, "EndMeasure")

    def force(self, label, b1, b2, p1, p2, direction, frame, expr):
        bid2 = 0 if b2 == "ground" else self.body_ids[b2]
        self.add("Force GenericForce", [
            ("ForceID", self.new_id()), ("BodyIDs", f"{self.body_ids[b1]},{bid2}"), ("Label", label),
            ("PointOfApplication_Body1", vec(p1)), ("PointOfApplication_Body2", vec(p2)),
            ("ForceDirectionType", frame), ("ForceDirection", direction), ("ForceExpression", expr)], "EndForce")

    def torque(self, label, b1, b2, axis, expr):
        self.add("Force GenericTorque", [
            ("ForceID", self.new_id()), ("BodyIDs", f"{self.body_ids[b1]},{self.body_ids[b2]}"), ("Label", label),
            ("AxisType", "BodyFixed"), ("Axis", axis), ("TorqueExpression", expr)], "EndForce")

    def parameter(self, label, value):
        self.add("DataObject Parameter", [("Label", label), ("VisualizationID", self.new_id()),
                                          ("InitialValue", repr(value))], "EndDataObject")


def car(f, c, y0):
    """Bodies, joints, measures and forces of car number c at lateral offset y0."""
    n = lambda s: f"{s}{c}"
    P = lambda s: f"PARVAL[{s}]"
    M = lambda s: f"MEAVAL[{n(s)}]"
    chassis = n("chassis")
    f.body(chassis, M_CHASSIS, (450.0, 1800.0, 2000.0), (1.3, y0, CM_HEIGHT))
    for w, (x, y) in WHEELS.items():
        y += y0
        center, top = (x, y, R_WHEEL), (x, y, CM_HEIGHT)
        carrier, wheel = n("carrier_" + w), n("wheel_" + w)
        f.body(carrier, M_CARRIER, (0.5, 0.5, 0.5), center)
        f.body(wheel, M_WHEEL, (0.6, 1.0, 0.6), center)
        f.joint(carrier, chassis, center, "dz")
        f.joint(wheel, carrier, center, "ay")
        # suspension: carrier height relative to the chassis (compression > 0) and its rate
        f.translational(n("dz_" + w), chassis, carrier, top, center, "disp", offset=True)
        f.translational(n("vz_" + w), chassis, carrier, top, center, "vel")
        f.force(n("susp_" + w), carrier, chassis, center, top, EZ, "BodyFixed",
                f"-{P('f_susp')}-{P('k_susp')}*{M('dz_' + w)}-{P('c_susp')}*{M('vz_' + w)}")
        # tire on the road profile: a bump of height bump (left) and 2/3 bump (right), rear later
        f.translational(n("zt_" + w), wheel, "ground", center, (x, y, 0.0), "disp", offset=True)
        f.translational(n("vt_" + w), wheel, "ground", center, (x, y, 0.0), "vel")
        # (legacy STEP takes numbers only, so the road is written with its values)
        t0 = ROAD["t_bump"] + (ROAD["t_rear"] if w[0] == "r" else 0.0)
        h = ROAD["bump"] * (1.0 if w[1] == "l" else 2 / 3)
        road = (f"STEP[SIMTIME[],{t0:g},0,{t0 + 0.05:g},{h:g},polynomial]"
                f"-STEP[SIMTIME[],{t0 + 0.1:g},0,{t0 + 0.15:g},{h:g},polynomial]")
        f.force(n("tire_" + w), wheel, "ground", center, (x, y, 0.0), EZ, "SpaceFixed",
                f"{P('f_tire')}-{P('k_tire')}*({M('zt_' + w)}-({road}))-{P('c_tire')}*{M('vt_' + w)}")
    # driveline bodies on revolute joints to the chassis
    engine, prop = n("engine"), n("propshaft")
    f.body(engine, M_ENGINE, (0.25, 0.3, 0.3), (2.2, y0, 0.45))
    f.joint(engine, chassis, (2.2, y0, 0.45), "ax")
    f.body(prop, M_PROP, (0.02, 0.5, 0.5), (1.3, y0, 0.35))
    f.joint(prop, chassis, (1.3, y0, 0.35), "ax")
    halves = {}
    for side, y in (("l", 0.4), ("r", -0.4)):
        halves[side] = n("halfshaft_" + side)
        f.body(halves[side], M_HALF, (0.1, 0.05, 0.1), (0.0, y0 + y, R_WHEEL))
        f.joint(halves[side], chassis, (0.0, y0 + y, R_WHEEL), "ay")
    for name, body, axis in (("e", engine, "x"), ("p", prop, "x"), ("hl", halves["l"], "y"), ("hr", halves["r"], "y")):
        f.rotational(n("phi_" + name), body, chassis, axis, "Angle")
        f.rotational(n("w_" + name), body, chassis, axis, "AngleVel")
    # gearbox: V = k/2 (phi_e/ig - phi_p)^2 -> torque T on the propeller shaft, -T/ig on the engine
    gear = (f"({P('k_gear')}*({M('phi_e')}/{P('i_gear')}-{M('phi_p')})"
            f"+{P('c_gear')}*({M('w_e')}/{P('i_gear')}-{M('w_p')}))")
    # differential: V = k/2 (phi_p/id - (phi_hl + phi_hr)/2)^2 -> T/2 per half shaft, -T/id on the propeller shaft
    diff = (f"({P('k_diff')}*({M('phi_p')}/{P('i_diff')}-0.5*({M('phi_hl')}+{M('phi_hr')}))"
            f"+{P('c_diff')}*({M('w_p')}/{P('i_diff')}-0.5*({M('w_hl')}+{M('w_hr')})))")
    f.torque(n("engine_torque"), engine, chassis, EX,
             f"{P('t_engine')}*STEP[SIMTIME[],0.5,0,1.0,1,polynomial]-{P('c_engine')}*{M('w_e')}-{gear}/{P('i_gear')}")
    f.torque(n("gear_out"), prop, chassis, EX, f"{gear}-{diff}/{P('i_diff')}")
    for side in ("l", "r"):
        w = "r" + side
        f.torque(n("diff_" + side), halves[side], chassis, EY, f"0.5*{diff}")
        # half shaft - wheel torsional spring (wheel angle relative to the half shaft)
        f.rotational(n("psi_" + w), n("wheel_" + w), halves[side], "y", "Angle")
        f.rotational(n("dpsi_" + w), n("wheel_" + w), halves[side], "y", "AngleVel")
        f.torque(n("shaft_" + w), n("wheel_" + w), halves[side], EY,
                 f"-{P('k_shaft')}*{M('psi_' + w)}-{P('c_shaft')}*{M('dpsi_' + w)}")
        # roller load torque on the wheel (relative to its carrier)
        f.rotational(n("ww_" + w), n("wheel_" + w), n("carrier_" + w), "y", "AngleVel")
        f.torque(n("roller_" + w), n("wheel_" + w), n("carrier_" + w), EY, f"-{P('c_roller')}*{M('ww_' + w)}")


def model(cars=1, end_time=5.0):
    f = Fds()
    for k, v in PARAMETERS.items():  # first: legacy FreeDyn evaluates expressions while reading
        f.parameter(k, v)
    for c in range(1, cars + 1):
        car(f, c, 3.0 * (c - 1))
    f.add("Force Gravity", [("ForceID", f.new_id()), ("GravityAccVector", "0.0,0.0,-9.81"),
                            ("ActivationTime", "0.0")], "EndForce")
    f.add("Simulation Dynamic", [
        ("SimulationTimeBegin", "0.0"), ("InitialPositionFromFile", "0"), ("InitialVelocityFromFile", "0"),
        ("SimulationTimeEnd", repr(end_time)), ("OutputTimeBegin", "0.0"), ("OutputTimeStepSize", "1.0e-02"),
        ("isExactOutputTimeEnforced", "yes"), ("WriteConstraintForceResultFile", "no"),
        ("WriteForceResultFile", "no"), ("WriteVelocityResultFile", "yes"), ("WriteStateResultFile", "yes"),
        ("WriteAccelerationResultFile", "yes"), ("WriteExtConstraintLagrangeResultFile", "no"),
        ("WriteMeasureResultFile", "no")], "EndSimulation")
    f.add("Solver HHT", [
        ("Alpha", "-0.1"), ("MaxNumberOfInnerIterations", "10,0"), ("GlobalErrorTolerance", "1.0e-06"),
        ("SolverMinTimeStep", "1.0e-14"), ("SolverMaxTimeStep", "1.0e-03"), ("ModifyNewmark", "no"),
        ("MaxNumOfTimeStepsUsingSameFactorizedJacobian", "0"), ("UsePardisoScalingAndMatching", "no"),
        ("DebugLevel", "0"), ("DebugTimeBegin", "0.0"), ("DebugTimeEnd", repr(end_time))], "EndSolver")
    header = (f"Header freeDynHeader\n% FreeDyn 2.0: {cars} fictitious car(s) on a chassis dynamometer "
              f"(examples/scaling/make_car.py)\nEndHeader")
    return header + "\n%\n" + "\n%\n".join(f.blocks) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cars", type=int, default=1)
    ap.add_argument("--end-time", type=float, default=5.0)
    ap.add_argument("-o", "--output", type=Path)
    args = ap.parse_args(argv)
    out = args.output or Path(f"car_{args.cars}.fds")
    out.write_text(model(args.cars, args.end_time), encoding="latin-1")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
