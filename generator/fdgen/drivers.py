"""Drivers: prescribed motion of a measure, Phi = m(q) - f(t) = 0.

Drivers along a curve of time (motion: <spline>(t), no ramp) on a displacement, distance or
angle are library element types (spline_driver_<measure>_<end>: compiled into the runtime, the
measure's geometry and the curve's segment are parameters, the table is data): no model
plugin. Any other driver is built in the model plugin.
"""

import re

import casadi as ca

from . import curve_forces
from .custom_element import CustomElement, PluginElement
from .forces import LibraryElement
from .kinematics import Connector, Ground
from .lagrange import ElementFunctions
from .measures import POSITION_MEASURES
from .splines import cubic, segment_of
from .splines import table as spline_table


def ramp(t, Ta, A, f1, f1d, f1dd):
    """Legacy TimeMotion activation (MBS_RiBo_Time_Motion): for t < Ta the 7th order polynomial
    from A (zero rate and acceleration) to the motion f, joining it C2 at Ta (f1, f1d, f1dd =
    f(Ta), f'(Ta), f''(Ta)); coefficients and evaluation in the legacy operation order."""
    B = -21. * A + 21. * f1 - 6. * f1d * Ta + (f1dd * Ta**2.) / 2.
    C = 35. * A - 35. * f1 + Ta * (11. * f1d - f1dd * Ta)
    D = -15.0 * A + 15. * f1 - 5. * f1d * Ta + (f1dd * Ta**2.) / 2.
    return A + (B * t**5.) / Ta**5. + (C * t**6.) / Ta**6. + (D * t**7.) / Ta**7.


def _build(model, name, d):
    """(CustomElement, element type name, Phi) of a driver."""
    measures = model.get("measures", {})
    if d["measure"] not in measures:
        raise ValueError(f"driver {name}: unknown measure {d['measure']!r}")
    spec = measures[d["measure"]]
    if spec["type"] not in POSITION_MEASURES:
        raise ValueError(f"driver {name}: measure must be a displacement, distance or angle")
    cx = CustomElement(model, f"driver {name}", [d["motion"]],
                       [spec["body1"]] + ([spec["body2"]] if spec.get("body2", "ground") != "ground" else []),
                       allow_measures=False)
    m = cx.measure(spec)
    f = cx.translate(d["motion"])
    Ta = float(d.get("ramp", 0.0))
    if Ta > 0:
        t = cx.c.t
        ft = ca.jacobian(f, t)
        f1, f1d, f1dd = (cx.value0(x, Ta) for x in (f, ft, ca.jacobian(ft, t)))
        A = cx.value0(m)
        consts = [cx.consts.add(v, k) for k, v in (("ramp", Ta), ("A", A), ("f1", f1), ("f1d", f1d), ("f1dd", f1dd))]
        f = ca.if_else(t < consts[0], ramp(t, *consts), f)
    tname = f"d_{model['model']}_{name}" + ("_euler" if cx.rep == "euler" else "")
    return cx, tname, m - f


# --- library drivers: motion along a curve of time ----------------------------------------------
#
# Phi = m(q) - curve(t): the measure's frame, component, points and axes are parameters (as for
# the curve forces), so one type per measure and end: spline_driver_<measure>_<end>.

_CURVE_OF_T = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(\s*t\s*\)\s*$")
LIBRARY_MEASURES = ("displacement", "distance", "angle")
KINDS = {f"spline_driver_{m}_{end}": (1 if end == "ground" else 2, curve_forces.N_MEASURE[m] + 5)
         for m in LIBRARY_MEASURES for end in ("ground", "bb")}


def build_library(kind):
    """ElementFunctions of a library driver type (fdgen.codegen)."""
    _, _, m, end = kind.split("_")
    nb, n_p = KINDS[kind]
    c = Connector("lie", nb, n_p)
    b1, b2 = c.bodies[0], (c.bodies[1] if nb == 2 else Ground())
    n = curve_forces.N_MEASURE[m]
    value, update = curve_forces.measure_value(c, m, b1, b2, c.p[0:n])
    funs = c.functions(kind, Phi=value - cubic(c.p[n:n + 5], c.t))
    funs["spline"] = ca.Function(f"fd_{kind}_spline", [c.q, c.v, c.p, c.t], [c.t], ["q", "qd", "p", "t"], ["x"])
    if update is not None:  # continuous angle: its reference after each accepted step
        new = ca.SX(n_p, 1)
        new[update[0]] = c.expand(update[1])
        funs["post"] = ca.Function(f"fd_{kind}_post", [c.q, c.v, c.p, c.t], [new], ["q", "qd", "p", "t"], ["p_new"])
    return ElementFunctions(kind, **funs)


def library_driver(model, name, d):
    """LibraryElement of a driver along a curve of time (motion: <spline>(t), no ramp) on a
    measure the library types cover, or None (the driver is built in the model plugin)."""
    m = _CURVE_OF_T.match(str(d["motion"]))
    splines = model.get("splines") or {}
    if not m or m.group(1) not in splines or float(d.get("ramp", 0.0)) > 0:
        return None
    spec = (model.get("measures") or {}).get(d["measure"])
    if spec is None or spec["type"] not in LIBRARY_MEASURES:
        return None
    fb = (spec["body1"], spec.get("body2", "ground"))
    if any(model["bodies"][b].get("element", "rigid_body") != "rigid_body" for b in fb if b != "ground"):
        return None
    mp = curve_forces.measure_params(model, spec, fb)
    if mp is None:
        return None
    sp = splines[m.group(1)]
    tab = spline_table(sp["x"], sp["y"], sp.get("type", "akima"))
    kind = f"spline_driver_{spec['type']}_{'ground' if fb[1] == 'ground' else 'bb'}"
    names = [fb[0]] + ([] if fb[1] == "ground" else [fb[1]])
    return LibraryElement(kind, names, [*mp, *segment_of(tab, 0.0)], [(len(mp), tab)])


def driver_elements(model):
    """[(driver name, element type, [bodies], p values, ElementFunctions)] of the model's drivers
    built in the model plugin (those library_driver does not cover): one constraint
    Phi = m(q) - f(t) per driver, m a position measure (displacement, distance, angle), f an
    expression of t, parameters, runtime inputs and splines. With ramp = Ta > 0, f is replaced
    for t < Ta by the legacy activation polynomial from the initial value of m (at rest), so a
    motion need not start at the initial configuration."""
    out = []
    for name, d in (model.get("drivers") or {}).items():
        if library_driver(model, name, d) is not None:
            continue
        cx, tname, phi = _build(model, name, d)
        out.append(PluginElement(name, tname, cx.bodies, cx.consts.values, cx.element(tname, Phi=phi),
                                 cx.consts.inputs, cx.spline_slots()))
    return out
