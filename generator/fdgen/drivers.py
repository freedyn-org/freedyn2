"""Drivers: prescribed motion of a measure, Phi = m(q) - f(t) = 0.

Drivers along a curve of time (motion: <spline>(t)) on the common measures are library element
types (spline_driver_*: compiled into the runtime, the table is data): no model plugin. Any
other driver is built in the model plugin.
"""

import re

import casadi as ca

from .custom_element import CustomElement, PluginElement
from .forces import LibraryElement
from .measures import POSITION_MEASURES


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

_CURVE_OF_T = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(\s*t\s*\)\s*$")


def _variants():
    """{type name: measure spec (bodies b1, b2 / ground)} of the library drivers."""
    out = {}
    for b2 in ("ground", "b2"):
        end = "ground" if b2 == "ground" else "bb"
        for off in ("none", "initial"):
            out[f"spline_driver_angle_{end}_{off}"] = {"type": "angle", "body1": "b1", "body2": b2,
                                                       "vector": [1, 0, 0], "plane": [[1, 0, 0], [0, 1, 0]],
                                                       "offset": off}
            out[f"spline_driver_distance_{end}_{off}"] = {"type": "distance", "body1": "b1", "body2": b2,
                                                          "offset": off}
            for comp in "xyz":
                for frame in ("global", "b1") + (("b2",) if b2 != "ground" else ()):
                    fname = {"global": "global", "b1": "body1", "b2": "body2"}[frame]
                    out[f"spline_driver_{comp}_{fname}_{end}_{off}"] = {
                        "type": "displacement", "body1": "b1", "body2": b2, "component": comp, "frame": frame,
                        "offset": off}
    return out


VARIANTS = _variants()


def _canonical(spec):
    bodies = {"b1": {"mass": 1.0, "inertia": [1, 1, 1], "position": [0.1, 0.2, 0.3], "orientation": [1, 0, 0, 0]},
              "b2": {"mass": 1.0, "inertia": [1, 1, 1], "position": [-0.3, 0.1, 0.2], "orientation": [1, 0, 0, 0]}}
    return {"model": "library", "bodies": bodies, "measures": {"m": spec},
            "splines": {"s": {"x": [0.0, 1.0, 2.0], "y": [0.0, 1.0, 0.0], "type": "akima"}}, "parameters": {}}


def _library_build(kind):
    model = _canonical(VARIANTS[kind])
    return _build(model, kind, {"measure": "m", "motion": "s(t)"})


def build_library(kind):
    """ElementFunctions of a library driver type (fdgen.codegen)."""
    cx, _, phi = _library_build(kind)
    return cx.element(kind, Phi=phi)


_KEYS = {}


def _library_keys():
    if not _KEYS:
        for kind in VARIANTS:
            cx, _, phi = _library_build(kind)
            cx.c.p = cx._p()
            _KEYS[cx._key("@", None, {"Phi": phi})] = kind
    return _KEYS


def library_driver(model, name, d):
    """LibraryElement of a driver along a curve of time on a measure a library type covers, or None."""
    m = _CURVE_OF_T.match(str(d["motion"]))
    if not m or m.group(1) not in (model.get("splines") or {}) or float(d.get("ramp", 0.0)) > 0:
        return None
    try:
        cx, _, phi = _build(model, name, d)
    except ValueError:
        return None
    if cx.rep != "lie":
        return None
    cx.c.p = cx._p()
    kind = _library_keys().get(cx._key("@", None, {"Phi": phi}))
    if kind is None:
        return None
    return LibraryElement(kind, cx.bodies, cx.consts.values, cx.spline_slots())


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
