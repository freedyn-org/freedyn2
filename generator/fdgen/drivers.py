"""Drivers (model plugin): prescribed motion of a measure, Phi = m(q) - f(t) = 0."""

import casadi as ca

from .custom_element import CustomElement, PluginElement
from .measures import POSITION_MEASURES


def ramp(t, Ta, A, f1, f1d, f1dd):
    """Legacy TimeMotion activation (MBS_RiBo_Time_Motion): for t < Ta the 7th order polynomial
    from A (zero rate and acceleration) to the motion f, joining it C2 at Ta (f1, f1d, f1dd =
    f(Ta), f'(Ta), f''(Ta)); coefficients and evaluation in the legacy operation order."""
    B = -21. * A + 21. * f1 - 6. * f1d * Ta + (f1dd * Ta**2.) / 2.
    C = 35. * A - 35. * f1 + Ta * (11. * f1d - f1dd * Ta)
    D = -15.0 * A + 15. * f1 - 5. * f1d * Ta + (f1dd * Ta**2.) / 2.
    return A + (B * t**5.) / Ta**5. + (C * t**6.) / Ta**6. + (D * t**7.) / Ta**7.


def driver_elements(model):
    """[(driver name, element type, [bodies], p values, ElementFunctions)] of the model's drivers
    (prescribed motion): one constraint Phi = m(q) - f(t) per driver, m a position measure
    (displacement, distance, angle), f an expression of t, parameters, runtime inputs and splines. With
    ramp = Ta > 0, f is replaced for t < Ta by the legacy activation polynomial from the initial
    value of m (at rest), so a motion need not start at the initial configuration."""
    out = []
    measures = model.get("measures", {})
    for name, d in (model.get("drivers") or {}).items():
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
        out.append(PluginElement(name, tname, cx.bodies, cx.consts.values, cx.element(tname, Phi=m - f),
                                 cx.consts.inputs))
    return out
