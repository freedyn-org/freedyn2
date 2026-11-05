"""Output channels (model plugin): named expressions of measures, parameters and inputs.

  outputs:
    lean_deg: asin(lean) * 180 / pi
    rpm: w_crank * 30 / pi

Each output is an element of the model plugin without loads and constraints (Q = 0): it
connects the bodies its measures refer to and writes one channel `<name>.value` per output
step. Measures are not result channels themselves; an output makes one of them (or any law of
them) a channel, for a program that reads the outputs at run time (fd::System::outputs).
"""

import casadi as ca

from .custom_element import CustomElement, PluginElement
from .expressions import names_in


def output_elements(model):
    """[PluginElement] of the model's outputs, in model order."""
    out = []
    measures = model.get("measures") or {}
    for name, expr in (model.get("outputs") or {}).items():
        if not names_in(expr) & set(measures):
            raise ValueError(f"output {name}: the expression uses no measure ({expr!r}); "
                             f"measures: {sorted(measures)}")
        cx = CustomElement(model, f"output {name}", [expr], [])
        s = cx.translate(expr)
        tname = f"o_{model['model']}_{name}" + ("_euler" if cx.rep == "euler" else "")
        elem = cx.element(tname, out={"value": s}, Q=ca.SX(cx.c.v.numel(), 1))
        out.append(PluginElement(name, tname, cx.bodies, cx.consts.values, elem, cx.consts.inputs))
    return out
