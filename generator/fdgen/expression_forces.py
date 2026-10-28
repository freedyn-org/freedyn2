"""Expression forces and torques (model plugin): force laws given as expressions.

  expression_force   value s of the law along a direction e on body1 at point1, -s e on body2
                     at point2: e line of sight (point1 to point2), a global direction, or a
                     direction fixed in body1 (frame body)
  expression_torque  s about an axis (global or fixed in body1) on body1, -s on body2

The law is an expression of t, model parameters, runtime inputs, measures and splines
(fdgen.expressions).
"""

import casadi as ca

from .custom_element import CustomElement, PluginElement
from .geometry import local_axis, local_point

EXPRESSION_TYPES = ("expression_force", "expression_torque")


def _direction(cx, f, vec):
    """Unit vector of a direction given globally at t = 0: fixed in space, or in body1 (frame body)."""
    if f.get("frame", "global") == "global":
        return cx.consts.add(local_axis(cx.model, "ground", vec), "e")
    return cx.c.bodies[0].axis(cx.consts.add(local_axis(cx.model, f["body1"], vec), "e"))


def _force_loads(cx, f, s):
    k1, k2 = cx.c.bodies[0], cx.kin(f["body2"])
    u1 = cx.consts.add(local_point(cx.model, f["body1"], f.get("point1")), "u1")
    x2 = cx.consts.add(local_point(cx.model, f["body2"], f.get("point2")), "u2")
    direction = f.get("direction", "line_of_sight")
    if direction == "line_of_sight":
        dr = k2.point(x2) - k1.point(u1)
        e = dr / ca.norm_2(dr)
    else:
        e = _direction(cx, f, direction)
    loads = [(0, s * e, u1, None)]
    if f["body2"] != "ground":
        loads.append((1, -s * e, x2, None))
    return loads, e


def _torque_loads(cx, f, s):
    e = _direction(cx, f, f["axis"])
    loads = [(0, None, None, s * e)]
    if f["body2"] != "ground":
        loads.append((1, None, None, -s * e))
    return loads, e


def expression_elements(model):
    """[(force name, element type, [bodies], p values, ElementFunctions)] of the model's
    expression forces, in model order."""
    out = []
    for f in model.get("forces", []):
        if f["type"] not in EXPRESSION_TYPES:
            continue
        is_force = f["type"] == "expression_force"
        law = f["force"] if is_force else f["torque"]
        cx = CustomElement(model, f"force {f['name']}", [law],
                           [f["body1"]] + ([f["body2"]] if f["body2"] != "ground" else []))
        s = cx.intermediate(cx.translate(law), "law")  # value of the law, chain rule as for measures
        loads, e = (_force_loads if is_force else _torque_loads)(cx, f, s)
        name = f"x_{model['model']}_{f['name']}" + ("_euler" if cx.rep == "euler" else "")
        vec_name = "force" if is_force else "torque"
        elem = cx.element(name, out={"value": s, vec_name: s * e}, Q=cx.c.generalized_force(loads))
        out.append(PluginElement(f["name"], name, cx.bodies, cx.consts.values, elem, cx.consts.inputs))
    return out
