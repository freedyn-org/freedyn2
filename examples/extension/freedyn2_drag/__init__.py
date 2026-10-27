"""An example extension of FreeDyn 2.0: a force type `aero_drag`, air drag at a point of a body,

  F = -c |v - w| (v - w)        v: velocity of the point, w: wind (global), c = rho cw A / 2

  forces:
    drag: {type: aero_drag, body1: car, point: [1.0, 0, 0.6], coefficient: 0.42, wind: [-5, 0, 0]}

The whole extension is this file and the entry point in pyproject.toml; docs/extensions.md
explains it step by step.
"""

import casadi as ca
import numpy as np

from fdgen import forces
from fdgen.errors import ModelError
from fdgen.extensions import Extension
from fdgen.geometry import local_point

N_P = 7  # parameters of the element: point u in body coordinates (3), c (1), wind w (3)


def drag_law(c):
    """The force law on a connector c (fdgen.kinematics.Connector) with one body: loads and outputs."""
    p, body = c.p, c.bodies[0]
    u, coef, wind = p[0:3], p[3], p[4:7]
    rel = body.point_velocity(u) - wind                   # velocity of the point against the air
    speed = ca.sqrt(ca.dot(rel, rel) + 1e-12)             # smooth at 0: the derivatives exist everywhere
    F = -coef * speed * rel
    loads = [(0, F, u, None)]                             # (body index, force, point, torque)
    outputs = {"force": F, "airspeed": speed}             # result channels drag.force_x/y/z, drag.airspeed
    return loads, outputs


class Drag(Extension):
    name = "drag"
    element_types = {  # generated and compiled into the model plugin (rep: "lie" or "euler" bodies)
        "aero_drag": lambda rep: forces.force_element_type("aero_drag", 1, N_P, drag_law, rep),
    }
    conventions = ("Air drag (drag extension): a force of type aero_drag on body1 at point (global, t = 0) "
                   "with coefficient c = rho cw A / 2 [kg/m] and an optional wind [m/s]; channels force_*, airspeed.")

    def schema(self, schema):
        force = schema["$defs"]["force"]
        force["properties"]["type"]["enum"].append("aero_drag")
        force["properties"]["coefficient"] = {"type": "number", "minimum": 0,
                                              "description": "aero_drag: rho cw A / 2 [kg/m] (drag extension)."}
        force["properties"]["wind"] = {"$ref": "#/$defs/vec3", "description": "aero_drag: wind velocity, global "
                                       "[m/s]. Default [0, 0, 0] (drag extension)."}

    def validate_force(self, f, base_dir):
        if f["type"] == "aero_drag":
            if "coefficient" not in f:
                raise ModelError(f"force {f['name']}: aero_drag needs coefficient")
            if f["body2"] != "ground":
                raise ModelError(f"force {f['name']}: aero_drag acts on body1 only")
        return f

    def force_element(self, force, bodies):
        if force["type"] != "aero_drag":
            return None
        model = {"bodies": bodies}  # local_point needs the bodies at t = 0
        u = local_point(model, force["body1"], force.get("point"))
        p = [*u, force["coefficient"], *np.asarray(force.get("wind", [0.0, 0.0, 0.0]), dtype=float)]
        return "aero_drag", [force["body1"]], p


extension = Drag()
