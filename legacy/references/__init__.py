"""Independent reference solutions for the physical comparison (RK4 with small steps, or exact).

Nothing here depends on the HHT formulation or the absolute coordinates of either program.
reference(model, t) picks the reference of a comparison model: by model name for the models
with their own reference (BY_MODEL), else by the model's content.
"""

import numpy as np

from .common import rotation
from .free_bodies import free_bodies_reference
from .free_body import rigid_body_reference
from .hinge_chain import hinge_tree_reference
from .linear import forced_oscillator_reference, torsion_disks_reference
from .prescribed import driven_base_reference, driven_rotor_reference, slider_reference

BY_MODEL = {
    "forced_oscillator": forced_oscillator_reference,
    "torsion_disks": torsion_disks_reference,
    "spline_spring": slider_reference,
    "driven_base": driven_base_reference,
    "driven_rotor": driven_rotor_reference,
}


def reference(model, t):
    """[(omega_body, A, R)] per body from the independent reference of the model."""
    if model["model"] in BY_MODEL:
        return BY_MODEL[model["model"]](model, t)
    if model["forces"]:
        return free_bodies_reference(model, t)
    if model["joints"]:
        return hinge_tree_reference(model, t)
    out = []
    for body in model["bodies"].values():
        w, th, R = rigid_body_reference(body, t)
        out.append((w, np.array([rotation(x) for x in th]), R))
    return out
