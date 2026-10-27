"""Simulation Dynamic and Solver HHT blocks, and the profiles (how a model is solved)."""

from ..errors import ModelError
from .blocks import flag, numbers

PROFILES = ("legacy", "v2")


def settings(sim, solver):
    """FreeDyn 2.0 solver settings of the legacy algorithm (profile legacy)."""
    if numbers(sim, "SimulationTimeBegin", 1, [0.0])[0] != 0.0:
        raise ModelError("SimulationTimeBegin must be 0")
    for key in ("InitialPositionFromFile", "InitialVelocityFromFile"):
        if numbers(sim, key, 1, [0.0])[0] != 0.0:
            raise ModelError(f"{key} is not supported")
    if numbers(solver, "MaxNumOfTimeStepsUsingSameFactorizedJacobian", 1, [0.0])[0] > 0:
        raise ModelError("MaxNumOfTimeStepsUsingSameFactorizedJacobian > 0 is not supported yet")
    iters = numbers(solver, "MaxNumberOfInnerIterations", 2)
    if iters[1] != 0:
        raise ModelError("forward/backward substitution only iterations (second value) are not supported yet")
    return {
        "alpha": numbers(solver, "Alpha", 1)[0],
        "end_time": numbers(sim, "SimulationTimeEnd", 1)[0],
        "step": numbers(solver, "SolverMaxTimeStep", 1, [1e3])[0],
        "h_min": numbers(solver, "SolverMinTimeStep", 1, [1e-14])[0],
        "eps": numbers(solver, "GlobalErrorTolerance", 1)[0],
        "max_newton": int(iters[0]),
        "output_step": numbers(sim, "OutputTimeStepSize", 1)[0],
        "output_begin": numbers(sim, "OutputTimeBegin", 1, [0.0])[0],
        "exact_output": flag(sim, "isExactOutputTimeEnforced", False),
        "newton": "legacy",
        "adaptive": True,
        "form": "negrut",
        "vproj_tol": 1e-5,  # SOL_InitialConditionManager::adaptVelocities
        "error_norm": "legacy",  # weights of the local error: largest magnitude so far
    }


def apply_profile(profile, s, bodies, solver):
    """Body element types and solver settings of the profile (the model itself is the same)."""
    if profile not in PROFILES:
        raise ModelError(f"profile must be one of {PROFILES}")
    if profile == "legacy":
        # Euler-parameter bodies as in the legacy kernel, with its modified Newmark option.
        modified = flag(solver, "ModifyNewmark", False)
        for b in bodies.values():
            b.update(element="rigid_body_euler", newmark="modified" if modified else "standard")
    else:
        # Lie group bodies integrate the angular velocity; ModifyNewmark has no meaning there.
        # Step size control: the legacy one, with absolute and relative tolerance in the error norm.
        s.update(form="arnold_bruls", newton="velocity", newton_tol=1e-10, max_newton=25, vproj_tol=1e-12,
                 error_norm="mixed")
        for b in bodies.values():
            b.update(element="rigid_body", jacobian="full")
