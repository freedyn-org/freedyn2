"""Physical comparison: FreeDyn 2.0 with its own defaults and legacy FreeDyn, each against an
independent reference (legacy/references: RK4 on Euler's equations for free bodies, on the
minimal-coordinate equations of motion for revolute joint chains).

FreeDyn 2.0 may use other methods than the legacy kernel (Arnold-Bruls instead of the Negrut
HHT form); both must solve the same physics, and FreeDyn 2.0 must be at least as accurate.
Skipped without the legacy build (FREEDYN_LEGACY_BIN) and freedyn2_run (FREEDYN2_RUN).
"""

import os
import sys
from pathlib import Path

import pytest

from conftest import ROOT

sys.path.insert(0, str(ROOT / "legacy"))
import compare  # noqa: E402
from fdgen.fds import load_fds  # noqa: E402

LEGACY = Path(os.environ.get("FREEDYN_LEGACY_BIN", ROOT / "build/legacy/build/FreeDyn"))
V2 = Path(os.environ.get("FREEDYN2_RUN", ROOT / "build/freedyn2_run"))
MODELS = sorted((ROOT / "examples" / "compare").glob("*.fds"))

# Relative omega error bound for FreeDyn 2.0 (about 5x its measured discretization error, second
# order) and a sanity bound for legacy FreeDyn, whose Negrut form is first order with alpha < 0
# (measured omega 2.4e-3 ... 1.3e-1, A up to 0.34 for step_control: alpha = -0.2, 40 rad of
# fast rotation): both solve the same problem.
V2_TOL = {"free_body": 5e-6, "rotated_gui_defaults": 1e-4, "step_control": 2e-4, "dzhanibekov": 5e-5,
          "fast_rotor": 2e-2, "fast_rotor_modified": 2e-2, "pendulum": 5e-4, "double_pendulum": 1e-3,
          "forced_oscillator": 2e-5, "spring_chain": 1e-3, "torsion_disks": 5e-4, "spline_spring": 5e-5,
          "driven_base": 5e-5, "driven_rotor": 2e-4}
LEGACY_SANITY = 0.5
# Fast rotor without ModifyNewmark: the legacy kernel loses ~28 % of the spin in 1 s and the
# orientation is completely wrong (A error ~2): the saturation problem of Euler parameters with
# HHT damping. Shown in the comparison, not bounded.
LEGACY_SANITY_EXEMPT = {"fast_rotor"}

pytestmark = pytest.mark.skipif(not (LEGACY.exists() and V2.exists()),
                                reason="legacy FreeDyn build or freedyn2_run not available")


@pytest.mark.parametrize("model", MODELS, ids=lambda p: p.stem)
def test_same_physics_as_legacy(model):
    d = compare.main([str(model), "--level", "physics", "--legacy", str(LEGACY), "--v2", str(V2),
                      "--repeat", "1"])[0]
    L, V = d["legacy"], d["v2"]
    if model.stem not in LEGACY_SANITY_EXEMPT:
        assert L["omega_rel"] < LEGACY_SANITY and L["A"] < LEGACY_SANITY
    assert V["omega_rel"] <= V2_TOL[model.stem], V
    assert V["omega_rel"] <= L["omega_rel"] and V["A"] <= L["A"]
    m = load_fds(model)
    if m["joints"] or m["forces"]:
        # Positions carry the integration error. Pure translation (forced_oscillator): both forms
        # are second order for constant mass, equal up to a few percent.
        assert V["R_rel"] <= V2_TOL[model.stem] and V["R_rel"] <= 1.1 * L["R_rel"]
    else:
        assert V["R_rel"] < 1e-12 and L["R_rel"] < 1e-12  # free flight: R exact in both
