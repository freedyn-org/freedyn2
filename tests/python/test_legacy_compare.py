"""FreeDyn 2.0 against the legacy FreeDyn kernel on the same .fds models.

Needs the Linux build of the legacy solver (legacy/build_legacy.py) and the FreeDyn 2.0
runner; skipped otherwise. Paths: FREEDYN_LEGACY_BIN, FREEDYN2_RUN.
"""

import os
import sys
from pathlib import Path

import pytest

from conftest import ROOT

sys.path.insert(0, str(ROOT / "legacy"))
import compare  # noqa: E402

LEGACY = Path(os.environ.get("FREEDYN_LEGACY_BIN", ROOT / "build/legacy/build/FreeDyn"))
V2 = Path(os.environ.get("FREEDYN2_RUN", ROOT / "build/freedyn2_run"))
MODELS = sorted((ROOT / "examples" / "compare").glob("*.fds"))

# Results are printed with 13 significant digits by the legacy kernel; agreement is at that
# level unless the model amplifies round-off. Each tolerance is the round-off floor measured by
# running FreeDyn 2.0 against itself with another LU factorization (sparse vs dense):
# - dzhanibekov: differences of ~1e-13 grow by e^(lambda t) whenever the body passes the
#   unstable axis (self-comparison 6.6e-7 after 20 s),
# - step_control: ~40 rad of fast rotation, round-off accumulates (self-comparison 2.7e-11),
# - double_pendulum: chaotic after release from the horizontal (self-comparison 1.0e-9),
# - spring_chain: the legacy kernel differentiates force expressions numerically
#   (FL_FunctionExpression::computeGradient) and its Newton rule stops after two corrections, so
#   its iteration matrix enters the result: 5e-13 after 0.1 s, 3e-8 in R and 1.7e-7 in A after
#   5 s (self-comparison 1e-13). Each force alone agrees to round-off over 0.5 s.
# - spline_spring: the same for a stiff nonlinear spring law from an Akima spline (2.8e-9;
#   each term alone at round-off).
# - driven_base, driven_rotor: the legacy activation polynomial uses numerical time derivatives
#   of the motion (FL_FunctionExpression), FreeDyn 2.0 exact ones (7e-10, 3e-10).
# - fast_rotor (300 rad/s): the round-off of the linear solver is amplified to 2e-11 over 1 s
#   (FreeDyn 2.0 against itself: KLU vs dense 2.1e-11, SparseLU vs dense 1.7e-11).
TOL = {"dzhanibekov": 1e-5, "fast_rotor": 5e-11, "step_control": 1e-10, "double_pendulum": 1e-8, "spring_chain": 1e-6,
       "spline_spring": 1e-7, "driven_base": 1e-8, "driven_rotor": 1e-8}

pytestmark = pytest.mark.skipif(not (LEGACY.exists() and V2.exists()),
                                reason="legacy FreeDyn build or freedyn2_run not available")


@pytest.mark.parametrize("model", MODELS, ids=lambda p: p.stem)
def test_same_results_as_legacy(model):
    d = compare.main([str(model), "--legacy", str(LEGACY), "--v2", str(V2), "--repeat", "1"])[0]
    assert d["frames"][0] == d["frames"][1] and d["t"] == 0.0
    assert d["steps"][0] == d["steps"][1] and d["rejected"][0] == d["rejected"][1]
    assert d["same_accept"] and d["same_iterations"]
    assert d["step_rel"] < 1e-4  # 5 significant digits in the legacy step log
    tol = TOL.get(model.stem, 1e-11)
    for key in ("R_rel", "A_rel", "Rdot_rel", "omega_body_rel"):
        assert d[key] <= tol, (key, d[key])
