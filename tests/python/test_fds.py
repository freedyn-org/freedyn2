"""Reader for legacy .fds models."""

import math

import numpy as np
import pytest

from conftest import ROOT
from fdgen.fds import euler_parameters, load_fds, orthonormalize_columns, parse_blocks
from fdgen.model import ModelError, write_fdm

COMPARE = ROOT / "examples" / "compare"


def shabana_A(t):
    t0, t1, t2, t3 = t
    return np.array(
        [
            [1 - 2 * (t2**2 + t3**2), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)],
            [2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1**2 + t3**2), 2 * (t2 * t3 - t0 * t1)],
            [2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1**2 + t2**2)],
        ]
    )


@pytest.mark.parametrize("path", sorted(COMPARE.glob("*.fds")), ids=lambda p: p.name)
def test_compare_models_load(path):
    m = load_fds(path)
    assert m["solver"]["newton"] == "legacy" and m["solver"]["adaptive"]
    for b in m["bodies"].values():
        assert math.isclose(sum(t * t for t in b["orientation"]), 1.0, rel_tol=0, abs_tol=1e-15)


def test_free_body_values():
    m = load_fds(COMPARE / "free_body.fds")
    b = m["bodies"]["body1"]
    assert b["orientation"] == [1.0, 0.0, 0.0, 0.0]
    assert b["angular_velocity"] == [0.5, 2.0, 0.1] and b["jacobian"] == "full"
    s = m["solver"]
    assert (s["alpha"], s["step"], s["eps"], s["max_newton"]) == (-0.1, 1e-3, 1e-6, 10)


def test_orientation_columns_and_gui_defaults():
    """OrientationPAI lists the columns of A; GUI writes 6 decimals (not exactly orthonormal)."""
    m = load_fds(COMPARE / "rotated_gui_defaults.fds")
    b = m["bodies"]["rotated"]
    assert b["jacobian"] == "no_qvv" and m["solver"]["alpha"] == -0.3
    cols = [0.866025, 0.5, 0.0, -0.353553, 0.612372, 0.707107, 0.353553, -0.612372, 0.707107]
    A_in = np.array(cols).reshape(3, 3).T
    A = shabana_A(b["orientation"])
    np.testing.assert_allclose(A, np.array(orthonormalize_columns(A_in.tolist())), atol=1e-15)
    np.testing.assert_allclose(A, A_in, atol=2e-6)


@pytest.mark.parametrize("seed", range(20))
def test_euler_parameter_cases(seed):
    """All four branches of the Diebel extraction reproduce the rotation matrix."""
    rng = np.random.default_rng(seed)
    t = rng.normal(size=4)
    t /= np.linalg.norm(t)
    A = shabana_A(t)
    np.testing.assert_allclose(shabana_A(euler_parameters(A.tolist())), A, atol=1e-14)


def test_parse_blocks_keys_with_spaces():
    blocks = parse_blocks("Body X\n\tGeometry Position = 1,2,3\n\tMass = 2\nEndBody\n% comment\n")
    assert blocks == [("Body", "X", {"Geometry Position": "1,2,3", "Mass": "2"})]


@pytest.mark.parametrize(
    "edit, message",
    [
        (lambda s: s.replace("EndSolver", "EndSolver\nForce TireForce\n\tForceID = 5\nEndForce"),
         "not supported"),
        (lambda s: s.replace("EndSolver", "EndSolver\nForce Gravity\n\tGravityAccVector = 0,0,-9.81\n\tActivationTime = 0.5\nEndForce"),
         "ActivationTime"),
        (lambda s: s.replace("EndSolver", "EndSolver\nConstraint Generic\n\tBodyIDs = 7, 0\n\tdxFixed = yes\nEndConstraint"),
         "unknown body ID"),
        (lambda s: s.replace("MaxNumberOfInnerIterations = 10,0", "MaxNumberOfInnerIterations = 10,3"), "substitution"),
        (lambda s: s.replace("EndBody", ""), "not closed"),
    ],
)
def test_unsupported_rejected(tmp_path, edit, message):
    p = tmp_path / "m.fds"
    p.write_text(edit((COMPARE / "free_body.fds").read_text()))
    with pytest.raises(ModelError, match=message):
        load_fds(p)


def test_fdm_from_fds(tmp_path):
    write_fdm(load_fds(COMPARE / "rotated_gui_defaults.fds"), tmp_path / "m.fdm")
    text = (tmp_path / "m.fdm").read_text()
    assert "newton legacy" in text and "adaptive 1" in text and "eps 1e-06" in text
    assert "jacobian no_qvv" in text


def test_profiles(tmp_path):
    """Same model, two ways to solve it: legacy algorithm or FreeDyn 2.0 defaults."""
    legacy = load_fds(COMPARE / "rotated_gui_defaults.fds")
    v2 = load_fds(COMPARE / "rotated_gui_defaults.fds", profile="v2")
    assert legacy["bodies"]["rotated"]["orientation"] == v2["bodies"]["rotated"]["orientation"]
    assert (legacy["solver"]["form"], legacy["solver"]["newton"], legacy["solver"]["adaptive"]) == (
        "negrut", "legacy", True)
    assert (v2["solver"]["form"], v2["solver"]["newton"], v2["solver"]["adaptive"]) == (
        "arnold_bruls", "velocity", True)  # the model's error tolerance is kept
    assert v2["bodies"]["rotated"]["jacobian"] == "full"
    write_fdm(v2, tmp_path / "v2.fdm")
    assert "hht_form arnold_bruls" in (tmp_path / "v2.fdm").read_text()
    # error weights of the step size control: legacy FreeDyn's, or absolute + relative tolerance
    assert (legacy["solver"]["error_norm"], v2["solver"]["error_norm"]) == ("legacy", "mixed")
    assert "error_norm mixed" in (tmp_path / "v2.fdm").read_text()
    with pytest.raises(ModelError, match="profile"):
        load_fds(COMPARE / "free_body.fds", profile="fast")


def test_element_types_per_profile():
    legacy = load_fds(COMPARE / "free_body.fds")["bodies"]["body1"]
    v2 = load_fds(COMPARE / "free_body.fds", profile="v2")["bodies"]["body1"]
    assert (legacy["element"], legacy["newmark"]) == ("rigid_body_euler", "standard")
    assert v2["element"] == "rigid_body"


def test_modify_newmark_maps_to_element_option(tmp_path):
    p = tmp_path / "m.fds"
    p.write_text((COMPARE / "free_body.fds").read_text().replace("ModifyNewmark = no", "ModifyNewmark = yes"))
    assert load_fds(p)["bodies"]["body1"]["newmark"] == "modified"
    write_fdm(load_fds(p), tmp_path / "m.fdm")
    assert "newmark modified" in (tmp_path / "m.fdm").read_text()


def test_yaml_default_form(tmp_path):
    from fdgen.model import load_yaml, validate

    m = validate(load_yaml(ROOT / "examples" / "free_body.yaml"))
    assert m["solver"]["form"] == "arnold_bruls"
    write_fdm(m, tmp_path / "m.fdm")
    assert "hht_form arnold_bruls" in (tmp_path / "m.fdm").read_text()
