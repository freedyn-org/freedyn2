"""Joint constraint elements: constraint values, Jacobians through the Lie group update, presets."""

import numpy as np
import pytest

from conftest import unit_quat
from fdgen import joints, rigid_body
from fdgen.geometry import rotation_matrix
from fdgen.joints import PRESETS as JOINT_PRESETS
from fdgen.joints import primitives as joint_primitives
from fdgen.model import validate

LIE = rigid_body.build()


def random_bodies(rng):
    return {
        name: {"position": list(rng.normal(size=3)), "orientation": list(unit_quat(rng))}
        for name in ("a", "b")
    }


def joint(fix, body2="b", rng=None):
    K = rotation_matrix(unit_quat(rng))
    return {"name": "j", "body1": "a", "body2": body2, "location": list(rng.normal(size=3)),
            "frame": K.tolist(), "fix": fix}


@pytest.mark.parametrize("body2", ["b", "ground"])
def test_constraints_vanish_initially(rng, body2):
    """The constants c make every primitive constraint zero at the initial configuration."""
    bodies = random_bodies(rng)
    for kind, _, names, p in joint_primitives(joint(JOINT_PRESETS["fixed"], body2, rng), bodies):
        q = np.concatenate([np.r_[bodies[n]["position"], bodies[n]["orientation"]] for n in names])
        e = joints.build(kind)
        phi = np.array(e.ic(q, np.zeros(6 * len(names)), 0, 0, p, 0, 0, 0)[2]).ravel()
        assert abs(phi[-1]) < 1e-14, kind


def test_preset_constraint_counts(rng):
    bodies = random_bodies(rng)
    for preset, n in (("fixed", 6), ("spherical", 3), ("revolute", 5), ("cylindrical", 4), ("prismatic", 5)):
        assert len(joint_primitives(joint(JOINT_PRESETS[preset], "b", rng), bodies)) == n


@pytest.mark.parametrize("seed", range(5))
@pytest.mark.parametrize("kind", list(joints.KINDS))
def test_iteration_matrix_vs_fd_through_body_updates(kind, seed):
    """J of a Lie group joint element vs FD of its residual with the bodies moved by their own
    exponential update: q_k = q_k,n o exp(incr_k), incr = incr0 + beta h^2 x.

    Step size: the constraint row is Phi / (beta h^2) ~ 3e3 here, so central differences carry
    round-off ~ 1e-16 * 3e3 / eps; eps = 1e-4 keeps it ~ 3e-9 while truncation stays ~ 1e-9."""
    rng = np.random.default_rng(seed)
    n_bodies, n_p = joints.KINDS[kind]
    e = joints.build(kind)
    beta, gamma, h = 0.2722, 0.54, 0.05
    coef = np.array([0.0, beta, gamma, h])
    q_n = [np.r_[rng.normal(size=3), unit_quat(rng)] for _ in range(n_bodies)]
    p = rng.normal(size=n_p)
    v0, x0 = rng.normal(size=6 * n_bodies), rng.normal(size=6 * n_bodies + 1)
    incr0 = 0.3 * rng.normal(size=6 * n_bodies)

    def state(x):
        incr = incr0 + beta * h * h * x[:-1]
        q = np.concatenate([np.array(LIE.update(q_n[k], incr[6 * k:6 * k + 6])).ravel() for k in range(n_bodies)])
        return q, v0 + gamma * h * x[:-1], incr

    def r(x):
        q, v, incr = state(x)
        return np.array(e.res(q, v, x[:-1], x[-1:], p, coef, incr, 0)[0]).ravel()

    q, v, incr = state(x0)
    J = np.array(e.iter(q, v, x0[:-1], x0[-1:], p, coef, incr, 0))
    eps = 1e-4
    J_fd = np.column_stack([(r(x0 + eps * d) - r(x0 - eps * d)) / (2 * eps) for d in np.eye(len(x0))])
    assert np.max(np.abs(J - J_fd)) / max(1, np.max(np.abs(J_fd))) < 1e-7


@pytest.mark.parametrize("kind", list(joints.KINDS))
def test_euler_and_lie_versions_agree(kind, rng):
    """Same constraint value; B (Lie) = Phi_q (Euler) mapped by thetadot = 1/2 Gbar^T omega_bar."""
    n_bodies, n_p = joints.KINDS[kind]
    q = np.concatenate([np.r_[rng.normal(size=3), unit_quat(rng)] for _ in range(n_bodies)])
    p = rng.normal(size=n_p)
    v = rng.normal(size=6 * n_bodies)
    qd = np.concatenate([np.r_[v[6 * k:6 * k + 3],
                               0.5 * np.array(rigid_body.g_body(q[7 * k + 3:7 * k + 7]).T @ v[6 * k + 3:6 * k + 6]).ravel()]
                         for k in range(n_bodies)])
    lie, eul = joints.build(kind), joints.build_euler(kind)
    _, _, phi_l, phid_l = (np.array(a) for a in lie.ic(q, v, 0, 0, p, 0, 0, 0))
    _, _, phi_e, phid_e = (np.array(a) for a in eul.ic(q, qd, 0, 0, p, 0, 0, 0))
    assert phi_l[-1, 0] == pytest.approx(phi_e[-1, 0], abs=1e-14)
    assert phid_l[-1, 0] == pytest.approx(phid_e[-1, 0], abs=1e-13)  # same constraint velocity


def test_yaml_joint_validation_and_frame():
    m = validate({
        "model": "m", "gravity": [0, 0, -9.81],
        "bodies": {"a": {"mass": 1, "inertia": [1, 1, 1]}},
        "joints": {"h": {"type": "revolute", "body1": "a", "orientation": [0.5**0.5, -(0.5**0.5), 0, 0]}},
        "solver": {"type": "HHT", "end_time": 1, "step": 0.01},
    })
    j = m["joints"][0]
    assert j["body2"] == "ground" and j["fix"] == JOINT_PRESETS["revolute"]
    np.testing.assert_allclose(np.array(j["frame"])[:, 2], [0, 1, 0], atol=1e-15)  # joint axis = global y
    assert m["gravity"] == [0, 0, -9.81]


@pytest.mark.parametrize("name", ["pendulum", "double_pendulum"])
def test_yaml_and_fds_models_agree(name):
    """examples/<name>.yaml and examples/compare/<name>.fds describe the same system: same
    bodies, gravity and primitive constraints (kinds, bodies, parameters)."""
    from conftest import ROOT
    from fdgen.fds import load_fds
    from fdgen.model import load_yaml

    y = validate(load_yaml(ROOT / "examples" / f"{name}.yaml"))
    f = load_fds(ROOT / "examples" / "compare" / f"{name}.fds", profile="v2")
    assert y["gravity"] == f["gravity"]
    for (_, by), (_, bf) in zip(y["bodies"].items(), f["bodies"].items()):
        for k in ("mass", "inertia", "position", "orientation", "velocity", "angular_velocity"):
            np.testing.assert_allclose(by[k], bf[k], atol=1e-15)
    assert len(y["joints"]) == len(f["joints"])
    for jy, jf in zip(y["joints"], f["joints"]):
        py, pf = joint_primitives(jy, y["bodies"]), joint_primitives(jf, f["bodies"])
        assert [p[0] for p in py] == [p[0] for p in pf]
        for a, b in zip(py, pf):
            assert list(y["bodies"]).index(a[2][0]) == list(f["bodies"]).index(b[2][0])
            np.testing.assert_allclose(a[3], b[3], atol=1e-15)
