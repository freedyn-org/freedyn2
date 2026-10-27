"""Named points and frames (fdgen.hardpoints): resolution, mirror and offsets, frames from
directions, use in every point and orientation field, errors, checks, viewer scene."""

import copy
from pathlib import Path

import numpy as np
import pytest
import yaml

from fdgen import checks, rigid_body
from fdgen.model import ModelError, validate
from fdgen.view import scene
from fdgen.view.server import apply_overrides

ROOT = Path(__file__).resolve().parents[2]


def base(**extra):
    m = {"model": "m", "parameters": {"zp": 0.5},
         "points": {"a": [1.0, 0.4, "=zp"], "b": {"mirror": "a"}, "c": {"from": "b", "offset": [0.0, 0.0, -0.2]}},
         "frames": {"f": {"origin": "c", "z": ["c", "a"]}},
         "bodies": {"p": {"mass": 1.0, "inertia": [1, 1, 1], "position": "a"}},
         "solver": {"type": "HHT", "end_time": 1.0, "step": 0.01}}
    m.update(extra)
    return m


def A(q):
    return np.array(rigid_body.rotation_matrix(q))


def test_points_formulas_mirror_offset():
    v = validate(base())
    np.testing.assert_allclose(v["points"]["a"], [1.0, 0.4, 0.5])
    np.testing.assert_allclose(v["points"]["b"], [1.0, -0.4, 0.5])
    np.testing.assert_allclose(v["points"]["c"], [1.0, -0.4, 0.3])
    np.testing.assert_allclose(v["bodies"]["p"]["position"], [1.0, 0.4, 0.5])


def test_frame_from_two_points_and_x_hint():
    v = validate(base(frames={"f": {"origin": "c", "z": ["c", "a"], "x": [0, 0, 1]}}))
    o, q = v["frames"]["f"]["origin"], v["frames"]["f"]["orientation"]
    R = A(q)
    z = np.array([0.0, 0.8, 0.2]) / np.linalg.norm([0.0, 0.8, 0.2])
    np.testing.assert_allclose(o, [1.0, -0.4, 0.3])
    np.testing.assert_allclose(R[:, 2], z, atol=1e-12)
    assert R[:, 0] @ z == pytest.approx(0.0, abs=1e-12) and R[2, 0] > 0  # x toward the hint, normal to z
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_names_in_every_point_and_orientation_field():
    m = base(
        joints={"j": {"type": "revolute", "body1": "p", "location": "f", "orientation": "f"}},
        forces={"s": {"type": "spring_damper", "body1": "p", "point1": "a", "point2": "c", "stiffness": 1.0},
                "b": {"type": "bushing", "body1": "p", "location": "a", "orientation": "f", "stiffness": 1.0,
                      "rot_stiffness": 1.0},
                "e": {"type": "expression_force", "body1": "p", "point": "a", "direction": [0, 0, 1], "force": "d"}},
        measures={"d": {"type": "displacement", "body1": "p", "point1": "a", "point2": "b", "component": "y"}},
        sensors={"acc": {"body1": "p", "point": "a", "orientation": "f"}})
    v = validate(m)
    f = v["frames"]["f"]
    joint = v["joints"][0]
    np.testing.assert_allclose(joint["location"], f["origin"])
    np.testing.assert_allclose(np.array(joint["frame"]), A(f["orientation"]), atol=1e-12)
    forces = {x["name"]: x for x in v["forces"]}
    assert forces["s"]["point1"] == v["points"]["a"] and forces["s"]["point2"] == v["points"]["c"]
    assert forces["b"]["orientation"] == f["orientation"] and forces["e"]["point"] == v["points"]["a"]
    assert v["measures"]["d"]["point2"] == v["points"]["b"]
    assert v["sensors"][0]["point"] == v["points"]["a"]
    refs = v["point_refs"]
    assert refs["a"]["users"] == ["acc", "b", "d", "e", "f", "p", "s"] and refs["a"]["body"] == "p"
    assert refs["c"]["users"] == ["f", "s"]  # the frame and the spring (point2: on the ground)
    assert refs["f"]["users"] == ["acc", "b", "j"]


@pytest.mark.parametrize("change, message", [
    ({"bodies": {"p": {"mass": 1.0, "inertia": [1, 1, 1], "position": "nowhere"}}}, "bodies.p.position: unknown point 'nowhere'"),
    ({"joints": {"j": {"type": "revolute", "body1": "p", "location": "a", "orientation": "a"}}},
     "joints.j.orientation: unknown frame 'a'"),
    ({"points": {"a": {"mirror": "b"}, "b": {"mirror": "a"}}}, "points and frames defined by each other: a -> b -> a"),
    ({"points": {"a": [1, 2]}}, "points.a: \\[1, 2\\] is not a point"),
    ({"frames": {"a": {}}}, "names used for a point and a frame: \\['a'\\]"),
    ({"frames": {"f": {"z": ["a", "a"]}}}, "frames.f.z: the direction has length zero"),
    ({"frames": {"f": {"z": [0, 0, 1], "x": [0, 0, 2]}}}, "frames.f: x is parallel to z"),
])
def test_errors(change, message):
    m = base()
    for k, v in change.items():
        m[k] = {**m.get(k, {}), **v} if k == "frames" and "f" not in v else v
    with pytest.raises(ModelError, match=message):
        validate(m)


def test_unused_points_are_reported():
    v = validate(base())  # b is used by c, c by f, f by nothing
    found = [f for f in checks.check_model(v)["findings"] if f["code"] == "unused_points"]
    assert found and found[0]["elements"] == ["f"]


def test_without_points_nothing_changes():
    m = base()
    del m["points"], m["frames"]
    m["bodies"]["p"]["position"] = [1.0, 0.4, 0.5]
    v = validate(m)
    assert v["points"] == {} and v["point_refs"] == {}


def test_double_wishbone_tuning_a_hardpoint_moves_its_joints():
    raw = yaml.safe_load((ROOT / "examples" / "double_wishbone.yaml").read_text())
    v = validate(copy.deepcopy(raw), base_dir=ROOT / "examples")
    assert v["point_refs"]["uca_front"]["users"] == ["uca_axis"] and v["point_refs"]["uca_axis"]["body"] == "uca"
    tuned = validate(apply_overrides(copy.deepcopy(raw), {"points.uca_outer.2": 0.50}), base_dir=ROOT / "examples")
    ball = {j["name"]: j for j in tuned["joints"]}["upper_ball"]
    assert ball["location"] == [0.0, 0.68, 0.50]
    s = scene.scene(v)
    pts = {p["name"]: p for p in s["points"]}
    assert pts["wheel_center"]["body"] == "wheel" and pts["wheel_axis"]["kind"] == "frame"
    np.testing.assert_allclose(pts["wheel_center"]["position"], 0, atol=1e-12)  # the wheel's center of mass
    assert not [f for f in checks.check_model(v)["findings"] if f["code"] == "unused_points"]


def test_points_and_frames_in_a_body():
    """in: body: coordinates in the body frame (center of mass, principal axes) at t = 0; the
    body's own position may be a hardpoint; frames in frames."""
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)  # body turned by 90 deg about z
    m = base(points={"cm": [1.0, 2.0, 3.0], "tip": {"in": "arm", "local": [0.5, 0.0, 0.0]},
                     "side": {"in": "hinge", "local": [0.0, 0.0, 0.2]}},
             frames={"hinge": {"in": "arm", "origin": [0.5, 0.0, 0.0], "z": [1, 0, 0]},
                     "tilted": {"in": "hinge", "orientation": [c, s, 0, 0]}},
             bodies={"arm": {"mass": 1.0, "inertia": [1, 1, 1], "position": "cm", "orientation": [c, 0, 0, s]}},
             sensors={"tip_sensor": {"body1": "arm", "point": "tip", "orientation": "tilted"}})
    v = validate(m)
    np.testing.assert_allclose(v["points"]["tip"], [1.0, 2.5, 3.0], atol=1e-12)  # body x = global y
    fo, fq = v["frames"]["hinge"]["origin"], v["frames"]["hinge"]["orientation"]
    np.testing.assert_allclose(fo, [1.0, 2.5, 3.0], atol=1e-12)
    np.testing.assert_allclose(A(fq)[:, 2], [0, 1, 0], atol=1e-12)  # z: body x
    np.testing.assert_allclose(v["points"]["side"], [1.0, 2.7, 3.0], atol=1e-12)
    At = A(v["frames"]["tilted"]["orientation"])
    np.testing.assert_allclose(At, A(fq) @ A([c, s, 0, 0]), atol=1e-12)
    np.testing.assert_allclose(v["sensors"][0]["point"], [1.0, 2.5, 3.0], atol=1e-12)
    refs = v["point_refs"]
    assert refs["tip"]["body"] == "arm" and refs["hinge"]["body"] == "arm"
    assert refs["hinge"]["users"] == ["side", "tilted"] and refs["cm"]["users"] == ["arm"]


def test_body_placed_at_a_point_in_itself_is_a_loop():
    m = base(points={"p": {"in": "b", "local": [0, 0, 1]}},
             bodies={"b": {"mass": 1.0, "inertia": [1, 1, 1], "position": "p"}})
    m.pop("frames")
    with pytest.raises(ModelError, match="defined by each other: p -> p|defined by each other"):
        validate(m)


def test_in_needs_a_body_or_frame():
    m = base(points={"p": {"in": "nothing", "local": [0, 0, 1]}})
    m.pop("frames")
    with pytest.raises(ModelError, match="points.p.in: unknown body or frame 'nothing'"):
        validate(m)
