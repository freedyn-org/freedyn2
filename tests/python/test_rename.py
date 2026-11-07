"""freedyn2 rename (fdgen.rename): an element and every reference to it, nothing else."""

import re

import pytest
import yaml

from conftest import ROOT
from fdgen import cli
from fdgen.errors import ModelError
from fdgen.model import validate
from fdgen.rename import SECTIONS, rename, rename_text

EXAMPLES = sorted(p for p in (ROOT / "examples").glob("*.yaml"))
# keys whose values are words of their own, not names (component: x, type: revolute, ...)
NOT_NAMES = {"type", "component", "id", "file", "crg", "crg_friction", "element", "jacobian", "newmark", "form",
             "newton", "error_norm", "frame_kind", "model", "name", "direction"}


def names(model):
    return [(s, n) for s in SECTIONS for n in (model.get(s) or {})]


def words(x, key=None, top=True):
    """The names of the elements and the words of the scalar strings of a loaded model."""
    if top:
        return {n for _, n in names(x)} | words(x, None, False)
    if isinstance(x, dict):
        return {w for k, v in x.items() for w in words(v, k, False)}
    if isinstance(x, list):
        return {w for v in x for w in words(v, key, False)}
    if isinstance(x, str) and key not in NOT_NAMES:
        return set(re.findall(r"(?<![A-Za-z0-9_.])[A-Za-z_][A-Za-z0-9_]*", x))
    return set()


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.stem)
def test_every_element_of_the_examples(path):
    """Renamed, the model is valid and the old name is nowhere left (but in comments)."""
    text = path.read_text(encoding="utf-8")
    model = yaml.safe_load(text)
    before = words(model)
    for section, old in names(model):
        new = f"{old}_renamed"
        out, n, where = rename_text(text, old, new, section)
        assert where == section and n >= 1
        m = yaml.safe_load(out)
        validate(m, path.parent)
        assert new in m[section] and old not in m[section]
        # the old name is nowhere left (unless another element has it too), and nothing else changed
        elsewhere = {old} if any(o == old for s, o in names(model) if s != section) else set()
        assert words(m) == (before - {old}) | {new} | elsewhere, (section, old)


MODEL = """\
model: m  # rod is the arm
gravity: [0, 0, -9.81]
parameters:
  k: 100.0
  k2: '=2 * k'   # k twice
points:
  pivot: [0, 0, 0]
  tip: {from: pivot, offset: [1, 0, 0]}
frames:
  hinge_frame: {origin: pivot, z: [pivot, tip]}
bodies:
  rod: {mass: 1, inertia: [0.01, 0.08, 0.08], position: [0.5, 0, 0], orientation: hinge_frame}
measures:
  phi: {type: angle, body1: rod, orientation: hinge_frame}
  x: {type: displacement, body1: rod, point1: tip, component: x}
joints:
  hinge: {type: revolute, body1: rod, location: pivot, orientation: hinge_frame}
forces:
  spring: {type: expression_torque, body1: rod, axis: [0, 1, 0], torque: "-k * phi - 0.1 * x"}
drivers: {}
outputs:
  angle: phi * 180 / pi
checks:
  values:
    - {channel: rod.R_z, value: 0, tolerance: 1}
    - {channel: rodx.R_z, value: 0, tolerance: 1}
solver: {type: HHT, end_time: 1.0, step: 1.0e-3}
"""


def test_references_and_nothing_else():
    out, n, section = rename_text(MODEL, "rod", "arm")
    assert section == "bodies" and n == 6
    assert "# rod is the arm" in out and "rodx.R_z" in out and "channel: arm.R_z" in out
    out, n, _ = rename_text(MODEL, "k", "stiffness")
    assert "'=2 * stiffness'   # k twice" in out and '"-stiffness * phi - 0.1 * x"' in out and "k2:" in out
    out, n, _ = rename_text(MODEL, "phi", "angle_rod")
    assert "angle: angle_rod * 180 / pi" in out and n == 3
    out, _, _ = rename_text(MODEL, "x", "travel")
    assert "component: x" in out and "0.1 * travel" in out and "z: [pivot, tip]" in out
    out, n, _ = rename_text(MODEL, "pivot", "origin")
    assert n == 5 and "location: origin" in out and "z: [origin, tip]" in out
    out, n, _ = rename_text(MODEL, "hinge_frame", "hf")
    assert n == 4


def test_errors():
    with pytest.raises(ModelError, match="no element named nope"):
        rename_text(MODEL, "nope", "x2")
    with pytest.raises(ModelError, match="already used in measures"):
        rename_text(MODEL, "rod", "phi")
    for bad in ("ground", "2x", "a-b", "t"):
        with pytest.raises(ModelError, match="not a name"):
            rename_text(MODEL, "rod", bad)


def test_file_crlf_and_invalid_result(tmp_path):
    p = tmp_path / "m.yaml"
    p.write_bytes((ROOT / "examples" / "pendulum.yaml").read_text().replace("\n", "\r\n").encode())
    assert rename(p, "rod", "arm") == (2, "bodies")
    assert b"body1: arm" in p.read_bytes() and b"\r\n" in p.read_bytes() and b"\n\n" not in p.read_bytes()
    assert cli.main(["rename", str(p), "hinge", "pivot"]) == 0
    assert "pivot:" in p.read_text()


def test_changes_for_the_editor():
    from fdgen.rename import changes

    edits, n, section = changes(MODEL, "rod", "arm")
    assert n == 6 and section == "bodies" and all(MODEL[c["from"]:c["to"]] == "rod" for c in edits)
    text = MODEL
    for c in reversed(edits):
        text = text[:c["from"]] + c["insert"] + text[c["to"]:]
    assert text == rename_text(MODEL, "rod", "arm")[0]


def test_renamed_by_hand():
    """The key of an element with an id changed by hand: the references left are found by the id."""
    from fdgen.rename import changes, renamed

    before = MODEL.replace("  rod: {mass: 1,", "  rod: {id: onshape:body:M1, mass: 1,")
    after = before.replace("  rod: {id:", "  arm: {id:")
    assert renamed(before, after) == [{"section": "bodies", "id": "onshape:body:M1", "old": "rod", "new": "arm",
                                       "references": 5}]
    edits, n, _ = changes(after, "rod", "arm", "bodies", references_only=True)
    assert n == 5
    assert renamed(before, before) == [] and renamed(before, "not: [yaml") == []
    # without id nothing can be told: a deleted element and a new one
    assert renamed(MODEL, MODEL.replace("  rod: {mass", "  arm: {mass")) == []
