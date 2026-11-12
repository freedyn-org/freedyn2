"""Element ids (the id key): optional, unique in the model, not seen by the generator."""

import pytest
import yaml

from conftest import ROOT
from fdgen.model import ModelError, element_ids, validate

PENDULUM = ROOT / "examples" / "pendulum.yaml"


def pendulum():
    return yaml.safe_load(PENDULUM.read_text())


def test_ids_are_collected_and_stripped():
    m = pendulum()
    m["bodies"]["rod"]["id"] = "onshape:M1/M2"
    m["bodies"]["rod"]["shapes"] = [{"type": "sphere", "radius": 0.1, "id": "onshape:M1/M2/shape"}]
    m["joints"]["hinge"]["id"] = "fds:constraint:1"
    v = validate(m, PENDULUM.parent)
    assert v["ids"] == {"onshape:M1/M2": ["bodies", "rod"], "onshape:M1/M2/shape": ["bodies", "rod", 0],
                        "fds:constraint:1": ["joints", "hinge"]}
    assert "id" not in v["bodies"]["rod"] and "id" not in v["bodies"]["rod"]["shapes"][0]
    assert "id" in m["bodies"]["rod"]  # the model given is not changed


def test_ids_are_optional():
    assert validate(pendulum(), PENDULUM.parent)["ids"] == {}


def test_an_id_used_twice_is_an_error():
    m = pendulum()
    m["bodies"]["rod"]["id"] = m["joints"]["hinge"]["id"] = "x1"
    with pytest.raises(ModelError, match="id x1 used twice: bodies rod and joints hinge"):
        validate(m, PENDULUM.parent)
    with pytest.raises(ModelError, match="used twice"):
        element_ids(m)


@pytest.mark.parametrize("bad", ["", "=k", "a b", "-x"])
def test_id_pattern(bad):
    m = pendulum()
    m["bodies"]["rod"]["id"] = bad
    with pytest.raises(ModelError):  # "=k" is taken for a formula first
        validate(m, PENDULUM.parent)


LONG = """\
model: m
gravity: [0, 0, -9.81]
parameters:
  k: {value: 100.0, id: p:k}
  k2: {value: '=2 * k', id: p:k2}
inputs:
  tq: {value: 0.0, id: i:tq}
points:
  pivot: {value: [0, 0, 0], id: pt:pivot}
  tip: {from: pivot, offset: [1, 0, 0], id: pt:tip}
bodies:
  rod: {mass: 1, inertia: [0.01, 0.08, 0.08], position: [0.5, 0, 0]}
measures:
  phi: {type: angle, body1: rod, orientation: [0.7071067811865476, -0.7071067811865476, 0, 0]}
joints:
  hinge: {type: revolute, body1: rod, location: pivot, orientation: [0.7071067811865476, -0.7071067811865476, 0, 0]}
forces:
  spring: {type: expression_torque, body1: rod, axis: [0, 1, 0], torque: "-k2 * phi + tq"}
outputs:
  angle: {value: phi * 180 / pi, id: o:angle}
solver: {type: HHT, end_time: 1.0, step: 1.0e-3}
"""


def short(text):
    """The same model in the short forms."""
    for a, b in (("{value: 100.0, id: p:k}", "100.0"), ("{value: '=2 * k', id: p:k2}", "'=2 * k'"),
                 ("{value: 0.0, id: i:tq}", "0.0"), ("{value: [0, 0, 0], id: pt:pivot}", "[0, 0, 0]"),
                 (", id: pt:tip}", "}"), ("{value: phi * 180 / pi, id: o:angle}", "phi * 180 / pi")):
        text = text.replace(a, b)
    return text


def test_long_forms_are_the_short_ones_with_an_id():
    v = validate(yaml.safe_load(LONG), PENDULUM.parent)
    w = validate(yaml.safe_load(short(LONG)), PENDULUM.parent)
    assert v["ids"] == {"p:k": ["parameters", "k"], "p:k2": ["parameters", "k2"], "i:tq": ["inputs", "tq"],
                        "pt:pivot": ["points", "pivot"], "pt:tip": ["points", "tip"], "o:angle": ["outputs", "angle"]}
    assert {k: x for k, x in v.items() if k != "ids"} == {k: x for k, x in w.items() if k != "ids"}
    assert v["parameters"] == {"k": 100.0, "k2": 200.0} and v["outputs"] == {"angle": "phi * 180 / pi"}


def test_long_form_has_value_and_id_only():
    m = yaml.safe_load(LONG)
    m["parameters"]["k"]["unit"] = "N/m"
    with pytest.raises(ModelError, match="parameters.k: the long form has value and id only, not unit"):
        validate(m, PENDULUM.parent)


def test_long_forms_are_renamed_and_tuned():
    from fdgen.rename import rename_text, renamed
    from fdgen.view.server import apply_overrides

    out, n, _ = rename_text(LONG, "phi", "angle_rod")
    assert "angle: {value: angle_rod * 180 / pi, id: o:angle}" in out and n == 3
    out, n, _ = rename_text(LONG, "k", "stiffness")
    assert "{value: '=2 * stiffness', id: p:k2}" in out
    assert renamed(LONG, LONG.replace("  k2: {", "  k_double: {")) == [
        {"section": "parameters", "id": "p:k2", "old": "k2", "new": "k_double", "references": 1}]
    m = apply_overrides(yaml.safe_load(LONG), {"parameters.k": 50.0, "parameters.k2.value": 7.0})
    assert m["parameters"]["k"] == {"value": 50.0, "id": "p:k"} and m["parameters"]["k2"]["value"] == 7.0
    assert validate(m, PENDULUM.parent)["parameters"] == {"k": 50.0, "k2": 7.0}
