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
