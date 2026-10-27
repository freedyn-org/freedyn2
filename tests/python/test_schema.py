import copy

import pytest

from conftest import ROOT
from fdgen.model import ModelError, load_yaml, validate, write_fdm

EXAMPLES = sorted((ROOT / "examples").glob("*.yaml"))


@pytest.fixture
def base():
    return load_yaml(ROOT / "examples" / "free_body.yaml")


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.name)
def test_examples_validate(path):
    validate(load_yaml(path), path.parent)


def test_defaults_filled(base):
    del base["bodies"]["body1"]["orientation"], base["solver"]["alpha"]
    m = validate(base)
    assert m["bodies"]["body1"]["orientation"] == [1.0, 0.0, 0.0, 0.0]
    assert m["solver"]["alpha"] == -0.1


@pytest.mark.parametrize(
    "mutate",
    [
        lambda m: m["bodies"]["body1"].pop("mass"),
        lambda m: m["bodies"]["body1"].update(mass=-1.0),
        lambda m: m["bodies"]["body1"].update(inertia=[0.1, 0.2]),
        lambda m: m["bodies"]["body1"].update(colour="red"),
        lambda m: m["solver"].update(alpha=-0.5),
        lambda m: m["solver"].update(type="RK4"),
        lambda m: m["solver"].update(output_step=1.5e-3),
        lambda m: m.update(joints={"j": {"type": "revolute", "body1": "nobody"}}),
        lambda m: m.update(joints={"j": {"type": "revolute", "body1": "ground", "body2": "body1"}}),
        lambda m: m.update(joints={"j": {"type": "generic", "body1": "body1"}}),
        lambda m: m.update(joints={"j": {"type": "hinge", "body1": "body1"}}),
        lambda m: m["bodies"]["body1"].update(orientation=[0, 0, 0, 0]),
    ],
)
def test_invalid_rejected(base, mutate):
    m = copy.deepcopy(base)
    mutate(m)
    with pytest.raises(ModelError):
        validate(m)


def test_orientation_normalized(base):
    base["bodies"]["body1"]["orientation"] = [2.0, 0.0, 0.0, 0.0]
    with pytest.warns(UserWarning):
        m = validate(base)
    assert m["bodies"]["body1"]["orientation"] == [1.0, 0.0, 0.0, 0.0]


def test_fdm_file(base, tmp_path):
    write_fdm(validate(base), tmp_path / "m.fdm")
    lines = (tmp_path / "m.fdm").read_text().splitlines()
    assert lines[0] == "freedyn2_model 1"
    assert "coords 0 1 2 3 4 5 6" in lines and "dofs 0 1 2 3 4 5" in lines and "cons " in lines
    assert "u 0.1 0.0 0.0 0.5 2.0 0.1" in lines
