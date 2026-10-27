"""Formulas in number fields (fdgen.formulas): "=k_susp", "=0.8 * k_front", in any section."""

import math

import pytest
import yaml

from fdgen.checks import unused_parameters
from fdgen.formulas import resolve
from fdgen.model import ModelError, validate

SOLVER = {"type": "HHT", "end_time": 0.1, "step": 1e-3}


def model(**extra):
    m = {"model": "m", "solver": SOLVER, "parameters": {"m0": 2.0, "k": 100.0, "h": 0.5},
         "bodies": {"b": {"mass": "=m0", "inertia": [1, 1, 1], "position": [0, 0, "=h + 0.25"]}},
         "forces": {"s": {"type": "spring_damper", "body1": "b", "point1": [0, 0, "=h"], "point2": [0, 0, 0],
                          "stiffness": "=2 * k", "damping": "=sqrt(k * m0) / pi"}}}
    m.update(extra)
    return m


def test_formulas_become_numbers():
    v = validate(model())
    b, (s,) = v["bodies"]["b"], v["forces"]
    assert b["mass"] == 2.0 and list(b["position"]) == [0, 0, 0.75]
    assert s["stiffness"] == 200.0 and s["damping"] == pytest.approx(math.sqrt(200.0) / math.pi)
    assert v["parameter_refs"] == ["h", "k", "m0"]


def test_parameters_from_parameters_and_unused():
    m = model()
    m["parameters"].update({"k_rear": "=0.8 * k_front", "k_front": "=k / 2", "spare": 1.0})
    m["forces"]["s"]["stiffness"] = "=k_rear"
    v = validate(m)
    assert v["parameters"]["k_front"] == 50.0 and v["forces"][0]["stiffness"] == 40.0
    assert unused_parameters(v) == ["spare"]


def test_the_given_model_is_not_changed():
    m = model()
    resolve(m)
    assert m["forces"]["s"]["stiffness"] == "=2 * k"


def test_yaml_without_quotes():
    text = "a: =k\nb: [1, 2, =h + 0.25]\n"
    assert yaml.safe_load(text) == {"a": "=k", "b": [1, 2, "=h + 0.25"]}


@pytest.mark.parametrize("change, message", [
    (lambda m: m["bodies"]["b"].update(mass="=m1"), "bodies.b.mass: unknown name 'm1'"),
    (lambda m: m["bodies"]["b"].update(mass="=sin(t)"), "force expressions, written without ="),
    (lambda m: m["parameters"].update(a="=b", b="=a"), "formulas that use each other"),
    (lambda m: m["parameters"].update(a="soft"), "neither a number nor a formula"),
    (lambda m: m["bodies"]["b"].update(mass=None), 'a "#" in YAML starts a comment'),
])
def test_errors(change, message):
    m = model()
    change(m)
    with pytest.raises(ModelError, match=message.replace("(", r"\(").replace(")", r"\)")):
        validate(m)


def test_tuning_a_parameter_moves_its_formulas(tmp_path):
    """The viewer's sliders set parameters; every number that is a formula of them follows."""
    from fdgen.view.scene import attach_sources, scene
    from fdgen.view.server import apply_overrides

    m = model()
    assert validate(apply_overrides(model(), {"parameters.k": 300.0}))["forces"][0]["stiffness"] == 600.0
    path = tmp_path / "m.yaml"
    path.write_text(yaml.safe_dump(m))
    s = scene(validate(m))
    attach_sources(s, path)
    t = {x["path"]: x for x in s["forces"][0]["tunable"]}
    assert t["forces.s.stiffness"] == {"path": "forces.s.stiffness", "formula": "=2 * k", "uses": ["k"]}
    assert t["forces.s.point1.2"]["uses"] == ["h"] and t["forces.s.point2.2"]["value"] == 0.0
