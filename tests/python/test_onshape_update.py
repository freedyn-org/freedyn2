"""freedyn2 import-onshape --update (fdgen.onshape_update): imported again, the person's changes kept."""

import copy

import pytest
import yaml

from fdgen import onshape
from fdgen.errors import ModelError
from fdgen.model import validate
from fdgen.rename import rename
from test_onshape import RY90, URL, Client, cs, instance, mate, pendulum, props, transform, mechanism


def double(asm, mass):
    """The pendulum with a second rod hinged at the end of the first (a new part and a new mate)."""
    asm, mass = copy.deepcopy(asm), dict(mass)
    root = asm["rootAssembly"]
    root["instances"].append(instance("I3", "Rod2 <1>", "JR2"))
    root["occurrences"].append({"path": ["I3"], "transform": transform(RY90, (1, 0, 0)), "fixed": False})
    hinge = cs([0, 0, 0], x=RY90.T @ [1, 0, 0], z=RY90.T @ [0, 1, 0])
    root["features"].append(mate("F2", "Revolute 2", "REVOLUTE", (["I3"], hinge),
                                 (["I2"], cs([0, 0, 1], x=RY90.T @ [1, 0, 0], z=RY90.T @ [0, 1, 0]))))
    mass["JR2"] = props(0.5, [0, 0, 0.5], [0.04, 0.04, 0.005])
    return asm, mass


def imported(tmp_path, asm, mass, **kw):
    out, warnings = onshape.import_onshape(URL, tmp_path / "p.yaml", client=Client(asm, mass), **kw)
    return out


def again(path, asm, mass, **kw):
    report = {}
    out, warnings = onshape.import_onshape(URL, path, client=Client(asm, mass), update=True, report=report, **kw)
    return report, warnings


def edit(path):
    """What a person does to an imported model: a new name, a spring, a comment, other solver settings."""
    rename(path, "rod_1", "pendel")
    text = path.read_text().replace("solver: {type: HHT, alpha: -0.1, end_time: 5.0,",
                                    "solver: {type: HHT, alpha: -0.1, end_time: 2.0,")
    text = text.replace("joints:\n", "# my comment about the joints\njoints:\n")
    text += ("forces:\n  spring:  # mine\n    {type: torsion_spring, body1: pendel, orientation: [0.7071067811865476, "
             "-0.7071067811865476, 0, 0], stiffness: 5}\n")
    path.write_text(text)


def test_nothing_changed_in_onshape_changes_nothing(tmp_path):
    asm, mass = pendulum()
    p = imported(tmp_path, asm, mass)
    first = p.read_text()
    report, _ = again(p, asm, mass)
    assert p.read_text() == first and report["updated"] == [] and report["unchanged"] == ["rod_1", "revolute_1"]
    edit(p)
    mine = p.read_text()
    report, warnings = again(p, asm, mass)
    assert p.read_text() == mine and warnings == [] and not report["added"] and not report["removed"]


def test_changes_in_onshape_and_mine(tmp_path):
    asm, mass = pendulum()
    p = imported(tmp_path, asm, mass)
    edit(p)
    mass["JR"] = props(2.0, [0, 0, 0.5], [0.17, 0.17, 0.02])  # a heavier rod
    asm, mass = double(asm, mass)  # and a second one
    report, warnings = again(p, asm, mass)
    text = p.read_text()
    m = yaml.safe_load(text)
    assert report["updated"] == ["pendel"] and report["unchanged"] == ["revolute_1"]
    assert report["added"] == ["rod2_1", "revolute_2"]
    assert m["bodies"]["pendel"]["mass"] == 2.0 and m["bodies"]["pendel"]["id"] == "onshape:body:I2"
    assert m["bodies"]["rod2_1"]["position"] == pytest.approx([1.5, 0, 0])
    assert m["joints"]["revolute_1"]["body1"] == "pendel"
    assert m["joints"]["revolute_2"]["body2"] == "pendel"  # the new joint knows the new name
    assert "# my comment about the joints" in text and "spring:  # mine" in text and "end_time: 2.0" in text
    assert m["forces"]["spring"]["body1"] == "pendel"
    validate(m, tmp_path)
    # the reference file follows the names of the model file
    assert '"pendel"' in (tmp_path / "p.onshape.json").read_text()


def test_removed_in_onshape(tmp_path):
    asm, mass = pendulum()
    two = double(asm, mass)
    p = imported(tmp_path, *two)
    report, warnings = again(p, asm, mass)  # Rod2 deleted in Onshape: the body and its joint go
    m = yaml.safe_load(p.read_text())
    assert sorted(report["removed"]) == ["revolute_2", "rod2_1"] and set(m["bodies"]) == {"rod_1"}
    assert set(m["joints"]) == {"revolute_1"}
    validate(m, tmp_path)
    # with a measure of mine on Rod2, it stays (and the model with it)
    p2 = tmp_path / "q.yaml"
    onshape.import_onshape(URL, p2, client=Client(*two))
    p2.write_text(p2.read_text() + "measures:\n  phi2: {type: angle, body1: rod2_1, body2: rod_1}\n")
    report, warnings = again(p2, asm, mass)
    m = yaml.safe_load(p2.read_text())
    assert "rod2_1" in m["bodies"] and report["kept"] == ["rod2_1"] and report["removed"] == ["revolute_2"]
    assert any("body rod2_1 is no longer in the Onshape assembly" in w for w in warnings)


def test_heaviest_part_changed(tmp_path):
    """The body of carriage and block: its id changes with the heaviest part; found by its parts."""
    asm, mass = mechanism()
    p = tmp_path / "m.yaml"
    onshape.import_onshape(URL, p, client=Client(asm, mass, name="Mechanism"))
    rename(p, "carriage_1", "slide")
    mass["JL"] = props(3.0, [0, 0, 0], [0.003, 0.003, 0.003])
    report = {}
    onshape.import_onshape(URL, p, client=Client(asm, mass), update=True, report=report)
    m = yaml.safe_load(p.read_text())
    assert "slide" in m["bodies"] and m["bodies"]["slide"]["mass"] == 4.0
    assert m["bodies"]["slide"]["id"] == "onshape:body:I4/S2" and not report["added"]
    assert m["joints"]["slider_1"]["body1"] == "slide"


def test_crlf_and_own_shapes(tmp_path):
    asm, mass = pendulum()
    p = imported(tmp_path, asm, mass)
    text = p.read_text().replace("    shapes:  # for the viewer (freedyn2 view) only\n",
                                 "    shapes:  # for the viewer (freedyn2 view) only\n      - {type: sphere, radius: 0.1}\n")
    p.write_bytes(text.replace("\n", "\r\n").encode())
    mass["JR"] = props(3.0, [0, 0, 0.5], [0.2, 0.2, 0.03])
    again(p, asm, mass)
    data = p.read_bytes()
    assert b"\r\n" in data and b"\n" not in data.replace(b"\r\n", b"")
    m = yaml.safe_load(data)
    assert m["bodies"]["rod_1"]["mass"] == 3.0
    assert m["bodies"]["rod_1"]["shapes"][0] == {"type": "sphere", "radius": 0.1} and len(m["bodies"]["rod_1"]["shapes"]) == 2


def test_file_without_ids(tmp_path):
    asm, mass = pendulum()
    p = imported(tmp_path, asm, mass)
    p.write_text("\n".join(x for x in p.read_text().splitlines() if "id: onshape" not in x and "onshape:" not in x))
    with pytest.raises(ModelError, match="no Onshape ids"):
        again(p, asm, mass)


def test_exists_hint(tmp_path):
    asm, mass = pendulum()
    p = imported(tmp_path, asm, mass)
    with pytest.raises(ModelError, match="--update merges"):
        onshape.import_onshape(URL, p, client=Client(asm, mass))


def test_flow_elements_and_keys_that_go(tmp_path):
    asm, mass = double(*pendulum())
    p = imported(tmp_path, asm, mass)
    m = yaml.safe_load(p.read_text())
    j = m["joints"]["revolute_2"]
    flow = "  revolute_2: {" + ", ".join(f"{k}: {v}" for k, v in j.items()) + "}  # in one line\n"
    text = p.read_text()
    start = text.index("  revolute_2:\n")
    end = text.index("solver:")
    p.write_text(text[:start] + flow + text[end:])
    # in Onshape, Rod2 is now hinged to the base instead of the first rod, elsewhere
    f = asm["rootAssembly"]["features"][1]["featureData"]["matedEntities"][1]
    f["matedOccurrence"], f["matedCS"] = ["I1"], cs([1, 0, 0.0], x=[1, 0, 0], z=[0, 1, 0])
    again(p, asm, mass)
    text = p.read_text()
    m = yaml.safe_load(text)
    assert "body2" not in m["joints"]["revolute_2"] and m["joints"]["revolute_2"]["body1"] == "rod2_1"
    assert "# in one line" in text and "  revolute_2: {id: onshape:mate:F2" in text
    validate(m, tmp_path)
