"""freedyn2 import-onshape (fdgen.onshape) against stand-in API responses.

The responses have the structure of the Onshape REST API v6 (assembly definition with mate
features, part mass properties, STL export), built by hand: no network, no API keys. The
pendulum assembly is examples/pendulum.yaml modelled in Onshape (the rod drawn along z in its
part studio, turned onto global x by its occurrence transform); imported, it must move as the
example does.
"""

import csv
import struct

import numpy as np
import pytest
import yaml

from conftest import ROOT
from fdgen import cli, onshape
from fdgen.errors import ModelError
from fdgen.geometry import rotation_matrix
from fdgen.model import validate

DID, WID, EID = "a" * 24, "b" * 24, "c" * 24
URL = f"https://cad.onshape.com/documents/{DID}/w/{WID}/e/{EID}"
STL = b"\0" * 80 + struct.pack("<I", 1) + struct.pack("<12fH", *([0.0] * 3 + [0, 0, 0, 1, 0, 0, 0, 1, 0]), 0)
RY90 = np.array([[0.0, 0, 1], [0, 1, 0], [-1, 0, 0]])  # about y by +90 deg: part z -> global x


def cs(origin, x, z):
    x, z = np.asarray(x, float), np.asarray(z, float)
    return {"origin": list(origin), "xAxis": x.tolist(), "yAxis": np.cross(z, x).tolist(), "zAxis": z.tolist()}


def transform(R=np.eye(3), p=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T.reshape(-1).tolist()


def instance(iid, name, part_id, element="e" * 24, kind="Part"):
    return {"id": iid, "name": name, "type": kind, "documentId": DID, "documentMicroversion": "f" * 24,
            "elementId": element, "partId": part_id, "fullConfiguration": "default", "suppressed": False}


def mate(fid, name, mate_type, *entities):
    return {"id": fid, "featureType": "mate", "suppressed": False, "featureData": {
        "name": name, "mateType": mate_type,
        "matedEntities": [{"matedOccurrence": list(o), "matedCS": c} for o, c in entities]}}


class Client:
    """Stand-in for OnshapeClient: the assembly definition, mass properties by part id, STL."""

    def __init__(self, assembly, mass, name="Pendulum"):
        self.assembly, self.mass, self.name, self.calls = assembly, mass, name, []

    def get_json(self, path, params=None):
        self.calls.append(path)
        if path.startswith("/assemblies/"):
            return self.assembly
        if path.startswith("/documents/"):
            return [{"name": self.name}]
        if path.endswith("/massproperties"):
            pid = path.split("/partid/")[1].split("/")[0]
            return {"bodies": {pid: self.mass[pid]}}
        raise AssertionError(path)

    def get_bytes(self, path, params=None):
        self.calls.append(path)
        assert path.endswith("/stl") and params["units"] == "meter"
        return STL


def props(mass, centroid, moments, axes=np.eye(3)):
    return {"hasMass": mass > 0, "mass": [mass, mass, mass], "centroid": list(centroid) * 3,
            "principalInertia": list(moments),
            "principalAxes": [{"x": a[0], "y": a[1], "z": a[2]} for a in np.asarray(axes, float).T]}


def pendulum():
    """examples/pendulum.yaml in Onshape: a fixed base, the rod (along z in its part studio)."""
    hinge_global = cs([0, 0, 0], x=[1, 0, 0], z=[0, 1, 0])
    hinge_rod = cs([0, 0, 0], x=RY90.T @ [1, 0, 0], z=RY90.T @ [0, 1, 0])  # the same frame, rod coordinates
    root = {"instances": [instance("I1", "Base <1>", "JB"), instance("I2", "Rod <1>", "JR")],
            "occurrences": [{"path": ["I1"], "transform": transform(), "fixed": True},
                            {"path": ["I2"], "transform": transform(RY90), "fixed": False}],
            "features": [mate("F1", "Revolute 1", "REVOLUTE", (["I2"], hinge_rod), (["I1"], hinge_global))],
            "patterns": []}
    mass = {"JB": props(0.0, [0, 0, 0], [0, 0, 0]),
            "JR": props(1.0, [0, 0, 0.5], [0.0833333333333333, 0.0833333333333333, 0.01])}
    return {"rootAssembly": root, "subAssemblies": [], "parts": []}, mass


def test_parse_url():
    assert onshape.parse_url(URL + "?configuration=List_1%3Da") == (
        "https://cad.onshape.com", DID, "w", WID, EID, "List_1=a")
    with pytest.raises(ModelError, match="not an Onshape assembly URL"):
        onshape.parse_url("https://cad.onshape.com/documents/xyz")


def test_api_keys_required(monkeypatch):
    monkeypatch.delenv("ONSHAPE_ACCESS_KEY", raising=False)
    with pytest.raises(ModelError, match="ONSHAPE_ACCESS_KEY"):
        onshape.OnshapeClient.from_env()


def test_pendulum(tmp_path):
    out, warnings = onshape.import_onshape(URL, tmp_path, client=Client(*pendulum()))
    assert out == tmp_path / "pendulum.yaml" and warnings == []
    m = yaml.safe_load(out.read_text())
    rod = m["bodies"]["rod_1"]
    assert rod["mass"] == 1.0
    assert rod["position"] == pytest.approx([0.5, 0, 0])
    A = np.asarray(rotation_matrix(rod["orientation"]))
    # the principal axis of the small moment (the rod's axis) is global x
    assert abs(A[:, int(np.argmin(rod["inertia"]))] @ [1, 0, 0]) == pytest.approx(1)
    hinge = m["joints"]["revolute_1"]
    assert hinge["type"] == "revolute" and hinge["body1"] == "rod_1" and "body2" not in hinge
    assert hinge["location"] == pytest.approx([0, 0, 0])
    Aj = rotation_matrix(hinge["orientation"])
    assert np.asarray(Aj)[:, 2] == pytest.approx([0, 1, 0])
    assert len(m["ground"]["shapes"]) == 1 and len(rod["shapes"]) == 1
    assert all((tmp_path / s["file"]).is_file() for s in rod["shapes"])
    refs = (tmp_path / "pendulum.onshape.json").read_text()
    assert '"partId": "JR"' in refs and DID in refs
    validate(m, tmp_path)


def test_pendulum_moves_as_the_example(tmp_path):
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    out, _ = onshape.import_onshape(URL, tmp_path, client=Client(*pendulum()), meshes=False)
    for model, csv_path in ((out, tmp_path / "a.csv"), (ROOT / "examples" / "pendulum.yaml", tmp_path / "b.csv")):
        assert cli.main(["run", str(model), "-o", str(csv_path), "--", "--end-time", "2"]) == 0
    a, b = (list(csv.DictReader(open(p))) for p in (tmp_path / "a.csv", tmp_path / "b.csv"))
    for axis in "xyz":
        xa = np.array([float(r[f"rod_1.R_{axis}"]) for r in a])
        xb = np.array([float(r[f"rod.R_{axis}"]) for r in b])
        assert xa == pytest.approx(xb, abs=1e-6)


def mechanism():
    """A crank on a fixed frame, with a massless screw fastened to it, a slider sub-assembly
    (carriage and a block in a mate group) on a rail, a Tangent mate and a gear relation."""
    sub_el = "d" * 24
    root = {"instances": [instance("I1", "Frame <1>", "JF"), instance("I2", "Crank <1>", "JC"),
                          instance("I3", "Screw <1>", "JS"),
                          {**instance("I4", "Slide <1>", "", element=sub_el, kind="Assembly"), "partId": None}],
            "occurrences": [{"path": ["I1"], "transform": transform(), "fixed": True},
                            {"path": ["I2"], "transform": transform(p=(0, 0, 1)), "fixed": False},
                            {"path": ["I3"], "transform": transform(p=(0.2, 0, 1)), "fixed": False},
                            {"path": ["I4"], "transform": transform(p=(1, 0, 0)), "fixed": False},
                            {"path": ["I4", "S1"], "transform": transform(p=(1, 0, 0)), "fixed": False},
                            {"path": ["I4", "S2"], "transform": transform(p=(1, 0, 0.1)), "fixed": False}],
            "features": [
                mate("F1", "Revolute 1", "REVOLUTE", (["I2"], cs([0, 0, 0], [1, 0, 0], [0, 0, 1])),
                     (["I1"], cs([0, 0, 1], [1, 0, 0], [0, 0, 1]))),
                mate("F2", "Fastened 1", "FASTENED", (["I3"], cs([0, 0, 0], [1, 0, 0], [0, 0, 1])),
                     (["I2"], cs([0.2, 0, 0], [1, 0, 0], [0, 0, 1]))),
                mate("F3", "Slider 1", "SLIDER", (["I4", "S1"], cs([0, 0, 0], [0, 0, 1], [1, 0, 0])),
                     (["I1"], cs([1, 0, 0], [0, 0, 1], [1, 0, 0]))),
                mate("F4", "Tangent 1", "TANGENT", (["I2"], cs([0, 0, 0], [1, 0, 0], [0, 0, 1])),
                     (["I4", "S1"], cs([0, 0, 0], [1, 0, 0], [0, 0, 1]))),
                {"id": "F5", "featureType": "mateRelation", "featureData": {"name": "Gear 1", "relationType": "GEAR"}}],
            "patterns": []}
    sub = {"documentId": DID, "documentMicroversion": "f" * 24, "elementId": sub_el, "fullConfiguration": "default",
           "instances": [instance("S1", "Carriage <1>", "JK"), instance("S2", "Block <1>", "JL")],
           "features": [{"id": "G1", "featureType": "mateGroup", "featureData": {
               "name": "Mate group 1", "occurrences": [{"occurrence": ["S1"]}, {"occurrence": ["S2"]}]}}],
           "patterns": []}
    mass = {"JF": props(0.0, [0, 0, 0], [0, 0, 0]), "JC": props(2.0, [0.1, 0, 0], [0.001, 0.01, 0.01]),
            "JS": props(0.0, [0, 0, 0], [0, 0, 0]), "JK": props(1.0, [0, 0, 0], [0.002, 0.002, 0.002]),
            "JL": props(1.0, [0, 0, 0], [0.001, 0.001, 0.001])}
    return {"rootAssembly": root, "subAssemblies": [sub], "parts": []}, mass


def test_mechanism(tmp_path):
    out, warnings = onshape.import_onshape(URL, tmp_path / "m.yaml", client=Client(*mechanism(), name="Mechanism"))
    m = yaml.safe_load(out.read_text())
    assert set(m["bodies"]) == {"crank_1", "carriage_1"}  # the screw is part of the crank, the block of the carriage
    crank, carriage = m["bodies"]["crank_1"], m["bodies"]["carriage_1"]
    assert crank["mass"] == 2.0 and crank["position"] == pytest.approx([0.1, 0, 1])
    # carriage + block: 2 kg, center between them, the parallel axis theorem about x and y
    assert carriage["mass"] == 2.0 and carriage["position"] == pytest.approx([1, 0, 0.05])
    assert sorted(carriage["inertia"]) == pytest.approx(sorted([0.003, 0.003 + 2 * 0.05 ** 2, 0.003 + 2 * 0.05 ** 2]))
    assert len(crank["shapes"]) == 2 and len(carriage["shapes"]) == 2
    j = m["joints"]
    assert set(j) == {"revolute_1", "slider_1"}
    assert j["revolute_1"]["body1"] == "crank_1" and j["revolute_1"]["location"] == pytest.approx([0, 0, 1])
    assert j["slider_1"]["type"] == "prismatic" and j["slider_1"]["body1"] == "carriage_1"
    assert j["slider_1"]["location"] == pytest.approx([1, 0, 0])
    assert any("Tangent 1" in w for w in warnings) and any("Gear 1" in w for w in warnings)
    text = out.read_text()
    assert "Tangent 1" in text and "Screw <1>" in text  # listed in the file
    validate(m, tmp_path)


def test_massless_body_is_an_error(tmp_path):
    asm, mass = pendulum()
    mass["JR"] = props(0.0, [0, 0, 0], [0, 0, 0])
    with pytest.raises(ModelError, match="no mass.*Rod <1>"):
        onshape.import_onshape(URL, tmp_path, client=Client(asm, mass))
    assert not (tmp_path / "pendulum.yaml").exists()


def test_output_must_be_yaml(tmp_path):
    with pytest.raises(ModelError, match=r"end in \.yaml"):
        onshape.import_onshape(URL, tmp_path / "m.yam", client=Client(*pendulum()))


def test_existing_file_needs_force(tmp_path):
    (tmp_path / "pendulum.yaml").write_text("x")
    with pytest.raises(ModelError, match="exists"):
        onshape.import_onshape(URL, tmp_path, client=Client(*pendulum()))
    onshape.import_onshape(URL, tmp_path, client=Client(*pendulum()), force=True)


def test_record(tmp_path):
    asm, mass = pendulum()
    mass["JR"] = props(0.0, [0, 0, 0], [0, 0, 0])
    with pytest.raises(ModelError):
        onshape.import_onshape(URL, tmp_path, client=Client(asm, mass), record=tmp_path / "rec")
    saved = sorted((tmp_path / "rec").glob("*.json"))
    assert len(saved) == 4 and '"Rod <1>"' in saved[1].read_text()


def test_names():
    """A part name used in several sub-assemblies gets their names in front; unique names stay short."""
    assert onshape._names(["arm <2> / upper_arm <1>", "arm <3> / upper_arm <1>", "arm <1> / upper_arm <1>",
                           "tool_base <1>", "Ground", "4 bolt"]) == [
        "arm_2_upper_arm_1", "arm_3_upper_arm_1", "arm_1_upper_arm_1", "tool_base_1", "ground_2", "part_4_bolt"]
    assert onshape._names(["x <1>", "a / x <1>"]) == ["x_1", "a_x_1"]
