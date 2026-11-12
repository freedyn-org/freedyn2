"""freedyn2 convert: every .fds model of the repository as YAML, run against the .fds itself.

For each model and profile: convert (into another directory than the .fds, so data files are
referenced relative to the YAML file), validate the YAML against the schema, run the .fds and
the YAML with freedyn2 run and compare the result CSVs channel by channel (renamed elements
mapped), and check the YAML with freedyn2 check.
"""

import csv
import math

import jsonschema
import numpy as np
import pytest
import yaml

from conftest import ROOT
from fdgen import cli
from fdgen.convert import convert, fds_to_yaml, number
from fdgen.model import ModelError, load_schema

MODELS = sorted(p for p in ROOT.joinpath("examples").rglob("*.fds"))
# The joint frames are written as quaternions and turned back into rotation matrices, which
# changes them at round-off (1e-16); everything else is the same number.
TOLERANCE = 1e-9


def _runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def _read(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    return rows[0], np.array(rows[1:], dtype=float)


def _channel(name, renames):
    """The YAML channel of a .fds channel (element.rest): the element renamed as convert did."""
    for kind in ("bodies", "joints", "forces", "drivers"):
        for old, new in renames[kind].items():
            if name.startswith(old + "."):
                return new + name[len(old):]
    return name


@pytest.mark.parametrize("profile", ["v2", "legacy"])
@pytest.mark.parametrize("path", MODELS, ids=lambda p: p.relative_to(ROOT / "examples").as_posix())
def test_converted_model_runs_as_the_fds(path, profile, tmp_path, capsys):
    _runtime()
    out = tmp_path / "yaml" / f"{path.stem}.yaml"
    out.parent.mkdir()
    assert cli.main(["convert", str(path), "-o", str(out), "--profile", profile]) == 0
    raw = yaml.safe_load(out.read_text(encoding="utf-8"))
    jsonschema.validate(raw, load_schema())
    assert out.read_text(encoding="utf-8").startswith(f"# {raw['model']}: converted from {path.name}")
    for spline in (raw.get("splines") or {}).values():
        if "file" in spline:
            assert not spline["file"].startswith("/") and (out.parent / spline["file"]).is_file()

    for model, csv_path in ((path, tmp_path / "fds.csv"), (out, tmp_path / "yaml.csv")):
        assert cli.main(["run", str(model), "-o", str(csv_path), "--profile", profile]) == 0
    head_fds, fds = _read(tmp_path / "fds.csv")
    head_yaml, data = _read(tmp_path / "yaml.csv")
    renames = fds_to_yaml(path, out, profile)[1]
    assert [_channel(c, renames) for c in head_fds] == head_yaml
    assert fds.shape == data.shape
    diff = float(np.max(np.abs(fds - data))) if fds.size else 0.0
    assert diff <= TOLERANCE * max(1.0, float(np.max(np.abs(fds))))

    assert cli.main(["check", str(out)]) == cli.main(["check", str(path), "--profile", profile])
    capsys.readouterr()
    print(f"{path.name} {profile}: max |fds - yaml| = {diff:.3g}, bitwise equal: {np.array_equal(fds, data)}")


def test_existing_file_needs_force(tmp_path):
    src = ROOT / "examples" / "compare" / "pendulum.fds"
    out = tmp_path / "pendulum.yaml"
    out.write_text("keep\n")
    with pytest.raises(ModelError, match="exists"):
        convert(src, out)
    assert out.read_text() == "keep\n"
    assert convert(src, out, force=True) == out
    assert convert(src, tmp_path, profile="legacy", force=True) == out  # a directory: <stem>.yaml in it
    assert "element: rigid_body_euler" in out.read_text()


def test_default_output_next_to_the_fds(tmp_path):
    src = tmp_path / "pendulum.fds"
    src.write_bytes((ROOT / "examples" / "compare" / "pendulum.fds").read_bytes())
    assert convert(src) == tmp_path / "pendulum.yaml"


def test_largest_step_need_not_divide_the_output_step(tmp_path):
    """Without SolverMaxTimeStep (1e3) the step size control alone sets the steps."""
    text = (ROOT / "examples" / "compare" / "pendulum.fds").read_text(encoding="latin-1")
    lines = [line for line in text.splitlines() if "SolverMaxTimeStep" not in line]
    src = tmp_path / "pendulum.fds"
    src.write_text("\n".join(lines) + "\n", encoding="latin-1")
    out = convert(src)
    assert yaml.safe_load(out.read_text())["solver"]["step"] == 1000.0
    _runtime()
    for model, name in ((src, "fds.csv"), (out, "yaml.csv")):
        assert cli.main(["run", str(model), "-o", str(tmp_path / name)]) == 0
    a, b = _read(tmp_path / "fds.csv")[1], _read(tmp_path / "yaml.csv")[1]
    assert np.max(np.abs(a - b)) <= TOLERANCE


def test_not_an_fds(tmp_path):
    with pytest.raises(ModelError, match="fds"):
        convert(ROOT / "examples" / "pendulum.yaml", tmp_path / "x.yaml")


def test_unconvertible_names_are_reported(tmp_path):
    text = (ROOT / "examples" / "compare" / "driven_base.fds").read_text(encoding="latin-1")
    src = tmp_path / "m.fds"
    src.write_text(text.replace("Label = k", "Label = kä").replace("PARVAL[k]", "PARVAL[kä]"),
                   encoding="latin-1")
    with pytest.raises(ModelError, match="parameters 'p_kä'"):
        convert(src)


@pytest.mark.parametrize("v", [0.0, -0.0, 1.0, -5.0, 0.1, 1e-6, 1e-14, 3041.1000000000004, 1e20, -2.5e-300,
                               0.7071067811865476])
def test_numbers_read_back_exactly(v):
    for in_list in (False, True):
        back = yaml.safe_load(f"[{number(v, in_list)}]")[0]
        assert float(back) == v and math.copysign(1, float(back)) == math.copysign(1, v)


def test_python_api(tmp_path):
    import freedyn2

    src = ROOT / "examples" / "compare" / "spline_spring.fds"
    out = freedyn2.convert(src, tmp_path / "a.yaml", profile="legacy")
    assert freedyn2.load(out).data["bodies"]["mass"]["element"] == "rigid_body_euler"
    m = freedyn2.load(src, profile="legacy")
    assert m.to_yaml(tmp_path / "b.yaml") == tmp_path / "b.yaml"
    assert (tmp_path / "b.yaml").read_text() == out.read_text()
    with pytest.raises(freedyn2.ModelError, match="exists"):
        m.to_yaml(tmp_path / "b.yaml")
    with pytest.raises(freedyn2.ModelError, match="YAML model already"):
        freedyn2.load(out).to_yaml(tmp_path / "c.yaml")


def test_legacy_ids_become_element_ids(tmp_path):
    """BodyID, ConstraintID, ... are written as the elements' ids (fds:body:1); an id the .fds
    uses twice is left out."""
    from fdgen.convert import _FdsIds
    from fdgen.model import load_yaml, validate

    out = convert(ROOT / "examples" / "compare" / "double_pendulum.fds", tmp_path / "dp.yaml")
    ids = validate(load_yaml(out), tmp_path)["ids"]
    assert ids["fds:body:1"] == ["bodies", "rod1"] and ids["fds:constraint:2"][0] == "joints"
    blocks = {k: [] for k in ("bodies", "joints", "motions", "forces", "measures")}
    blocks["measures"] = [("Measure", "Rotational", {"MeasureID": "4"}), ("Measure", "Rotational", {"MeasureID": "4"}),
                          ("Measure", "Rotational", {"MeasureID": "5"})]
    f = _FdsIds(blocks)
    assert [f.entry(b, "measure") for b in blocks["measures"]] == [[], [], [("id", "fds:measure:5", "")]]
