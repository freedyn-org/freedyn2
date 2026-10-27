"""freedyn2 command line (fdgen.cli), the model plugin cache and the benchmark measures."""

import csv
import sys
from pathlib import Path

import numpy as np
import pytest

from fdgen import cli, plugin

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "legacy"))

import benchmark  # noqa: E402


def _runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


@pytest.mark.parametrize("model", ["examples/compare/pendulum.fds", "examples/forced_oscillator.yaml"])
def test_run(model, tmp_path, capsys):
    _runtime()
    out = tmp_path / "r.csv"
    assert cli.main(["run", str(ROOT / model), "-o", str(out), "--", "--end-time", "0.1"]) == 0
    rows = list(csv.DictReader(open(out)))
    assert float(rows[-1]["t"]) == pytest.approx(0.1)
    assert "build" in capsys.readouterr().out


def test_paths_with_spaces(tmp_path):
    """Model file, plugin and OpenCRG road in a directory with spaces (Windows user directories)."""
    _runtime()
    import numpy as np
    import yaml

    from fdgen.crg import write_crg

    d = tmp_path / "Max Mustermann" / "my models"
    d.mkdir(parents=True)
    write_crg(d / "flat road.crg", 0.5, np.linspace(-3, 3, 7), np.zeros((41, 7)), start=(-5.0, 0.0, 0.0))
    m = yaml.safe_load((ROOT / "examples" / "driven_rotor.yaml").read_text())
    m["model"] = "spaced"
    path = d / "spaced model.yaml"
    path.write_text(yaml.safe_dump(m))
    workdir = d / "work dir"
    out = d / "result file.csv"
    assert cli.main(["run", str(path), "-o", str(out), "--workdir", str(workdir), "--", "--end-time", "0.05"]) == 0
    assert "plugin spaced_plugin" in (workdir / "spaced.fdm").read_text()
    road = {"crg": "flat road.crg"}
    model, _, _ = cli.build(_vehicle_on(road, d), workdir, "v2")
    assert "road_crg " + str(d / "flat road.crg") in model.read_text()
    cli.simulate(cli.find_runtime(), model, d / "vehicle.csv", ["--end-time", "0.01"])


def _vehicle_on(road, d):
    import yaml

    from conftest import car

    m = car(road, 0.1)
    path = d / "vehicle on crg.yaml"
    path.write_text(yaml.safe_dump(m))
    return path


def test_resources_in_the_repository():
    """Without an installed package the schema, the examples and the build come from the
    repository (an installed wheel has them in fdgen/schema, fdgen/examples, fdgen/bin)."""
    from fdgen import resources

    assert resources.path("schema/model.schema.json") == ROOT / "schema" / "model.schema.json"
    assert resources.path("examples/double_pendulum.yaml").is_file()
    assert resources.path("nothing/here").parent.parent == resources.PACKAGE


def test_run_reports_errors(tmp_path, capsys):
    bad = tmp_path / "bad.yaml"
    bad.write_text("model: bad\n")
    assert cli.main(["run", str(bad), "--runtime", __file__]) == 1
    assert "freedyn2:" in capsys.readouterr().err


def test_plugin_cache(tmp_path, monkeypatch):
    src = tmp_path / "f.c"
    src.write_text("double f(double x) { return 2 * x; }\n")
    monkeypatch.setenv("FREEDYN2_PLUGIN_CACHE", str(tmp_path / "cache"))
    plugin.compile_library([src], tmp_path / "a.so")
    cached = list((tmp_path / "cache").glob("*"))
    assert len(cached) == 1
    def no_compiler(cmd):
        raise AssertionError("compiled although cached")

    monkeypatch.setattr(plugin, "_run", no_compiler)
    plugin.compile_library([src], tmp_path / "b.so")
    assert (tmp_path / "b.so").read_bytes() == (tmp_path / "a.so").read_bytes()
    src.write_text("double f(double x) { return 3 * x; }\n")  # other source: compiled
    with pytest.raises(AssertionError):
        plugin.compile_library([src], tmp_path / "c.so")


def _result(t, phase):
    c, s = np.cos(phase), np.sin(phase)
    A = np.zeros((len(t), 3, 3))
    A[:, 0, 0], A[:, 0, 1], A[:, 1, 0], A[:, 1, 1], A[:, 2, 2] = c, -s, s, c, 1
    w = np.zeros((len(t), 3))
    w[:, 2] = 1.0
    return {"t": t, "bodies": [{"R": np.stack([2 * c, 2 * s, 0 * t], 1), "A": A, "omega_body": w,
                                "Rdot": np.stack([-2 * s, 2 * c, 0 * t], 1)}]}


def test_deviations_and_horizon():
    t = np.linspace(0, 10, 101)
    ref = _result(t, t)
    res = _result(t, t + 1e-4 * np.exp(t))  # perturbation growing like an unstable motion
    dev = benchmark.deviations(res, ref, benchmark._scales(ref))
    assert dev.shape == (101, 3)
    delta = 1e-4 * np.exp(t)
    small = delta < 1e-2  # rotation deviation: largest entry change, between delta / sqrt(2) and delta
    assert np.all(dev[small, 1] >= 0.7 * delta[small]) and np.all(dev[small, 1] <= 1.01 * delta[small])
    h = benchmark.horizon(dev, t)
    assert np.max(dev[t <= h]) <= benchmark.HORIZON_TOL < np.max(dev[t <= h + 0.1 + 1e-12])
    assert benchmark.horizon(dev[:10], t[:10]) is None
    with pytest.raises(ValueError):
        benchmark.deviations(_result(t[:50], t[:50]), ref, benchmark._scales(ref))


def test_identical_laws_share_one_element_type(tmp_path):
    """Copies of a subsystem compile each law once (examples/scaling: two cars)."""
    sys.path.insert(0, str(ROOT / "examples" / "scaling"))
    import make_car
    from fdgen.fds import load_fds

    path = tmp_path / "car_2.fds"
    path.write_text(make_car.model(2), encoding="latin-1")
    elems = plugin.plugin_elements(load_fds(path, "v2"))
    types = {}
    for name, tname, *_ in elems:
        types.setdefault(tname, []).append(name)
    assert len(elems) == 32 and len(types) == 11  # the four tires differ in their road profile
    assert sorted(types[next(t for t in types if t.endswith("susp_fl1"))]) == sorted(
        f"susp_{w}{c}" for w in ("fl", "fr", "rl", "rr") for c in (1, 2))


def test_car_model_runs(tmp_path):
    """examples/car_on_crg.yaml on an analytic road: validates, builds and runs a few steps,
    rolling at 15 m/s with all wheels loaded."""
    _runtime()
    import yaml

    from conftest import ROUGH_ROAD, car

    path = tmp_path / "car.yaml"
    path.write_text(yaml.safe_dump(car(ROUGH_ROAD, 0.05)))
    out = tmp_path / "r.csv"
    assert cli.main(["run", str(path), "-o", str(out)]) == 0
    rows = list(csv.DictReader(open(out)))
    assert float(rows[-1]["t"]) == pytest.approx(0.05)
    assert float(rows[-1]["chassis.Rdot_x"]) == pytest.approx(15.0, abs=0.1)
    for w in ("fl", "fr", "rl", "rr"):
        assert 1500 < float(rows[-1][f"tire_{w}.force_z"]) < 5500  # a rough road: about 3.4 kN each


def test_newton_fixed_from_yaml(tmp_path, capsys):
    """Real-time mode in the model file: solver.newton fixed, exactly newton_iterations
    corrections per step; the result agrees with converged Newton far below the step's error."""
    import yaml

    _runtime()
    m = yaml.safe_load(open(ROOT / "examples" / "double_pendulum.yaml"))
    m["solver"].update(end_time=0.5, newton="fixed", newton_iterations=2)
    path = tmp_path / "rt.yaml"
    path.write_text(yaml.safe_dump(m))
    assert cli.main(["run", str(path), "-o", str(tmp_path / "rt.csv"), "--workdir", str(tmp_path)]) == 0
    text = (tmp_path / f"{m['model']}.fdm").read_text()
    assert "newton fixed" in text and "newton_iterations 2" in text
    out = capsys.readouterr().out
    assert "Newton iterations per step: 2.000" in out and "largest velocity change" in out
    m["solver"].update(newton="velocity")
    path.write_text(yaml.safe_dump(m))
    assert cli.main(["run", str(path), "-o", str(tmp_path / "conv.csv")]) == 0
    a, b = (np.loadtxt(tmp_path / f, delimiter=",", skiprows=1) for f in ("rt.csv", "conv.csv"))
    assert np.max(np.abs(a - b)) < 1e-8


def test_examples_lists_and_copies(tmp_path, capsys):
    assert cli.main(["examples"]) == 0
    assert "quarter_car" in capsys.readouterr().out
    assert cli.main(["examples", "quarter_car", "-o", str(tmp_path)]) == 0
    assert (tmp_path / "quarter_car.yaml").read_text() == (ROOT / "examples" / "quarter_car.yaml").read_text()
    assert cli.main(["examples", "quarter_car", "-o", str(tmp_path)]) == 1  # exists: not overwritten
    assert cli.main(["examples", "nothing"]) == 1


def test_examples_copy_their_data_files(tmp_path):
    assert cli.main(["examples", "car_on_crg", "-o", str(tmp_path)]) == 0
    crg = tmp_path / "data" / "test_track.crg"
    assert crg.read_bytes() == (ROOT / "examples" / "data" / "test_track.crg").read_bytes()
    from fdgen.model import data_files

    assert data_files(tmp_path / "car_on_crg.yaml") == [Path("data/test_track.crg")]
