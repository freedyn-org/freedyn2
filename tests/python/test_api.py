"""The Python API (freedyn2): load, run with parameter values, .fds models, checks, sweeps,
examples, errors."""

import gc
import importlib.util

import numpy as np
import pytest

import freedyn2
from fdgen import cli
from fdgen.view.scene import read_results


@pytest.fixture(scope="module")
def runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def test_version_and_names():
    assert freedyn2.__version__ and freedyn2.__version__ != "unknown"
    assert all(hasattr(freedyn2, n) for n in freedyn2.__all__)


def test_examples():
    found = freedyn2.examples()
    assert "pendulum" in found and found["pendulum"].suffix == ".yaml"
    assert freedyn2.example_path("pendulum") == found["pendulum"]
    assert freedyn2.example_path("compare/pendulum.fds").is_file()
    with pytest.raises(freedyn2.ModelError):
        freedyn2.example_path("no_such_model")


def test_load():
    m = freedyn2.load(freedyn2.example_path("forced_oscillator"))
    assert m.name == "forced_oscillator" and m.parameters["k"] == 200.0
    assert (m.bodies, m.joints, m.forces, m.measures) == (["mass"], ["slider"], ["drive"], ["z", "vz"])
    m2 = m.with_parameters({"k": 400.0}, d=1)
    assert m2.parameters["k"] == 400.0 and m2.parameters["d"] == 1.0 and m.parameters["k"] == 200.0
    assert m2.overrides == {"parameters.k": 400.0, "parameters.d": 1.0}
    with pytest.raises(freedyn2.ModelError, match="no parameter"):
        m.with_parameters(kk=1)
    with pytest.raises(freedyn2.ModelError):
        m.with_parameters({"forces.nothing.stiffness": 1})
    with pytest.raises(ValueError):  # ModelError is a ValueError
        m.with_parameters({"bodies.mass.mass": -1})
    fds = freedyn2.load(freedyn2.example_path("compare/pendulum.fds"), profile="legacy")
    assert fds.profile == "legacy" and len(fds.bodies) == 1
    with pytest.raises(freedyn2.ModelError, match=".yaml models only"):
        fds.with_parameters(k=1)


def test_run_with_parameters(runtime, tmp_path):
    path = freedyn2.example_path("forced_oscillator")
    r = freedyn2.run(path, end_time=0.5, output_step=0.01, workdir=tmp_path)
    assert r.csv_path.parent == tmp_path and r.fdm_path.is_file() and r.simulation_seconds > 0
    assert r.t[-1] == pytest.approx(0.5) and len(r) == len(r.t) == 51
    assert "mass.R_z" in r.channels and "t" not in r.channels and "mass.R_z" in r
    assert np.array_equal(r["mass.R_z"], read_results(r.csv_path)["mass.R_z"])
    with pytest.raises(KeyError, match="no channel"):
        r["mass.nothing"]
    stiff = freedyn2.run(path, end_time=0.5, params={"k": 800.0})
    assert stiff.model.parameters["k"] == 800.0
    assert stiff.kpi("mass.R_z", "max") < r.kpi("mass.R_z", "max")  # stiffer: smaller amplitude
    assert freedyn2.load(path).run(end_time=0.5, params={"k": 800.0}).kpi("mass.R_z", "max") == \
        pytest.approx(stiff.kpi("mass.R_z", "max"))
    df = r.to_pandas() if _has_pandas() else None
    if df is not None:
        assert df.index.name == "t" and np.array_equal(df["mass.R_z"].to_numpy(), r["mass.R_z"])


def _has_pandas():
    return importlib.util.find_spec("pandas") is not None


def test_temporary_workdir_lives_with_the_result(runtime):
    r = freedyn2.run(freedyn2.example_path("pendulum"), end_time=0.05)
    d = r.workdir
    assert r.csv_path.is_file()
    del r
    gc.collect()
    assert not d.exists()


def test_run_fds_model(runtime):
    path = freedyn2.example_path("compare/pendulum.fds")
    v2 = freedyn2.run(path, end_time=0.2)
    legacy = freedyn2.run(path, end_time=0.2, profile="legacy")
    yaml = freedyn2.run(freedyn2.example_path("pendulum"), end_time=0.2)
    name = v2.model.bodies[0]
    assert legacy.model.profile == "legacy"
    assert v2[f"{name}.R_z"][-1] < -0.01  # it falls
    assert np.allclose(v2[f"{name}.R_z"][-1], legacy[f"{name}.R_z"][-1], atol=1e-3)
    assert v2[f"{name}.R_z"][-1] == pytest.approx(yaml["rod.R_z"][-1], abs=1e-3)  # the same pendulum


def test_check(runtime):
    path = freedyn2.example_path("double_pendulum")
    report = freedyn2.check(path)
    assert report.summary["bodies"] == 2 and report.passed and "degrees of freedom" in str(report)
    r = freedyn2.run(path, end_time=0.1)
    assert r.check().passed and freedyn2.check(path, r.csv_path).passed
    assert freedyn2.check(path, run=True, end_time=0.1).results_report is not None


def test_errors(runtime, tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("model: bad\nbodies: {}\n")
    with pytest.raises(freedyn2.ModelError):
        freedyn2.load(bad)
    with pytest.raises(freedyn2.SimulationError):
        freedyn2.run(freedyn2.example_path("pendulum"), options=["--no-such-option"])
    with pytest.raises(freedyn2.RuntimeNotFoundError):
        freedyn2.run(freedyn2.example_path("pendulum"), runtime=tmp_path / "missing", workdir=tmp_path)
    assert issubclass(freedyn2.RuntimeNotFoundError, FileNotFoundError)


def test_sweep(runtime):
    path = freedyn2.example_path("forced_oscillator")
    rs = freedyn2.sweep(path, "k", [100.0, 400.0], end_time=0.5, params={"d": 2.0}, jobs=2)
    assert [r.model.parameters["k"] for r in rs] == [100.0, 400.0]
    assert all(r.model.parameters["d"] == 2.0 for r in rs)
    assert rs[0].kpi("mass.R_z", "max") > rs[1].kpi("mass.R_z", "max")
    with pytest.raises(freedyn2.SimulationError, match="1 of 2 values failed"):
        freedyn2.sweep(path, "bodies.mass.mass", [1.0, -1.0], end_time=0.1)
    rs = freedyn2.sweep(path, "bodies.mass.mass", [1.0, -1.0], end_time=0.1, strict=False)
    assert rs[0] is not None and rs[1] is None
    with pytest.raises(freedyn2.ModelError, match="sweeps need"):
        freedyn2.sweep(freedyn2.example_path("compare/pendulum.fds"), "k", [1.0])
