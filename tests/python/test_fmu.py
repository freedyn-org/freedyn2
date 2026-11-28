"""FMUs of freedyn2 models (fdgen.fmu, runtime/fmu): FMI 2.0 and 3.0 co-simulation, checked with
FMPy (validation of the modelDescription.xml, simulation) against the runtime itself."""

import csv
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

fmpy = pytest.importorskip("fmpy")
from fmpy.validation import validate_fmu  # noqa: E402

from fdgen import cli, fmu  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def tools():
    try:
        return cli.find_runtime(), fmu.find_library()
    except FileNotFoundError as e:
        pytest.skip(str(e))


def run(tools, tmp_path, model, *options):
    """Results of freedyn2_run (Eigen's SparseLU, as in the FMU) by channel."""
    work = tmp_path / "run"
    work.mkdir(exist_ok=True)
    fdm, _, _ = cli.build(ROOT / "examples" / f"{model}.yaml", work)
    out = work / f"{model}.csv"
    subprocess.run([str(tools[0]), str(fdm), "-o", str(out), "--linear-solver", "sparselu", *options], check=True,
                   capture_output=True)
    rows = list(csv.DictReader(open(out)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


@pytest.mark.parametrize("fmi", ["2", "3"])
def test_fmu_is_the_model(tools, tmp_path, fmi):
    """input_oscillator (a model plugin, runtime inputs, a driver): the FMU passes FMPy's
    validation, and with constant inputs its outputs at the communication points are those of
    freedyn2_run (the same steps: to round-off)."""
    path, v = fmu.export(ROOT / "examples" / "input_oscillator.yaml", tmp_path / "osc.fmu", fmi)
    assert validate_fmu(str(path)) == []
    assert v["inputs"] == ["F", "z_set"] and "mass.R_z" in v["outputs"]
    md = fmpy.read_model_description(str(path))
    assert md.fmiVersion == f"{fmi}.0" and md.coSimulation.modelIdentifier == "input_oscillator"
    names = ["mass.R_z", "force.value", "position.lambda"]
    r = fmpy.simulate_fmu(str(path), stop_time=3.0, output_interval=0.01, start_values={"F": 1.5, "z_set": 0.02},
                          output=names)
    ref = run(tools, tmp_path, "input_oscillator", "--input", "F=1.5", "--input", "z_set=0.02")
    np.testing.assert_allclose(r["time"], ref["t"], atol=1e-12)
    for n in names:
        np.testing.assert_allclose(r[n], ref[n], rtol=1e-9, atol=1e-12 * np.abs(ref[n]).max())


def test_inputs_drive_the_fmu(tmp_path, tools):
    """An input signal from the master: the force F steps from 0 to 1.5 N at t = 1 s; the mass
    (k = 200 N/m, lightly damped) settles at F / k."""
    path, _ = fmu.export(ROOT / "examples" / "input_oscillator.yaml", tmp_path / "osc.fmu", "3")
    signal = np.array([(0.0, 0.0), (1.0, 0.0), (1.0, 1.5), (12.0, 1.5)], dtype=[("time", float), ("F", float)])
    r = fmpy.simulate_fmu(str(path), stop_time=12.0, output_interval=0.01, input=signal, output=["mass.R_z"])
    z, t = r["mass.R_z"], r["time"]
    assert np.abs(z[t <= 1.0]).max() < 1e-12
    assert z[-1] == pytest.approx(1.5 / 200.0, rel=1e-3)


def test_contact_fmu(tmp_path, tools):
    """bouncing_ball: events within the communication steps. After an event the FMU goes on with
    full steps from it (freedyn2_run with the rest of its grid step), so the steps differ; both
    are as close to a run at a ten times smaller step."""
    path, _ = fmu.export(ROOT / "examples" / "bouncing_ball.yaml", tmp_path / "ball.fmu", "2")
    z = fmpy.simulate_fmu(str(path), stop_time=1.0, output_interval=1e-3, output=["ball.R_z"])["ball.R_z"]
    run_z = run(tools, tmp_path, "bouncing_ball", "--end-time", "1")["ball.R_z"]
    fine_z = run(tools, tmp_path, "bouncing_ball", "--end-time", "1", "--step", "1e-5")["ball.R_z"]
    assert np.abs(z - fine_z).max() <= np.abs(run_z - fine_z).max()
    assert z.min() > 0.04  # it bounces, it does not fall through the floor


def test_road_file_in_the_fmu(tmp_path, tools):
    """car_on_crg: the OpenCRG road file goes into the resources of the FMU and is found there."""
    path, _ = fmu.export(ROOT / "examples" / "car_on_crg.yaml", tmp_path / "car.fmu", "3")
    names = ["chassis.R_z", "chassis.Rdot_x"]
    r = fmpy.simulate_fmu(str(path), stop_time=0.3, output_interval=0.02, output=names)
    ref = run(tools, tmp_path, "car_on_crg", "--end-time", "0.3")
    for n in names:
        np.testing.assert_allclose(r[n], ref[n][::2], rtol=1e-7, atol=1e-9)  # the run writes every 0.01 s


def test_errors_of_the_fmu(tmp_path, tools):
    """A step from another time than the slave is at (no rollback) is an error, logged."""
    from fmpy.fmi3 import FMU3Slave

    path, _ = fmu.export(ROOT / "examples" / "pendulum.yaml", tmp_path / "p.fmu", "3")
    md = fmpy.read_model_description(str(path))
    unzipped = fmpy.extract(str(path), unzipdir=str(tmp_path / "unzipped"))
    messages = []
    slave = FMU3Slave(guid=md.guid, unzipDirectory=unzipped, modelIdentifier=md.coSimulation.modelIdentifier,
                      instanceName="p")
    slave.instantiate(logMessage=lambda *a: messages.append(a))
    slave.enterInitializationMode()
    slave.exitInitializationMode()
    slave.doStep(0.0, 0.01)
    with pytest.raises(Exception):
        slave.doStep(0.5, 0.01)
    assert any(b"no rollback" in (m[-1] if isinstance(m[-1], bytes) else str(m[-1]).encode()) for m in messages)
    slave.terminate()
    slave.freeInstance()
    shutil.rmtree(unzipped, ignore_errors=True)
