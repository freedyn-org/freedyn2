"""OpenCRG roads (fdgen.crg writes them; the runtime reads and evaluates them with the official
ASAM OpenCRG C library: fd::Road, freedyn2_road): data formats, curved reference line with
slope and banking, consistency of the (x, y) and (u, v) evaluations, gradients, placement, the
friction file, and a vehicle on a CRG road against the same road given analytically."""

import csv
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.crg import write_crg
from fdgen.model import ModelError, validate

from conftest import car

ROOT = Path(__file__).resolve().parents[2]

DU, NU = 0.05, 241
U = np.arange(NU) * DU
V = np.linspace(-2.0, 2.0, 41)
K = 1 / 15.0  # curvature of the reference line [1/m]
PHI = 0.3 + K * (U[:-1] + DU / 2)  # segment headings: an arc
START = (10.0, -5.0, 0.3)


def wave(u, v):
    return 0.02 * np.sin(2 * np.pi * u / 3.0) * np.cos(2 * np.pi * v / 2.5)


def road_tool():
    exe = cli.find_tool("freedyn2_road")
    if exe is None:
        pytest.skip("freedyn2_road not built")
    return exe


def probe(exe, crg, pts, *opts):
    r = subprocess.run([str(exe), str(crg), *opts], input="\n".join(f"{float(a)!r} {float(b)!r}" for a, b in pts),
                       capture_output=True, text=True, check=True)
    return np.array([[float(x) for x in line.split()] for line in r.stdout.splitlines()])


def bilinear(Z, u, v):
    i = min(int(u / DU), NU - 2)
    j = min(np.searchsorted(V, v) - 1, len(V) - 2)
    a, b = (u - U[i]) / DU, (v - V[j]) / (V[j + 1] - V[j])
    return (1 - a) * ((1 - b) * Z[i, j] + b * Z[i, j + 1]) + a * ((1 - b) * Z[i + 1, j] + b * Z[i + 1, j + 1])


@pytest.mark.parametrize("fmt,tol", [("LDFI", 2e-7), ("KDBI", 2e-7), ("KRBI", 2e-7), ("LRFI", 1e-4)])
def test_evaluation_in_all_formats(fmt, tol, tmp_path):
    """Curved reference line, slope and banking channels: z(u, v) = bilinear grid + 0.03 u +
    0.05 v (the library keeps grid values in single precision), the same z at the (x, y) of
    (u, v), and the gradient is the slope of the surface."""
    exe = road_tool()
    Z = wave(U[:, None], V[None, :])
    crg = tmp_path / f"arc_{fmt}.crg"
    write_crg(crg, DU, V, Z, phi=PHI, slope=np.full(NU, 0.03), bank=np.full(NU, 0.05), start=START, fmt=fmt)
    rng = np.random.default_rng(1)
    uv = np.c_[rng.uniform(0.2, U[-1] - 0.2, 200), rng.uniform(-1.9, 1.9, 200)]
    o = probe(exe, crg, uv, "--uv")
    expected = np.array([bilinear(Z, u, v) for u, v in uv]) + 0.03 * uv[:, 0] + 0.05 * uv[:, 1]
    assert np.max(np.abs(o[:, 4] - expected)) < tol
    xy = probe(exe, crg, o[:, 2:4])
    assert np.max(np.abs(xy[:, 2] - o[:, 4])) < 1e-9  # (x, y) -> (u, v) -> z as (u, v) -> z
    assert np.all(xy[:, 5] == 1.0)  # friction: default constant
    # gradient of a plane in road coordinates (no waves): slope along, banking across the line
    write_crg(crg, DU, V, 0 * Z, phi=PHI, slope=np.full(NU, 0.03), bank=np.full(NU, 0.05), start=START, fmt=fmt)
    g = probe(exe, crg, o[:, 2:4])
    phi = np.interp(uv[:, 0], U[1:] - DU / 2, PHI)  # heading of the reference line at u
    t, n = np.c_[np.cos(phi), np.sin(phi)], np.c_[-np.sin(phi), np.cos(phi)]
    curv = 1 / (1 - K * uv[:, 1])  # du/ds on a curve: the slope along u is steeper outside
    g_expected = 0.03 * curv[:, None] * t + 0.05 * n
    assert np.max(np.abs(g[:, 3:5] - g_expected)) < 2e-3


def test_placement_and_friction_file(tmp_path):
    exe = road_tool()
    Z = wave(U[:, None], V[None, :])
    write_crg(tmp_path / "h.crg", DU, V, Z, phi=PHI, start=START)
    write_crg(tmp_path / "mu.crg", DU, V, 0.5 + 0.1 * np.cos(U[:, None]) + 0 * V[None, :], phi=PHI, start=START)
    rng = np.random.default_rng(2)
    uv = np.c_[rng.uniform(0.2, U[-1] - 0.2, 50), rng.uniform(-1.9, 1.9, 50)]
    pts = probe(exe, tmp_path / "h.crg", uv, "--uv")[:, 2:4]
    ref = probe(exe, tmp_path / "h.crg", pts, "--friction-crg", str(tmp_path / "mu.crg"))
    assert np.all((ref[:, 5] > 0.39) & (ref[:, 5] < 0.61))
    # the same road moved by (3, 4, 0.5) and turned by 0.7 rad about z
    c, s = math.cos(0.7), math.sin(0.7)
    moved = pts @ np.array([[c, s], [-s, c]]) + [3.0, 4.0]
    o = probe(exe, tmp_path / "h.crg", moved, "--place", "3", "4", "0.5", "0.7")
    assert np.max(np.abs(o[:, 2] - 0.5 - ref[:, 2])) < 1e-12
    g = ref[:, 3:5] @ np.array([[c, s], [-s, c]])  # gradient turned with the road
    assert np.max(np.abs(o[:, 3:5] - g)) < 1e-12


def test_vehicle_on_crg_equals_analytic_road(tmp_path):
    """Waves in the grid, slope and banking as reference line channels: the vehicle drives as on
    the analytic road with the same harmonics and slope (bilinear error of a 2 m wave on 5 cm)."""
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    road = {"harmonics": [{"amplitude": 0.01, "wavelength": 5.0, "heading": 0.3, "phase": 0.2},
                          {"amplitude": 0.004, "wavelength": 2.0, "heading": -0.7, "phase": 1.0}],
            "slope": [0.02, 0.04], "friction": 0.9}

    def h(x, y):
        return sum(k["amplitude"] * np.sin(2 * np.pi * (x * np.cos(k["heading"]) + y * np.sin(k["heading"]))
                                           / k["wavelength"] + k["phase"]) for k in road["harmonics"])

    du = 0.05
    x, v = np.arange(-1.0, 18.0 + 1e-9, du), np.arange(-2.0, 3.0 + 1e-9, du)
    X, Y = np.meshgrid(x, v, indexing="ij")
    # reference line z from the slope channel starts at 0 at x = -1: the grid holds the offset
    write_crg(tmp_path / "road.crg", du, v, h(X, Y) + 0.02 * x[0], slope=np.full(len(x), 0.02),
              bank=np.full(len(x), 0.04), start=(-1.0, 0.0, 0.0), fmt="KDBI")
    rows = {}
    for name, rd in (("analytic", road), ("crg", {"crg": "road.crg", "friction": 0.9})):
        m = car(rd, 1.0)
        m["model"] = f"vehicle_{name}"
        path = tmp_path / f"{name}.yaml"
        path.write_text(yaml.safe_dump(m))
        fdm, model, _ = cli.build(path, tmp_path, "v2")
        cli.simulate(runtime, fdm, tmp_path / f"{name}.csv", ["--newton-fixed", "2"])
        rows[name] = list(csv.DictReader(open(tmp_path / f"{name}.csv")))
    assert "road_crg" in fdm.read_text() and "element tire_local tire_fl" in fdm.read_text()
    a, b = rows["analytic"], rows["crg"]

    def dev(pattern):
        return max(abs(float(ra[k]) - float(rb[k])) for ra, rb in zip(a, b) for k in ra if pattern in k)

    assert dev("road_height") < 3e-5
    assert dev(".R_z") < 1e-4 and dev(".R_y") < 1e-4
    assert dev("tire_fl.force_z") < 0.005 * 3400.0  # N, of about 3.4 kN
    assert dev("friction") < 1e-12


def test_model_validation(tmp_path):
    base = {"model": "m", "bodies": {"w": {"mass": 1.0, "inertia": [1.0, 1.0, 1.0]}},
            "solver": {"type": "HHT", "end_time": 1.0, "step": 1e-3}}
    with pytest.raises(ModelError, match="not found"):
        validate(dict(base, road={"crg": "missing.crg"}), tmp_path)
    (tmp_path / "r.crg").write_text("x")
    with pytest.raises(ModelError, match="analytic road keys"):
        validate(dict(base, road={"crg": "r.crg", "harmonics": []}), tmp_path)
    with pytest.raises(ModelError, match="belong to an OpenCRG road"):
        validate(dict(base, road={"heading": 0.1}), tmp_path)
    m = validate(dict(base, road={"crg": "r.crg"}), tmp_path)
    assert Path(m["road"]["crg"]).is_absolute()


def test_test_track_is_what_its_script_writes(tmp_path):
    sys.path.insert(0, str(ROOT / "examples" / "data"))
    import make_test_track

    make_test_track.main(tmp_path / "t.crg")
    assert (tmp_path / "t.crg").read_bytes() == (ROOT / "examples" / "data" / "test_track.crg").read_bytes()


def test_car_on_the_test_track(tmp_path):
    """examples/car_on_crg.yaml (the OpenCRG tutorial): starts at rest on its tires, the speed
    bump unloads the front wheels, the pothole is under the left wheels only."""
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    model = yaml.safe_load((ROOT / "examples" / "car_on_crg.yaml").read_text())
    model["solver"]["end_time"] = 9.5
    model["road"]["crg"] = str(ROOT / "examples" / "data" / "test_track.crg")
    path = tmp_path / "car.yaml"
    path.write_text(yaml.safe_dump(model))
    assert cli.main(["run", str(path), "-o", str(tmp_path / "car.csv")]) == 0
    rows = list(csv.DictReader(open(tmp_path / "car.csv")))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = col("t")
    rest, bump, hole = t < 1.5, (t > 1.6) & (t < 2.6), (t > 8.3) & (t < 9.2)
    np.testing.assert_allclose(col("head.specific_force_z")[rest], 9.81, atol=0.5)  # settles by a few mm
    assert col("tire_fl.force_z")[bump].min() < 0.1 * col("tire_fl.force_z")[0]
    assert np.ptp(col("tire_fl.force_z")[hole]) > 10 * np.ptp(col("tire_fr.force_z")[hole])
    assert col("chassis.Rdot_x")[-1] == pytest.approx(15.0, abs=0.3)  # the speed controller holds it


def test_viewer_watches_the_road(tmp_path):
    from fdgen.view import server

    view = server.View(ROOT / "examples" / "car_on_crg.yaml", run=False)
    assert (ROOT / "examples" / "data" / "test_track.crg") in view._watched()
