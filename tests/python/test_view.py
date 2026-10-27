"""Viewer (fdgen.view, fdgen.shapes): shapes of bodies in their frames, default shapes, the scene
(joint frames, force points, analytic and OpenCRG road grids), the server's API and live reload,
the exported HTML file; shapes do not change the runtime model file."""

import base64
import json
import math
import struct
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pytest
import yaml

from conftest import ROUGH_ROAD, car, road_height
from fdgen import cli
from fdgen.crg import write_crg
from fdgen.geometry import rotation_matrix
from fdgen.model import ModelError, validate, write_fdm
from fdgen.shapes import euler_parameters, inertia_box
from fdgen.view import server
from fdgen.view.scene import read_results, scene

ROOT = Path(__file__).resolve().parents[2]

C = math.sqrt(0.5)
SOLVER = {"type": "HHT", "end_time": 0.2, "step": 1e-3, "output_step": 1e-2}


def floats(s):
    return np.frombuffer(base64.b64decode(s), dtype="<f4")


def pendulum(**body):
    return {"model": "p", "gravity": [0, 0, -9.81], "solver": SOLVER,
            "bodies": {"rod": dict({"mass": 1.0, "inertia": [0.01, 0.1, 0.1], "position": [0.5, 0, 0],
                                    "orientation": [C, 0, 0, C]}, **body)},
            "joints": {"pin": {"type": "revolute", "body1": "rod", "location": [0, 0, 0],
                               "orientation": [C, -C, 0, 0]}}}


def test_euler_parameters_of_rotation_matrices():
    rng = np.random.default_rng(3)
    for t in [*rng.normal(size=(50, 4)), [0, 1, 0, 0], [0, 0, 0, 1], [1e-9, 0, 1, 0]]:
        t = np.asarray(t) / np.linalg.norm(t)
        assert np.allclose(rotation_matrix(euler_parameters(rotation_matrix(t))), rotation_matrix(t), atol=1e-12)


def test_inertia_box():
    m, a, b, c = 3.0, 0.8, 0.3, 0.2
    inertia = [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12]
    assert np.allclose(inertia_box(m, inertia), [a, b, c])
    thin = inertia_box(1.0, [1.0, 0.5, 0.5])  # a disc's inertia: no real box, limited edge
    assert min(thin) > 0


def test_shapes_in_the_body_frame(tmp_path):
    (tmp_path / "part.stl").write_bytes(b"\0" * 84)
    m = validate(pendulum(shapes=[
        {"type": "box", "size": [1, 0.1, 0.1]},  # default: the body frame
        {"type": "sphere", "radius": 0.1, "position": [1, 0, 0]},  # global at t = 0
        {"type": "cylinder", "radius": 0.1, "length": 0.2, "frame": "body", "position": [0.5, 0, 0],
         "orientation": [C, 0, C, 0]},
        {"type": "mesh", "file": "part.stl", "scale": 0.001, "position": [0, 0, 0], "orientation": [1, 0, 0, 0]},
    ]), tmp_path)
    box, sphere, cyl, mesh = m["bodies"]["rod"]["shapes"]
    assert np.allclose(box["position"], 0) and np.allclose(box["orientation"], [1, 0, 0, 0])
    # body turned 90 deg about z: the global point (1, 0, 0) is 0.5 along -y of the body
    assert np.allclose(sphere["position"], [0, -0.5, 0])
    assert np.allclose(cyl["position"], [0.5, 0, 0]) and np.allclose(cyl["orientation"], [C, 0, C, 0])
    # a CAD mesh in global coordinates: its frame is the global frame, seen from the body
    A = rotation_matrix(m["bodies"]["rod"]["orientation"])
    assert np.allclose(rotation_matrix(mesh["orientation"]), A.T)
    assert np.allclose(mesh["position"], A.T @ -np.array([0.5, 0, 0]))
    assert mesh["scale"] == [0.001] * 3 and Path(mesh["file"]).is_absolute()


@pytest.mark.parametrize("shape,message", [
    ({"type": "box"}, "needs size"),
    ({"type": "mesh", "file": "missing.stl"}, "not found"),
    ({"type": "mesh", "file": "part.step"}, ".stl or .obj"),
])
def test_shape_errors(shape, message, tmp_path):
    with pytest.raises(ModelError, match=message):
        validate(pendulum(shapes=[shape]), tmp_path)


def test_ground_shapes_are_global(tmp_path):
    m = pendulum()
    m["ground"] = {"shapes": [{"type": "box", "size": [1, 1, 0.1], "position": [0, 0, -1]}]}
    assert validate(m, tmp_path)["ground_shapes"][0]["position"] == [0, 0, -1]
    m["ground"]["shapes"][0]["frame"] = "body"
    with pytest.raises(ModelError, match="no body frame"):
        validate(m, tmp_path)


def test_shapes_do_not_change_the_model_file(tmp_path):
    plain, shaped = pendulum(), pendulum(shapes=[{"type": "box", "size": [1, 0.1, 0.1]}])
    shaped["ground"] = {"shapes": [{"type": "sphere", "radius": 0.1}]}
    write_fdm(validate(plain, tmp_path), tmp_path / "a.fdm")
    write_fdm(validate(shaped, tmp_path), tmp_path / "b.fdm")
    assert (tmp_path / "a.fdm").read_text() == (tmp_path / "b.fdm").read_text()


def test_scene_of_the_vehicle():
    """Default shapes (wheels: cylinders about body y), joint frames on both bodies, spring points,
    the analytic road with its friction patch around the wheels."""
    m = validate(car(ROUGH_ROAD, shapes=False), ROOT / "examples")
    driven = {"tire_fl.contact_x": np.array([0.0, 120.0]), "tire_fl.contact_y": np.array([0.0, 10.0])}
    s = scene(m, driven)  # the road around where the tire drove: over the friction patch
    bodies = {b["name"]: b for b in s["bodies"]}
    wheel = bodies["wheel_fl"]["shapes"][0]
    assert wheel["type"] == "cylinder" and wheel["radius"] == 0.3
    assert np.allclose(np.abs(rotation_matrix(wheel["orientation"]) @ [0, 0, 1]), [0, 1, 0])  # axis: body y
    assert bodies["chassis"]["shapes"][0]["type"] == "box" and bodies["chassis"]["default_shapes"]
    for j in s["joints"]:  # the joint frame seen from body1 and from body2 is the same frame at t = 0
        frames = []
        for k in ("1", "2"):
            b = j["body" + k]
            R, A = (np.zeros(3), np.eye(3)) if b == "ground" else \
                (np.array(m["bodies"][b]["position"]), rotation_matrix(m["bodies"][b]["orientation"]))
            frames.append((R + A @ j["position" + k], A @ rotation_matrix(j["orientation" + k])))
        assert np.allclose(frames[0][0], frames[1][0]) and np.allclose(frames[0][1], frames[1][1])
    springs = [f for f in s["forces"] if f["type"] == "spring_damper"]
    assert springs and all(len(f["point1"]) == 3 and len(f["point2"]) == 3 for f in springs)
    road = s["road"]
    assert road["source"] == "analytic"
    P, mu = floats(road["points"]).reshape(-1, 3), floats(road["friction"])
    assert len(P) == road["rows"] * road["cols"] == len(mu)
    expected = np.array([road_height(float(x), float(y)) for x, y in P[::50, :2]])
    assert np.max(np.abs(P[::50, 2] - expected)) < 1e-5  # float32
    assert P[:, 0].min() < 0 and P[:, 0].max() > 120
    assert np.isclose(mu.min(), 0.3, atol=1e-3) and mu.max() == 1.0  # the friction patch


def test_crg_road_grid(tmp_path):
    tool = cli.find_tool("freedyn2_road")
    if tool is None:
        pytest.skip("freedyn2_road not built")
    du, v = 0.1, np.linspace(-2, 2, 21)
    u = np.arange(101) * du
    z = 0.05 * np.sin(u)[:, None] + 0.02 * v[None, :]
    write_crg(tmp_path / "r.crg", du, v, z, phi=0.2 + 0.05 * u[:-1], start=(1.0, 2.0, 0.2))
    write_crg(tmp_path / "mu.crg", du, v, 0.5 + 0 * z, phi=0.2 + 0.05 * u[:-1], start=(1.0, 2.0, 0.2))
    road = {"crg": str(tmp_path / "r.crg"), "crg_friction": str(tmp_path / "mu.crg"),
            "position": [5.0, -3.0, 0.4], "heading": 0.6}
    m = {"forces": [{"type": "tire", "body1": "w", "name": "t"}], "road": road}
    grid = scene(dict(m, model="r", bodies={}), road_tool=tool)["road"]
    P = floats(grid["points"]).astype(float)
    assert grid["source"] == "crg" and grid["rows"] * grid["cols"] * 3 == len(P)
    inner = P.reshape(grid["rows"], grid["cols"], 3)[1:-1, 1:-1].reshape(-1, 3)[::7]  # border: extrapolated
    probe = cli_probe(tool, road, inner[:, :2])
    assert np.max(np.abs(probe[:, 2] - inner[:, 2])) < 1e-5  # heights where the runtime evaluates them
    assert np.allclose(floats(grid["friction"]), 0.5)


def cli_probe(tool, road, xy):
    import subprocess

    args = [str(tool), road["crg"], "--place", *map(str, road["position"]), str(road["heading"])]
    out = subprocess.run(args, input="\n".join(f"{a!r} {b!r}" for a, b in xy.tolist()),
                         capture_output=True, text=True, check=True).stdout
    return np.array([line.split() for line in out.splitlines()], dtype=float)


def write_pendulum(path, stiffness=None):
    m = pendulum(shapes=[{"type": "box", "size": [1, 0.05, 0.05]}])
    if stiffness:
        m["forces"] = {"s": {"type": "spring_damper", "body1": "rod", "point1": [1, 0, 0], "stiffness": stiffness}}
    path.write_text(yaml.safe_dump(m))


def get(url):
    with urllib.request.urlopen(url, timeout=10) as r:
        return r.read()


def test_server_api_and_reload(tmp_path):
    """Results given: the API serves scene and CSV; a saved model file makes a new version, an
    invalid one an error over the last scene."""
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    model = tmp_path / "p.yaml"
    write_pendulum(model)
    fdm, _, _ = cli.build(model, tmp_path, "v2")
    cli.simulate(runtime, fdm, tmp_path / "p.csv")
    view = server.View(model, tmp_path / "p.csv", runtime=runtime)
    view.rebuild()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server._handler(view))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}/"
    try:
        state = json.loads(get(base + "api/state"))
        assert state["status"] == "ready" and state["results"] and state["scene"]
        s = json.loads(get(base + "api/scene"))
        assert s["bodies"][0]["name"] == "rod" and s["joints"][0]["type"] == "revolute"
        assert s["checks"]["summary"]["degrees_of_freedom"] == 1  # checks of model and results
        line = s["bodies"][0]["source"]["line"]
        assert model.read_text().splitlines()[line - 1].strip() == "rod:"
        assert s["bodies"][0]["source"]["yaml"].startswith("rod:")
        assert s["joints"][0]["source"]["yaml"].startswith("pin:")
        assert [r["id"] for r in state["runs"]] == [1] and state["current"] == 1 and state["reference"] is None
        csv_text = get(base + "api/results.csv").decode()
        assert csv_text.startswith("t,rod.R_x")
        assert b"Three.js Authors" in get(base + "vendor/three.module.min.js")[:200]
        assert b"FreeDyn 2 Viewer" in get(base)
        with pytest.raises(urllib.error.HTTPError):
            get(base + "../../model.py")
        time.sleep(0.05)
        write_pendulum(model, stiffness=10.0)
        assert view.changed()
        view.rebuild()
        assert view.state()["version"] > state["version"]
        assert [f["name"] for f in json.loads(get(base + "api/scene"))["forces"]] == ["s"]
        state = view.state()  # the run before stays, with its own results
        assert [r["id"] for r in state["runs"]] == [1, 2] and state["current"] == 2
        assert [r["edit"] for r in state["runs"]] == [1, 2]  # two versions of the model file
        assert get(base + "api/runs/1.csv").decode() == csv_text
        model.write_text(model.read_text().replace("stiffness", "stifness"))
        view.rebuild()
        state = view.state()
        assert state["status"] == "error" and "stifness" in state["error"] and state["scene"]
    finally:
        httpd.shutdown()


def test_export_is_one_file(tmp_path):
    model = tmp_path / "p.yaml"
    write_pendulum(model)
    (tmp_path / "p.csv").write_text("t,rod.R_x\n0,0.5\n0.1,0.4\n")
    page = server.export(server.View(model, tmp_path / "p.csv"), tmp_path / "p.html")
    html = page.read_text()
    assert "window.FD_STATIC" in html and "importmap" in html
    assert 'src="./' not in html and "vendor/" not in html.split("window.FD_STATIC")[0]
    data = json.loads(html.split("window.FD_STATIC = ", 1)[1].split(";</script>", 1)[0].replace("<\\/", "</"))
    assert data["scene"]["model"] == "p" and data["csv"].startswith("t,rod.R_x")


def test_read_results(tmp_path):
    (tmp_path / "r.csv").write_text("t,a.x\n0,1\n0.5,2\n")
    r = read_results(tmp_path / "r.csv")
    assert list(r) == ["t", "a.x"] and r["a.x"].tolist() == [1.0, 2.0]


def test_stl_header_only_mesh_is_accepted(tmp_path):
    # an STL with no triangles is valid input for the schema and the validation (the viewer draws nothing)
    (tmp_path / "e.stl").write_bytes(b"\0" * 80 + struct.pack("<I", 0))
    m = validate(pendulum(shapes=[{"type": "mesh", "file": "e.stl"}]), tmp_path)
    assert m["bodies"]["rod"]["shapes"][0]["file"].endswith("e.stl")


def test_apply_overrides():
    raw = {"parameters": {"k": 1.0}, "bodies": {"rod": {"mass": 1.0, "position": [0.5, 0, 0]}},
           "forces": {"s": {"type": "spring_damper", "stiffness": 10.0}}}
    server.apply_overrides(raw, {"parameters.k": 2, "bodies.rod.position.2": -0.1, "forces.s.damping": 3})
    assert raw["parameters"]["k"] == 2.0 and raw["bodies"]["rod"]["position"] == [0.5, 0, -0.1]
    assert raw["forces"]["s"]["damping"] == 3.0  # a default made explicit
    with pytest.raises(ValueError, match="no nope"):
        server.apply_overrides(raw, {"bodies.nope.mass": 1})


def test_tunable_numbers_and_parameters(tmp_path):
    from fdgen.view.scene import attach_sources

    m = pendulum()
    m["parameters"] = {"k": 5.0, "unused": 1.0}
    m["forces"] = {"f": {"type": "expression_force", "body1": "rod", "force": "k * sin(t)", "direction": [0, 0, 1]}}
    path = tmp_path / "p.yaml"
    path.write_text(yaml.safe_dump(m))
    s = scene(validate(m, tmp_path))
    attach_sources(s, path)
    rod = {x["path"]: x["value"] for x in s["bodies"][0]["tunable"]}
    assert rod["bodies.rod.mass"] == 1.0 and rod["bodies.rod.position.0"] == 0.5 and rod["bodies.rod.inertia.1"] == 0.1
    assert not any("orientation" in p for p in rod)  # quaternion components are no knobs
    assert {x["path"] for x in s["forces"][0]["tunable"]} == {"forces.f.direction.0", "forces.f.direction.1",
                                                             "forces.f.direction.2"}
    assert s["parameters"] == [{"path": "parameters.k", "value": 5.0}, {"path": "parameters.unused", "value": 1.0}]


def serve_view(view):
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server._handler(view))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, f"http://127.0.0.1:{httpd.server_address[1]}/"


def wait_ready(base, seconds=60):
    """api/state once the background rebuild is done."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        state = json.loads(get(base + "api/state"))
        if state["status"] not in ("simulating", "loading"):
            return state
        time.sleep(0.05)
    raise TimeoutError("still simulating")


def post(url, body, origin=None):
    req = urllib.request.Request(url, json.dumps(body).encode(), {"Content-Type": "application/json"})
    if origin:
        req.add_header("Origin", origin)
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())


def test_tuned_values_are_simulated(tmp_path):
    """POST api/overrides: the model is simulated with them in the background, the file stays;
    every simulation is a run (with its tuned values), one can be the reference."""
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    model = tmp_path / "p.yaml"
    write_pendulum(model, stiffness=10.0)
    text = model.read_text()
    view = server.View(model, runtime=runtime)
    view.rebuild()
    httpd, base = serve_view(view)
    try:
        before = read_results(view.csv)["rod.R_z"]
        state = post(base + "api/overrides", {"bodies.rod.mass": 3.0})
        assert state["status"] == "simulating" and state["overrides"] == {"bodies.rod.mass": 3.0}
        state = wait_ready(base)
        assert [r["overrides"] for r in state["runs"]] == [{}, {"bodies.rod.mass": 3.0}] and state["current"] == 2
        after = read_results(view.csv)["rod.R_z"]
        assert np.max(np.abs(after - before)) > 1e-3 and model.read_text() == text
        assert json.loads(get(base + "api/state"))["overrides"] == {"bodies.rod.mass": 3.0}
        write_pendulum(model, stiffness=10.0)  # a save of the file keeps the tuned values
        view.rebuild()
        assert np.allclose(read_results(view.csv)["rod.R_z"], after)
        assert post(base + "api/overrides", {})["overrides"] == {}
        wait_ready(base)
        assert np.allclose(read_results(view.csv)["rod.R_z"], before)
        state = post(base + "api/runs/2", {"reference": True})
        assert state["reference"] == 2
        state = post(base + "api/runs/1", {"delete": True})
        assert [r["id"] for r in state["runs"]] == [2, 3, 4] and state["reference"] == 2
        with pytest.raises(urllib.error.HTTPError):
            post(base + "api/runs/4", {"delete": True})  # the current run stays
        assert post(base + "api/runs/2", {"reference": False})["reference"] is None
        with pytest.raises(urllib.error.HTTPError) as e:
            post(base + "api/overrides", {}, origin="http://evil.example")  # only the page of this server
        assert e.value.code == 403
        post(base + "api/overrides", {"bodies.nope.mass": 1.0})
        state = wait_ready(base)  # shown on the page, over the last scene
        assert state["status"] == "error" and "nope" in state["error"] and state["scene"]
    finally:
        httpd.shutdown()


def test_notes_from_the_viewer(tmp_path):
    from fdgen.view import notes

    model = tmp_path / "p.yaml"
    write_pendulum(model)
    view = server.View(model, run=False)
    view.rebuild()
    httpd, base = serve_view(view)
    try:
        stamp = view.state()["notes"]
        png = base64.b64encode(b"\x89PNG\r\n\x1a\nfake").decode()
        n = post(base + "api/notes", {"text": "too soft", "context": {"time": 0.5, "selection": "rod",
                                                                       "overrides": {"forces.s.stiffness": 40.0}},
                                      "image": "data:image/png;base64," + png})
        assert n["id"] == 1 and n["status"] == "open" and n["image"] == "note_1.png"
        assert view.state()["notes"] != stamp
        assert (tmp_path / "p.notes" / "notes.json").is_file()
        assert get(base + "api/notes/note_1.png").startswith(b"\x89PNG")
        text = notes.note_text(notes.read_notes(model)[0], model)
        assert "too soft" in text and "t = 0.5 s" in text and "selected: rod" in text
        assert "forces.s.stiffness = 40" in text and str(tmp_path / "p.notes" / "note_1.png") in text
        notes.answer_note(model, 1, "stiffness 40 N/m in the model file now")
        listed = json.loads(get(base + "api/notes"))
        assert listed[0]["status"] == "done" and listed[0]["reply"].startswith("stiffness 40")
        assert post(base + "api/notes/1", {"done": False})["status"] == "open"
        post(base + "api/notes/1", {"delete": True})
        assert json.loads(get(base + "api/notes")) == [] and not (tmp_path / "p.notes" / "note_1.png").exists()
        with pytest.raises(urllib.error.HTTPError):
            post(base + "api/notes", {"text": "  "})
    finally:
        httpd.shutdown()


def test_eigenmodes_of_a_hanging_pendulum(tmp_path):
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    m, l, I = 1.0, 0.5, 0.1
    model = {"model": "hanging", "gravity": [0, 0, -9.81], "solver": SOLVER,
             "bodies": {"bob": {"mass": m, "inertia": [I, I, 0.01], "position": [0, 0, -l]}},
             "joints": {"pin": {"type": "revolute", "body1": "bob", "location": [0, 0, 0], "orientation": [C, -C, 0, 0]}}}
    path = tmp_path / "hanging.yaml"
    path.write_text(yaml.safe_dump(model))
    view = server.View(path, runtime=runtime)
    view.rebuild()
    httpd, base = serve_view(view)
    try:
        data = json.loads(get(base + "api/modes?t=0.1"))
        assert data["time"] == pytest.approx(0.1)
        (mode,) = data["modes"]
        assert mode["frequency"] == pytest.approx(math.sqrt(m * 9.81 * l / (I + m * l * l)) / (2 * math.pi), rel=1e-6)
        assert abs(mode["damping"]) < 1e-6
        x = np.array(mode["bodies"]["bob"])[:, 0]  # real parts: swinging in x about y
        assert abs(x[0]) > 0.1 and abs(x[4]) > 0.1 and np.allclose(x[[1, 2, 3, 5]], 0, atol=1e-9)
        assert np.max(np.abs(np.array(mode["bodies"]["bob"]))) == pytest.approx(1.0)
    finally:
        httpd.shutdown()
    with pytest.raises(ValueError, match="need results"):
        server.View(path, run=False).modes(0.0)


def test_a_new_request_stops_the_running_simulation(tmp_path):
    """Tuned values while a simulation runs: it is stopped (no run), the new values are simulated;
    meanwhile the state has the progress and api/running.csv the rows so far."""
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    model = tmp_path / "p.yaml"
    write_pendulum(model)
    view = server.View(model, runtime=runtime, options=["--end-time", "200", "--step", "1e-4"])  # a long run
    view.set_overrides({"bodies.rod.mass": 2.0}, wait=False)
    httpd, base = serve_view(view)
    try:
        t0 = time.time()
        while time.time() - t0 < 30:
            state = json.loads(get(base + "api/state"))
            if state["progress"] and state["progress"]["t"] > 0:
                break
            time.sleep(0.05)
        assert state["status"] == "simulating" and state["progress"]["end"] == 200.0
        rows = get(base + "api/running.csv").decode()
        assert rows.startswith("t,rod.R_x") and rows.endswith("\n") and len(rows.splitlines()) > 1
        view.options = ["--end-time", "0.1"]
        view.set_overrides({"bodies.rod.mass": 3.0}, wait=False)
        state = wait_ready(base)
        assert state["status"] == "ready" and [r["overrides"] for r in state["runs"]] == [{"bodies.rod.mass": 3.0}]
        assert state["progress"] is None and get(base + "api/running.csv") == b""
    finally:
        httpd.shutdown()


def test_eigenvalue_preview_and_root_locus(tmp_path):
    """POST api/modes with other values and POST api/locus: eigenvalues of the model with them,
    about the state of the current run (exact here: the hanging pendulum stays in equilibrium)."""
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    l, I = 0.5, 0.1
    model = {"model": "hanging", "gravity": [0, 0, -9.81], "solver": SOLVER, "parameters": {"m": 1.0},
             "bodies": {"bob": {"mass": "=m", "inertia": [I, I, 0.01], "position": [0, 0, -l]}},
             "joints": {"pin": {"type": "revolute", "body1": "bob", "location": [0, 0, 0], "orientation": [C, -C, 0, 0]}}}
    path = tmp_path / "hanging.yaml"
    path.write_text(yaml.safe_dump(model))
    view = server.View(path, runtime=runtime)
    view.rebuild()
    httpd, base = serve_view(view)
    f = lambda m: math.sqrt(m * 9.81 * l / (I + m * l * l)) / (2 * math.pi)
    try:
        data = post(base + "api/modes", {"t": 0.1, "overrides": {"parameters.m": 3.0}})
        assert [mode["frequency"] for mode in data["modes"]] == [pytest.approx(f(3.0), rel=1e-6)]
        assert view.state()["overrides"] == {}  # a preview: nothing tuned, nothing simulated
        loc = post(base + "api/locus", {"t": 0.1, "path": "parameters.m", "values": [0.5, 1.0, 2.0]})
        assert loc["values"] == [0.5, 1.0, 2.0] and loc["path"] == "parameters.m"
        assert [ev[0][1] / (2 * math.pi) for ev in loc["eigenvalues"]] == [pytest.approx(f(m), rel=1e-6) for m in (0.5, 1, 2)]
        with pytest.raises(urllib.error.HTTPError):
            post(base + "api/modes", {"t": 0.1, "overrides": {"parameters.nope": 1.0, "bodies.nope.mass": 1.0}})
    finally:
        httpd.shutdown()


def test_sweep_in_the_viewer(tmp_path):
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    model = tmp_path / "p.yaml"
    write_pendulum(model, stiffness=10.0)
    view = server.View(model, runtime=runtime)
    view.rebuild()
    httpd, base = serve_view(view)
    try:
        state = post(base + "api/sweep", {"path": "forces.s.stiffness", "values": [5.0, 50.0, 500.0]})
        assert state["sweep"]["path"] == "forces.s.stiffness" and state["sweep"]["values"] == [5.0, 50.0, 500.0]
        t0 = time.time()
        while json.loads(get(base + "api/state"))["sweep"]["status"] == "running" and time.time() - t0 < 60:
            time.sleep(0.05)
        sw = json.loads(get(base + "api/state"))["sweep"]
        assert sw["status"] == "done" and sw["results"] == ["ok", "ok", "ok"]
        assert get(base + "api/sweep/2.csv").decode().startswith("t,rod.R_x")
        assert json.loads(get(base + "api/state"))["runs"][-1]["id"] == 1  # a sweep is not a run
        with pytest.raises(urllib.error.HTTPError):
            post(base + "api/sweep", {"path": "forces.s.stiffness", "values": [1.0]})  # 2 values at least
        assert post(base + "api/sweep", {"clear": True})["sweep"] is None
        with pytest.raises(urllib.error.HTTPError):
            get(base + "api/sweep/0.csv")
    finally:
        httpd.shutdown()
