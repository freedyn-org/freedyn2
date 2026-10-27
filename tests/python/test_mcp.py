"""MCP server (fdgen.mcp_server): the tools as Python calls, and the protocol over stdio."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from fdgen import cli
from fdgen.mcp_server import Session

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = str(ROOT / "examples" / "double_pendulum.yaml")


@pytest.fixture
def session(tmp_path):
    try:
        runtime = cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    s = Session(tmp_path / "runs", runtime)
    yield s
    s.close()


def test_guide_and_schema():
    s = Session()
    g = s.guide()
    assert "GLOBALLY AT t = 0" in g and "## force" in g and "double_pendulum" in g
    assert json.loads(s.schema("joint"))["properties"]["type"]["enum"][0] == "fixed"
    assert "checks" in json.loads(s.schema())["properties"]
    with pytest.raises(ValueError, match="no section"):
        s.schema("nonsense")


def test_validate():
    s = Session()
    assert s.validate(PENDULUM).startswith("valid\n2 bodies")
    assert s.validate("missing.yaml").startswith("INVALID")


def test_simulate_results_compare_check(session, tmp_path):
    out = session.simulate(PENDULUM, end_time=1.0)
    assert out.startswith("run r1: double_pendulum, t = 0 .. 1 s, 101 outputs")
    assert "rod2.R_z" in session.channels("r1", "rod2 r_z")
    table = session.results("r1", ["rod2.R_z"], t_start=0.5, points=6)
    lines = table.splitlines()
    assert lines[1].startswith("rod2.R_z: ") and lines[3] == "t,rod2.R_z" and len(lines) == 4 + 6
    assert float(lines[4].split(",")[0]) == pytest.approx(0.5)
    with pytest.raises(ValueError, match="unknown channels"):
        session.results("r1", ["nope"])
    heavier = tmp_path / "heavier.yaml"
    heavier.write_text(Path(PENDULUM).read_text().replace("mass: 1.0\n    inertia: [0.01, 0.0833333333333333, "
                                                          "0.0833333333333333]\n    position: [1.5",
                                                          "mass: 1.5\n    inertia: [0.01, 0.0833333333333333, "
                                                          "0.0833333333333333]\n    position: [1.5"))
    session.simulate(str(heavier), end_time=1.0)
    session.simulate(PENDULUM, end_time=1.0)
    assert session.compare("r1", "r3").endswith("70 the same (up to round-off)")
    diff = session.compare("r1", "r2", top=3).splitlines()
    assert "largest differences" in diff[0] and len(diff) == 4 and "max |difference|" in diff[1]
    assert "degrees of freedom" in session.check(PENDULUM, "r1")
    with pytest.raises(ValueError, match="unknown run"):
        session.compare("r1", "r9")


def test_view_serves_the_model(session):
    import urllib.request

    msg = session.view(PENDULUM, open_browser=False)
    url = msg.split()[1]
    with urllib.request.urlopen(url + "api/state", timeout=10) as r:
        assert json.loads(r.read())["status"] == "ready"
    assert session.view(PENDULUM, open_browser=False).startswith("viewer already open")


def test_notes_and_modes(session, tmp_path):
    from fdgen.view import notes

    model = tmp_path / "pendulum.yaml"
    model.write_text(Path(PENDULUM).read_text())
    assert session.notes(str(model)).startswith("no notes")
    notes.add_note(model, "rod2 swings too far", {"time": 1.25, "selection": "rod2"}, b"\x89PNG\r\n\x1a\nx")
    out = session.notes(str(model))
    assert out.startswith("note 1 (open") and "rod2 swings too far" in out and "selected: rod2" in out
    assert session.note_image(str(model), 1).startswith(b"\x89PNG")
    assert "answered (done)" in session.answer_note(str(model), 1, "added damping to hinge2")
    assert session.notes(str(model)) == "no open notes"
    assert "answer: added damping" in session.notes(str(model), everything=True)
    session.simulate(str(model), end_time=0.1)
    text = session.modes(str(model), "r1", 0.0)
    assert text.startswith("eigenmodes about t = 0 s")
    table = session.locus(str(model), "r1", "bodies.rod2.mass", [0.5, 2.0], 0.0)
    lines = table.splitlines()
    assert lines[0].startswith("eigenfrequencies (damping ratio) about the state of r1")
    assert lines[1].startswith("bodies.rod2.mass = 0.5: ") and lines[2].startswith("bodies.rod2.mass = 2: ")
    table = session.sweep(str(model), "bodies.rod2.mass", [0.5, 2.0], ["rod2.R_z"], "min", 0.0, 1.0, end_time=1.0).splitlines()
    assert table[0].startswith("min over t = 0.0 .. 1.0 s") and table[1] == "run,bodies.rod2.mass,rod2.R_z"
    run_a, _, z_a = table[2].split(",")
    run_b, _, z_b = table[3].split(",")
    assert float(z_a) != float(z_b) and session.run(run_b).results["t"][-1] == pytest.approx(1.0)


def test_screenshot(session):
    pytest.importorskip("playwright")
    if not os.environ.get("FREEDYN2_CHROMIUM"):
        pytest.skip("set FREEDYN2_CHROMIUM (a Chromium for playwright) to test screenshots")
    session.simulate(PENDULUM, end_time=1.0)
    png = session.screenshot(PENDULUM, "r1", time=0.5, channels=["rod2.R_z"], select="rod2", width=800, height=600)
    assert png[:8] == b"\x89PNG\r\n\x1a\n" and len(png) > 20000


def test_stdio_protocol(tmp_path):
    pytest.importorskip("mcp")
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    env = dict(os.environ, PYTHONPATH=str(ROOT / "generator"))
    p = subprocess.Popen([sys.executable, "-m", "fdgen.cli", "mcp", "--workdir", str(tmp_path)], cwd=ROOT,
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)

    def call(method, params=None, id=None):
        msg = {"jsonrpc": "2.0", "method": method, **({"params": params} if params is not None else {}),
               **({"id": id} if id is not None else {})}
        p.stdin.write(json.dumps(msg) + "\n")
        p.stdin.flush()
        return json.loads(p.stdout.readline()) if id is not None else None

    try:
        init = call("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                   "clientInfo": {"name": "test", "version": "0"}}, id=1)
        assert init["result"]["serverInfo"]["name"] == "freedyn2"
        call("notifications/initialized")
        tools = {t["name"] for t in call("tools/list", {}, id=2)["result"]["tools"]}
        assert tools == {"guide", "schema", "validate", "simulate", "channels", "results", "compare", "check",
                         "view", "screenshot", "modes", "locus", "sweep", "notes", "note_image", "answer_note"}
        r = call("tools/call", {"name": "simulate", "arguments": {"model_file": PENDULUM, "end_time": 0.5}}, id=3)
        assert r["result"]["content"][0]["text"].startswith("run r1: double_pendulum")
        r = call("tools/call", {"name": "results", "arguments": {"run_id": "r1", "channels": ["rod1.R_x"],
                                                                 "points": 3}}, id=4)
        assert "t,rod1.R_x" in r["result"]["content"][0]["text"]
    finally:
        p.stdin.close()
        p.wait(timeout=30)
