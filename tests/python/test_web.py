"""fdgen.web (the browser demo): YAML to .fdm and the viewer page without CasADi, as in Pyodide."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# Runs in a fresh interpreter in which `import casadi` fails (sys.modules entry None).
SCRIPT = r"""
import sys
sys.modules["casadi"] = None
sys.path.insert(0, sys.argv[1] + "/generator")
from fdgen import web
from fdgen.errors import ModelError

fdm = web.prepare(open(sys.argv[1] + "/examples/pendulum.yaml").read())
assert fdm.startswith("freedyn2_model 1"), fdm[:40]
assert "casadi" not in sys.modules or sys.modules["casadi"] is None
html = web.page("t,rod.R_x\n0,0.5\n0.1,0.5\n")
assert "window.FD_STATIC" in html and "pendulum" in html
try:
    web.prepare(open(sys.argv[1] + "/examples/driven_rotor.yaml").read())
    raise SystemExit("driven_rotor needs CasADi but was prepared")
except ModelError as e:
    assert "desktop freedyn2" in str(e), e
print("ok")
"""


def test_web_without_casadi():
    r = subprocess.run([sys.executable, "-c", SCRIPT, str(ROOT)], capture_output=True, text=True)
    assert r.returncode == 0 and r.stdout.strip() == "ok", r.stdout + r.stderr


def test_fdm_without_casadi_equals_with(tmp_path):
    """The lazy CasADi import (fdgen._casadi) changes nothing in the written model file."""
    from fdgen import web
    from fdgen.model import load_yaml, validate, write_fdm

    y = ROOT / "examples/double_pendulum.yaml"
    write_fdm(validate(load_yaml(y), y.parent), tmp_path / "m.fdm")
    assert web.prepare(y.read_text()) == (tmp_path / "m.fdm").read_text()


def test_web_files(tmp_path):
    """Meshes next to the model (add_file, add_zip), in the viewer page and in the bundle."""
    import io
    import zipfile

    import pytest

    from fdgen import web
    from fdgen.errors import ModelError

    stl = (b"x" * 80) + (0).to_bytes(4, "little")  # a binary STL without triangles
    yaml_text = (ROOT / "examples/pendulum.yaml").read_text().replace(
        "position: [0.5, 0, 0]                                     # center of mass",
        "position: [0.5, 0, 0]\n    shapes: [{type: mesh, file: parts/rod.stl, scale: 0.001}]")
    web.clear_files()
    with pytest.raises(ModelError, match="parts/rod.stl not found") as e:
        web.prepare(yaml_text, "p.yaml")
    assert str(web.WORK) not in str(e.value)
    web.add_file("parts/rod.stl", stl)
    web.prepare(yaml_text, "p.yaml")
    assert web.files() == ["parts/rod.stl"]
    assert "api/mesh/0.stl" in web.page("t,rod.R_x\n0,0.5\n")
    data = web.bundle(yaml_text, "p.yaml")
    assert sorted(zipfile.ZipFile(io.BytesIO(data)).namelist()) == ["p.yaml", "parts/rod.stl"]

    # a zip with one folder around everything (as an export): paths relative to the model
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("export/p.yaml", yaml_text)
        z.writestr("export/parts/rod.stl", stl)
    web.clear_files()
    assert list(web.add_zip(buf.getvalue())) == ["export/p.yaml"]
    assert web.files() == ["parts/rod.stl"]
    with pytest.raises(ValueError):
        web.add_file("../outside.stl", stl)
    web.clear_files()
