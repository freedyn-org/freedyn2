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
