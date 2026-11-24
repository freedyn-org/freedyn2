"""Golden results of flexible body models: the legacy check-in tests 03_FlyingFleBo,
04_FlexRotUHackerl and 106_V8_FlexConRod (me_testing; 8 flexible connecting rods on massless
bodies, mm-t-s: Newton at the round-off floor), as freedyn2 computes them. The comparison with legacy FreeDyn
that these results were checked with is documented in docs/findings.md; it is not repeated.

FD_UPDATE_GOLDEN=1 python -m pytest tests/python/test_flex_golden.py  writes them anew.
"""

import csv
import os

import numpy as np
import pytest

from fdgen import cli

from conftest import ROOT

DATA = ROOT / "tests" / "data" / "flex"
MODELS = {"flying_beam": ("flex_beam", 26, 1), "rotating_hook": ("struct1", 20, 20),  # body, modes, every n-th row
          "v8_flex_conrods": ("con_rod.1", 21, 5)}
# Tolerances (body frame channels, modal coordinates), relative to the largest value of a
# channel; default 1e-6 for both. 106_V8_FlexConRod runs Newton at the round-off floor
# (massless bodies, mm-t-s): its results are defined to about 1e-6 (body frame) and 1e-5 (modal
# coordinates) only. Another linear solver on the same machine (--linear-solver sparselu or
# dense instead of klu) moves the body frame by 1.2e-6 and eta1 by 8e-6; MSVC on Windows eta1
# by 4e-6.
TOLERANCE = {"v8_flex_conrods": (5e-6, 3e-5)}


@pytest.fixture
def runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def channels(body, nm):
    out = [f"{body}.R_{c}" for c in "xyz"] + [f"{body}.theta{k}" for k in range(4)]
    return out + [f"{body}.eta{i + 1}.eta" for i in range(nm)]


@pytest.mark.parametrize("name", MODELS)
def test_golden(tmp_path, runtime, name):
    body, nm, every = MODELS[name]
    fdm, _, _ = cli.build(DATA / f"{name}.fds", tmp_path, "v2")
    cli.simulate(runtime, fdm, tmp_path / "r.csv")
    with open(tmp_path / "r.csv", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))[::every]
    cols = ["t"] + channels(body, nm)
    got = np.array([[float(r[c]) for c in cols] for r in rows])
    golden = DATA / f"{name}.golden.csv"
    if os.environ.get("FD_UPDATE_GOLDEN"):
        with open(golden, "w", encoding="utf-8", newline="\n") as f:
            f.write(",".join(cols) + "\n")
            for r in got:
                f.write(",".join(repr(float(x)) for x in r) + "\n")
    with open(golden, encoding="utf-8") as f:
        ref_rows = list(csv.reader(f))
    assert ref_rows[0] == cols
    ref = np.array(ref_rows[1:], dtype=float)
    assert got.shape == ref.shape and np.allclose(got[:, 0], ref[:, 0], rtol=0, atol=1e-12)
    # per channel: relative to its largest value (round-off of other platforms and compilers)
    scale = np.maximum(np.abs(ref).max(axis=0), 1e-30)
    err = (np.abs(got - ref) / scale).max(axis=0)
    body_tol, eta_tol = TOLERANCE.get(name, (1e-6, 1e-6))
    tol = np.array([eta_tol if ".eta" in c else body_tol for c in cols])
    worst = int(np.argmax(err / tol))
    assert err[worst] < tol[worst], f"{cols[worst]}: {err[worst]:.2e}"
