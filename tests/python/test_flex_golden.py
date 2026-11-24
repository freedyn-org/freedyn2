"""Golden results of flexible body models: the legacy check-in tests 03_FlyingFleBo and
04_FlexRotUHackerl (me_testing), as freedyn2 computes them. The comparison with legacy FreeDyn
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
MODELS = {"flying_beam": ("flex_beam", 26, 1), "rotating_hook": ("struct1", 20, 20)}  # body, modes, every n-th row


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
    worst = int(np.argmax(err))
    assert err[worst] < 1e-6, f"{cols[worst]}: {err[worst]:.2e}"
