"""Generated C code compiled with the system C compiler, called via ctypes, vs CasADi."""

import ctypes

import numpy as np
import pytest

from conftest import COEF, P, random_state
from fdgen import plugin
from fdgen.codegen import generate_elements


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    try:
        plugin.find_compiler()
    except RuntimeError:
        pytest.skip("no C compiler")
    d = tmp_path_factory.mktemp("gen")
    generate_elements(d)
    so = d / ("librb" + plugin.LIB_SUFFIX)
    plugin.compile_library([d / "rigid_body.c", d / "rigid_body_euler.c"], so, cache=False)
    return ctypes.CDLL(str(so))


def call(lib, name, inputs, sizes):
    f = getattr(lib, name)
    dbl_p = ctypes.POINTER(ctypes.c_double)
    ins = [np.ascontiguousarray(a, dtype=float) for a in inputs]
    outs = [np.zeros(n) for n in sizes]
    arg = (dbl_p * len(ins))(*[a.ctypes.data_as(dbl_p) for a in ins])
    res = (dbl_p * len(outs))(*[o.ctypes.data_as(dbl_p) for o in outs])
    assert f(arg, res, None, None, 0) == 0
    return outs


def test_generated_c_matches_casadi(lib, elem, lie, rng):
    for _ in range(3):
        q, qd, qdd, lam = random_state(rng, unit=False)
        args = [q, qd, qdd, lam, P, COEF, rng.normal(size=7), [0.7]]
        for fun in (elem.res, elem.iter, elem.iter_noqv, elem.ic, elem.jac_qd):
            sizes = [fun.sparsity_out(i).nnz() for i in range(fun.n_out())]
            got = call(lib, fun.name(), args, sizes)
            ref = fun.call(args)
            for g, r in zip(got, ref):
                np.testing.assert_allclose(g, np.array(r.nz).ravel(), rtol=1e-13, atol=1e-13)
        # Lie group body: 6 velocity dofs, no constraint; rotation increments up to ~1 rad.
        args = [q, qd[:6], qdd[:6], np.zeros(0), P, COEF, rng.normal(size=6), [0.7]]
        for fun in (lie.res, lie.iter, lie.ic):
            sizes = [fun.sparsity_out(i).nnz() for i in range(fun.n_out())]
            got = call(lib, fun.name(), args, sizes)
            ref = fun.call(args)
            for g, r in zip(got, ref):
                np.testing.assert_allclose(g, np.array(r.nz).ravel(), rtol=1e-13, atol=1e-13)


def test_no_casadi_dependency(tmp_path):
    generate_elements(tmp_path)
    src = (tmp_path / "rigid_body.c").read_text()
    assert "#include <casadi" not in src
    reg = (tmp_path / "fd_registry.c").read_text()
    for role in ("res", "iter", "ic", "init", "out", "update", "scale"):
        assert f"fd_rigid_body_{role}_sparsity_out" in reg
    for role in ("iter_noqv", "mn_qd", "jac_qd"):
        assert f"fd_rigid_body_euler_{role}_sparsity_out" in reg


def test_cmake_lists_all_element_types():
    import re

    from conftest import ROOT
    from fdgen.codegen import ELEMENT_BUILDERS

    text = (ROOT / "CMakeLists.txt").read_text()
    listed = re.search(r"set\(FD_ELEMENT_TYPES(.*?)\)", text, re.S).group(1).split()
    assert sorted(listed) == sorted(ELEMENT_BUILDERS)
