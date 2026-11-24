"""Own linear FEM for flexible bodies (fdgen.fem): elements, Nastran reader, RBE2 / RBE3,
Craig-Bampton reduction and inertia invariants against analytic results.

Checked once against MSC Nastran / Adams MTX files of the legacy check-in tests (docs/findings.md);
here: a steel beam 200 x 10 x 10 mm (mm, t, s), meshed in the test."""

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.errors import ModelError
from fdgen.fem import assemble, elements, read_nastran, reduce
from fdgen.fem.reduce import find_modal

E, NU, RHO = 210000.0, 0.3, 7.85e-9
L, H = 200.0, 10.0


def _rigid(X):
    out = []
    for a in range(3):
        u = np.zeros_like(X)
        u[:, a] = 1
        out.append(u.ravel())
        e = np.zeros(3)
        e[a] = 1
        out.append(np.cross(e, X).ravel())
    return np.array(out).T


@pytest.mark.parametrize("kind", ["tet4", "tet10", "hex8"])
def test_patch_and_rigid(kind):
    """Rigid body motions free of stress, a constant strain exactly (also on distorted
    elements, HEX8 with incompatible modes), the mass."""
    rng = np.random.default_rng(1)
    if kind == "hex8":
        X = elements.HEX_CORNERS * [2, 1, 0.5] + 0.15 * rng.normal(size=(8, 3))
    else:
        X = np.array([[0, 0, 0], [2, 0, 0], [0, 3, 0], [0, 0, 1.5]], float) + 0.1 * rng.normal(size=(4, 3))
        if kind == "tet10":
            X = np.vstack([X] + [(X[i] + X[j]) / 2 for i, j in ((0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3))])
    K, M, (_, w, _) = elements.solid_matrices(X[None], E, NU, RHO)
    K, M = K[0], M[0]
    assert np.abs(K @ _rigid(X)).max() < 1e-12 * np.abs(K).max()
    G = 1e-3 * rng.normal(size=(3, 3))
    u = (X @ G.T).ravel()
    eps = np.array([G[0, 0], G[1, 1], G[2, 2], G[0, 1] + G[1, 0], G[1, 2] + G[2, 1], G[0, 2] + G[2, 0]])
    vol = w.sum() / RHO
    assert u @ K @ u == pytest.approx(eps @ elements.elasticity(E, NU) @ eps * vol, rel=1e-12)
    assert M.sum() / 3 == pytest.approx(RHO * vol, rel=1e-12)


def test_beam_element():
    """Cantilever of one element: tip deflections and rotations with shear deformation
    (Timoshenko), rigid body motions free of stress."""
    r, Lb = 1.0, 4.0
    A, I = np.pi * r * r, np.pi * r ** 4 / 4
    K, _, _ = elements.beam_matrices(np.zeros((1, 3)), np.array([[Lb, 0, 0]]), np.array([[0, 0, 1.0]]), E, NU,
                                     RHO, A, I, I, 2 * I)
    K = K[0]
    u = np.linalg.solve(K[6:, 6:], [0, 1.0, 0, 0, 0, 0])
    kappa, G = 6 * (1 + NU) / (7 + 6 * NU), E / (2 * (1 + NU))
    assert u[1] == pytest.approx(Lb ** 3 / (3 * E * I) + Lb / (kappa * G * A), rel=1e-12)
    assert u[5] == pytest.approx(Lb ** 2 / (2 * E * I), rel=1e-12)


def beam_bdf(path, nx=40, ny=2, nz=2, rbe="rbe2", tet=False):
    """Bulk data of the beam [0, L] x [-H/2, H/2]^2: HEX8 (or 6 TET10 per cube), an RBE2 (or RBE3)
    at each end to the nodes 1001 (x = 0) and 1002 (x = L)."""
    xs, ys, zs = np.linspace(0, L, nx + 1), np.linspace(-H / 2, H / 2, ny + 1), np.linspace(-H / 2, H / 2, nz + 1)
    nid = lambda i, j, k: 1 + i + (nx + 1) * (j + (ny + 1) * k)
    lines = ["$ beam", "BEGIN BULK"]
    nodes = {}
    for k, z in enumerate(zs):
        for j, y in enumerate(ys):
            for i, x in enumerate(xs):
                nodes[nid(i, j, k)] = (x, y, z)
    cells = []
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                cells.append([nid(i, j, k), nid(i + 1, j, k), nid(i + 1, j + 1, k), nid(i, j + 1, k),
                              nid(i, j, k + 1), nid(i + 1, j, k + 1), nid(i + 1, j + 1, k + 1), nid(i, j + 1, k + 1)])
    eid, mids, solids = 1, {}, []
    if tet:  # 6 tetrahedra per cube (Kuhn), midside nodes
        nxt = [10001]  # midside nodes, after the masters 1001, 1002

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in mids:
                mids[key] = nxt[0]
                nodes[nxt[0]] = tuple((np.array(nodes[a]) + nodes[b]) / 2)
                nxt[0] += 1
            return mids[key]
        for c in cells:
            for t in ((0, 1, 2, 6), (0, 2, 3, 6), (0, 3, 7, 6), (0, 7, 4, 6), (0, 4, 5, 6), (0, 5, 1, 6)):
                v = [c[i] for i in t]
                p = [np.array(nodes[i]) for i in v]
                if np.dot(np.cross(p[1] - p[0], p[2] - p[0]), p[3] - p[0]) < 0:
                    v[1], v[2] = v[2], v[1]
                solids.append(v + [mid(v[0], v[1]), mid(v[1], v[2]), mid(v[2], v[0]), mid(v[0], v[3]),
                                   mid(v[1], v[3]), mid(v[2], v[3])])
    else:
        solids = cells
    nodes[1001], nodes[1002] = (0.0, 0.0, 0.0), (L, 0.0, 0.0)
    for i, (x, y, z) in nodes.items():
        lines.append(f"GRID,{i},,{float(x)!r},{float(y)!r},{float(z)!r}")
    for s in solids:
        lines.append(f"{'CTETRA' if tet else 'CHEXA'},{eid},1," + ",".join(map(str, s)))
        eid += 1
    lines += ["PSOLID,1,1", f"MAT1,1,{E},,{NU},{RHO}"]
    for master, i in ((1001, 0), (1002, nx)):
        face = [nid(i, j, k) for j in range(ny + 1) for k in range(nz + 1)]
        if rbe == "rbe2":
            lines.append(f"RBE2,{eid},{master},123456," + ",".join(map(str, face)))
        else:
            lines.append(f"RBE3,{eid},,{master},123456,1.0,123," + ",".join(map(str, face)))
        eid += 1
    lines.append("ENDDATA")
    path.write_text("\n".join(lines) + "\n")
    return path


@pytest.fixture
def modal_tool():
    try:
        return find_modal()
    except FileNotFoundError:
        pytest.skip("freedyn2_modal not built")


def test_reader(tmp_path):
    fe = read_nastran(beam_bdf(tmp_path / "b.bdf", nx=4, ny=1, nz=1, rbe="rbe3"))
    assert len(fe.solids) == 4 and fe.materials[1] == (E, NU, RHO)
    assert fe.rbe3[0][1] == 1001 and fe.rbe3[0][3][0][1] == "123" and len(fe.rbe3[0][3][0][2]) == 4
    # small and large field, implicit exponents, continuations
    (tmp_path / "s.bdf").write_text(
        "GRID           1       0      0.      0.      0.\n"
        "GRID*                  2               0   1.000000e+001   0.000000e+000\n"
        "*          0.000000e+000\n"
        "MAT1           1  2.1+5              .3  7.85-9\n")
    fe = read_nastran(tmp_path / "s.bdf")
    assert fe.nodes[2] == (10.0, 0.0, 0.0) and fe.materials[1] == pytest.approx((2.1e5, 0.3, 7.85e-9))


def euler_bernoulli(beta):
    I, A = H ** 4 / 12, H * H
    return beta ** 2 / (2 * np.pi * L ** 2) * np.sqrt(E * I / (RHO * A))


@pytest.mark.parametrize("tet", [False, True])
def test_free_beam(tmp_path, modal_tool, tet):
    """Free-free beam (one interface, Craig-Bampton): mass, centre of mass and inertia of the box
    exactly, the first bending frequencies (two equal ones, square section) near Euler-Bernoulli
    (beta L = 4.730, 7.853; shear and the mesh lower them by a few per cent), mass-normalized
    modes with no translation of the centre of mass (I3 = 0)."""
    a = assemble(read_nastran(beam_bdf(tmp_path / "b.bdf", nx=20 if tet else 40, tet=tet)))
    d = reduce(a, [1001], 12, tool=modal_tool)
    m = RHO * L * H * H
    assert d.mass == pytest.approx(m, rel=1e-12)
    assert d.center_of_mass() == pytest.approx([L / 2, 0, 0], abs=1e-9)
    c = d.center_of_mass()
    Jc = d.I7 - m * (c @ c * np.eye(3) - np.outer(c, c))
    assert np.diag(Jc) == pytest.approx([m * 2 * H * H / 12, m * (L * L + H * H) / 12, m * (L * L + H * H) / 12],
                                        rel=1e-10)
    f = d.frequencies
    assert f[0] == pytest.approx(f[1], rel=1e-3 if tet else 1e-9)  # the tet mesh is not symmetric
    assert f[0] == pytest.approx(euler_bernoulli(4.7300), rel=0.03)
    assert np.abs(d.I3).max() < 1e-9 * np.sqrt(m)  # mass-normalized modes: I3 ~ sqrt(m)
    assert np.allclose(d.I6, np.eye(d.nm), atol=1e-9)
    assert np.allclose(d.K, np.diag((2 * np.pi * f) ** 2))
    # symmetries of the invariants (fdgen.flex.modal)
    assert np.allclose(d.I5, -d.I5.transpose(2, 1, 0), atol=1e-9 * np.abs(d.I5).max())
    assert np.allclose(d.I9, d.I9.transpose(1, 0, 3, 2), atol=1e-9 * np.abs(d.I9).max())


def test_craig_bampton_converges_to_free_modes(tmp_path, modal_tool):
    """The orthonormalized Craig-Bampton modes are Ritz approximations of the free-free modes:
    above them, and equal to them with enough fixed-interface modes; RBE2 and RBE3 at both ends."""
    a = assemble(read_nastran(beam_bdf(tmp_path / "b.bdf", nx=20)))
    free = reduce(a, [], 16, tool=modal_tool).frequencies
    for nm, n, tol in ((4, 4, 0.02), (24, 6, 1e-3)):
        f = reduce(a, [1001, 1002], nm, tool=modal_tool).frequencies[:n]
        assert np.all(f >= free[:n] * (1 - 1e-9)) and f == pytest.approx(free[:n], rel=tol)
    a3 = assemble(read_nastran(beam_bdf(tmp_path / "b3.bdf", nx=20, rbe="rbe3")))
    free3 = reduce(a3, [], 16, tool=modal_tool).frequencies
    f3 = reduce(a3, [1001, 1002], 24, tool=modal_tool).frequencies[:6]
    assert f3 == pytest.approx(free3[:6], rel=1e-3)
    assert np.all(free3[:6] < free[:6])  # RBE3 does not stiffen the end faces, RBE2 does


def test_mechanism_is_an_error(tmp_path, modal_tool):
    """A model in two pieces: more than 6 rigid body modes."""
    p = beam_bdf(tmp_path / "b.bdf", nx=4, ny=1, nz=1)
    text = p.read_text().replace("ENDDATA", "GRID,5001,,0.,20.,0.\nGRID,5002,,1.,20.,0.\nGRID,5003,,0.,21.,0.\n"
                                            "GRID,5004,,0.,20.,1.\nCTETRA,99,1,5001,5002,5003,5004\nENDDATA")
    p.write_text(text)
    with pytest.raises(ModelError, match="rigid body modes|restrain"):
        reduce(assemble(read_nastran(p)), [1001], 4, tool=modal_tool)


def test_cantilever_from_fem(tmp_path, modal_tool, monkeypatch):
    """A flexible body from the mesh (flex.fem) clamped at node 1001: the first bending
    frequency of the cantilever (beta L = 1.875) from the linearized model."""
    try:
        cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")
    from fdgen.linearize import analyze
    monkeypatch.setenv("FREEDYN2_HOME", str(tmp_path / "home"))
    beam_bdf(tmp_path / "beam.bdf", nx=40)
    m = {"model": "cantilever", "bodies": {"beam": {"flex": {"fem": "beam.bdf", "interfaces": [1001, 1002],
                                                             "modes": 10}}},
         "joints": {"clamp": {"type": "fixed", "body1": "beam", "node": 1001}},
         "solver": {"type": "HHT", "alpha": -0.05, "step": 1e-5, "end_time": 2e-5}}
    path = tmp_path / "cantilever.yaml"
    path.write_text(yaml.safe_dump(m))
    fdm, model, _ = cli.build(path, tmp_path, "v2")
    assert model["joints"][0]["location"] == pytest.approx([0, 0, 0])
    cli.simulate(cli.find_runtime(), fdm, tmp_path / "r.csv")
    lam, _ = analyze(path, fdm, tmp_path / "r.csv", 0.0)
    f = np.sort(np.abs(lam.imag[lam.imag > 1e-6])) / (2 * np.pi)
    assert f[0] == pytest.approx(f[1], rel=1e-6)
    assert f[0] == pytest.approx(euler_bernoulli(1.8751), rel=0.02)
    assert (tmp_path / "cms" / "beam.mtx").is_file() and (tmp_path / "cms" / "beam.json").is_file()  # kept
