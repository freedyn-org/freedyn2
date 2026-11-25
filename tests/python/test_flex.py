"""Flexible bodies from Adams MTX files: reader, joints at nodes, .fds import, runs.

examples/data/flex_beam.mtx: the beam of the legacy check-in test 03_FlyingFleBo (me_testing),
200 mm x 10.4 mm x 10.4 mm steel, C3D10 elements, mm-kg-s, two interface nodes 10000 (z = 100)
and 10001 (z = -100), 26 Craig-Bampton modes (orthonormalized, mass normalized).
"""

import casadi as ca
import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen import flex_body as fb
from fdgen import rigid_body as rb
from fdgen.errors import ModelError
from fdgen.flex import read_mtx
from fdgen.fds import load_fds
from fdgen.linearize import analyze

from conftest import ROOT

MTX = ROOT / "examples" / "data" / "flex_beam.mtx"


@pytest.fixture(scope="module")
def beam():
    return read_mtx(MTX)


def test_mtx_reader(beam):
    assert beam.nm == 26 and list(beam.node_labels) == [10000, 10001]
    assert beam.mass == pytest.approx(0.1572)
    assert np.allclose(beam.node_positions, [[0, 0, 100], [0, 0, -100]])
    assert beam.frequencies[0] == pytest.approx(1289.79635591523)
    # mass normalized modes, K = omega^2
    assert np.allclose(beam.I6, np.eye(26))
    assert np.allclose(np.diag(beam.K), (2 * np.pi * beam.frequencies) ** 2, rtol=1e-9)
    # symmetries of the invariants (a check of the block layout): I5_j[:, k] = -I5_k[:, j],
    # I9_jk = I9_kj^T, I8 rows
    assert np.abs(beam.I5).max() > 0 and np.allclose(beam.I5, -beam.I5.transpose(2, 1, 0))
    assert np.abs(beam.I9).max() > 0 and np.allclose(beam.I9, beam.I9.transpose(1, 0, 3, 2))
    assert np.allclose(beam.I7, beam.I7.T) and beam.I7[0, 0] == pytest.approx(525.5196)


@pytest.mark.parametrize("kind", ["transfix_bb", "rotfix_bb", "transfix_ground", "rotfix_ground"])
def test_node_terms_add_up(kind):
    """Rigid form of a joint primitive at the node position plus the mode terms = the primitive
    with the node point u + S eta and the node axes (I + skew(Psi eta)) b of the flexible body."""
    from fdgen.joints import build
    rng = np.random.default_rng(7)
    nm = 3
    S, Psi = rng.normal(size=(3, nm)), 0.1 * rng.normal(size=(3, nm))
    two = kind.endswith("_bb")
    nb = 2 if two else 1
    qs = []
    for _ in range(nb):
        th = rng.normal(size=4)
        qs.append(np.concatenate([rng.normal(size=3), th / np.linalg.norm(th)]))
    eta = 0.2 * rng.normal(size=nm)
    p = rng.normal(size=build(kind).res.size1_in(4))
    A = [np.array(rb.rotation_matrix(ca.DM(q[3:7]))) for q in qs]
    R = [q[0:3] for q in qs]
    # the same primitive written with the deformed node (fdgen.joints conventions, flexible body last)
    d = S @ eta
    rot = np.eye(3) + np.array(rb.skew(ca.DM(Psi @ eta)))
    if kind == "transfix_bb":
        ref = (R[1] + A[1] @ (p[6:9] + d) - R[0]) @ (A[0] @ p[0:3]) - p[9]
    elif kind == "rotfix_bb":
        ref = (A[0] @ p[0:3]) @ (A[1] @ rot @ p[3:6]) - p[6]
    elif kind == "transfix_ground":
        ref = p[0:3] @ (R[0] + A[0] @ (p[3:6] + d)) - p[6]
    else:
        ref = (A[0] @ rot @ p[0:3]) @ p[3:6] - p[6]

    def phi(fun, q, n_v):
        _, _, phi_, _ = fun.ic(q, np.zeros(n_v), [], [], np.asarray(fun_p[fun]), np.zeros(4), [], 0)
        return float(np.array(phi_).ravel()[-1])

    base = build(kind)
    fun_p = {base: p}
    total = phi(base, np.concatenate(qs), 6 * nb)
    for i, (nkind, pp) in enumerate(fb.node_terms(kind, p, S, Psi)):
        f = fb.build_node(nkind)
        fun_p[f] = pp
        total += phi(f, np.concatenate([*qs, [eta[i]]]), 6 * nb + 1)
    assert total == pytest.approx(ref, rel=1e-12, abs=1e-12)


def test_fds_import(tmp_path, beam):
    """Body MBS_FlexBody_Full and Constraint FixRiBoFleBo with a Windows path to the MTX file
    (found next to the model)."""
    (tmp_path / "flex_beam.mtx").write_bytes(MTX.read_bytes())
    fds = tmp_path / "m.fds"
    fds.write_text("""Body Rigid_EulerParameter_PAI
  BodyName = dummy
  BodyID = 1
  Mass = 1.0
  Inertia = 1, 1, 1
  LocationCenterMass = 0, 0, -100
  OrientationPAI = 1,0,0,0,1,0,0,0,1
  InitialVelocityVector = 0, 0, 0
  InitialOmegaVector = 0, 0, 0
EndBody
Body MBS_FlexBody_Full
  BodyID = 2
  BodyName = beam
  Path2MtxFile = C:\\somewhere\\03_FlyingFleBo\\flex_beam.mtx
  Is_Itf_active = yes
  Is_QuadraticTermOF_Itt_active = no
  LocationCenterMass = 0, 0, 0
  OrientationPAI = 1,0,0,0,1,0,0,0,1
  InitialVelocityVector = 0, 0, 0
  InitialOmegaVector = 0, 0, 0
  GlobalDampingFactor = 0.02, 0.05
EndBody
Constraint FixRiBoFleBo
  ConstraintID = 1
  Location = 0, 0, -100
  BodyIDs = 1, 2
  InterfaceNode = 10001
EndConstraint
Simulation Dynamic
  SimulationTimeEnd = 1e-5
  OutputTimeStepSize = 1e-6
EndSimulation
Solver HHT
  Alpha = -0.05
  MaxNumberOfInnerIterations = 10, 0
  GlobalErrorTolerance = 1e-6
  SolverMaxTimeStep = 1e-6
EndSolver
""")
    m = load_fds(fds, "v2")
    b = m["bodies"]["beam"]
    assert b["flex"]["damping"] == [0.02, 0.05] and b["flex"]["rotation_coupling"]
    assert not b["flex"]["quadratic_inertia"]
    assert b["mass"] == pytest.approx(beam.mass)
    j = m["joints"][0]
    assert (j["body1"], j["body2"], j["node"], j["fix"]) == ("dummy", "beam", 10001, ["dx", "dy", "dz", "ax", "ay", "az"])
    with pytest.raises(ModelError, match="profile v2"):
        load_fds(fds, "legacy")


# --- runs (need the runtime) ---------------------------------------------------------------------


@pytest.fixture
def runtime():
    try:
        return cli.find_runtime()
    except FileNotFoundError:
        pytest.skip("freedyn2_run not built")


def build_run(model, work):
    path = work / f"{model['model']}.yaml"
    path.write_text(yaml.safe_dump(model))
    fdm, _, _ = cli.build(path, work, "v2")
    cli.simulate(cli.find_runtime(), fdm, work / "r.csv")
    return path, fdm, work / "r.csv"


def beam_model(name, **extra):
    m = {"model": name, "bodies": {"beam": {"flex": {"modal": str(MTX)}}},
         "solver": {"type": "HHT", "alpha": -0.05, "step": 1e-6, "end_time": 2e-6}}
    m.update(extra)
    return m


def frequencies(path, fdm, csv):
    lam, _ = analyze(path, fdm, csv, 0.0)
    return np.sort(np.abs(lam.imag[lam.imag > 1e-6])) / (2 * np.pi)


def test_free_beam_frequencies(tmp_path, runtime, beam):
    """Linearized free body: exactly the frequencies of the modal data (6 rigid body modes 0)."""
    f = frequencies(*build_run(beam_model("free"), tmp_path))
    assert np.allclose(f, beam.frequencies, rtol=1e-10)


def test_cantilever_frequencies(tmp_path, runtime, beam):
    """Clamped at node 10001 by a fixed joint: the first two bending frequencies of a cantilever,
    Euler-Bernoulli from the free-free one: f_c = f_ff (beta_c / beta_ff)^2, within the shear
    correction of a beam with L/h = 19 (about 1 %)."""
    m = beam_model("cantilever", joints={"clamp": {"type": "fixed", "body1": "beam", "node": 10001}})
    f = frequencies(*build_run(m, tmp_path))
    f_ff = beam.frequencies[0]
    f1, f2 = f_ff * (1.875104 / 4.730041) ** 2, f_ff * (4.694091 / 4.730041) ** 2
    assert f[0] == pytest.approx(f[1], rel=1e-9)  # square section: two equal bending modes
    assert f[0] == pytest.approx(f1, rel=0.02)
    assert f[2] == pytest.approx(f2, rel=0.02)


def kinetic_and_elastic(d, row, name="beam"):
    """T + V (no gravity) of a flexible body from the result columns."""
    get = lambda k: np.array([float(row[f"{name}.{k}{c}"]) for c in ("_x", "_y", "_z")])
    th = np.array([float(row[f"{name}.theta{k}"]) for k in range(4)])
    A = np.array(rb.rotation_matrix(ca.DM(th)))
    Rd, w = get("Rdot"), get("omega_body")
    e = np.array([float(row[f"{name}.eta{i + 1}.eta"]) for i in range(d.nm)])
    ed = np.array([float(row[f"{name}.eta{i + 1}.etadot"]) for i in range(d.nm)])
    S = d.I2 + d.I3 @ e
    J = d.I7 - np.einsum("j,jab->ab", e, d.I8 + d.I8.transpose(0, 2, 1)) - np.einsum("j,k,jkab->ab", e, e, d.I9)
    Irf = d.I4 + np.einsum("j,jak->ak", e, d.I5)
    sk = np.array(rb.skew(ca.DM(S)))
    T = (0.5 * d.mass * Rd @ Rd + Rd @ A @ (-sk @ w + d.I3 @ ed) + 0.5 * w @ J @ w + w @ Irf @ ed
         + 0.5 * ed @ d.I6 @ ed)
    return T + 0.5 * e @ d.K @ e


def test_flying_beam_conserves_energy(tmp_path, runtime, beam):
    """The beam of 03_FlyingFleBo (spinning at 5000 rad/s, full coupling, no damping) without
    gravity: kinetic plus elastic energy constant to the accuracy of the time integration."""
    import csv
    m = beam_model("flying", solver={"type": "HHT", "alpha": -0.0001, "step": 1e-7, "end_time": 1e-4,
                                     "output_step": 1e-5})
    m["bodies"]["beam"].update(velocity=[0.0, 100.0, 0.0], angular_velocity=[0.0, 1000.0, 5000.0])
    _, _, out = build_run(m, tmp_path)
    rows = list(csv.DictReader(open(out, encoding="utf-8")))
    E = np.array([kinetic_and_elastic(beam, r) for r in rows])
    eta = max(abs(float(rows[-1][f"beam.eta{i + 1}.eta"])) for i in range(beam.nm))
    assert eta > 1e-4  # the beam deforms
    assert np.max(np.abs(E - E[0])) < 1e-6 * E[0]


def test_fdflex_roundtrip(tmp_path, beam):
    """.fdflex keeps the modal data exactly and its meta; MTX -> fdflex -> MTX."""
    from fdgen.flex import read_fdflex, write_fdflex, write_mtx
    from fdgen.flex.fdflex import read_meta
    f = write_fdflex(beam, tmp_path / "b.fdflex", {"source": "test"})
    d = read_fdflex(f)
    for k in ("I2", "I3", "I4", "I5", "I6", "I7", "I8", "I9", "K", "frequencies", "node_labels", "node_positions",
              "Phi", "Psi"):
        assert np.array_equal(getattr(d, k), getattr(beam, k)), k
    assert d.mass == beam.mass and d.has_I5 and read_meta(f) == {"source": "test"} and d.viz is None
    write_mtx(d, tmp_path / "b.mtx")
    assert np.allclose(read_mtx(tmp_path / "b.mtx").I9, beam.I9, rtol=1e-13, atol=0)
    (tmp_path / "x.fdflex").write_bytes(b"PK no")
    with pytest.raises(Exception):
        read_fdflex(tmp_path / "x.fdflex")


def test_set_interface_edits_only_the_flex_block():
    """fdgen.flex.edit: an explicit interface set, changed and removed again; flow and block
    style kept, the rest of the file (comments) untouched."""
    from fdgen.flex.edit import set_interface
    flow = "model: a  # c\nbodies:\n  arm:\n    mass: 1   # rigid\n    flex: {material: steel, mesh_size: 0.004}   # f\n" \
           "    position: [0, 0, 0]\n  other: {mass: 1}\n"
    block = "bodies:\n  arm:\n    flex:\n      material: steel\n      mesh_size: 0.004\n    position: [0, 0, 0]\nsolver: {type: HHT}\n"
    for text in (flow, block):
        a = set_interface(text, "arm", [0, 0, 0.1], radius=0.01, kind="rbe3")
        assert yaml.safe_load(a)["bodies"]["arm"]["flex"]["interfaces"] == [
            {"point": [0.0, 0.0, 0.1], "radius": 0.01, "type": "rbe3"}]
        b = set_interface(a, "arm", [0, 0, 0.1], radius=0.02)
        assert yaml.safe_load(b)["bodies"]["arm"]["flex"]["interfaces"] == [{"point": [0.0, 0.0, 0.1], "radius": 0.02}]
        c = set_interface(b, "arm", [0, 0, 0.1], remove=True)
        assert yaml.safe_load(c) == yaml.safe_load(text)
    f = set_interface(flow, "arm", [1, 0, 0], radius=1)
    assert "# c" in f and "# rigid" in f and "# f" in f and "  other: {mass: 1}" in f
    assert "      - point: [0.0, 0.0, 0.1]" in set_interface(block, "arm", [0, 0, 0.1], radius=0.01)
    with pytest.raises(ModelError, match="no flex block"):
        set_interface(flow, "other", [0, 0, 0])
