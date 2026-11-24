"""Rigid to flex: a body meshed from its STL (fdgen.fem.surface, Netgen), interfaces made for its
joints and forces."""

import struct

import numpy as np
import pytest
import yaml

from fdgen import cli
from fdgen.fem.stl import check_closed, read_stl
from fdgen.errors import ModelError

L, B, H = 0.2, 0.02, 0.01  # steel arm [m]
E, RHO = 210e9, 7850.0


def box_stl(path, size=(L, B, H)):
    """Closed binary STL of a box centred at the origin."""
    lx, ly, lz = size
    v = np.array([[x, y, z] for z in (-lz / 2, lz / 2) for y in (-ly / 2, ly / 2) for x in (-lx / 2, lx / 2)])
    f = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6),
         (1, 3, 5), (3, 7, 5)]
    with open(path, "wb") as o:
        o.write(b"\0" * 80 + struct.pack("<I", len(f)))
        for a, b, c in f:
            o.write(struct.pack("<12fH", 0, 0, 0, *v[a], *v[b], *v[c], 0))
    return path


def arm(flex=None, name="arm"):
    """An arm on a hinge about y at its end x = 0, a spring at its tip; rigid, or flex."""
    m = RHO * L * B * H
    body = {"mass": m, "inertia": [m * (B * B + H * H) / 12, m * (L * L + H * H) / 12, m * (L * L + B * B) / 12],
            "position": [L / 2, 0, 0], "shapes": [{"type": "mesh", "file": "arm.stl", "frame": "body"}]}
    if flex:
        body["flex"] = flex
    return {"model": name, "gravity": [0, 0, -9.81], "bodies": {"arm": body},
            "joints": {"pivot": {"type": "revolute", "body1": "arm", "location": [0, 0, 0],
                                 "orientation": [0.70710678, -0.70710678, 0, 0]}},
            "forces": {"spring": {"type": "spring_damper", "body1": "arm", "point1": [L, 0, 0], "point2": [L, 0, 0.1],
                                  "stiffness": 2000.0, "damping": 1.0, "length": 0.1}},
            "solver": {"type": "HHT", "alpha": -0.05, "step": 1e-4, "end_time": 2e-4}}


@pytest.fixture
def env(tmp_path, monkeypatch):
    pytest.importorskip("netgen.stl")
    try:
        cli.find_runtime()
        from fdgen.fem.reduce import find_modal
        find_modal()
    except FileNotFoundError:
        pytest.skip("runtime programs not built")
    monkeypatch.setenv("FREEDYN2_HOME", str(tmp_path / "home"))
    box_stl(tmp_path / "arm.stl")
    return tmp_path


def frequencies(m, work):
    from fdgen.linearize import analyze
    path = work / f"{m['model']}.yaml"
    path.write_text(yaml.safe_dump(m))
    fdm, model, _ = cli.build(path, work, "v2")
    cli.simulate(cli.find_runtime(), fdm, work / f"{m['model']}.csv")
    lam, _ = analyze(path, fdm, work / f"{m['model']}.csv", 0.0)
    return np.sort(np.abs(lam.imag[lam.imag > 1e-6])) / (2 * np.pi), model


def test_closed_surface_check(tmp_path):
    tri = read_stl(box_stl(tmp_path / "b.stl"))
    check_closed(tri, "box")
    with pytest.raises(ModelError, match="not closed"):
        check_closed(tri[:-1], "box")


def test_rigid_to_flex(env):
    """The same model with flex: {material, mesh_size}: the hinge acts at an interface node, the
    spring on a massless body fixed to another one. The pendulum mode on the spring as for the
    rigid arm; the bending about the weak axis (hinged at one end, free with the soft spring at
    the other: Euler-Bernoulli beta L = 3.9266) within the shear and mesh effects; mass of the
    FE model = mass of the rigid body."""
    f_rigid, _ = frequencies(arm(name="rigid"), env)
    flex = {"material": {"E": E, "nu": 0.3, "rho": RHO}, "mesh_size": 0.005, "modes": 8}
    f_flex, model = frequencies(arm(flex, name="flex"), env)
    b = model["bodies"]["arm"]
    assert b["mass"] == pytest.approx(RHO * L * B * H, rel=1e-6)  # STL coordinates are float32
    assert model["joints"][0]["node"] == b["flex"]["nodes"][0]
    assert model["forces"][0]["body1"] == "arm_if2" and model["bodies"]["arm_if2"]["mass"] == 0.0
    assert f_flex[0] == pytest.approx(f_rigid[0], rel=2e-3)
    I = B * H ** 3 / 12
    f_weak = 3.9266 ** 2 / (2 * np.pi * L ** 2) * np.sqrt(E * I / (RHO * B * H))
    assert any(abs(f / f_weak - 1) < 0.03 for f in f_flex[1:4])
    # the CMS result is kept next to the model
    assert {f.name for f in (env / "cms").iterdir()} == {"arm.fdflex"}


def test_material_name_and_errors(env):
    m = arm({"material": "unobtainium", "mesh_size": 0.005})
    path = env / "m.yaml"
    path.write_text(yaml.safe_dump(m))
    with pytest.raises(ModelError, match="material|schema"):
        cli.build(path, env, "v2")


def test_viewer_scene(env):
    """The scene of a flexible body: its modal coordinate channels and the mode shapes on the
    vertices of its STL (three per triangle), in the shape's coordinates; the massless interface
    body without shapes; mesh shapes with their extent."""
    import base64
    from fdgen.view.scene import scene
    path = env / "flex.yaml"
    path.write_text(yaml.safe_dump(arm({"material": {"E": E, "nu": 0.3, "rho": RHO}, "mesh_size": 0.005, "modes": 4},
                                       name="flex")))
    s = scene(cli.load(path))
    b = {x["name"]: x for x in s["bodies"]}
    flex = b["arm"]["flex"]
    nm = 2 * 6 + 4 - 6  # two interfaces, 4 fixed-interface modes, without the rigid body modes
    assert flex["eta"] == [f"arm.eta{i}.eta" for i in range(1, nm + 1)]
    phi = np.frombuffer(base64.b64decode(flex["modes"]["0"]), "<f4").reshape(nm, 12 * 3, 3)
    assert np.abs(phi).max() > 0
    assert b["arm"]["shapes"][0]["extent"] == pytest.approx(np.sqrt((L / 2) ** 2 + (B / 2) ** 2 + (H / 2) ** 2), rel=1e-6)
    assert b["arm_if2"]["shapes"] == []


def test_export_and_own_fe_model(env):
    """flex-export writes the meshed body as Nastran bulk data; used with flex.fem the joints and
    forces attach to its interface nodes by their points: the same frequencies."""
    from fdgen.fem.surface import export
    flex = {"material": {"E": E, "nu": 0.3, "rho": RHO}, "mesh_size": 0.005, "modes": 6}
    m = arm(flex, name="meshed")
    (env / "meshed.yaml").write_text(yaml.safe_dump(m))
    bdf = export(env / "meshed.yaml", "arm")
    text = bdf.read_text()
    assert "90000001: [0.0, 0.0, 0.0]  rbe2  (joint pivot)" in text and "(force spring)" in text
    f1, _ = frequencies(m, env)
    m2 = arm({"fem": str(bdf), "modes": 6}, name="own")
    f2, model = frequencies(m2, env)
    assert model["joints"][0]["node"] == 90000001 and model["forces"][0]["body1"] == "arm_if2"
    assert f2[:4] == pytest.approx(f1[:4], rel=1e-6)


def test_torque_needs_a_point(env):
    m = arm({"material": {"E": E, "nu": 0.3, "rho": RHO}, "mesh_size": 0.005, "modes": 2}, name="tq")
    m["forces"]["drive"] = {"type": "torque", "body1": "arm", "vector": [0, 1.0, 0]}
    path = env / "tq.yaml"
    path.write_text(yaml.safe_dump(m))
    with pytest.raises(ModelError, match="give point"):
        cli.build(path, env, "v2")
    m["forces"]["drive"]["point"] = [0.1, 0, 0]
    path.write_text(yaml.safe_dump(m))
    _, model, _ = cli.build(path, env, "v2")
    assert model["forces"][1]["body1"] == "arm_if3"


def test_cms_result_kept_and_reused(env):
    """The reduction is kept in cms/<body>.fdflex and reused while the inputs are the same (made again
    when they change); another model uses it with flex.mtx, the viewer data come with it."""
    from fdgen.flex.fdflex import read_meta
    from fdgen.view.scene import scene
    flex = {"material": {"E": E, "nu": 0.3, "rho": RHO}, "mesh_size": 0.005, "modes": 4}
    path = env / "a.yaml"
    path.write_text(yaml.safe_dump(arm(flex, name="a")))
    cli.build(path, env, "v2")
    mtx = env / "cms" / "arm.fdflex"
    info = read_meta(mtx)
    assert info["modes"] == 10 and info["source"] == "surface" and len(info["interfaces"]) == 2
    stamp = mtx.stat().st_mtime_ns
    cli.build(path, env, "v2")
    assert mtx.stat().st_mtime_ns == stamp  # reused
    flex["modes"] = 6
    path.write_text(yaml.safe_dump(arm(flex, name="a")))
    cli.build(path, env, "v2")
    assert read_meta(mtx)["modes"] == 12  # made again
    # reuse in another model, with its own place for it
    m = arm({"modal": "cms/arm.fdflex"}, name="b")
    (env / "b.yaml").write_text(yaml.safe_dump(m))
    _, model, _ = cli.build(env / "b.yaml", env, "v2")
    assert model["joints"][0]["node"] == 90000001
    assert scene(model)["bodies"][0]["flex"]["modes"]  # deformation in the viewer
    flex["cms"] = "parts/arm_cms.fdflex"
    path.write_text(yaml.safe_dump(arm(flex, name="a")))
    cli.build(path, env, "v2")
    assert (env / "parts" / "arm_cms.fdflex").is_file()


def plate_with_hole(path, r=0.005, n=32, m=4, nz=2, nx=12):
    """Closed STL of the arm [-a, L] x [-a, a] x [-H/2, H/2] with a bore of radius r at the
    origin (axis z): the surface of a structured hexahedral mesh, an O-grid from the bore to
    the square of half width a (n around, a multiple of 8, m across) and the rest of the arm on
    the square's right edge (nx along). Independent of a CAD kernel."""
    from fdgen.fem.assemble import boundary
    from fdgen.fem.nastran import FeModel
    from fdgen.fem.stl import write_stl
    a = 0.015
    phi = 2 * np.pi * np.arange(n) / n
    c, s_ = np.cos(phi), np.sin(phi)
    sq = np.column_stack([c, s_]) / np.maximum(np.abs(c), np.abs(s_))[:, None] * a
    rg = np.column_stack([c, s_]) * r
    zs = np.linspace(-H / 2, H / 2, nz + 1)
    fe, nid = FeModel(), {}

    def node(key, x, y, z):
        if key not in nid:
            nid[key] = len(nid) + 1
            fe.nodes[nid[key]] = (x, y, z)
        return nid[key]
    for i in range(n):  # O-grid: radial lines from the bore to the square
        for j in range(m + 1):
            p = rg[i] + (sq[i] - rg[i]) * j / m
            for k, z in enumerate(zs):
                node(("o", i, j, k), p[0], p[1], z)
    right = [i for i in range(n) if abs(sq[i][0] - a) < 1e-12]
    right = sorted(right, key=lambda i: sq[i][1])
    for q, i in enumerate(right):  # the arm: from the square's right edge to x = L
        for g in range(1, nx + 1):
            for k, z in enumerate(zs):
                node(("a", q, g, k), a + (L - a) * g / nx, sq[i][1], z)
    hexes = []
    for i in range(n):
        i1 = (i + 1) % n
        for j in range(m):
            for k in range(nz):
                hexes.append([nid[("o", ii, jj, kk)] for kk in (k, k + 1) for ii, jj in
                              ((i, j), (i1, j), (i1, j + 1), (i, j + 1))])
    edge = lambda q, g, k: nid[("o", right[q], m, k)] if g == 0 else nid[("a", q, g, k)]
    for q in range(len(right) - 1):
        for g in range(nx):
            for k in range(nz):
                hexes.append([edge(qq, gg, kk) for kk in (k, k + 1) for qq, gg in
                              ((q, g), (q, g + 1), (q + 1, g + 1), (q + 1, g))])
    fe.solids = [(e + 1, 1, h) for e, h in enumerate(hexes)]
    ids = np.array(sorted(fe.nodes))
    xyz = fe.node_array(ids)
    used, tri = boundary(fe, ids, xyz)
    write_stl(path, xyz[used][tri])
    return path


def test_bore_spider(env):
    """A revolute joint in a bore: its spider takes the bore surface (all around, through the
    thickness), the explicit radius a patch; spiders in the .fdflex and in the scene, with the
    joint at them."""
    from fdgen.flex.fdflex import read_fdflex
    from fdgen.view.scene import scene
    r = 0.005
    plate_with_hole(env / "hole.stl", r)
    m = {"model": "bore", "gravity": [0, -9.81, 0],
         "bodies": {"arm": {"mass": 0.5, "inertia": [1e-5, 1e-4, 1e-4],
                            "shapes": [{"type": "mesh", "file": "hole.stl", "frame": "body"}],
                            "flex": {"material": "steel", "mesh_size": 0.004, "modes": 4}}},
         "joints": {"pivot": {"type": "revolute", "body1": "arm", "location": [0, 0, 0]}},
         "solver": {"type": "HHT", "alpha": -0.05, "step": 1e-4, "end_time": 2e-4}}
    path = env / "bore.yaml"
    path.write_text(yaml.safe_dump(m))
    model = cli.load(path)
    d = read_fdflex(model["bodies"]["arm"]["flex"]["modal"])
    (node, kind, pts), = d.spiders
    rho = np.hypot(pts[:, 0], pts[:, 1])
    assert kind == "rbe2" and len(pts) > 30
    assert rho.max() == pytest.approx(r, rel=1e-6) and rho.min() > 0.9 * r  # on the bore (facets: midside nodes)
    assert pts[:, 2].min() == pytest.approx(-H / 2, rel=1e-6) and pts[:, 2].max() == pytest.approx(H / 2, rel=1e-6)
    sp = scene(model)["bodies"][0]["flex"]["spiders"][0]
    assert sp["elements"] == ["pivot"] and sp["n"] == len(pts) and sp["center"] == pytest.approx([0, 0, 0])
    # an explicit radius: the nodes within it (a patch of the bore wall near the point)
    m["bodies"]["arm"]["flex"]["interfaces"] = [{"point": [0, 0, 0], "radius": 1.05 * r}]
    path.write_text(yaml.safe_dump(m))
    (_, _, pts2), = read_fdflex(cli.load(path)["bodies"]["arm"]["flex"]["modal"]).spiders
    assert np.linalg.norm(pts2, axis=1).max() <= 1.05 * r * (1 + 1e-9) and len(pts2) < len(pts)


def test_mcp_flex_tools(env):
    """The MCP tools for flexible bodies on the arm made flexible: info (modes, interfaces, spiders,
    what is attached), convergence over the number of modes (the model's cms file untouched),
    deformation in a run, the elastic mode in an eigenmode, export."""
    from fdgen.mcp_server import Session
    flex = {"material": {"E": E, "nu": 0.3, "rho": RHO}, "mesh_size": 0.005, "modes": 4}
    m = arm(flex, name="mcp")
    m["solver"]["end_time"] = 0.02
    path = env / "mcp.yaml"
    path.write_text(yaml.safe_dump(m))
    s = Session(env / "runs", cli.find_runtime())
    try:
        info = s.flex_info(str(path))
        assert "body arm: flexible, meshed from its surface" in info and "10 elastic modes" in info
        assert "interface 90000001 at [0, 0, 0]: RBE2 patch" in info and "; pivot" in info
        assert "interface 90000002 at [0.2, 0, 0]" in info and "; spring" in info
        made = (env / "cms" / "arm.fdflex").stat().st_mtime_ns
        conv = s.flex_convergence(str(path), "arm", modes=[2, 6], count=3)
        assert "modes 2:" in conv and "modes 6:" in conv and conv.rstrip().endswith("(max change 0.00%)")
        assert (env / "cms" / "arm.fdflex").stat().st_mtime_ns == made
        run = s.simulate(str(path))
        assert run.startswith("run r1")
        dfm = s.flex_deformation(str(path), "r1")
        assert "largest deformation" in dfm and "modes with the most elastic energy" in dfm
        assert "deforms mostly in its mode" in s.modes(str(path), "r1")
        out = s.flex_export(str(path), "arm", str(env / "arm.bdf"))
        assert out.startswith("wrote") and (env / "arm.bdf").is_file()
        with pytest.raises(ValueError, match="not a flexible body"):
            s.flex_info(str(path), "arm_if2")
    finally:
        s.close()
