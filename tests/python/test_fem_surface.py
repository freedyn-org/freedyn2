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
    # cached: the second build does not mesh again
    assert len(list((env / "home").rglob("*.mtx"))) == 1


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
