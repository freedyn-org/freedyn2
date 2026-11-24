"""A flexible body from the surface of a part (its STL) in a model: "rigid to flex".

A body with `flex: {material, mesh_size, ...}` is meshed from its STL shape (or flex.stl, placed
as a shape), reduced with interfaces at every point a joint or a force attaches to it (and the
explicit flex.interfaces), and becomes a flexible body with the modal data of that reduction:

- joints at the body act at its interface node (`node`); a joint whose body1 is the flexible
  body and body2 another body has its bodies swapped (the flexible body is body2, fdgen.flex_body),
  for the preset types (their constraints do not depend on which body carries the joint frame);
- forces at the body act on a massless body fixed to the interface node (`<body>_if<k>`), so that
  every force law works unchanged;
- position and orientation of the body stay as they are: the FE model lives in its frame.

Meshing and reduction are cached by their inputs (fdgen.userdirs.cache_dir()/flex).
"""

import hashlib
import warnings

import numpy as np

from ..errors import ModelError
from ..geometry import body_frame, rotation_matrix
from . import FEM_VERSION, assemble, reduce
from .stl import INTERFACE_IDS, MATERIALS, fe_model, mesh_stl, read_surface

SYMMETRIC_JOINTS = ("fixed", "spherical", "revolute", "cylindrical", "prismatic")


def is_surface(body):
    return "flex" in body and "material" in body["flex"]


def _material(flex, name):
    m = flex["material"]
    if isinstance(m, str):
        if m not in MATERIALS:
            raise ModelError(f"body {name}: material {m}: one of {sorted(MATERIALS)} (SI units) or {{E, nu, rho}}")
        return MATERIALS[m]
    return float(m["E"]), float(m["nu"]), float(m["rho"])


def _surface(name, body, base_dir):
    """Triangles of the part in the body frame: flex.stl (a mesh shape) or the body's one mesh
    shape (STL or OBJ)."""
    from ..shapes import validate_shapes
    if "stl" in body["flex"]:
        s = body["flex"]["stl"]
        shape = validate_shapes(f"body {name}: flex.stl", [s if isinstance(s, dict) else
                                                           {"type": "mesh", "file": s}], body, base_dir)[0]
    else:
        meshes = [s for s in body.get("shapes", []) if s["type"] == "mesh"]
        if len(meshes) != 1:
            raise ModelError(f"body {name}: flex needs one mesh shape (STL or OBJ) or flex.stl, it has {len(meshes)}")
        shape = meshes[0]
    tri = read_surface(shape["file"]) * np.asarray(shape["scale"], float)
    A = rotation_matrix(np.array(shape["orientation"], float))
    return tri @ A.T + np.asarray(shape["position"], float)


def _attachments(name, joints, forces):
    """[(global point, what)] of the joints and forces at the body."""
    out = []
    for j in joints:
        if name in (j["body1"], j["body2"]) and "node" not in j:
            out.append((np.array(j["location"], float), ("joint", j)))
    for f in forces:
        pts = []
        if f["type"] == "spring_damper":
            pts = [k for k, b in (("point1", "body1"), ("point2", "body2")) if f[b] == name]
        elif f["type"] == "bushing" and name in (f["body1"], f["body2"]):
            pts = ["location"]
        elif f["type"] == "force" and f["body1"] == name:
            pts = ["point"]
        elif name in (f["body1"], f["body2"]):
            warnings.warn(f"force {f['name']}: {f['type']} on the flexible body {name} acts on its body frame")
        for k in pts:
            if k not in f:
                raise ModelError(f"force {f['name']}: give {k} explicitly for the flexible body {name}")
            out.append((np.array(f[k], float), ("force", f, k)))
    return out


def prepare(model_bodies, joints, forces, base_dir, log=None):
    """Make the surface bodies flexible (in place): modal data, joint nodes, force bodies.
    Returns the massless interface bodies to add {name: body}."""
    from ..model import _flex, _joint_node
    added = {}
    for name, body in list(model_bodies.items()):
        if not is_surface(body):
            continue
        flex = body["flex"]
        R, A = body_frame(body)
        tri = _surface(name, body, base_dir)
        size = float(np.ptp(tri.reshape(-1, 3), axis=0).max())
        # interfaces: explicit ones, then joints and forces; coinciding points share a node
        specs, owners = [], []
        for k, i in enumerate(flex.get("interfaces") or []):
            specs.append((A.T @ (np.array(i["point"], float) - R), i.get("type", "rbe2"), i.get("radius")))
            owners.append([])
        for p, what in _attachments(name, joints, forces):
            u = A.T @ (p - R)
            for k, s in enumerate(specs):
                if np.linalg.norm(s[0] - u) <= 1e-6 * size:
                    owners[k].append(what)
                    break
            else:
                specs.append((u, flex.get("interface_type", "rbe2"), None))
                owners.append([what])
        if not specs:
            raise ModelError(f"body {name}: flexible without interfaces (no joint or force acts on it)")
        material = _material(flex, name)
        h = float(flex["mesh_size"])
        modes = int(flex.get("modes", 20))
        flex["mtx"] = str(_modal_file(name, tri, material, h, specs, modes, log))
        flex["nodes"] = [INTERFACE_IDS + k + 1 for k in range(len(specs))]
        m_rigid = body.get("mass")
        body.pop("mass", None), body.pop("inertia", None)
        _flex(name, body, base_dir)
        if m_rigid and abs(body["mass"] / m_rigid - 1) > 0.05:
            warnings.warn(f"body {name}: mass of the FE model {body['mass']:g} differs from the rigid body mass "
                          f"{m_rigid:g} (density, units, STL scale?)")
        for k, whats in enumerate(owners):
            node = INTERFACE_IDS + k + 1
            for what in whats:
                if what[0] == "joint":
                    j = what[1]
                    if j["body1"] == name and j["body2"] != "ground":
                        if j["type"] not in SYMMETRIC_JOINTS:
                            raise ModelError(f"joint {j['name']}: the flexible body {name} must be body2 "
                                             f"(a {j['type']} joint cannot swap its bodies)")
                        j["body1"], j["body2"] = j["body2"], j["body1"]
                    _joint_node(j, {"node": node, "location": j["location"]}, model_bodies)
                else:
                    f, key = what[1], what[2]
                    aux = f"{name}_if{k + 1}"
                    if aux not in added:
                        p = R + A @ specs[k][0]
                        added[aux] = {"mass": 0.0, "inertia": [0.0, 0.0, 0.0], "position": p.tolist(),
                                      "orientation": list(body["orientation"]), "velocity": list(body["velocity"]),
                                      "angular_velocity": list(body["angular_velocity"]), "interface_of": name}
                        joints.append({"name": f"{aux}_fix", "type": "fixed", "body1": aux, "body2": name,
                                       "location": p.tolist(), "frame": np.eye(3).tolist(),
                                       "fix": ["dx", "dy", "dz", "ax", "ay", "az"], "node": node})
                    which = {"point1": "body1", "point2": "body2", "location": "body1", "point": "body1"}[key]
                    if key == "location":  # bushing: whichever body is the flexible one
                        which = "body1" if f["body1"] == name else "body2"
                    f[which] = aux
    model_bodies.update(added)
    return added


def _modal_file(name, tri, material, h, specs, modes, log):
    from ..flex.mtx import write_mtx
    from ..userdirs import cache_dir
    key = hashlib.sha256(np.ascontiguousarray(tri, float).tobytes())
    key.update(repr((FEM_VERSION, "surface", tuple(material), h, modes,
                     [(tuple(np.round(u, 12)), t, r) for u, t, r in specs])).encode())
    out = cache_dir() / "flex" / f"{name}_{key.hexdigest()[:16]}.mtx"
    if out.is_file():
        return out
    xyz, tets, surf = mesh_stl(tri, h, f"body {name}")
    fe = fe_model(xyz, tets, surf, material, specs, h)
    a = assemble(fe)
    ifs = [INTERFACE_IDS + k + 1 for k in range(len(specs))]
    if log:
        log(f"body {name}: {len(tets)} TET10, {a.n} dofs, {len(ifs)} interfaces, {modes} modes")
    data = reduce(a, ifs, modes, output_nodes=ifs)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    write_mtx(data, tmp, title=f"body {name}: Netgen TET10 h = {h}, freedyn2 FEM, Craig-Bampton")
    tmp.replace(out)
    return out
