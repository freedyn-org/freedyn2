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
    """[(global point, what)] of the joints and forces at the body without a node."""
    out = []
    for j in joints:
        if name in (j["body1"], j["body2"]) and "node" not in j:
            out.append((np.array(j["location"], float), ("joint", j)))
    for f in forces:
        pts = []
        if f["type"] == "spring_damper":
            pts = [k for k, b in (("point1", "body1"), ("point2", "body2")) if f[b] == name]
        elif f["type"] in ("bushing", "torsion_spring") and name in (f["body1"], f["body2"]):
            pts = ["location"]
        elif f["type"] in ("force", "torque") and f["body1"] == name:
            pts = ["point"]
        elif name in (f["body1"], f["body2"]):
            warnings.warn(f"force {f['name']}: {f['type']} on the flexible body {name} acts on its body frame")
        for k in pts:
            if k not in f:
                raise ModelError(f"force {f['name']}: give {k} for the flexible body {name} (where it acts on it)")
            out.append((np.array(f[k], float), ("force", f, k)))
    return out


def plan(name, body, joints, forces, base_dir):
    """Surface (triangles in the body frame) and interfaces of a body to mesh:
    (tri, specs [(point body frame, type, radius)], owners [[what]] per interface)."""
    flex = body["flex"]
    R, A = body_frame(body)
    tri = _surface(name, body, base_dir)
    size = float(np.ptp(tri.reshape(-1, 3), axis=0).max())
    # explicit interfaces, then joints and forces; coinciding points share one. A revolute or
    # cylindrical joint gives its axis (joint z): its spider takes the bore around it
    specs, owners = [], []
    for i in flex.get("interfaces") or []:
        axis = A.T @ np.array(i["axis"], float) if "axis" in i else None
        specs.append([A.T @ (np.array(i["point"], float) - R), i.get("type", "rbe2"), i.get("radius"), axis])
        owners.append([])
    for p, what in _attachments(name, joints, forces):
        u = A.T @ (p - R)
        axis = None
        if what[0] == "joint" and what[1]["type"] in ("revolute", "cylindrical"):
            axis = A.T @ np.array(what[1]["frame"], float)[:, 2]
        for k, sp in enumerate(specs):
            if np.linalg.norm(sp[0] - u) <= 1e-6 * size:
                owners[k].append(what)
                if sp[3] is None and axis is not None:
                    sp[3] = axis
                break
        else:
            specs.append([u, flex.get("interface_type", "rbe2"), None, axis])
            owners.append([what])
    specs = [tuple(sp) for sp in specs]
    if not specs:
        raise ModelError(f"body {name}: flexible without interfaces (no joint or force acts on it)")
    return tri, specs, owners


def _connect(name, body, owners, nodes, positions, joints, model_bodies, added):
    """Joints and forces at the interface nodes (owners[k] at nodes[k], global positions)."""
    from ..model import _joint_node
    for k, whats in enumerate(owners):
        node = nodes[k]
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
                    p = np.asarray(positions[k], float)
                    added[aux] = {"mass": 0.0, "inertia": [0.0, 0.0, 0.0], "position": p.tolist(),
                                  "orientation": list(body["orientation"]), "velocity": list(body["velocity"]),
                                  "angular_velocity": list(body["angular_velocity"]), "interface_of": name}
                    joints.append({"name": f"{aux}_fix", "type": "fixed", "body1": aux, "body2": name,
                                   "location": p.tolist(), "frame": np.eye(3).tolist(),
                                   "fix": ["dx", "dy", "dz", "ax", "ay", "az"], "node": node})
                if key in ("point1", "point2"):
                    which = "body1" if key == "point1" else "body2"
                else:  # bushing, torsion spring, force, torque: whichever body is the flexible one
                    which = "body1" if f["body1"] == name else "body2"
                f[which] = aux


def _existing_interfaces(name, body, joints, forces):
    """Joints and forces without a node at a body with modal data (modal, fem): each at the
    interface node at its point (nodes with rotations, within 1 % of the body size)."""
    from ..flex import load_modal
    d = load_modal(body["flex"]["modal"])
    R, A = body_frame(body)
    has_rot = np.abs(d.Psi).reshape(len(d.node_labels), -1).max(axis=1) > 0
    labels, pos = d.node_labels[has_rot], d.node_positions[has_rot]
    if not len(labels):
        return [], [], []
    size = np.sqrt(np.trace(d.I7) / d.mass)
    nodes, positions, owners = [], [], []
    for p, what in _attachments(name, joints, forces):
        u = A.T @ (p - R)
        k = int(np.argmin(np.linalg.norm(pos - u, axis=1)))
        if np.linalg.norm(pos[k] - u) > 0.01 * size:
            continue  # stays on the body frame (fdgen.model warns)
        if labels[k] not in nodes:
            nodes.append(int(labels[k])), positions.append(R + A @ pos[k]), owners.append([])
        owners[nodes.index(int(labels[k]))].append(what)
    return nodes, positions, owners


def prepare(model_bodies, joints, forces, base_dir, log=None):
    """Flexible bodies (in place): bodies meshed from their surface get their modal data; joints
    and forces without a node act at the interface node at their point (joints: node; forces: a
    massless body fixed to it). Returns the added interface bodies {name: body}."""
    from ..model import _flex
    added = {}
    for name, body in list(model_bodies.items()):
        if "flex" not in body or body.get("interface_of"):
            continue
        if is_surface(body):
            flex = body["flex"]
            tri, specs, owners = plan(name, body, joints, forces, base_dir)
            material, h, modes = _material(flex, name), float(flex["mesh_size"]), int(flex.get("modes", 20))
            flex["modal"] = str(_modal_file(name, tri, material, h, specs, modes, log, base_dir, flex))
            nodes = [INTERFACE_IDS + k + 1 for k in range(len(specs))]
            flex["nodes"] = nodes
            _flex(name, body, base_dir)  # mass and inertia from the FE model (a warning if they differ)
            R, A = body_frame(body)
            positions = [R + A @ sp[0] for sp in specs]
        else:
            nodes, positions, owners = _existing_interfaces(name, body, joints, forces)
        _connect(name, body, owners, nodes, positions, joints, model_bodies, added)
    model_bodies.update(added)
    for j in joints:
        for b in (j["body1"], j["body2"]):
            if b != "ground" and "flex" in model_bodies[b] and "node" not in j:
                warnings.warn(f"joint {j['name']}: on the body frame of the flexible body {b} (no interface node "
                              "at its location)")
    return added


def fe_of(name, tri, material, h, specs):
    """FE model of a body meshed from its surface, with its interfaces."""
    xyz, tets, surf = mesh_stl(tri, h, f"body {name}")
    return fe_model(xyz, tets, surf, material, specs, h)


def _modal_file(name, tri, material, h, specs, modes, log, base_dir=None, flex=None):
    from . import cms_file
    key = hashlib.sha256(np.ascontiguousarray(tri, float).tobytes())
    key.update(repr((FEM_VERSION, "surface", tuple(material), h, modes,
                     [(tuple(np.round(u, 12)), t, r, None if ax is None else tuple(np.round(ax, 12)))
                      for u, t, r, ax in specs])).encode())

    def compute():
        fe = fe_of(name, tri, material, h, specs)
        a = assemble(fe)
        ifs = [INTERFACE_IDS + k + 1 for k in range(len(specs))]
        if log:
            log(f"body {name}: {len(fe.solids)} TET10, {a.n} dofs, {len(ifs)} interfaces, {modes} modes")
        return reduce(a, ifs, modes, output_nodes=ifs), {
            "source": "surface", "mesh_size": h, "material": list(material), "dofs": int(a.n),
            "elements": len(fe.solids), "interfaces": [{"node": n, "point_body": [float(x) for x in sp[0]],
                                                         "type": sp[1], "radius": sp[2],
                                                         "axis_body": None if sp[3] is None else [float(x) for x in sp[3]]}
                                                        for n, sp in zip(ifs, specs)]}

    return cms_file(base_dir, name, flex or {}, key.hexdigest(), compute,
                    f"body {name}: Netgen TET10 h = {h}, freedyn2 FEM, Craig-Bampton")


def export(model_path, body_name, out=None):
    """The FE model of a body meshed from its surface, as Nastran bulk data (interfaces and their
    joints and forces as comments): to inspect or change it (spiders, mesh) and use it with
    flex.fem. Returns the written path."""
    from pathlib import Path

    from ..model import load_yaml, validate
    from .nastran import write_nastran
    model_path = Path(model_path)
    m = validate(load_yaml(model_path), model_path.parent, prepare_flex=False)
    body = m["bodies"].get(body_name)
    if body is None or not is_surface(body):
        raise ModelError(f"{body_name}: not a body with flex: {{material, mesh_size}}")
    tri, specs, owners = plan(body_name, body, m["joints"], m["forces"], model_path.parent)
    fe = fe_of(body_name, tri, _material(body["flex"], body_name), float(body["flex"]["mesh_size"]), specs)
    out = Path(out or model_path.with_name(f"{body_name}.bdf"))
    R, A = body_frame(body)
    notes = [f"freedyn2 flex-export: body {body_name} of {model_path.name}, in its body frame (model units)",
             f"{len(fe.solids)} TET10, material {body['flex']['material']}, mesh size {body['flex']['mesh_size']}",
             "use it with  flex: {fem: <this file>, interfaces: [...], modes: ...}; joints and forces attach to",
             "the interface node at their point. Interfaces (RBE master / reference node: global point):"]
    for k, (sp, whats) in enumerate(zip(specs, owners)):
        names = ", ".join(f"{w[0]} {w[1]['name']}" for w in whats) or "explicit"
        notes.append(f"  {INTERFACE_IDS + k + 1}: {np.round(R + A @ sp[0], 9).tolist()}  {sp[1]}  ({names})")
    return write_nastran(fe, out, notes)
