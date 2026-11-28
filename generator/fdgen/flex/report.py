"""What a flexible body of a validated model is made of, for tools (MCP) and the viewer:
its modal data, its interfaces with their spiders and the joints and forces at them, and the
deformation in a run."""

import numpy as np

from ..geometry import body_frame
from . import load_modal


def elements_at(model, name, node):
    """Names of the joints and forces at interface node `node` of the flexible body `name`
    (forces act on its massless interface body, fixed to the node)."""
    out = []
    for j in model.get("joints", []):
        if j.get("node") != node or name not in (j["body1"], j["body2"]):
            continue
        aux = model["bodies"].get(j["body1"], {}).get("interface_of") == name and j["body1"]
        out += [f["name"] for f in model.get("forces", []) if aux in (f["body1"], f["body2"])] if aux else [j["name"]]
    return out


def _around(pts, centre, axis):
    """Angle (degrees) the points cover around the axis through centre."""
    e = np.asarray(axis, float) / np.linalg.norm(axis)
    u = np.cross(e, [1.0, 0, 0] if abs(e[0]) < 0.9 else [0, 1.0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(e, u)
    d = pts - centre
    ang = np.arctan2(d @ v, d @ u)
    bins = np.unique(np.floor((ang + np.pi) / (2 * np.pi) * 36).astype(int) % 36)
    return 10.0 * len(bins)


def interfaces(model, name):
    """[{node, position (global, t = 0), type, nodes, distance (min, max from the node), shape
    (bore / patch / given), axis (global) or None, elements}] of a flexible body."""
    b = model["bodies"][name]
    d = load_modal(b["flex"]["modal"])
    R, A = body_frame(b)
    meta = getattr(d, "meta", None) or {}
    axes = {i["node"]: i.get("axis_body") for i in meta.get("interfaces", [])}
    radii = {i["node"]: i.get("radius") for i in meta.get("interfaces", [])}
    labels = d.node_labels.tolist()
    out = []
    spiders = getattr(d, "spiders", None) or []
    for node, kind, pts in spiders:
        c = d.node_positions[labels.index(node)] if node in labels else pts.mean(axis=0)
        r = np.linalg.norm(pts - c, axis=1)
        axis = axes.get(node)
        shape = "given radius" if radii.get(node) else "patch"
        if axis is not None and not radii.get(node) and _around(pts, c, axis) >= 270:
            shape = "bore"
        out.append({"node": int(node), "position": (R + A @ c).tolist(), "type": kind, "nodes": len(pts),
                    "distance": [float(r.min()), float(r.max())], "shape": shape,
                    "axis": None if axis is None else (A @ np.asarray(axis, float)).tolist(),
                    "elements": elements_at(model, name, node)})
    if not spiders:  # modal data without spiders (an MTX file): the nodes with rotations
        has_rot = np.abs(d.Psi).reshape(len(labels), -1).max(axis=1) > 0
        for k in np.flatnonzero(has_rot):
            node = labels[k]
            out.append({"node": int(node), "position": (R + A @ d.node_positions[k]).tolist(), "type": "?",
                        "nodes": None, "distance": None, "shape": "from the modal data", "axis": None,
                        "elements": elements_at(model, name, node)})
    return out


def summary(model, name, frequencies=20):
    """Text: source, modes and frequencies, mass, interfaces."""
    b = model["bodies"][name]
    flex = b["flex"]
    d = load_modal(flex["modal"])
    meta = getattr(d, "meta", None) or {}
    src = meta.get("source")
    if src == "surface":
        how = (f"meshed from its surface (mesh size {meta.get('mesh_size')}, material E, nu, rho = "
               f"{meta.get('material')}), {meta.get('elements')} TET10, {meta.get('dofs')} dofs")
    elif src:
        how = f"FE model {src} ({meta.get('elements')} elements, {meta.get('dofs')} dofs)"
    else:
        how = f"modal data file {flex.get('modal_all', flex['modal'])}"
    lines = [f"body {name}: flexible, {how}"]
    if meta.get("made"):
        lines.append(f"  reduced (Craig-Bampton) {meta.get('made')} in {meta.get('seconds')} s; kept in {flex['modal']} "
                     "(reused while mesh, material, interfaces and modes stay the same)")
    f = d.frequencies
    shown = ", ".join(f"{x:.4g}" for x in f[:frequencies]) + (" ..." if len(f) > frequencies else "")
    lines.append(f"  {d.nm} elastic modes, free-free (Hz): {shown}")
    damping = ("default (1 % / 10 % / 100 % below 100 Hz / below 1000 Hz / above)" if flex.get("damping_default")
               else flex.get("damping", 0.0))
    if flex.get("modal_all"):
        lines.append(f"  modes selected from {flex['modal_all']}: {d.meta.get('kept_modes')} (max_frequency "
                     f"{flex.get('max_frequency')}, exclude_modes {flex.get('exclude_modes')})")
    lines.append(f"  mass {d.mass:.6g}, damping ratio {damping}, coupling: rotation "
                 f"{'on' if flex.get('rotation_coupling', True) else 'off'}, quadratic inertia "
                 f"{'on' if flex.get('quadratic_inertia', True) else 'off'}")
    for i in interfaces(model, name):
        pos = ", ".join(f"{x:.6g}" for x in i["position"])
        spider = (f"{i['type'].upper()} {i['shape']}, {i['nodes']} surface nodes at {i['distance'][0]:.4g} .. "
                  f"{i['distance'][1]:.4g} from it" if i["nodes"] else i["shape"])
        lines.append(f"  interface {i['node']} at [{pos}]: {spider}; "
                     f"{', '.join(i['elements']) or 'nothing attached'}")
    return "\n".join(lines)


def deformation(model, name, results):
    """Largest deformation of a flexible body in a run: on its surface (the FE surface, if the
    modal data have it, else the nodes of the modal data), relative to the rigid motion that fits
    the displacement best (the body frame floats with the body), and the modal coordinates."""
    d = load_modal(model["bodies"][name]["flex"]["modal"])
    eta = np.column_stack([results[f"{name}.eta{i + 1}.eta"] for i in range(d.nm)])  # (nt, nm)
    viz = getattr(d, "viz", None)
    xyz, phi, where = (viz[0], viz[2], "surface") if viz is not None else (d.node_positions, d.Phi, "nodes")
    t = results["t"]
    c = xyz.mean(axis=0)
    r = xyz - c
    best = (0.0, 0, 0)
    step = max(1, len(t) // 400)
    for k in range(0, len(t), step):
        u = np.einsum("nam,m->na", phi, eta[k])
        tr = u.mean(axis=0)
        v = u - tr
        S = (r ** 2).sum() * np.eye(3) - r.T @ r
        w = np.linalg.lstsq(S, np.cross(r, v).sum(axis=0), rcond=None)[0]
        e = np.linalg.norm(v - np.cross(w, r), axis=1)
        if e.max() > best[0]:
            best = (float(e.max()), k, int(e.argmax()))
    umax, k, n = best
    R, A = body_frame(model["bodies"][name])
    omega = 2 * np.pi * d.frequencies
    energy = 0.5 * (omega ** 2) * (eta ** 2) * np.diag(d.I6)
    share = energy.max(axis=0)
    top = np.argsort(-share)[:3]
    lines = [f"body {name}: largest deformation {umax:.4g} at t = {t[k]:.4g} s, at [{', '.join(f'{x:.5g}' for x in R + A @ xyz[n])}] "
             f"(global at t = 0; {where}, relative to the best-fitting rigid motion)",
             "  modes with the most elastic energy: " + ", ".join(
                 f"mode {m + 1} ({d.frequencies[m]:.4g} Hz, max |eta| {np.abs(eta[:, m]).max():.3g})" for m in top)]
    return "\n".join(lines)
