"""Mode shapes of a flexible body on the vertices of its shapes, for the viewer.

Source: the mode shapes on the surface of the FE model (<mtx>.viz.npz, written by the own FE
reduction), else those at the nodes of the modal data (an MTX file with all nodes; with only a
few interface nodes there is nothing to show). Each vertex of an STL shape gets the inverse
distance weighted mode shapes of the 4 nearest of those nodes; vertices farther from them than
5 % of the body size get none (the shape and the FE model do not match).

A flexible body without an STL shape but with an FE surface gets that surface as its shape
(an STL file next to the modal data).
"""

from pathlib import Path

import numpy as np

from ..geometry import rotation_matrix
from . import load_modal

K_NEAREST = 4


def source(flex):
    """(node positions (n, 3) body frame, triangles or None, mode shapes (n, 3, nm)) or None."""
    from ..fem import viz_path
    p = viz_path(flex["mtx"])
    if p.is_file():
        z = np.load(p)
        return z["xyz"], z["tri"], z["phi"]
    d = load_modal(flex["mtx"])
    if len(d.node_labels) < 8:
        return None
    return d.node_positions, None, d.Phi


def _nearest(points, nodes, k, chunk=2000):
    """Indices (m, k) and distances of the k nearest nodes of each point (brute force in chunks)."""
    k = min(k, len(nodes))
    idx = np.empty((len(points), k), int)
    dist = np.empty((len(points), k))
    nn = np.einsum("ij,ij->i", nodes, nodes)
    for c in range(0, len(points), chunk):
        p = points[c:c + chunk]
        d2 = np.einsum("ij,ij->i", p, p)[:, None] - 2 * p @ nodes.T + nn[None, :]
        part = np.argpartition(d2, k - 1, axis=1)[:, :k]
        dd = np.take_along_axis(d2, part, axis=1)
        order = np.argsort(dd, axis=1)
        idx[c:c + chunk] = np.take_along_axis(part, order, axis=1)
        dist[c:c + chunk] = np.sqrt(np.maximum(np.take_along_axis(dd, order, axis=1), 0))
    return idx, dist


def interpolate(points, nodes, phi):
    """Mode shapes (m, 3, nm) at points from those at nodes; None where too far from them."""
    idx, dist = _nearest(points, nodes, K_NEAREST)
    size = float(np.ptp(nodes, axis=0).max()) or 1.0
    if np.max(dist[:, 0]) > 0.05 * size:
        return None
    w = 1.0 / np.maximum(dist, 1e-12 * size) ** 2
    w /= w.sum(axis=1, keepdims=True)
    return np.einsum("mk,mkan->man", w, phi[idx])


def write_surface_stl(path, xyz, tri):
    from ..fem.stl import write_stl
    write_stl(path, xyz[tri])


def shape_modes(body, shapes):
    """{shape index: float32 array (nm, nv, 3) in the shape's own coordinates (file coordinates,
    before its scale)}, and extra shapes (the FE surface for a body without STL shapes). nv:
    the vertices of the STL in file order, three per triangle (as three.js's STLLoader)."""
    from ..fem.stl import read_stl
    src = source(body["flex"])
    if src is None:
        return {}, []
    nodes, tri, phi = src
    extra = []
    if not any(s["type"] == "mesh" for s in shapes) and tri is not None and len(tri):
        f = Path(body["flex"]["mtx"]).with_suffix(".surface.stl")
        if not f.is_file():
            write_surface_stl(f, nodes, tri)
        extra = [{"type": "mesh", "file": str(f), "position": [0.0, 0.0, 0.0], "orientation": [1.0, 0.0, 0.0, 0.0],
                  "scale": [1.0, 1.0, 1.0]}]
    out = {}
    for k, s in enumerate(list(shapes) + extra):
        if s["type"] != "mesh" or not s["file"].lower().endswith(".stl"):
            continue
        v = read_stl(s["file"]).reshape(-1, 3)
        As = rotation_matrix(np.array(s["orientation"], float))
        scale = np.asarray(s.get("scale", [1.0, 1.0, 1.0]), float)
        pb = (v * scale) @ As.T + np.asarray(s["position"], float)
        ph = interpolate(pb, nodes, phi)
        if ph is None:
            continue
        local = np.einsum("ba,mbn->man", As, ph) / scale[None, :, None]  # A_s^T phi / scale
        out[k] = np.ascontiguousarray(np.transpose(local, (2, 0, 1)), dtype=np.float32)
    return out, extra
