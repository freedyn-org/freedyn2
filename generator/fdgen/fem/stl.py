"""FE model of a part from its surface (STL): volume mesh of quadratic tetrahedra by Netgen
(netgen-mesher, LGPL; imported only here), interface nodes with RBE2 / RBE3 spiders.

The STL is placed in the body frame (shape position, orientation, scale) before meshing, so
that the FE model, and with it the modal data, live in the body frame of the flexible body.

Interfaces: a point (body frame) becomes a node with 6 dofs, connected by an RBE2 (rigid,
default) or an RBE3 (interpolating, adds no stiffness) to the surface nodes within `radius`.
Default radius: the distance to the nearest surface node plus a mesh size (a ring in a bore whose
centre is the point, a patch around a point on the surface), widened until it holds at least 6
nodes that are not on one line.
"""

import struct
import tempfile
from pathlib import Path

import numpy as np

from ..errors import ModelError
from .nastran import FeModel

INTERFACE_IDS = 90_000_000  # node ids of the interface nodes: INTERFACE_IDS + 1, + 2, ... (I8 field of MTX)

MATERIALS = {  # SI units (m, kg, s): E [Pa], nu, rho [kg/m^3]
    "steel": (210e9, 0.3, 7850.0),
    "aluminium": (70e9, 0.33, 2700.0),
    "titanium": (114e9, 0.34, 4430.0),
}


def read_stl(path):
    """Triangles (nt, 3, 3) of a binary or ASCII STL file."""
    data = Path(path).read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:1000]:
        v = [list(map(float, line.split()[1:4])) for line in data.decode("latin-1").splitlines()
             if line.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    if len(rec) != n:
        raise ModelError(f"{path}: truncated binary STL")
    return rec["v"].astype(float)


def read_obj(path):
    """Triangles of a Wavefront OBJ file (polygons fanned)."""
    V, out = [], []
    for line in Path(path).read_text(encoding="latin-1").splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            V.append([float(x) for x in p[1:4]])
        elif p[0] == "f":
            idx = [int(x.split("/")[0]) for x in p[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            out += [(idx[0], idx[k], idx[k + 1]) for k in range(1, len(idx) - 1)]
    return np.array(V, float)[np.array(out, int)]


def read_surface(path):
    """Triangles of an STL or OBJ file."""
    return read_obj(path) if str(path).lower().endswith(".obj") else read_stl(path)


def write_stl(path, tri):
    with open(path, "wb") as f:
        f.write(b"freedyn2".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tri)))
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1), 1e-300)[:, None]
        rec = np.zeros(len(tri), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
        rec["n"], rec["v"] = n, tri
        f.write(rec.tobytes())


def check_closed(tri, where):
    """Every edge in exactly two triangles (a closed surface), else an error."""
    key = np.round(tri.reshape(-1, 3) / max(np.ptp(tri.reshape(-1, 3), axis=0).max(), 1e-300) * 1e9).astype(np.int64)
    _, vid = np.unique(key, axis=0, return_inverse=True)
    vid = vid.reshape(-1, 3)
    e = np.sort(np.concatenate([vid[:, [0, 1]], vid[:, [1, 2]], vid[:, [2, 0]]]), axis=1)
    _, count = np.unique(e, axis=0, return_counts=True)
    bad = int(np.sum(count != 2))
    if bad:
        raise ModelError(f"{where}: the surface is not closed ({bad} edges not shared by exactly two triangles); "
                         "repair the STL (watertight, no duplicate or missing faces)")


def mesh_stl(tri, maxh, where="STL"):
    """Quadratic tetrahedra of the closed surface tri (nt, 3, 3): (xyz (n, 3), tets (ne, 10) in
    the Nastran node order, surface node indices)."""
    try:
        from netgen.stl import STLGeometry
    except ImportError:
        raise ModelError("meshing an STL needs Netgen: pip install netgen-mesher") from None
    check_closed(tri, where)
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "part.stl"
        write_stl(f, tri)
        try:
            mesh = STLGeometry(str(f)).GenerateMesh(maxh=float(maxh))
        except Exception as e:  # Netgen raises plain exceptions
            raise ModelError(f"{where}: Netgen could not mesh the surface: {e}") from None
    if len(mesh.Elements3D()) == 0:
        raise ModelError(f"{where}: Netgen made no volume elements (surface not closed or self-intersecting?)")
    mesh.SecondOrder()
    xyz = np.array([p.p for p in mesh.Points()], float)
    raw = np.array([[p.nr - 1 for p in el.points] for el in mesh.Elements3D()], int)
    surf = np.unique([p.nr - 1 for el in mesh.Elements2D() for p in el.points])
    return xyz, _tet10_order(xyz, raw), surf


def _tet10_order(xyz, raw):
    """Midside nodes in the Nastran order (1-2, 2-3, 3-1, 1-4, 2-4, 3-4), assigned by geometry
    (the node nearest to each edge midpoint); corners turned to a positive volume."""
    c = raw[:, :4].copy()
    X = xyz[c]
    vol = np.einsum("ij,ij->i", np.cross(X[:, 1] - X[:, 0], X[:, 2] - X[:, 0]), X[:, 3] - X[:, 0])
    neg = vol < 0
    c[neg, 1], c[neg, 2] = c[neg, 2], c[neg, 1].copy()
    mids = raw[:, 4:]
    out = np.empty((len(raw), 10), int)
    out[:, :4] = c
    for k, (i, j) in enumerate(((0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3))):
        m = (xyz[c[:, i]] + xyz[c[:, j]]) / 2
        d = np.linalg.norm(xyz[mids] - m[:, None, :], axis=2)
        out[:, 4 + k] = mids[np.arange(len(raw)), np.argmin(d, axis=1)]
    return out


def bore_nodes(xyz, surf, point, axis, h):
    """Surface nodes of the bore around the axis through point: the nodes at the smallest
    distance r0 from the axis near the point (faceted: within 3 % of r0 plus h / 20), along the
    axis as far as that surface goes on without a gap of more than 2 h, and around at least
    270 degrees. None if there is no such bore (the point on a face, an open slot)."""
    e = np.asarray(axis, float) / np.linalg.norm(axis)
    d = xyz[surf] - point
    a = d @ e
    rho = np.linalg.norm(d - np.outer(a, e), axis=1)
    near = np.abs(a) <= 3 * h
    if not near.any():
        return None
    r0 = rho[near].min()
    if r0 < 0.5 * h:
        return None
    on = rho <= r0 * 1.03 + 0.05 * h
    av = np.sort(a[on])
    if not len(av):
        return None
    lo = hi = np.searchsorted(av, 0.0)
    lo, hi = max(min(lo, len(av) - 1), 0), max(min(hi, len(av) - 1), 0)
    while lo > 0 and av[lo] - av[lo - 1] <= 2 * h:
        lo -= 1
    while hi < len(av) - 1 and av[hi + 1] - av[hi] <= 2 * h:
        hi += 1
    if av[lo] > 2 * h or av[hi] < -2 * h:  # the bore surface does not reach the point
        return None
    sel = on & (a >= av[lo] - 1e-12) & (a <= av[hi] + 1e-12)
    u = np.cross(e, [1.0, 0, 0] if abs(e[0]) < 0.9 else [0, 1.0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(e, u)
    ang = np.arctan2(d[sel] @ v, d[sel] @ u)
    if len(np.unique(np.floor((ang + np.pi) / (2 * np.pi) * 12).astype(int) % 12)) < 9:
        return None
    return surf[sel]


def spider_nodes(xyz, surf, point, radius=None, h=1.0, axis=None):
    """Surface nodes (indices) of an interface at point: with an axis (a revolute or cylindrical
    joint) the bore around it (bore_nodes), if there is one; else within radius, by default the
    distance to the nearest surface node plus h, widened until 6 nodes not on one line."""
    if axis is not None and radius is None:
        sel = bore_nodes(xyz, surf, np.asarray(point, float), axis, h)
        if sel is not None and len(sel) >= 6:
            return sel
    d = np.linalg.norm(xyz[surf] - point, axis=1)
    r = d.min() + h if radius is None else float(radius)
    while True:
        sel = surf[d <= r * (1 + 1e-9)]
        if len(sel) >= 6:
            P = xyz[sel] - xyz[sel].mean(axis=0)
            s = np.linalg.svd(P, compute_uv=False)
            if s[1] > 1e-6 * s[0]:
                return sel
        if radius is not None:
            raise ModelError(f"interface at {list(point)}: fewer than 6 surface nodes (or all on a line) "
                             f"within the radius {radius}")
        r *= 1.25


def fe_model(xyz, tets, surf, material, interfaces, h):
    """FeModel of the tet mesh with material (E, nu, rho) and interfaces [(point, type,
    radius, axis)]: interface nodes INTERFACE_IDS + 1, ... in that order."""
    fe = FeModel()
    fe.nodes = {i + 1: tuple(x) for i, x in enumerate(xyz)}
    fe.solids = [(k + 1, 1, list(t + 1)) for k, t in enumerate(tets)]
    fe.psolid = {1: 1}
    fe.materials = {1: tuple(material)}
    for k, (point, kind, radius, axis) in enumerate(interfaces):
        nid = INTERFACE_IDS + k + 1
        fe.nodes[nid] = tuple(float(x) for x in point)
        sel = [int(i) + 1 for i in spider_nodes(xyz, surf, np.asarray(point, float), radius, h, axis)]
        if kind == "rbe2":
            fe.rbe2.append((nid, nid, "123456", sel))
        elif kind == "rbe3":
            fe.rbe3.append((nid, nid, "123456", [(1.0, "123", sel)]))
        else:
            raise ModelError(f"interface type {kind}: rbe2 or rbe3")
    return fe
