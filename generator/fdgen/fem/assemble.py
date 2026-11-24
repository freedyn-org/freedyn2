"""Assembly of a linear FE model: element matrices into sparse (COO) K and M, the dependencies of
RBE2 and RBE3 as u_dep = G u_indep, and the quadrature data for the inertia invariants.

Dofs: 3 translations per node of the solids; 6 (translations and rotations) for the nodes of
beams and the master / reference nodes of RBE2 and RBE3. Nodes no element or RBE uses get none.
"""

from dataclasses import dataclass, field

import numpy as np

from ..errors import ModelError
from . import elements


@dataclass
class Assembly:
    node_ids: np.ndarray                 # nodes with dofs
    xyz: np.ndarray                      # their positions
    start: np.ndarray                    # first dof of each node
    ndof: np.ndarray                     # 3 or 6
    n: int
    K: tuple                             # (rows, cols, vals)
    M: tuple
    G: tuple = ((), (), ())              # dependent rows, independent cols, vals
    quad: list = field(default_factory=list)  # [(x (ne, nq, 3), w (ne, nq), interp, dofs (ne, nd))]
    masters: dict = field(default_factory=dict)  # RBE master / reference node -> element id
    surface: tuple = None                # boundary of the solids: (node indices, triangles into them)
    spiders: list = field(default_factory=list)  # [(master / reference node, "rbe2" | "rbe3", [node ids])]

    def index(self, node):
        k = np.searchsorted(self.node_ids, node)
        if k >= len(self.node_ids) or self.node_ids[k] != node:
            raise ModelError(f"node {node} has no dofs (not used by an element)")
        return int(k)

    def dofs(self, node, comps=range(6)):
        k = self.index(node)
        return [int(self.start[k] + c) for c in comps if c < self.ndof[k]]


def _coo(blocks, dofs):
    """COO of element matrices (ne, nd, nd) at the element dofs (ne, nd)."""
    ne, nd = dofs.shape
    rows = np.repeat(dofs, nd, axis=1).ravel()
    cols = np.tile(dofs, (1, nd)).ravel()
    return rows, cols, blocks.reshape(ne, nd * nd).ravel()


def assemble(fe, chunk=4000):
    """Assembly of a fdgen.fem.nastran.FeModel (or any object with its fields)."""
    six = set()
    for _, _, ga, gb, _ in fe.bars:
        six |= {ga, gb}
    for _, master, _, _ in fe.rbe2:
        six.add(master)
    for _, ref, _, _ in fe.rbe3:
        six.add(ref)
    used = set(six)
    for _, _, ids in fe.solids:
        used.update(ids)
    for _, master, _, slaves in fe.rbe2:
        used.update(slaves)
    for _, _, _, groups in fe.rbe3:
        for g in groups:
            used.update(g[2])
    missing = [i for i in used if i not in fe.nodes]
    if missing:
        raise ModelError(f"nodes used but not defined: {sorted(missing)[:10]}")
    node_ids = np.array(sorted(used))
    ndof = np.array([6 if i in six else 3 for i in node_ids])
    start = np.concatenate([[0], np.cumsum(ndof)[:-1]])
    a = Assembly(node_ids, fe.node_array(node_ids), start, ndof, int(ndof.sum()), None, None)
    Kc, Mc = [], []

    # solids, grouped by node count and material
    groups = {}
    for eid, pid, ids in fe.solids:
        if pid not in fe.psolid:
            raise ModelError(f"element {eid}: property {pid} is not a PSOLID")
        groups.setdefault((len(ids), fe.psolid[pid]), []).append(ids)
    for (nn, mid), conn in groups.items():
        if mid not in fe.materials:
            raise ModelError(f"material {mid} (MAT1) missing")
        E, nu, rho = fe.materials[mid]
        conn = np.array(conn)
        for c0 in range(0, len(conn), chunk):
            c = conn[c0:c0 + chunk]
            k = np.searchsorted(node_ids, c)
            X = a.xyz[k]
            try:
                Ke, Me, (xq, wq, N) = elements.solid_matrices(X, E, nu, rho)
            except ValueError as e:
                raise ModelError(f"{elements.SOLIDS[nn][0]} elements: {e}") from None
            dofs = (a.start[k][:, :, None] + np.arange(3)).reshape(len(c), 3 * nn)
            Kc.append(_coo(Ke, dofs))
            Mc.append(_coo(Me, dofs))
            a.quad.append((xq, wq, ("solid", N), dofs))

    a.surface = boundary(fe, node_ids, a.xyz)

    # beams
    if fe.bars:
        bars = {}
        for eid, pid, ga, gb, v in fe.bars:
            bars.setdefault(pid, []).append((ga, gb, v))
        for pid, items in bars.items():
            mid, A, Iy, Iz, J = fe.pbar[pid]
            E, nu, rho = fe.materials[mid]
            ka = np.searchsorted(node_ids, [g[0] for g in items])
            kb = np.searchsorted(node_ids, [g[1] for g in items])
            v = np.array([g[2] for g in items])
            Ke, Me, (xq, wq, H) = elements.beam_matrices(a.xyz[ka], a.xyz[kb], v, E, nu, rho, A, Iy, Iz, J)
            dofs = np.hstack([a.start[ka][:, None] + np.arange(6), a.start[kb][:, None] + np.arange(6)])
            Kc.append(_coo(Ke, dofs))
            Mc.append(_coo(Me, dofs))
            a.quad.append((xq, wq, ("beam", H), dofs))
    a.K = tuple(np.concatenate([c[i] for c in Kc]) for i in range(3))
    a.M = tuple(np.concatenate([c[i] for c in Mc]) for i in range(3))
    a.G = _constraints(fe, a)
    return a


def _skew(r):
    return np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])


def _constraints(fe, a):
    rows, cols, vals = [], [], []
    dependent = set()

    def add(dep, indep, coef):
        if dep in dependent:
            raise ModelError(f"dof {dep} is dependent in two RBEs")
        dependent.add(dep)
        for j, c in zip(indep, coef):
            if c != 0.0:
                rows.append(dep), cols.append(j), vals.append(c)

    for eid, master, comps, slaves in fe.rbe2:
        a.masters[master] = eid
        a.spiders.append((master, "rbe2", list(slaves)))
        xm = a.xyz[a.index(master)]
        mt, mr = a.dofs(master, range(3)), a.dofs(master, range(3, 6))
        cs = [int(c) - 1 for c in comps]
        for s in slaves:
            k = a.index(s)
            r = a.xyz[k] - xm
            for c in cs:
                if c >= a.ndof[k]:
                    continue
                dep = int(a.start[k] + c)
                if c < 3:  # u_s = u_m + theta_m x r = u_m - skew(r) theta_m
                    add(dep, [mt[c], *mr], [1.0, *(-_skew(r)[c])])
                else:
                    add(dep, [mr[c - 3]], [1.0])
    for eid, ref, refc, groups in fe.rbe3:
        a.masters[ref] = eid
        a.spiders.append((ref, "rbe3", [n for g in groups for n in g[2]]))
        if refc != "123456":
            raise ModelError(f"RBE3 {eid}: reference components {refc}, only 123456 is supported")
        xr = a.xyz[a.index(ref)]
        sdofs, Bs, ws = [], [], []
        for w, comp, nodes in groups:
            if comp != "123":
                raise ModelError(f"RBE3 {eid}: components {comp}, only 123 is supported")
            for s in nodes:
                r = a.xyz[a.index(s)] - xr
                sdofs += a.dofs(s, range(3))
                Bs.append(np.hstack([np.eye(3), -_skew(r)]))  # rigid motion of the reference at s
                ws.append(w)
        B = np.vstack(Bs)                       # (3 ns, 6)
        Wd = np.repeat(ws, 3)
        Nmat = B.T @ (Wd[:, None] * B)          # least squares fit of a rigid motion
        W = np.linalg.solve(Nmat, B.T * Wd)     # [u_r; theta_r] = W u_s   (6 x 3 ns)
        # the reference node stays independent (a boundary node): 6 slave dofs become dependent,
        # chosen by pivoted Gram-Schmidt on the columns of W
        R = W.copy()
        piv = []
        for _ in range(6):
            nrm = np.linalg.norm(R, axis=0)
            nrm[piv] = -1
            j = int(np.argmax(nrm))
            piv.append(j)
            q = R[:, j] / np.linalg.norm(R[:, j])
            R -= np.outer(q, q @ R)
        rest = [j for j in range(W.shape[1]) if j not in piv]
        Wd_inv = np.linalg.inv(W[:, piv])
        # u_piv = W_piv^-1 ([u_r; theta_r] - W_rest u_rest)
        refd = a.dofs(ref)
        for i, j in enumerate(piv):
            add(sdofs[j], refd + [sdofs[k] for k in rest],
                [*Wd_inv[i], *(-(Wd_inv[i] @ W[:, rest]))])
    return np.array(rows, int), np.array(cols, int), np.array(vals, float)


# faces of the solids by their corner nodes (local indices), as triangles; outward by geometry
_FACES = {4: [(0, 1, 2), (0, 1, 3), (1, 2, 3), (0, 2, 3)], 10: [(0, 1, 2), (0, 1, 3), (1, 2, 3), (0, 2, 3)],
          8: [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]}


def boundary(fe, node_ids, xyz):
    """Boundary of the solids (faces of one element only): (indices of its corner nodes into
    node_ids, triangles (nt, 3) into those), oriented outwards. For the viewer."""
    faces, owner = [], []
    for _, _, ids in fe.solids:
        for f in _FACES[len(ids)]:
            faces.append(tuple(ids[i] for i in f))
            owner.append(ids[:8] if len(ids) == 8 else ids[:4])
    if not faces:
        return np.zeros(0, int), np.zeros((0, 3), int)
    count = {}
    for f in faces:
        key = tuple(sorted(f))
        count[key] = count.get(key, 0) + 1
    tris = []
    for f, corners in zip(faces, owner):
        if count[tuple(sorted(f))] != 1:
            continue
        k = np.searchsorted(node_ids, f)
        c = xyz[np.searchsorted(node_ids, corners)].mean(axis=0)
        n = np.cross(xyz[k[1]] - xyz[k[0]], xyz[k[2]] - xyz[k[0]])
        if np.dot(n, xyz[k].mean(axis=0) - c) < 0:
            k = k[::-1]
        tris += [(k[0], k[1], k[2])] if len(k) == 3 else [(k[0], k[1], k[2]), (k[0], k[2], k[3])]
    tris = np.array(tris, int)
    used, inv = np.unique(tris, return_inverse=True)
    return used, inv.reshape(-1, 3)
