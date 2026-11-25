"""Modal reduction of an assembled FE model into the modal data of a flexible body.

Craig-Bampton with the interface nodes (6 dofs each) as boundary: constraint modes and
fixed-interface normal modes (freedyn2_modal), then orthonormalized as Adams/Flex does: the
eigenproblem of the reduced K and M gives mass-normalized modes with diagonal stiffness, the
6 rigid body modes (contained in the constraint modes) are removed. Without interface nodes,
free-free normal modes.

The inertia invariants (fdgen.flex.modal) are integrals over the body, here sums over the
quadrature points of the elements with the consistent mass (exact for the FE discretization,
unlike the lumped node masses of Adams).
"""

import subprocess
import tempfile
from pathlib import Path

import numpy as np

from ..errors import ModelError
from ..flex.modal import ModalData
from . import stress


def find_modal(path=None):
    from ..cli import find_tool

    tool = find_tool("freedyn2_modal", path, "FREEDYN2_MODAL")
    if tool is None:
        raise FileNotFoundError("freedyn2_modal not found: build it (cmake) or set FREEDYN2_MODAL")
    return tool


def _solve(a, boundary, nmodes, shift, tol=1e-5, maxit=200, tool=None):
    """Run freedyn2_modal: (eigenvalues, V (n x ncols), Kr, Mr)."""
    tool = tool or find_modal()
    with tempfile.TemporaryDirectory() as tmp:
        fin, fout = Path(tmp) / "in.bin", Path(tmp) / "out.bin"
        with open(fin, "wb") as f:
            np.array([0x464D4F44, 1, a.n, len(a.K[0]), len(a.M[0]), len(a.G[0]), len(boundary), nmodes, maxit],
                     np.int64).tofile(f)
            np.array([shift, tol], np.float64).tofile(f)
            for mat in (a.K, a.M, a.G):
                np.asarray(mat[0], np.int64).tofile(f)
                np.asarray(mat[1], np.int64).tofile(f)
                np.asarray(mat[2], np.float64).tofile(f)
            np.asarray(boundary, np.int64).tofile(f)
        r = subprocess.run([str(tool), str(fin), str(fout)], capture_output=True, text=True)
        if r.returncode != 0:
            raise ModelError(r.stderr.strip() or "freedyn2_modal failed")
        raw = fout.read_bytes()
    n, nc, nb, _ = np.frombuffer(raw[:32], np.int64)
    m = nc - nb
    d = np.frombuffer(raw[32:], np.float64)
    lam, d = d[:m], d[m:]
    V, d = d[:n * nc].reshape(nc, n).T, d[n * nc:]
    Kr, Mr = d[:nc * nc].reshape(nc, nc).T, d[nc * nc:2 * nc * nc].reshape(nc, nc).T
    return lam, V, Kr, Mr


def _typical_eigenvalue(a):
    """Median of diag(K) / diag(M) over the dofs with both (an eigenvalue scale of the mesh)."""
    dk, dm = np.zeros(a.n), np.zeros(a.n)
    for (r, c, v), d in ((a.K, dk), (a.M, dm)):
        on = r == c
        np.add.at(d, r[on], v[on])
    ok = (dk > 0) & (dm > 0)
    return float(np.median(dk[ok] / dm[ok]))


def _orthonormalize(V, Kr, Mr):
    """Mass-normalized modes of the reduced problem, ascending: (lambda, V Q)."""
    L = np.linalg.cholesky((Mr + Mr.T) / 2)
    Li = np.linalg.inv(L)
    A = Li @ ((Kr + Kr.T) / 2) @ Li.T
    lam, Y = np.linalg.eigh((A + A.T) / 2)
    Q = Li.T @ Y
    return lam, V @ Q


def reduce(a, interfaces, nmodes, output_nodes=None, rigid="auto", shift=None, tool=None):
    """ModalData of the assembly a: interfaces: node ids of the boundary (RBE master / reference
    nodes, 6 dofs each), nmodes: fixed-interface normal modes (free-free modes without
    interfaces), output_nodes: nodes whose mode shapes are kept (default: interfaces and all
    nodes with dofs)."""
    boundary = []
    for nid in interfaces:
        d = a.dofs(nid)
        if len(d) != 6:
            raise ModelError(f"interface node {nid} has no rotational dofs (not an RBE master / reference node)")
        boundary += d
    if shift is None:  # free body: below 0, scaled with the stiffness to mass ratio of the model
        shift = 0.0 if boundary else -1e-8 * _typical_eigenvalue(a)
    lam_fixed, V, Kr, Mr = _solve(a, boundary, nmodes, shift, tool=tool)
    lam, Phi = _orthonormalize(V, Kr, Mr)
    # an unsupported body has 6 rigid body modes; the constraint modes contain them exactly,
    # numerically their eigenvalues are small but not 0
    nr = 6 if rigid == "auto" else int(rigid)
    if nr and len(lam) > nr and abs(lam[nr - 1]) > 1e-3 * lam[nr]:
        raise ModelError(f"the {nr} lowest modes are not rigid body modes (frequencies "
                         f"{np.sqrt(np.abs(lam[:nr + 1])) / (2 * np.pi)} Hz): a mechanism or a model in pieces?")
    lam, Phi = lam[nr:], Phi[:, nr:]
    if np.any(lam <= 0):
        raise ModelError("negative or zero eigenvalues after removing the rigid body modes (mechanism?)")
    data = invariants(a, Phi)
    data.K = np.diag(lam)
    data.frequencies = np.sqrt(lam) / (2 * np.pi)
    nodes = list(interfaces) + [i for i in (a.node_ids if output_nodes is None else output_nodes)
                                if i not in set(interfaces)]
    idx = [a.index(i) for i in nodes]
    data.node_labels = np.array(nodes, int)
    data.node_positions = a.xyz[idx]
    nm = Phi.shape[1]
    data.Phi = np.stack([Phi[a.start[k]:a.start[k] + 3] for k in idx])
    data.Psi = np.stack([Phi[a.start[k] + 3:a.start[k] + 6] if a.ndof[k] == 6 else np.zeros((3, nm)) for k in idx])
    data.removed_modes = list(range(1, nr + 1))
    # the spiders of the interfaces (viewer): node, type, positions of the nodes it connects
    data.spiders = [(int(m), kind, a.xyz[[a.index(n) for n in nodes]]) for m, kind, nodes in a.spiders
                    if m in set(int(i) for i in interfaces)]
    if a.surface is not None and len(a.surface[0]):  # mode shapes on the boundary, for the viewer
        k = a.surface[0]
        data.viz = (a.xyz[k], a.surface[1], np.stack([Phi[a.start[i]:a.start[i] + 3] for i in k]))
    if a.solids:  # the mesh of the solids and the modal stresses at its nodes (fatigue)
        used, sig = stress.nodal_stresses(a, Phi)
        remap = np.full(len(a.node_ids), -1)
        remap[used] = np.arange(len(used))
        cells = {}
        for conn, _, _ in a.solids:  # one array per node count (several materials)
            cells[conn.shape[1]] = np.vstack([cells[conn.shape[1]], remap[conn]]) if conn.shape[1] in cells else remap[conn]
        data.mesh = (a.node_ids[used], a.xyz[used], cells, sig)
    data.fixed_interface_frequencies = np.sqrt(np.maximum(lam_fixed, 0)) / (2 * np.pi)
    return data


def _sk(v):
    """skew matrices of vectors v (..., 3) -> (..., 3, 3)."""
    z = np.zeros(v.shape[:-1])
    return np.stack([np.stack([z, -v[..., 2], v[..., 1]], -1), np.stack([v[..., 2], z, -v[..., 0]], -1),
                     np.stack([-v[..., 1], v[..., 0], z], -1)], -2)


def invariants(a, Phi):
    """Inertia invariants of the mode shapes Phi (n dofs x nm) by quadrature (ModalData without
    K, frequencies and nodes)."""
    nm = Phi.shape[1]
    I1, I2, I3, I4 = 0.0, np.zeros(3), np.zeros((3, nm)), np.zeros((3, nm))
    I5, I6, I7 = np.zeros((nm, 3, nm)), np.zeros((nm, nm)), np.zeros((3, 3))
    I8, I9 = np.zeros((nm, 3, 3)), np.zeros((nm, nm, 3, 3))
    for x, w, (kind, op), dofs in a.quad:
        ue = Phi[dofs]  # (ne, nd, nm)
        if kind == "solid":
            ne, nd, _ = ue.shape
            u = np.einsum("qn,enam->eqam", op, ue.reshape(ne, nd // 3, 3, nm), optimize=True)
        else:
            u = np.einsum("eqad,edm->eqam", op, ue, optimize=True)
        x, w, u = x.reshape(-1, 3), w.ravel(), u.reshape(-1, 3, nm)
        S, U = _sk(x), _sk(np.moveaxis(u, 2, 1))  # (P, 3, 3), (P, nm, 3, 3)
        I1 += w.sum()
        I2 += w @ x
        I3 += np.einsum("p,pam->am", w, u)
        I4 += np.einsum("p,pab,pbm->am", w, S, u, optimize=True)
        I5 += np.einsum("p,pjab,pbk->jak", w, U, u, optimize=True)
        I6 += np.einsum("p,pam,pak->mk", w, u, u, optimize=True)
        I7 += np.einsum("p,pba,pbc->ac", w, S, S, optimize=True)
        I8 += np.einsum("p,pab,pjbc->jac", w, S, U, optimize=True)
        I9 += np.einsum("p,pjab,pkbc->jkac", w, U, U, optimize=True)
    return ModalData(mass=float(I1), I2=I2, I3=I3, I4=I4, I5=I5, I6=I6, I7=I7, I8=I8, I9=I9,
                     K=np.zeros((nm, nm)), frequencies=np.zeros(nm), node_labels=np.zeros(0, int),
                     node_positions=np.zeros((0, 3)), Phi=np.zeros((0, 3, nm)), Psi=np.zeros((0, 3, nm)),
                     units=(), source="freedyn2 FEM")
