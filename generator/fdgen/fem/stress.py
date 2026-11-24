"""Modal stresses: the stress tensor of each mode shape at the nodes of the solids, for fatigue
(modal superposition sigma(t) = sum_i sigma_i eta_i(t)).

Per element the stress sigma = D B(xi_node) u_e at its own nodes (the strain of the displacement
interpolation there), then averaged over the elements at a node. For TET10 the strain is linear
in an element, so the nodal values are exact for the element; TET4 is constant; HEX8 includes
its incompatible modes (recovered from the element displacements as in the condensation).
Order of the components: xx yy zz xy yz zx.
"""

import numpy as np

from . import elements

# natural coordinates of the element nodes (the element shape functions' node order)
_TET4 = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float)
_TET10 = np.vstack([_TET4] + [(_TET4[i] + _TET4[j]) / 2 for i, j in ((0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3))])
NODES = {4: _TET4, 10: _TET10, 8: elements.HEX_CORNERS}


def nodal_stresses(a, Phi, chunk=4000):
    """(node indices into a.node_ids (n,), stresses (n, 6, nm)) of the mode shapes Phi (all dofs
    x nm), averaged over the solid elements at each node."""
    nm = Phi.shape[1]
    acc = np.zeros((len(a.node_ids), 6, nm))
    count = np.zeros(len(a.node_ids))
    for conn, E, nu in a.solids:
        nn = conn.shape[1]
        _, shape, _ = elements.SOLIDS[nn]
        N, dN = shape(NODES[nn])  # dN: (nn points, nn functions, 3)
        D = elements.elasticity(E, nu)
        for c0 in range(0, len(conn), chunk):
            c = conn[c0:c0 + chunk]
            X = a.xyz[c]                                             # (ne, nn, 3)
            J = np.einsum("pfk,efj->epjk", dN, X)                    # at each node point p
            dNdx = np.einsum("pfk,epkj->epfj", dN, np.linalg.inv(J))
            B = elements._bmat(dNdx)                                 # (ne, p, 6, 3 nn)
            dofs = (a.start[c][:, :, None] + np.arange(3)).reshape(len(c), 3 * nn)
            u = Phi[dofs]                                            # (ne, 3 nn, nm)
            eps = np.einsum("epjk,ekm->epjm", B, u, optimize=True)      # (ne, p, 6, nm)
            if nn == 8:  # incompatible modes: alpha = -K_aa^-1 K_au u
                pg, wg = elements.hex_rule()
                Ng, dNg = shape(pg)
                Jg = np.einsum("qfk,efj->eqjk", dNg, X)
                detg = np.linalg.det(Jg)
                Bg = elements._bmat(np.einsum("qfk,eqkj->eqfj", dNg, np.linalg.inv(Jg)))
                Kaa, Kau = elements.incompatible_matrices(pg, wg, Jg, detg, Bg, D)
                alpha = -np.linalg.solve(Kaa, np.einsum("eak,ekm->eam", Kau, u))
                Ga = elements.incompatible_strain(NODES[8], J, np.linalg.det(J), elements.centre_jacobian(Jg, wg))
                eps += np.einsum("epja,eam->epjm", Ga, alpha)
            sig = np.einsum("ij,epjm->epim", D, eps)                     # (ne, p, 6, nm)
            np.add.at(acc, c.ravel(), sig.reshape(-1, 6, nm))
            np.add.at(count, c.ravel(), 1.0)
    used = np.flatnonzero(count)
    return used, acc[used] / count[used][:, None, None]


def von_mises(s):
    """von Mises stress of tensors (..., 6) (xx yy zz xy yz zx)."""
    xx, yy, zz, xy, yz, zx = (s[..., k] for k in range(6))
    return np.sqrt(0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2) + 3 * (xy ** 2 + yz ** 2 + zx ** 2))
