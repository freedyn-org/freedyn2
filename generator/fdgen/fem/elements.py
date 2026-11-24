"""Element matrices of linear elasticity (small strains, isotropic material), vectorized over
the elements of one type.

Solids (isoparametric, 3 translational dofs per node, consistent mass):
    TET4, TET10  collapsed Gauss rule (Duffy transform of a 4 x 4 x 4 Gauss-Legendre rule): exact
                 for polynomials of degree 4 on straight tetrahedra (stiffness and mass of TET10)
    HEX8         2 x 2 x 2 Gauss; with incompatible modes (Wilson-Taylor, the Jacobian at the
                 centre, which passes the patch test) by default, as Nastran's CHEXA, so that
                 bending is not locked
Beam (2 nodes, 6 dofs each): 3D Timoshenko stiffness, consistent mass (with the polar inertia).

All functions also return the quadrature points with their mass weights rho detJ w and the
shape function values, for the inertia invariants (fdgen.fem.modal).
"""

import numpy as np


# --- quadrature ---------------------------------------------------------------------------------


def gauss(n):
    x, w = np.polynomial.legendre.leggauss(n)
    return (x + 1) / 2, w / 2  # on [0, 1]


def tet_rule(n=4):
    """Points (volume coordinates L1..L3, L0 = 1 - sum) and weights of the unit tetrahedron
    (volume 1/6) by the Duffy transform of a Gauss rule with n points per direction."""
    x, w = gauss(n)
    u, v, t = np.meshgrid(x, x, x, indexing="ij")
    wu, wv, wt = np.meshgrid(w, w, w, indexing="ij")
    a = u
    b = v * (1 - u)
    c = t * (1 - u) * (1 - v)
    wt_ = wu * wv * wt * (1 - u) ** 2 * (1 - v)
    return np.column_stack([a.ravel(), b.ravel(), c.ravel()]), wt_.ravel()


def hex_rule(n=2):
    x, w = np.polynomial.legendre.leggauss(n)
    p = np.array(np.meshgrid(x, x, x, indexing="ij")).reshape(3, -1).T
    ww = np.prod(np.array(np.meshgrid(w, w, w, indexing="ij")).reshape(3, -1), axis=0)
    return p, ww


# --- shape functions ----------------------------------------------------------------------------


def tet4_shape(p):
    """N (nq, 4), dN/dxi (nq, 4, 3) at points p = (xi, eta, zeta); node 1 at the origin."""
    xi, eta, zeta = p.T
    N = np.column_stack([1 - xi - eta - zeta, xi, eta, zeta])
    d = np.array([[-1, -1, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float)
    return N, np.broadcast_to(d, (len(p), 4, 3)).copy()


def tet10_shape(p):
    """Nastran node order: corners 1-4, then the midside nodes 1-2, 2-3, 3-1, 1-4, 2-4, 3-4."""
    xi, eta, zeta = p.T
    L = np.column_stack([1 - xi - eta - zeta, xi, eta, zeta])
    dL = np.array([[-1, -1, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float)
    edges = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
    N = np.empty((len(p), 10))
    dN = np.empty((len(p), 10, 3))
    for i in range(4):
        N[:, i] = L[:, i] * (2 * L[:, i] - 1)
        dN[:, i] = (4 * L[:, i] - 1)[:, None] * dL[i]
    for k, (i, j) in enumerate(edges):
        N[:, 4 + k] = 4 * L[:, i] * L[:, j]
        dN[:, 4 + k] = 4 * (L[:, j][:, None] * dL[i] + L[:, i][:, None] * dL[j])
    return N, dN


HEX_CORNERS = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float)


def hex8_shape(p):
    c = HEX_CORNERS
    f = 1 + p[:, None, :] * c[None, :, :]  # (nq, 8, 3)
    N = np.prod(f, axis=2) / 8
    dN = np.empty((len(p), 8, 3))
    for a in range(3):
        o = [b for b in range(3) if b != a]
        dN[:, :, a] = c[None, :, a] * f[:, :, o[0]] * f[:, :, o[1]] / 8
    return N, dN


# --- solids -------------------------------------------------------------------------------------


def elasticity(E, nu):
    lam, mu = E * nu / ((1 + nu) * (1 - 2 * nu)), E / (2 * (1 + nu))
    D = np.zeros((6, 6))
    D[:3, :3] = lam
    D[np.arange(3), np.arange(3)] += 2 * mu
    D[np.arange(3, 6), np.arange(3, 6)] = mu
    return D


def _bmat(dNdx):
    """Strain-displacement matrices (..., 6, 3 nn) from dN/dx (..., nn, 3); strain order
    xx yy zz xy yz zx (engineering shear)."""
    *lead, nn, _ = dNdx.shape
    B = np.zeros((*lead, 6, 3 * nn))
    x, y, z = dNdx[..., 0], dNdx[..., 1], dNdx[..., 2]
    B[..., 0, 0::3] = x
    B[..., 1, 1::3] = y
    B[..., 2, 2::3] = z
    B[..., 3, 0::3], B[..., 3, 1::3] = y, x
    B[..., 4, 1::3], B[..., 4, 2::3] = z, y
    B[..., 5, 0::3], B[..., 5, 2::3] = z, x
    return B


SOLIDS = {4: ("TET4", tet4_shape, tet_rule), 10: ("TET10", tet10_shape, tet_rule), 8: ("HEX8", hex8_shape, hex_rule)}


def solid_matrices(X, E, nu, rho, incompatible=True):
    """K, M (ne, 3 nn, 3 nn) of solids with the node coordinates X (ne, nn, 3), and the
    quadrature data (points x (ne, nq, 3), mass weights (ne, nq), N (nq, nn))."""
    ne, nn, _ = X.shape
    _, shape, rule = SOLIDS[nn]
    p, w = rule()
    N, dN = shape(p)
    J = np.einsum("qnk,enj->eqjk", dN, X)  # dx_j / dxi_k
    det = np.linalg.det(J)
    if np.any(det <= 0):
        bad = int(np.sum(np.any(det <= 0, axis=1)))
        raise ValueError(f"{bad} elements with a non-positive Jacobian (inverted or badly shaped)")
    Jinv = np.linalg.inv(J)
    dNdx = np.einsum("qnk,eqkj->eqnj", dN, Jinv)
    B = _bmat(dNdx)
    D = elasticity(E, nu)
    dv = det * w
    K = np.einsum("eqik,ij,eqjl,eq->ekl", B, D, B, dv, optimize=True)
    if nn == 8 and incompatible:
        K = K - _incompatible_condensed(p, w, J, det, B, D)
    Ms = np.einsum("qa,qb,eq->eab", N, N, rho * dv)
    M = np.zeros((ne, 3 * nn, 3 * nn))
    for a in range(3):
        M[:, a::3, a::3] = Ms
    xq = np.einsum("qn,enj->eqj", N, X)
    return K, M, (xq, rho * dv, N)


def _incompatible_condensed(p, w, J, det, B, D):
    """Wilson-Taylor incompatible modes of HEX8 (1 - xi^2 per direction and component, 9
    internal dofs), condensed: K_uu - K_ua K_aa^-1 K_au; returns K_ua K_aa^-1 K_au. The modes
    use the Jacobian at the centre (Taylor), scaled by det J0 / det J: the patch test holds."""
    J0 = np.einsum("eqjk->ejk", J * w[None, :, None, None]) / w.sum()  # mean Jacobian (= centre for a parallelepiped)
    det0 = np.linalg.det(J0)
    J0inv = np.linalg.inv(J0)
    dP = np.zeros((len(p), 3, 3))  # d(1 - xi_k^2)/dxi_k per point, modes k
    for k in range(3):
        dP[:, k, k] = -2 * p[:, k]
    dPdx = np.einsum("qmk,ekj->eqmj", dP, J0inv) * (det0[:, None] / det)[:, :, None, None]
    G = _bmat(dPdx)  # (ne, nq, 6, 9)
    dv = det * w
    Kaa = np.einsum("eqik,ij,eqjl,eq->ekl", G, D, G, dv, optimize=True)
    Kau = np.einsum("eqik,ij,eqjl,eq->ekl", G, D, B, dv, optimize=True)
    return np.einsum("eki,ekl->eil", Kau, np.linalg.solve(Kaa, Kau), optimize=True)


# --- beams --------------------------------------------------------------------------------------


def beam_matrices(XA, XB, v, E, nu, rho, A, Iy, Iz, J, shear=None, nq=4):
    """K, M (ne, 12, 12) in global dofs [u_A, theta_A, u_B, theta_B] of 3D beams from XA to XB,
    the local y axis in the plane of the axis and v (CBAR). Stiffness with shear deformation
    (Timoshenko) with the shear factor `shear` (default: solid circle, Cowper
    6 (1 + nu) / (7 + 6 nu); 0: Euler-Bernoulli), consistent mass of Euler-Bernoulli. Quadrature
    data for the invariants: points along the axis with the mass rho A ds and the
    interpolation of the translation (Hermite for the bending), as (x, weight, H (ne, nq, 3, 12))."""
    G = E / (2 * (1 + nu))
    if shear is None:
        shear = 6 * (1 + nu) / (7 + 6 * nu)
    d = XB - XA
    L = np.linalg.norm(d, axis=1)
    ex = d / L[:, None]
    ez = np.cross(ex, v)
    ez /= np.linalg.norm(ez, axis=1)[:, None]
    ey = np.cross(ez, ex)
    R = np.stack([ex, ey, ez], axis=1)  # rows: local axes
    ne = len(L)
    k = np.zeros((ne, 12, 12))
    m = np.zeros((ne, 12, 12))
    EA, GJ, EIy, EIz = E * A / L, G * J / L, E * Iy, E * Iz
    for i, j, s in ((0, 0, 1), (0, 6, -1), (6, 6, 1)):
        k[:, i, j] = s * EA
        k[:, i + 3, j + 3] = s * GJ
    for (u, t, EI, sg) in ((1, 5, EIz, 1), (2, 4, EIy, -1)):  # v-thz plane, w-thy plane
        # Timoshenko: shear flexibility phi = 12 EI / (kappa G A L^2) (kappa = 0: none)
        phi = 12 * EI / (shear * G * A * L ** 2) if shear else 0.0 * L
        a, b, c = 12 * EI / L ** 3 / (1 + phi), 6 * EI / L ** 2 / (1 + phi), EI / L / (1 + phi)
        idx = [u, t, u + 6, t + 6]
        kk = np.array([[a, sg * b, -a, sg * b], [sg * b, (4 + phi) * c, -sg * b, (2 - phi) * c],
                       [-a, -sg * b, a, -sg * b], [sg * b, (2 - phi) * c, -sg * b, (4 + phi) * c]])
        for r in range(4):
            for q in range(4):
                k[:, idx[r], idx[q]] += kk[r, q]
    # consistent mass
    mL = rho * A * L
    for i, j, s in ((0, 0, 2), (0, 6, 1), (6, 6, 2)):
        m[:, i, j] = s * mL / 6
        m[:, i + 3, j + 3] = s * rho * J * L / 6
    for (u, t, sg) in ((1, 5, 1), (2, 4, -1)):
        idx = [u, t, u + 6, t + 6]
        mm = np.array([[156, sg * 22 * L, 54, -sg * 13 * L], [sg * 22 * L, 4 * L * L, sg * 13 * L, -3 * L * L],
                       [54, sg * 13 * L, 156, -sg * 22 * L], [-sg * 13 * L, -3 * L * L, -sg * 22 * L, 4 * L * L]],
                      dtype=object)
        for r in range(4):
            for q in range(4):
                m[:, idx[r], idx[q]] += np.asarray(mm[r, q], float) * mL / 420
    for M_ in (k, m):  # symmetric fill
        il = np.tril_indices(12, -1)
        M_[:, il[0], il[1]] = M_[:, il[1], il[0]]
    T = np.zeros((ne, 12, 12))
    for b in range(4):
        T[:, 3 * b:3 * b + 3, 3 * b:3 * b + 3] = R
    K = np.einsum("eji,ejk,ekl->eil", T, k, T, optimize=True)
    M = np.einsum("eji,ejk,ekl->eil", T, m, T, optimize=True)
    # quadrature along the axis for the invariants: translation (local) by linear / Hermite shapes
    s, ws = gauss(nq)
    H = np.zeros((ne, nq, 3, 12))
    h1, h2, h3, h4 = 1 - 3 * s ** 2 + 2 * s ** 3, (s - 2 * s ** 2 + s ** 3), 3 * s ** 2 - 2 * s ** 3, (-s ** 2 + s ** 3)
    H[:, :, 0, 0], H[:, :, 0, 6] = 1 - s, s
    H[:, :, 1, 1], H[:, :, 1, 5], H[:, :, 1, 7], H[:, :, 1, 11] = h1, h2 * L[:, None], h3, h4 * L[:, None]
    H[:, :, 2, 2], H[:, :, 2, 4], H[:, :, 2, 8], H[:, :, 2, 10] = h1, -h2 * L[:, None], h3, -h4 * L[:, None]
    Hg = np.einsum("eja,eqjk,ekl->eqal", R, H, T, optimize=True)  # global translation from global dofs
    x = XA[:, None, :] + s[None, :, None] * d[:, None, :]
    wq = (rho * A * L)[:, None] * ws[None, :]
    return K, M, (x, wq, Hg)
