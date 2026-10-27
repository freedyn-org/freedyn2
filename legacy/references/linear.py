"""Linear models with closed-form solutions: forced damped oscillator, torsion disks."""

import math

import numpy as np

def forced_oscillator_reference(model, times):
    """[(omega_body, A, R)] of examples/compare/forced_oscillator.fds: m z'' = -k z - d z' +
    F0 sin(w t) on a prismatic joint along z, z(0) = z'(0) = 0 (analytic)."""
    (body,) = model["bodies"].values()
    m = body["mass"]
    p = model["parameters"]
    k, d, F0, w = p["p_k"], p["p_d"], p["p_f0"], p["p_w"]
    a, b = np.linalg.solve([[k - m * w * w, -d * w], [d * w, k - m * w * w]], [F0, 0.0])
    wn = math.sqrt(k / m)
    s = d / (2 * m)
    wd = math.sqrt(wn * wn - s * s)
    C = -b
    D = (s * C - a * w) / wd
    t = np.asarray(times)
    z = a * np.sin(w * t) + b * np.cos(w * t) + np.exp(-s * t) * (C * np.cos(wd * t) + D * np.sin(wd * t))
    R = np.zeros((len(t), 3))
    R[:, 2] = z
    return [(np.zeros((len(t), 3)), np.repeat(np.eye(3)[None], len(t), 0), R)]


def torsion_disks_reference(model, times):
    """[(omega_body, A, R)] of examples/compare/torsion_disks.fds, exact: absolute angles
    theta1 (disk 1 to the ground), theta2 (disk 2), both about the global z axis,
        J1 theta1'' = -k1 theta1 - (c1 + c3) theta1' + k2 (theta2 - theta1) + c2 (theta2' - theta1')
        J2 theta2'' = -k2 (theta2 - theta1) - c2 (theta2' - theta1')
    (spring 2 acts on disk 2 and, opposite, on disk 1), solved with the matrix exponential."""
    b1, b2 = model["bodies"]["disk1"], model["bodies"]["disk2"]
    J1, J2 = b1["inertia"][2], b2["inertia"][2]
    p = model["parameters"]
    k1, k2, c1, c2, c3 = (p[f"p_{n}"] for n in ("k1", "k2", "c1", "c2", "c3"))
    M = np.diag([J1, J2])
    K = np.array([[k1 + k2, -k2], [-k2, k2]])
    C = np.array([[c1 + c3 + c2, -c2], [-c2, c2]])
    Ainv = np.linalg.inv(M)
    S = np.block([[np.zeros((2, 2)), np.eye(2)], [-Ainv @ K, -Ainv @ C]])
    x0 = np.array([0.0, 0.0, b1["angular_velocity"][2], b2["angular_velocity"][2]])
    w, V = np.linalg.eig(S)
    coef = np.linalg.solve(V, x0)
    X = np.real(np.array([V @ (coef * np.exp(w * t)) for t in times]))  # (nt, 4)
    out = []
    for k, b in ((0, b1), (1, b2)):
        th, om = X[:, k], X[:, 2 + k]
        c, s = np.cos(th), np.sin(th)
        A = np.zeros((len(times), 3, 3))
        A[:, 0, 0], A[:, 0, 1], A[:, 1, 0], A[:, 1, 1], A[:, 2, 2] = c, -s, s, c, 1.0
        W = np.zeros((len(times), 3))
        W[:, 2] = om
        out.append((W, A, np.tile(np.array(b["position"], dtype=float), (len(times), 1))))
    return out
