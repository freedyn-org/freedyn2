"""Helpers of the reference solutions (numpy; independent of the generator's CasADi code)."""

import math

import casadi as ca
import numpy as np


def rotation_matrix(th):
    """Shabana 2005 Eq. 2.13 for Euler parameters th (nested lists)."""
    t0, t1, t2, t3 = th
    return [[1 - 2 * (t2**2 + t3**2), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)],
            [2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1**2 + t3**2), 2 * (t2 * t3 - t0 * t1)],
            [2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1**2 + t2**2)]]


def rotation(th):
    return np.array(rotation_matrix(th))


def rotation_about(a, phi):
    """Rodrigues: rotation by phi about the unit vector a (numpy array), CasADi expression."""
    S = ca.DM([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return ca.DM.eye(3) + ca.sin(phi) * S + (1 - ca.cos(phi)) * (S @ S)


def rk4(rhs, y0, times, h):
    """Classical RK4 on y' = rhs(t, y), sampled at times (substeps of at most h)."""
    y, t, out = np.array(y0, dtype=float), 0.0, []
    for target in times:
        n = max(1, math.ceil((target - t) / h - 1e-9)) if target > t else 0
        dt = (target - t) / n if n else 0.0
        for _ in range(n):
            k1 = rhs(t, y)
            k2 = rhs(t + dt / 2, y + dt / 2 * k1)
            k3 = rhs(t + dt / 2, y + dt / 2 * k2)
            k4 = rhs(t + dt, y + dt * k3)
            y = y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            t += dt
        t = target
        out.append(y.copy())
    return np.array(out)
