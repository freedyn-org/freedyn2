"""Free rigid body: RK4 on Euler's equations (omega_bar, theta' = 1/2 Gbar^T omega_bar), R = R0 + v0 t."""

import math

import numpy as np

def _rhs(J, w, th):
    J1, J2, J3 = J
    w1, w2, w3 = w
    dw = ((J2 - J3) / J1 * w2 * w3, (J3 - J1) / J2 * w3 * w1, (J1 - J2) / J3 * w1 * w2)
    t0, t1, t2, t3 = th
    # theta' = 1/2 Gbar^T omega_bar, Gbar rows [-t1, t0, t3, -t2], [-t2, -t3, t0, t1], [-t3, t2, -t1, t0]
    dth = (
        0.5 * (-t1 * w1 - t2 * w2 - t3 * w3),
        0.5 * (t0 * w1 - t3 * w2 + t2 * w3),
        0.5 * (t3 * w1 + t0 * w2 - t1 * w3),
        0.5 * (-t2 * w1 + t1 * w2 + t0 * w3),
    )
    return dw, dth


def rigid_body_reference(body, times, h=1e-4):
    """omega_body (n, 3), theta (n, 4) and R (n, 3) at `times` for a body dict of fdgen.

    `body` holds inertia, position, orientation (Euler parameters), velocity and
    angular_velocity (global), as returned by fdgen.fds.load_fds.
    """
    th = tuple(body["orientation"])
    A = np.array(
        [
            [1 - 2 * (th[2] ** 2 + th[3] ** 2), 2 * (th[1] * th[2] - th[0] * th[3]), 2 * (th[1] * th[3] + th[0] * th[2])],
            [2 * (th[1] * th[2] + th[0] * th[3]), 1 - 2 * (th[1] ** 2 + th[3] ** 2), 2 * (th[2] * th[3] - th[0] * th[1])],
            [2 * (th[1] * th[3] - th[0] * th[2]), 2 * (th[2] * th[3] + th[0] * th[1]), 1 - 2 * (th[1] ** 2 + th[2] ** 2)],
        ]
    )
    w = tuple(A.T @ np.array(body["angular_velocity"]))
    J = tuple(body["inertia"])
    t = 0.0
    out_w, out_th = [], []
    for target in times:
        n = max(1, math.ceil((target - t) / h - 1e-9)) if target > t else 0
        dt = (target - t) / n if n else 0.0
        for _ in range(n):
            k1w, k1t = _rhs(J, w, th)
            k2w, k2t = _rhs(J, [w[i] + 0.5 * dt * k1w[i] for i in range(3)], [th[i] + 0.5 * dt * k1t[i] for i in range(4)])
            k3w, k3t = _rhs(J, [w[i] + 0.5 * dt * k2w[i] for i in range(3)], [th[i] + 0.5 * dt * k2t[i] for i in range(4)])
            k4w, k4t = _rhs(J, [w[i] + dt * k3w[i] for i in range(3)], [th[i] + dt * k3t[i] for i in range(4)])
            w = tuple(w[i] + dt / 6 * (k1w[i] + 2 * k2w[i] + 2 * k3w[i] + k4w[i]) for i in range(3))
            th = tuple(th[i] + dt / 6 * (k1t[i] + 2 * k2t[i] + 2 * k3t[i] + k4t[i]) for i in range(4))
        t = target
        out_w.append(w)
        out_th.append(th)
    times = np.asarray(times)
    R = np.array(body["position"]) + np.outer(times, body["velocity"])
    return np.array(out_w), np.array(out_th), R
