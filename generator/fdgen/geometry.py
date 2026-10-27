"""Geometry of the initial configuration (numpy): rotations, body coordinates of points and axes.

Model input gives points and directions globally at t = 0; elements store them in body
coordinates (or globally for the ground).
"""

import numpy as np


def rotation_matrix(t):
    """Shabana 2005 Eq. 2.13 for Euler parameters t (numpy), as fdgen.rigid_body.rotation_matrix."""
    t0, t1, t2, t3 = t
    return np.array([
        [1 - 2 * (t2**2 + t3**2), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)],
        [2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1**2 + t3**2), 2 * (t2 * t3 - t0 * t1)],
        [2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1**2 + t2**2)],
    ])


def body_frame(body):
    """Position and rotation matrix at t = 0 of a body (its model dict)."""
    return np.array(body["position"], dtype=float), rotation_matrix(body["orientation"])


def local_point(model, body, point):
    """Body coordinates of a global point (t = 0); the point itself for the ground; None: the
    center of mass / the origin."""
    if point is None:
        return np.zeros(3)
    if body == "ground":
        return np.asarray(point, dtype=float)
    R, A = body_frame(model["bodies"][body])
    return A.T @ (np.asarray(point, dtype=float) - R)


def local_axis(model, body, vec, normalize=True):
    """Body components of a global direction (t = 0); the direction itself for the ground."""
    v = np.asarray(vec, dtype=float)
    if normalize:
        v = v / np.linalg.norm(v)
    return v if body == "ground" else body_frame(model["bodies"][body])[1].T @ v
