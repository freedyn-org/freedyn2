"""Geometry of bodies and the ground for the viewer (freedyn2 view); no effect on the simulation.

Shapes are given as all model input, globally at t = 0 (or with frame: body in the body frame),
and stored in the body frame: position and Euler parameters of the shape frame relative to the
body frame. Without shapes the viewer draws a default: the box with the body's mass and
principal inertia, or for the wheel of a tire a cylinder of the tire radius and width.
"""

from pathlib import Path

import numpy as np

from . import extensions
from .errors import ModelError
from .geometry import body_frame, rotation_matrix

MESH_SUFFIXES = (".stl", ".obj")
_NEEDS = {"box": ("size",), "cylinder": ("radius", "length"), "sphere": ("radius",), "mesh": ("file",)}


def euler_parameters(A):
    """Euler parameters [theta0, theta1, theta2, theta3] (theta0 >= 0) of a rotation matrix."""
    t0 = 0.5 * np.sqrt(max(0.0, 1.0 + np.trace(A)))
    if t0 > 0.1:
        t = np.array([t0, (A[2, 1] - A[1, 2]) / (4 * t0), (A[0, 2] - A[2, 0]) / (4 * t0),
                      (A[1, 0] - A[0, 1]) / (4 * t0)])
    else:  # near 180 deg: from the largest diagonal element
        i = int(np.argmax(np.diag(A)))
        j, k = (i + 1) % 3, (i + 2) % 3
        ti = 0.5 * np.sqrt(max(0.0, 1.0 + A[i, i] - A[j, j] - A[k, k]))
        t = np.zeros(4)
        t[0] = (A[k, j] - A[j, k]) / (4 * ti)
        t[1 + i], t[1 + j], t[1 + k] = ti, (A[j, i] + A[i, j]) / (4 * ti), (A[k, i] + A[i, k]) / (4 * ti)
    t = t / np.linalg.norm(t)
    return t if t[0] >= 0 else -t


def validate_shapes(owner, shapes, body, base_dir):
    """Shapes of a body (its model dict; None: the ground) in its frame; mesh files absolute."""
    R, A = (np.zeros(3), np.eye(3)) if body is None else body_frame(body)
    out = []
    for k, s in enumerate(shapes or []):
        where = f"{owner}: shape {k + 1} ({s['type']})"
        for key in _NEEDS[s["type"]]:
            if key not in s:
                raise ModelError(f"{where} needs {key}")
        s = dict(s)
        frame = s.pop("frame", "global")
        if frame == "body" and body is None:
            raise ModelError(f"{where}: the ground has no body frame (frame: global)")
        t = np.array(s.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=float)
        if np.linalg.norm(t) == 0:
            raise ModelError(f"{where}: orientation must not be zero")
        As = rotation_matrix(t / np.linalg.norm(t))
        if frame == "global":  # defaults: the body frame
            As = As if "orientation" in s else A
            p, As = A.T @ (np.asarray(s.get("position", R), dtype=float) - R), A.T @ As
        else:
            p = np.asarray(s.get("position", [0.0, 0.0, 0.0]), dtype=float)
        s["position"], s["orientation"] = p.tolist(), euler_parameters(As).tolist()
        if s["type"] == "mesh":
            path = Path(s["file"])
            path = (path if path.is_absolute() else Path(base_dir or ".") / path).resolve()
            if path.suffix.lower() not in MESH_SUFFIXES:
                raise ModelError(f"{where}: {path.name}: meshes are .stl or .obj files")
            if not path.is_file():
                raise ModelError(f"{where}: file {path} not found")
            s["file"] = str(path)
            scale = s.get("scale", 1.0)
            s["scale"] = [float(scale)] * 3 if np.isscalar(scale) else list(scale)
        out.append(s)
    return out


def inertia_box(mass, inertia):
    """Edge lengths of the homogeneous box with this mass and principal inertia (a box for any
    inertia: lengths that would be imaginary or tiny are limited to a tenth of the largest)."""
    I1, I2, I3 = inertia
    sq = 6.0 / mass * np.array([I2 + I3 - I1, I1 + I3 - I2, I1 + I2 - I3])
    a = np.sqrt(np.maximum(sq, 0.0))
    return np.maximum(a, 0.1 * max(a.max(), 1e-3)).tolist()


def default_shapes(model):
    """Shapes of the bodies without any: {body: [shape]}. Wheels of tires: a cylinder about the
    wheel axis (body y) of the tire radius and width, other bodies: the inertia box."""
    wheels = {}
    for f in model.get("forces", []):
        if f["type"] == "tire":
            radius = f.get("radius", 0.3)
            wheels[f["body1"]] = extensions.wheel(f) or (radius, 0.4 * radius)
    out = {}
    for name, b in model["bodies"].items():
        if b.get("shapes"):
            continue
        if name in wheels:
            r, w = wheels[name]  # cylinder axis z -> body y: rotation by -90 deg about x
            c = np.sqrt(0.5)
            out[name] = [{"type": "cylinder", "radius": r, "length": w, "position": [0.0, 0.0, 0.0],
                          "orientation": [c, -c, 0.0, 0.0], "color": "#303030"}]
        else:
            out[name] = [{"type": "box", "size": inertia_box(b["mass"], b["inertia"]),
                          "position": [0.0, 0.0, 0.0], "orientation": [1.0, 0.0, 0.0, 0.0]}]
    return out
