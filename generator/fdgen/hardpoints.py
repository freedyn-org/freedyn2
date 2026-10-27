"""Named points (hardpoints) and frames of the model file.

  points:
    uca_outer_fl: [2.55, 0.72, 0.62]
    uca_outer_fr: {mirror: uca_outer_fl}                  # y negated: the other side
    lca_outer_fl: {from: uca_outer_fl, offset: [0.05, 0.06, -0.37]}
    tie_outer_fl: {in: carrier_fl, local: [-0.12, -0.01, -0.07]}  # in the frame of a body or a frame
  frames:
    kingpin_fl: {origin: lca_outer_fl, z: [lca_outer_fl, uca_outer_fl]}
  joints:
    upper_ball_fl: {type: spherical, body1: carrier_fl, body2: uca_fl, location: uca_outer_fl}
    steer_fl: {type: revolute, body1: carrier_fl, body2: knuckle_fl, location: kingpin_fl, orientation: kingpin_fl}

A point or frame given in a body (in: body) is in the body frame (center of mass, principal
axes) at t = 0; in a frame, in its axes from its origin. Wherever a point is given
(POINT_KEYS) its name may stand instead of the numbers, a frame name
for its origin; wherever an orientation is given (ORIENTATION_KEYS) a frame name. Points may
use formulas of the parameters (=x_uca), so a hardpoint is tuned or swept like any number.
Names are replaced when the model is read, after the formulas and before the schema check, so
everything after it sees numbers only, as with the formulas.
"""

import numpy as np

from .errors import ModelError
from .geometry import rotation_matrix
from .shapes import euler_parameters

# section -> keys holding a point / an orientation (global at t = 0)
POINT_KEYS = {"bodies": ("position",), "joints": ("location",), "forces": ("point", "point1", "point2", "location"),
              "sensors": ("point",), "measures": ("point1", "point2")}
ORIENTATION_KEYS = {section: ("orientation",) for section in ("bodies", "joints", "forces", "sensors", "measures")}


def _vec3(v, where):
    a = np.asarray(v, dtype=float) if isinstance(v, list) and len(v) == 3 else None
    if a is None or not np.all(np.isfinite(a)):
        raise ModelError(f"{where}: {v!r} is not a point [x, y, z]")
    return a


class _Resolver:
    """Values of points, frames and body frames at t = 0, resolved on demand: points and frames
    may be defined by each other and in the frame of a body (in: body), whose position and
    orientation may themselves be points and frames; a loop is an error."""

    def __init__(self, points, frames, bodies):
        self.raw = {"point": points, "frame": frames}
        self.bodies = bodies
        self.values, self.visiting = {}, []
        self.used = {name: [] for name in [*points, *frames]}  # name -> what uses it
        self.carrier = {}  # point or frame -> the body it is given in (in: body)

    def _get(self, kind, name, compute):
        key = (kind, name)
        if key not in self.values:
            if key in self.visiting:
                loop = self.visiting[self.visiting.index(key):]
                raise ModelError("points and frames defined by each other: " + " -> ".join(n for _, n in loop + [key]))
            self.visiting.append(key)
            self.values[key] = compute()
            self.visiting.pop()
        return self.values[key]

    def use(self, name, user):
        self.used[name].append(user)

    def point(self, name, user=None):
        """Global position of a point, or of the origin of a frame."""
        if name in self.raw["frame"]:
            if user:
                self.use(name, user)
            return self.frame(name)[0]
        if name not in self.raw["point"]:
            return None
        if user:
            self.use(name, user)
        return self._get("point", name, lambda: self._point(name, self.raw["point"][name]))

    def _point(self, name, d):
        where = f"points.{name}"
        if not isinstance(d, dict):
            return _vec3(d, where)
        if "in" in d:
            o, A = self.in_frame(d["in"], name, where + ".in")
            return o + A @ _vec3(d.get("local"), where + ".local")
        ref = d.get("mirror", d.get("from"))
        if not isinstance(ref, str):
            raise ModelError(f"{where}: give [x, y, z], {{mirror: name}}, {{from: name, offset: [dx, dy, dz]}} "
                             "or {in: body or frame, local: [x, y, z]}")
        p = self.point(ref, name)
        if p is None:
            raise ModelError(f"{where}: unknown point {ref!r}")
        return p * [1, -1, 1] if "mirror" in d else p + _vec3(d.get("offset"), where + ".offset")

    def frame(self, name, user=None):
        """(origin, rotation matrix) of a frame."""
        if user:
            self.use(name, user)
        return self._get("frame", name, lambda: self._frame(name, self.raw["frame"][name]))

    def body(self, name):
        """(position, rotation matrix) of a body at t = 0 (its position and orientation may be names)."""
        def compute():
            b = self.bodies[name]
            R = self.place(b.get("position", [0.0, 0.0, 0.0]), f"bodies.{name}.position", name)
            o = b.get("orientation", [1.0, 0.0, 0.0, 0.0])
            if isinstance(o, str):
                if o not in self.raw["frame"]:
                    raise ModelError(f"bodies.{name}.orientation: unknown frame {o!r}")
                return R, self.frame(o, name)[1]
            q = np.asarray(o, dtype=float)
            return R, rotation_matrix(q / np.linalg.norm(q))
        return self._get("body", name, compute)

    def in_frame(self, ref, user, where):
        """(origin, rotation matrix) of the body or frame a definition is given in."""
        if not isinstance(ref, str):
            raise ModelError(f"{where}: the name of a body or a frame")
        if ref in self.raw["frame"]:
            if ref in self.bodies:
                raise ModelError(f"{where}: {ref!r} is a body and a frame")
            return self.frame(ref, user)
        if ref in self.bodies:
            self.carrier.setdefault(user, ref)
            return self.body(ref)
        raise ModelError(f"{where}: unknown body or frame {ref!r}")

    def place(self, spec, where, user):
        """A point field: [x, y, z] or a name."""
        if isinstance(spec, str):
            p = self.point(spec, user)
            if p is None:
                raise ModelError(f"{where}: unknown point {spec!r}")
            return p
        return _vec3(spec, where)

    def direction(self, spec, A, user, where):
        """[x, y, z] in the axes A, or two point names [from, to] (global)."""
        if isinstance(spec, list) and len(spec) == 2 and all(isinstance(s, str) for s in spec):
            a, b = (self.point(s, user) for s in spec)
            for s, v in zip(spec, (a, b)):
                if v is None:
                    raise ModelError(f"{where}: unknown point {s!r}")
            d = b - a
        else:
            d = A @ _vec3(spec, where)
        n = np.linalg.norm(d)
        if not n > 0:
            raise ModelError(f"{where}: the direction has length zero")
        return d / n

    def _frame(self, name, d):
        where = f"frames.{name}"
        if not isinstance(d, dict):
            raise ModelError(f"{where}: give origin and orientation or z (and x)")
        o0, A0 = self.in_frame(d["in"], name, where + ".in") if "in" in d else (np.zeros(3), np.eye(3))
        o = d.get("origin", [0.0, 0.0, 0.0])
        origin = self.place(o, where + ".origin", name) if isinstance(o, str) else o0 + A0 @ _vec3(o, where + ".origin")
        if "orientation" in d:
            if "z" in d or "x" in d:
                raise ModelError(f"{where}: orientation or z / x, not both")
            q = np.asarray(d["orientation"], dtype=float)
            return origin, A0 @ rotation_matrix(q / np.linalg.norm(q))
        if "z" not in d:
            return origin, A0
        z = self.direction(d["z"], A0, name, where + ".z")
        if "x" in d:
            x = self.direction(d["x"], A0, name, where + ".x")
        else:
            x = A0[:, 0] if abs(A0[:, 0] @ z) < 0.9 else A0[:, 1]
        x = x - (x @ z) * z
        if not np.linalg.norm(x) > 1e-9:
            raise ModelError(f"{where}: x is parallel to z")
        x /= np.linalg.norm(x)
        return origin, np.column_stack([x, np.cross(z, x), z])


def resolve(model):
    """(model with point and frame names replaced by numbers, {point or frame: {"users": the
    elements, points and frames using it, "body": the body that carries it (that of its first
    use in an element; None: none)}}). The model must have its formulas resolved; it is changed
    in place."""
    if not isinstance(model, dict) or not (model.get("points") or model.get("frames")):
        return model, {}
    raw_points, raw_frames = model.get("points") or {}, model.get("frames") or {}
    if not isinstance(raw_points, dict) or not isinstance(raw_frames, dict):
        raise ModelError("points and frames are maps of names")
    clash = set(raw_points) & set(raw_frames)
    if clash:
        raise ModelError(f"names used for a point and a frame: {sorted(clash)}")
    bodies = model.get("bodies") if isinstance(model.get("bodies"), dict) else {}
    r = _Resolver(raw_points, raw_frames, {k: v for k, v in bodies.items() if isinstance(v, dict)})
    points = {name: r.point(name) for name in raw_points}
    frames = {name: r.frame(name) for name in raw_frames}
    carrier = dict(r.carrier)
    for section in POINT_KEYS:  # bodies first: a body's position is carried by the body
        entries = model.get(section)
        if not isinstance(entries, dict):
            continue
        for element, entry in entries.items():
            if not isinstance(entry, dict):
                continue
            for key in POINT_KEYS.get(section, ()):
                ref = entry.get(key)
                if not isinstance(ref, str) or ref.lstrip().startswith("="):
                    continue
                entry[key] = r.place(ref, f"{section}.{element}.{key}", element).tolist()
                carrier.setdefault(ref, element if section == "bodies" else
                                   entry.get("body2", "ground") if key == "point2" else entry.get("body1"))
            for key in ORIENTATION_KEYS.get(section, ()):
                ref = entry.get(key)
                if not isinstance(ref, str):
                    continue
                if ref not in frames:
                    raise ModelError(f"{section}.{element}.{key}: unknown frame {ref!r}")
                entry[key] = euler_parameters(r.frame(ref, element)[1]).tolist()
    # the definitions themselves, as numbers (the viewer draws them)
    model["points"] = {name: v.tolist() for name, v in points.items()}
    model["frames"] = {name: {"origin": o.tolist(), "orientation": euler_parameters(A).tolist()}
                       for name, (o, A) in frames.items()}
    return model, {name: {"users": sorted(set(els)), "body": carrier.get(name)} for name, els in r.used.items()}
