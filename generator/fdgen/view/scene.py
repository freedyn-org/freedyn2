"""The scene of the viewer: a validated model (and optionally its results) as JSON-ready data.

  scene(model, results=None, road_tool=None) -> dict

Everything that moves is given in the frame of its body (the viewer places it with the body's
position R and Euler parameters theta per output time): shapes, joint frames, force points.
The road is a grid of points and friction values (base64 float32, row-major): the analytic road around the region the
tires drive on, an OpenCRG road along its reference line (sampled by freedyn2_road).
"""

import base64
import copy
import csv
import subprocess
from pathlib import Path

import numpy as np
import yaml

from .. import formulas
from ..model import LOADER
from ..geometry import local_point, rotation_matrix
from ..shapes import default_shapes, euler_parameters

DUMPER = getattr(yaml, "CSafeDumper", yaml.SafeDumper)
PALETTE = ["#4e79a7", "#f28e2b", "#59a14f", "#e15759", "#76b7b2", "#edc948", "#b07aa1", "#9c755f"]
MAX_ROAD_POINTS = 120_000  # of the road grid
ROAD_MARGIN = 5.0  # [m] around the tire contact points


def read_results(path):
    """Columns of a result CSV: {name: numpy array}."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        return {}
    data = np.array(rows[1:], dtype=float).reshape(-1, len(rows[0]))
    return {name: data[:, i] for i, name in enumerate(rows[0])}


def _frame_on(model, body, location, A_global):
    """Position and Euler parameters on body (or the ground) of a frame given globally at t = 0."""
    p = local_point(model, body, location)
    if body == "ground":
        return p.tolist(), euler_parameters(A_global).tolist()
    A = rotation_matrix(model["bodies"][body]["orientation"])
    return p.tolist(), euler_parameters(A.T @ A_global).tolist()


def _bodies(model):
    defaults = default_shapes(model)
    out = []
    for k, (name, b) in enumerate(model["bodies"].items()):
        color = PALETTE[k % len(PALETTE)]
        shapes = [dict(s, color=s.get("color", color)) for s in (b.get("shapes") or defaults[name])]
        out.append({"name": name, "mass": b["mass"], "shapes": shapes, "default_shapes": not b.get("shapes"),
                    "position": list(b["position"]), "orientation": list(b["orientation"])})
    return out


def _joints(model):
    out = []
    for j in model.get("joints", []):
        F = np.asarray(j["frame"], dtype=float)
        p1, t1 = _frame_on(model, j["body1"], j["location"], F)
        p2, t2 = _frame_on(model, j["body2"], j["location"], F)
        out.append({"name": j["name"], "type": j.get("type", "generic"), "fix": j["fix"],
                    "body1": j["body1"], "position1": p1, "orientation1": t1,
                    "body2": j["body2"], "position2": p2, "orientation2": t2})
    return out


def _forces(model):
    """Lines (springs, dampers, bushings) between points of two bodies, points of applied loads,
    tires (their contact point is a result)."""
    out = []
    for f in model.get("forces", []):
        kind, b1, b2 = f["type"], f["body1"], f.get("body2", "ground")
        entry = {"name": f["name"], "type": kind, "body1": b1, "body2": b2}
        if kind == "spring_damper":
            entry["point1"] = local_point(model, b1, f.get("point1")).tolist()
            entry["point2"] = local_point(model, b2, f.get("point2")).tolist()
        elif kind == "bushing":  # its force and torque outputs are in its frame on body2 (orientation2)
            loc = f.get("location", model["bodies"][b1]["position"])
            o = np.asarray(f.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=float)
            entry["point1"] = local_point(model, b1, loc).tolist()
            entry["point2"], entry["orientation2"] = _frame_on(model, b2, loc, rotation_matrix(o / np.linalg.norm(o)))
        elif kind in ("force", "torque", "expression_force", "expression_torque") or "point" in f:
            entry["point1"] = local_point(model, b1, f.get("point")).tolist()  # also types of extensions
        elif kind != "tire":
            entry["point1"] = [0.0, 0.0, 0.0]
        out.append(entry)
    for sn in model.get("sensors", []):  # drawn and inspected like the forces: a point on body1
        out.append({"name": sn["name"], "type": "sensor", "body1": sn["body1"], "body2": sn["body1"],
                    "point1": local_point(model, sn["body1"], sn.get("point")).tolist()})
    return out


def _points(model):
    """Named points and frames (fdgen.hardpoints) at their place on the body that carries them."""
    out = []
    for kind, section in (("point", "points"), ("frame", "frames")):
        for name, v in (model.get(section) or {}).items():
            ref = (model.get("point_refs") or {}).get(name, {})
            body = ref.get("body") or "ground"
            r = v if kind == "point" else v["origin"]
            out.append({"name": name, "kind": kind, "body": body, "position": local_point(model, body, r).tolist(),
                        "global": list(r), "users": ref.get("users", [])})
    return out


def _float32(a):
    """Base64 of the float32 values of a (the viewer decodes it to a Float32Array)."""
    return base64.b64encode(np.ascontiguousarray(a, dtype="<f4").tobytes()).decode("ascii")


def _grid(n_points, extent_u, extent_v, cell):
    """Numbers of grid lines (>= 2) along u and v: cell size, at most n_points points."""
    nu, nv = max(2, int(extent_u / cell) + 1), max(2, int(extent_v / cell) + 1)
    if nu * nv > n_points:
        f = np.sqrt(nu * nv / n_points)
        nu, nv = max(2, int(nu / f)), max(2, int(nv / f))
    return nu, nv


def _analytic_road(road, region):
    """Height and friction (fdgen.tire road_height, friction) on a grid over region."""
    (x0, x1), (y0, y1) = region
    waves = road.get("harmonics", [])
    cell = min([0.25] + [w["wavelength"] / 8 for w in waves])
    nx, ny = _grid(MAX_ROAD_POINTS, x1 - x0, y1 - y0, cell)
    X, Y = np.meshgrid(np.linspace(x0, x1, nx), np.linspace(y0, y1, ny), indexing="ij")
    gx, gy = road.get("slope", [0.0, 0.0])
    Z = gx * X + gy * Y
    for w in waves:
        psi = w.get("heading", 0.0)
        Z += w["amplitude"] * np.sin(2 * np.pi * (X * np.cos(psi) + Y * np.sin(psi)) / w["wavelength"]
                                     + w.get("phase", 0.0))
    mu = np.full_like(X, road.get("friction", 1.0))
    patch = road.get("patch")
    if patch:
        e = patch.get("edge", 0.5)
        box = lambda s, a, b: 0.25 * (1 + np.tanh((s - a) / e)) * (1 - np.tanh((s - b) / e))
        mu += (patch.get("friction", 1.0) - mu) * box(X, *patch["x"]) * box(Y, *patch["y"])
    return {"rows": nx, "cols": ny, "points": _float32(np.stack([X, Y, Z], -1)), "friction": _float32(mu)}


def _crg_road(road, tool):
    """OpenCRG road on a (u, v) grid over the file, placed as the runtime places it (freedyn2_road --grid)."""
    args = [str(tool), road["crg"], "--grid", "0.1", str(MAX_ROAD_POINTS), "--place",
            *(repr(float(v)) for v in road.get("position", [0.0, 0.0, 0.0])), repr(float(road.get("heading", 0.0)))]
    if "crg_friction" in road:
        args += ["--friction-crg", road["crg_friction"]]
    out = subprocess.run(args, capture_output=True, text=True, check=True, stdin=subprocess.DEVNULL).stdout.split("\n", 1)
    rows, cols = (int(w) for w in out[0].split())
    data = np.array(out[1].split(), dtype=float).reshape(-1, 4)
    mu = data[:, 3] if "crg_friction" in road else np.full(len(data), road.get("friction", 1.0))
    return {"rows": rows, "cols": cols, "points": _float32(data[:, :3]), "friction": _float32(mu)}


def _tire_region(model, results):
    """x and y range the tires drive on (their contact points, else the wheel centers), with a margin."""
    tires = [f for f in model.get("forces", []) if f["type"] == "tire"]
    xs, ys = [], []
    for f in tires:
        for prefix in (f["name"] + ".contact", f["body1"] + ".R"):
            if results and f"{prefix}_x" in results:
                xs += [results[f"{prefix}_x"].min(), results[f"{prefix}_x"].max()]
                ys += [results[f"{prefix}_y"].min(), results[f"{prefix}_y"].max()]
                break
        else:
            x, y = model["bodies"][f["body1"]]["position"][:2]
            xs += [x, x]
            ys += [y, y]
    m = ROAD_MARGIN
    return (min(xs) - m, max(xs) + m), (min(ys) - m, max(ys) + m)


def road_grid(model, results=None, road_tool=None):
    """The road under the tires (None: no tires). road_tool: freedyn2_road, for OpenCRG roads."""
    if not any(f["type"] == "tire" for f in model.get("forces", [])):
        return None
    road = model.get("road") or {}
    if "crg" in road:
        if road_tool is None:
            return None
        return dict(_crg_road(road, road_tool), source="crg")
    return dict(_analytic_road(road, _tire_region(model, results)), source="analytic")


def tunable(entry, path):
    """Paths (section.name.key[.index]) and values of the numbers in a YAML entry: what the
    viewer offers to tune (fdgen.view.server.apply_overrides); a formula (=..., fdgen.formulas)
    gives its text and the parameters it uses (they are tuned instead)."""
    out = []
    if isinstance(entry, bool):
        return out
    if isinstance(entry, (int, float)):
        return [{"path": path, "value": float(entry)}]
    if formulas.is_formula(entry):
        try:
            uses = sorted(formulas.names(entry))
        except ValueError:
            uses = []
        return [{"path": path, "formula": entry.strip(), "uses": uses}]
    if isinstance(entry, dict):
        for k, v in entry.items():
            if k not in ("shapes", "checks", "orientation"):  # quaternion components: no
                out += tunable(v, f"{path}.{k}")
    elif isinstance(entry, list) and len(entry) <= 4 and all(isinstance(v, (int, float)) or formulas.is_formula(v)
                                                             for v in entry):
        for i, v in enumerate(entry):
            out += tunable(v, f"{path}.{i}")
    return out


_SOURCES = {}  # (model file, modification time) -> what attach_sources adds (the file changes rarely)


def _sources(path):
    path = Path(path).resolve()
    key = (path, path.stat().st_mtime_ns)
    if key not in _SOURCES:
        text = path.read_text(encoding="utf-8")
        raw = yaml.load(text, Loader=LOADER) or {}
        root = yaml.compose(text, Loader=LOADER)
        lines = {}
        for k, section in getattr(root, "value", []):
            for name, _ in getattr(section, "value", []) if isinstance(section, yaml.MappingNode) else []:
                lines[(k.value, name.value)] = name.start_mark.line + 1
        items = {}
        for section in ("bodies", "joints", "forces", "sensors", "points", "frames"):
            for name, entry in (raw.get(section) or {}).items():
                if entry is not None:
                    items[(section, name)] = {
                        "source": {"line": lines.get((section, name)),
                                   "yaml": yaml.dump({name: entry}, Dumper=DUMPER, sort_keys=False, width=100)},
                        "tunable": tunable(entry, f"{section}.{name}")}
        _SOURCES.clear()  # one version of one file is enough
        _SOURCES[key] = (items, tunable(raw.get("parameters") or {}, "parameters"))
    return _SOURCES[key]


def attach_sources(s, path):
    """Each body, joint and force of the scene gets its YAML text, its line in the model file and
    its numbers (tunable); the scene gets the model parameters."""
    items, parameters = _sources(path)
    for section in ("bodies", "joints", "forces", "points"):
        for item in s.get(section, []):
            key = "sensors" if item.get("type") == "sensor" else item["kind"] + "s" if section == "points" else section
            extra = items.get((key, item["name"]))
            if extra:
                item.update(copy.deepcopy(extra))
    s["parameters"] = copy.deepcopy(parameters)


def scene(model, results=None, road_tool=None):
    """Viewer data of a validated model; results: columns of its result CSV (read_results)."""
    return {
        "model": model["model"],
        "gravity": list(model.get("gravity", [0.0, 0.0, 0.0])),
        "bodies": _bodies(model),
        "ground_shapes": [dict(s, color=s.get("color", "#9a9a9a")) for s in model.get("ground_shapes", [])],
        "joints": _joints(model),
        "forces": _forces(model),
        "points": _points(model),
        "road": road_grid(model, results, road_tool),
    }
