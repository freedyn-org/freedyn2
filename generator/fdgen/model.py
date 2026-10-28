"""Read and validate a YAML model, write the runtime model file (.fdm)."""

import functools
import json
import math
import warnings
from pathlib import Path

import jsonschema
import numpy as np
import yaml

from . import forces
from . import joints as joint_types
from .geometry import rotation_matrix
from . import extensions, formulas, hardpoints, plugin, resources, rigid_body, shapes, splines
from .measures import POSITION_MEASURES
from .errors import ModelError  # noqa: F401  (re-exported: fdgen.model.ModelError)

SCHEMA_PATH = resources.path("schema/model.schema.json")
FDM_VERSION = 1


def load_schema():
    """The JSON schema of the model file, with what the extensions add (fdgen.extensions)."""
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    for ext in extensions.extensions():
        ext.schema(schema)
    return schema


@functools.lru_cache(maxsize=1)
def _validator():
    """The schema's validator, made once (jsonschema.validate checks the schema itself on every call)."""
    schema = load_schema()
    cls = jsonschema.validators.validator_for(schema)
    cls.check_schema(schema)
    return cls(schema)


LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)  # libyaml if PyYAML has it (much faster)


def load_yaml(path):
    with open(path, encoding="utf-8") as f:
        return yaml.load(f, Loader=LOADER)


def _data_path(path, base_dir):
    path = Path(path)
    return path if path.is_absolute() else Path(base_dir or ".") / path


def _check_bodies(owner, names, bodies):
    for b in names:
        if b != "ground" and b not in bodies:
            raise ModelError(f"{owner}: unknown body {b}")


def _solver(s):
    s = dict(s)
    s.setdefault("alpha", -0.1)
    s.setdefault("form", "arnold_bruls")
    s.setdefault("output_step", s["step"])
    s.setdefault("newton_tol", 1e-10)
    s.setdefault("max_newton", 25)
    if s.get("newton") == "fixed":
        s.setdefault("newton_iterations", 2)
    ratio = s["output_step"] / s["step"]
    if abs(ratio - round(ratio)) > 1e-9 * ratio:
        raise ModelError("solver.output_step must be an integer multiple of solver.step")
    return s


def _body(name, b, base_dir):
    b = dict(b)
    b.setdefault("position", [0.0, 0.0, 0.0])
    b.setdefault("orientation", [1.0, 0.0, 0.0, 0.0])
    b.setdefault("velocity", [0.0, 0.0, 0.0])
    b.setdefault("angular_velocity", [0.0, 0.0, 0.0])
    n = math.sqrt(sum(t * t for t in b["orientation"]))
    if n == 0:
        raise ModelError(f"body {name}: orientation must not be zero")
    if abs(n - 1) > 1e-9:
        warnings.warn(f"body {name}: orientation |theta| = {n}, normalized")
    b["orientation"] = [t / n for t in b["orientation"]]
    if "shapes" in b:
        b["shapes"] = shapes.validate_shapes(f"body {name}", b["shapes"], b, base_dir)
    return b


def _joint(name, j, bodies):
    fix = joint_types.PRESETS.get(j["type"]) if j["type"] != "generic" else j.get("fix")
    if fix is None:
        raise ModelError(f"joint {name}: type generic needs a fix list")
    _check_bodies(f"joint {name}", (j["body1"], j.get("body2", "ground")), bodies)
    if j["body1"] == "ground":
        raise ModelError(f"joint {name}: body1 must be a body (body2 may be ground)")
    t = np.array(j.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=float)
    return {
        "name": name, "type": j["type"], "body1": j["body1"], "body2": j.get("body2", "ground"),
        "location": list(j.get("location", [0.0, 0.0, 0.0])),
        "frame": rotation_matrix(t / np.linalg.norm(t)).tolist(), "fix": list(fix),
    }


_FORCE_KEYS = {"spring_damper": ["stiffness"], "torsion_spring": ["stiffness"], "force": ["vector"],
               "torque": ["vector"], "bushing": ["stiffness", "rot_stiffness"], "expression_force": ["force"],
               "expression_torque": ["torque", "axis"], "tire": []}


def _force(name, f, bodies, road, base_dir):
    f = dict(f, name=name)
    f.setdefault("body2", "ground")
    _check_bodies(f"force {name}", (f["body1"], f["body2"]), bodies)
    if f["body1"] == "ground":
        raise ModelError(f"force {name}: body1 must be a body (body2 may be ground)")
    for key in _FORCE_KEYS.get(f["type"], []):  # types of extensions check their keys themselves
        if key not in f:
            raise ModelError(f"force {name}: {f['type']} needs {key}")
    if f["type"] == "tire":
        if f["body2"] != "ground":
            raise ModelError(f"force {name}: a tire acts between a wheel and the ground")
        f["road"] = dict(road or {})
    if f["type"] in ("force", "torque") and f["body2"] != "ground":
        raise ModelError(f"force {name}: {f['type']} acts on body1 only")
    for ext in extensions.extensions():
        f = ext.validate_force(f, base_dir)
    return f


def _sensor(name, sn, bodies):
    _check_bodies(f"sensor {name}", [sn["body1"]], bodies)
    if sn["body1"] == "ground":
        raise ModelError(f"sensor {name}: body1 must be a body")
    o = np.asarray(sn.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=float)
    if not np.linalg.norm(o) > 0:
        raise ModelError(f"sensor {name}: orientation must not be zero")
    return dict(sn, name=name, orientation=list(o / np.linalg.norm(o)))


def _measure(name, m, bodies):
    m = dict(m)
    m.setdefault("body2", "ground")
    _check_bodies(f"measure {name}", [m[k] for k in ("body1", "body2", "frame") if m.get(k) not in (None, "global")],
                  bodies)
    if m["type"] == "direction" and "vector" not in m:
        raise ModelError(f"measure {name}: direction needs vector")
    if m["type"] in ("displacement", "velocity", "direction", "angular_velocity") and "component" not in m:
        raise ModelError(f"measure {name}: {m['type']} needs component")
    return m


def _spline(name, sp, base_dir):
    sp = dict(sp)
    if "file" in sp:
        if "x" in sp or "y" in sp:
            raise ModelError(f"spline {name}: give either x / y or file")
        cols = sp.get("columns", [0, 1])
        try:
            sp["x"], sp["y"] = splines.read_table(_data_path(sp["file"], base_dir), sp.get("header_lines", 0),
                                                  cols[0], cols[1])
        except (OSError, ValueError) as e:
            raise ModelError(f"spline {name}: {e}") from None
    elif "x" not in sp or "y" not in sp:
        raise ModelError(f"spline {name}: needs x and y or file")
    try:
        splines.coefficients(sp["x"], sp["y"], sp.get("type", "akima"))
    except ValueError as e:
        raise ModelError(f"spline {name}: {e}") from None
    if len(sp["x"]) != len(sp["y"]):
        raise ModelError(f"spline {name}: x and y need the same length")
    return sp


def _road(road, base_dir):
    """Road of the tires: the analytic road (harmonics, slope, patch) or an OpenCRG file."""
    road = dict(road or {})
    if "crg" not in road:
        if "crg_friction" in road or "position" in road or "heading" in road:
            raise ModelError("road: crg_friction, position and heading belong to an OpenCRG road (crg)")
        return road
    mixed = {"harmonics", "slope", "patch"} & set(road)
    if mixed:
        raise ModelError(f"road: {sorted(mixed)} are analytic road keys, not for an OpenCRG road")
    for key in ("crg", "crg_friction"):
        if key in road:
            path = _data_path(road[key], base_dir).resolve()
            if not path.is_file():
                raise ModelError(f"road: {key} file {path} not found")
            road[key] = str(path)
    return road


def _checks(spec, base_dir):
    """References of the results (fdgen.checks); curve files absolute."""
    spec = dict(spec or {})
    curves = []
    for c in spec.get("curves", []):
        path = _data_path(c["file"], base_dir).resolve()
        if not path.is_file():
            raise ModelError(f"checks: curve file {path} not found")
        curves.append(dict(c, file=str(path)))
    if curves:
        spec["curves"] = curves
    return spec


def _driver(name, d, measures):
    if d["measure"] not in measures:
        raise ModelError(f"driver {name}: unknown measure {d['measure']}")
    if measures[d["measure"]]["type"] not in POSITION_MEASURES:
        raise ModelError(f"driver {name}: the measure must be a displacement, distance or angle")
    return dict(d)


def validate(model, base_dir=None):
    """Validate against the JSON schema and the model rules; fill defaults. base_dir: directory
    relative data files (spline tables, .tir files) are read from (default: the current one)."""
    model, parameter_refs = formulas.resolve(model)  # "=k_susp" in number fields
    model, point_refs = hardpoints.resolve(model)  # "location: uca_outer" (points, frames)
    try:
        error = jsonschema.exceptions.best_match(_validator().iter_errors(model))
        if error is not None:
            raise error
    except jsonschema.ValidationError as e:
        hint = ' (empty: a "#" in YAML starts a comment; formulas start with "=")' if e.instance is None else ""
        raise ModelError(f"schema: {e.message} at {list(e.absolute_path)}{hint}") from None
    solver = _solver(model["solver"])
    section = lambda key: (model.get(key) or {}).items()
    bodies = {name: _body(name, b, base_dir) for name, b in model["bodies"].items()}
    ground = shapes.validate_shapes("ground", (model.get("ground") or {}).get("shapes"), None, base_dir)
    joints = [_joint(name, j, bodies) for name, j in section("joints")]
    road = _road(model.get("road"), base_dir)
    forces = [_force(name, f, bodies, road, base_dir) for name, f in section("forces")]
    measures = {name: _measure(name, m, bodies) for name, m in section("measures")}
    sensors = [_sensor(name, sn, bodies) for name, sn in section("sensors")]
    parameters = dict(model.get("parameters") or {})
    inputs = {name: float(v) for name, v in (model.get("inputs") or {}).items()}
    model_splines = {name: _spline(name, sp, base_dir) for name, sp in section("splines")}
    drivers = {name: _driver(name, d, measures) for name, d in section("drivers")}
    names = [set(parameters), set(measures), set(inputs)]
    clash = {n for i, a in enumerate(names) for b in names[i + 1:] for n in a & b}
    clash |= {"t", "pi"} & set().union(*names)
    if clash:
        raise ModelError(f"names used twice or reserved (t, pi): {sorted(clash)}")
    return {"model": model["model"], "bodies": bodies, "joints": joints, "forces": forces,
            "parameters": parameters, "inputs": inputs, "measures": measures, "sensors": sensors, "splines": model_splines, "drivers": drivers,
            "gravity": list(model.get("gravity", [0, 0, 0])), "solver": solver, "road": road,
            "ground_shapes": ground, "checks": _checks(model.get("checks"), base_dir),
            "parameter_refs": sorted(parameter_refs), "points": dict(model.get("points") or {}),
            "frames": dict(model.get("frames") or {}), "point_refs": point_refs}


def _fmt(values):
    return " ".join(repr(float(v)) for v in values)


def _connected(names, layout):
    """coords and dofs lines of an element connecting the bodies names; their element type."""
    types = {layout[b][0] for b in names}
    return types, [
        "coords " + " ".join(str(i) for b in names for i in layout[b][1]),
        "dofs " + " ".join(str(i) for b in names for i in layout[b][2]),
    ]


def write_fdm(model, path):
    """Write the plain-text model file read by the runtime.

    Global layout: stored coordinates q (nq), x = [qdd (nv); lam (nc)]. Each element lists its
    global coordinate, velocity dof and constraint indices and its parameters p; bodies also
    their initial coordinates q and the user velocity input u that the element's generated init
    function maps to qd. Joints are written as their primitive constraint elements, which
    reference the coordinates and dofs of the connected bodies.
    """
    if model["joints"] and model["solver"].get("alpha", 0.0) == 0.0:
        # Trapezoidal rule with index-3 constraints: a (-1)^n mode of lambda grows (docs/findings.md).
        warnings.warn(f"{model['model']}: alpha = 0 is unstable with joints, use alpha < 0 (e.g. -0.01)")
    s = model["solver"]
    keys = {"form": "hht_form"}  # model key -> runtime key, where they differ
    solver = " ".join(
        f"{keys.get(k, k)} {int(s[k]) if isinstance(s[k], bool) else s[k]!r}".replace("'", "")
        for k in ("alpha", "form", "end_time", "step", "output_step", "output_begin", "newton_tol",
                  "max_newton", "newton", "newton_iterations", "adaptive", "h_min", "eps", "exact_output", "vproj_tol",
                  "error_norm")
        if k in s
    )
    lines = [f"freedyn2_model {FDM_VERSION}", f"name {model['model']}", f"solver {solver}"]
    road = model.get("road") or {}
    if "crg" in road:  # OpenCRG road, evaluated by the runtime (fd::Road)
        place = road.get("position", [0.0, 0.0, 0.0])
        lines += [f"road_crg {road['crg']}", f"road_friction {float(road.get('friction', 1.0))!r}",
                  f"road_place {_fmt([*place, road.get('heading', 0.0)])}"]
        if "crg_friction" in road:
            lines.append(f"road_friction_crg {road['crg_friction']}")
    # runtime inputs (fd::System::set_input): name and default; each element that uses one
    # lists its offset in p ("inputs name:offset")
    lines += [f"input {name} {float(v)!r}" for name, v in (model.get("inputs") or {}).items()]
    gravity = model.get("gravity", [0.0, 0.0, 0.0])
    nq = nv = nc = 0  # stored coordinates, velocity dofs, constraints
    layout = {}  # body -> (element type, coords, dofs)
    for name, b in model["bodies"].items():
        element = b.get("element", "rigid_body")
        if element == "rigid_body":  # Lie group: 7 stored coordinates, 6 velocity dofs
            n_coords, n_dofs, n_cons = rigid_body.NQ, rigid_body.NV, 0
        elif element == "rigid_body_euler":  # absolute coordinates + normalization constraint
            n_coords, n_dofs, n_cons = rigid_body.NQ, rigid_body.NQ, 1
        else:
            raise ModelError(f"body {name}: unknown element type {element}")
        layout[name] = (element, list(range(nq, nq + n_coords)), list(range(nv, nv + n_dofs)))
        lines += [
            f"element {element} {name}",
            "coords " + " ".join(map(str, range(nq, nq + n_coords))),
            "dofs " + " ".join(map(str, range(nv, nv + n_dofs))),
            "cons " + " ".join(map(str, range(nc, nc + n_cons))),
            "p " + _fmt([b["mass"], *b["inertia"], *gravity]),
            "q " + _fmt([*b["position"], *b["orientation"]]),
            "u " + _fmt([*b["velocity"], *b["angular_velocity"]]),
            f"jacobian {b.get('jacobian', 'full')}",
            f"newmark {b.get('newmark', 'standard')}",
            "end",
        ]
        nq, nv, nc = nq + n_coords, nv + n_dofs, nc + n_cons

    def connector(element, name, names, p, constrained, owner, extra=()):
        """Lines of an element without coordinates of its own (joint primitive, force, driver)."""
        nonlocal nc
        types, idx = _connected(names, layout)
        if len(types) != 1:
            raise ModelError(f"{owner}: bodies of different element types")
        cons = f"cons {nc}" if constrained else "cons "
        nc += constrained
        return [f"element {element} {name}", *idx, cons, "p " + _fmt(p), *extra, "end"]

    euler = lambda names: "_euler" if {layout[b][0] for b in names} == {"rigid_body_euler"} else ""
    for joint in model.get("joints", []):
        for kind, suffix, names, p in joint_types.primitives(joint, model["bodies"]):
            lines += connector(kind + euler(names), f"{joint['name']}.{suffix}", names, p, True,
                               f"joint {joint['name']}")
    # model plugin: expression forces, drivers and the element types of extensions, shared types
    custom = plugin.plugin_elements(model)
    if custom:
        lib = plugin.generate_plugin(model, Path(path).parent, elements=custom)
        lines.insert(3, f"plugin {lib.name}")
    in_plugin = {pe[0]: (pe[1], pe[2], pe[3]) for pe in custom}
    input_slots = {pe[0]: getattr(pe, "inputs", {}) for pe in custom}

    def input_lines(name):
        slots = input_slots.get(name)
        return ["inputs " + " ".join(f"{k}:{o}" for k, o in slots.items())] if slots else []

    for force in model.get("forces", []):
        if force["name"] in in_plugin:
            kind, names, p = in_plugin[force["name"]]
        else:
            kind, names, p = forces.element(force, model["bodies"])
            kind += euler(names)
        # tires on the OpenCRG road: the runtime sets their road plane at this offset of p
        offset = forces.LOCAL_ROAD_OFFSET.get(kind.removesuffix("_euler"), extensions.road_offset(kind))
        extra = [f"road {offset}"] if offset is not None else []
        extra += input_lines(force["name"])
        lines += connector(kind, force["name"], names, p, False, f"force {force['name']}", extra)
    for sensor in model.get("sensors", []):
        kind, names, p = forces.sensor_element(sensor, model["bodies"], gravity)
        lines += connector(kind + euler(names), sensor["name"], names, p, False, f"sensor {sensor['name']}")
    for name in model.get("drivers") or {}:  # prescribed motion: one constraint each
        tname, names, p = in_plugin[name]
        lines += connector(tname, name, names, p, True, f"driver {name}", input_lines(name))
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def data_files(model_path):
    """Relative paths of the data files a model file reads (crg, crg_friction, tir, file, data)
    that exist next to it."""
    found = []

    def walk(x):
        if isinstance(x, dict):
            for k, v in x.items():
                if k in ("crg", "crg_friction", "tir", "file", "data") and isinstance(v, str):
                    p = Path(v)
                    if not p.is_absolute() and ".." not in p.parts and (model_path.parent / p).is_file():
                        found.append(p)
                else:
                    walk(v)
        elif isinstance(x, list):
            for v in x:
                walk(v)

    try:
        walk(load_yaml(model_path))
    except (OSError, yaml.YAMLError):
        return []
    return sorted(set(found))
