"""freedyn2 convert: a legacy FreeDyn model (.fds) as a FreeDyn 2 model file (YAML).

The importer (fdgen.fds.load_fds) reads the .fds into the internal model of a profile; this
module writes that model as YAML, in the format of the hand-written examples: the elements in
the order of the .fds, comments naming the .fds blocks they come from, numbers written so that
they read back exactly (repr), data files (spline tables) referenced relative to the YAML file.

Nothing is interpreted here: what the YAML says is what the importer made of the .fds. The
written file is read back with the YAML loader (schema and model rules) and compared with the
imported model, key by key; any difference, and any part of the imported model this module does
not know how to write, is an error naming the element, so nothing is dropped silently. The
profile's solver settings (step size control, HHT form, Newton rule) and, for --profile legacy,
the Euler-parameter bodies are written too, so the YAML runs as `freedyn2 run model.fds
--profile <profile>` does.

Element names must be identifiers in YAML: bodies, joints, forces and drivers whose .fds names
are not (constraints are named by their ConstraintID, e.g. 1) are renamed (joint_1), which
renames their result channels; the comment at the element gives the .fds name. renames(path)
gives the mapping.
"""

import math
import os
import re
from pathlib import Path

import numpy as np
import yaml

from .errors import ModelError
from .fds import euler_parameters, load_fds, parse_blocks
from .fds.blocks import ident, numbers
from .fds.loads import measure_name, parameter

IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
LINE = 100  # a mapping that fits in this many characters is written on one line

# keys of the imported elements, in the order they are written (anything else is an error)
BODY_KEYS = ("mass", "inertia", "position", "orientation", "velocity", "angular_velocity", "element", "jacobian",
             "newmark")
BODY_DEFAULTS = {"position": [0.0] * 3, "orientation": [1.0, 0.0, 0.0, 0.0], "velocity": [0.0] * 3,
                 "angular_velocity": [0.0] * 3, "element": "rigid_body", "jacobian": "full", "newmark": "standard"}
JOINT_KEYS = ("name", "body1", "body2", "location", "frame", "fix")
FORCE_KEYS = ("type", "body1", "body2", "point1", "point2", "direction", "axis", "frame", "force", "torque")
MEASURE_KEYS = ("type", "body1", "body2", "point1", "point2", "component", "frame", "vector", "plane", "normalize",
                "form", "offset")
MEASURE_DEFAULTS = {"normalize": True}
DRIVER_KEYS = ("measure", "motion", "ramp")
SOLVER_KEYS = ("alpha", "form", "end_time", "step", "output_step", "output_begin", "newton", "newton_tol",
               "max_newton", "adaptive", "eps", "h_min", "error_norm", "exact_output", "vproj_tol")
# the runtime's values of solver keys a model file leaves out (fd::SolverSettings); they are left
# out of the YAML file too, but for alpha, adaptive and eps
SOLVER_DEFAULTS = {"form": "arnold_bruls", "output_begin": 0.0, "newton": "velocity", "newton_tol": 1e-10,
                   "max_newton": 25, "h_min": 1e-14, "error_norm": "mixed", "exact_output": True, "vproj_tol": 1e-12}
MODEL_KEYS = ("model", "bodies", "joints", "forces", "parameters", "measures", "splines", "drivers", "gravity",
              "solver")

PROFILE_NOTE = {
    "v2": "FreeDyn 2 methods (Lie group bodies, Arnold-Bruls HHT, velocity Newton criterion)\n"
          "with the step size control of legacy FreeDyn (eps: GlobalErrorTolerance, step: SolverMaxTimeStep)",
    "legacy": "the legacy algorithm (Euler-parameter bodies, Negrut HHT, legacy Newton rule, step size\n"
              "control and error norm): the same steps as legacy FreeDyn",
}


# --- scalars ------------------------------------------------------------------------------------

def number(v, in_list=False):
    """A number as YAML that reads back to the same float: repr, with a '.0' where YAML 1.1 needs
    one (1e-06 -> 1.0e-06); whole numbers in lists without it ([0, 0, -9.81])."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    v = float(v)
    if not math.isfinite(v):
        raise ModelError(f"the number {v} cannot be written to the model file")
    if v == int(v) and abs(v) < 1e15 and not (v == 0 and math.copysign(1.0, v) < 0):  # -0.0 stays
        return str(int(v)) if in_list else f"{int(v)}.0"
    r = repr(v)
    mantissa, e, exponent = r.partition("e")
    if e and "." not in mantissa:
        r = f"{mantissa}.0e{exponent}"
    return r


def string(s, flow=False):
    """A string as a plain YAML scalar if it reads back as itself, else single-quoted."""
    plain = bool(s) and "#" not in s and not (flow and any(c in s for c in ",[]{}"))
    if plain:
        try:
            plain = yaml.safe_load(f"k: {s}") == {"k": s}
        except yaml.YAMLError:
            plain = False
    return s if plain else "'" + s.replace("'", "''") + "'"


def value(v, flow=True):
    """A value in flow style: number, string, list, mapping."""
    if isinstance(v, dict):
        return "{" + ", ".join(f"{k}: {value(x)}" for k, x in v.items()) + "}"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(number(x, True) if isinstance(x, (int, float, np.number)) and not isinstance(x, bool)
                               else value(x) for x in v) + "]"
    if isinstance(v, str):
        return string(v, flow)
    return number(v)


def _comment(text):
    return f"  # {text}" if text else ""


class _Writer:
    def __init__(self):
        self.lines = []

    def comment(self, text, indent=""):
        for line in text.splitlines() or [""]:
            self.lines.append(f"{indent}# {line}".rstrip())

    def key(self, key, v, comment="", indent=""):
        self.lines.append(f"{indent}{key}: {value(v, flow=not isinstance(v, str))}{_comment(comment)}")

    def element(self, name, entries, comment="", indent="  "):
        """name: {...} on one line if short and without key comments, else a block."""
        flow = f"{indent}{name}: {value(dict((k, v) for k, v, _ in entries))}"
        if not any(c for _, _, c in entries) and len(flow) + len(_comment(comment)) <= LINE:
            self.lines.append(flow + _comment(comment))
            return
        self.lines.append(f"{indent}{name}:{_comment(comment)}")
        for k, v, c in entries:
            self.key(k, v, c, indent + "  ")

    def text(self):
        return "\n".join(self.lines) + "\n"


# --- the .fds blocks behind the imported elements ------------------------------------------------

def _blocks(path):
    """Header comment and the blocks of the .fds by kind, in file order (as fdgen.fds.load_fds)."""
    text = Path(path).read_text(encoding="latin-1")
    header, inside = [], False
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("Header"):
            inside = True
        elif line == "EndHeader":
            break
        elif inside and line.startswith("%"):
            header.append(line.lstrip("%").strip())
    kinds = {"bodies": [], "joints": [], "motions": [], "forces": [], "measures": [], "splines": [],
             "parameters": [], "gravity": []}
    for b in parse_blocks(text):
        key = {("Body", "Rigid_EulerParameter_PAI"): "bodies", ("Constraint", "Generic"): "joints",
               ("Constraint", "TimeMotion"): "motions", ("Force", "GenericForce"): "forces",
               ("Force", "GenericTorque"): "forces", ("Measure", "Translational"): "measures",
               ("Measure", "Rotational"): "measures", ("DataObject", "Spline_1D"): "splines",
               ("DataObject", "Parameter"): "parameters", ("Force", "Gravity"): "gravity"}.get((b[0], b[1]))
        if key:
            kinds[key].append(b)
    return [h for h in header if h], kinds


def _spline_sources(blocks, model_dir):
    """{label as in the spline names: {label, file, header_lines, columns}} of the Spline_1D
    blocks; file (absolute, as the importer reads it) only for data in a file."""
    out = {}
    for b in blocks:
        label = b[2].get("Label") or b[2].get("Name") or ""
        src = {"label": label}
        if "NumDataPts" not in b[2] and "PathToSplineData" in b[2]:
            f = Path(b[2]["PathToSplineData"].replace("\\", "/"))
            src.update(file=f if f.is_absolute() else Path(model_dir) / f,
                       header_lines=int(numbers(b, "NumberOfHeaderLines", 1)[0]),
                       columns=[int(numbers(b, "ColumnXData", 1)[0]), int(numbers(b, "ColumnYData", 1)[0])])
        out[ident("", label.lower())] = src
    return out


def _ids(b, *keys):
    return ", ".join(f"{k} {b[2][k]}" for k in keys if b[2].get(k))


# --- names ---------------------------------------------------------------------------------------

def _rename(names, prefix):
    """{old: new} for names that are not identifiers (or are 'ground'): characters other than
    letters, digits and _ replaced, a prefix where the name starts with a digit, unique."""
    out, taken = {}, set(names)
    for n in names:
        if IDENT.match(n) and n != "ground" and n not in out.values():
            out[n] = n
            continue
        base = re.sub(r"[^A-Za-z0-9_]", "_", n)
        base = base if IDENT.match(base) and base != "ground" else f"{prefix}_{base}"
        new, k = base, 2
        while new in taken or new in out.values():
            new, k = f"{base}_{k}", k + 1
        out[n] = new
        taken.add(new)
    return out


def _require_idents(kind, names):
    bad = [n for n in names if not IDENT.match(n)]
    if bad:
        raise ModelError(f"{kind} {', '.join(map(repr, bad))}: not an identifier; used in expressions, so it "
                         "cannot be renamed by the converter (rename it in the .fds)")


def _unknown(owner, d, known):
    extra = [k for k in d if k not in known]
    if extra:
        raise ModelError(f"{owner}: {', '.join(extra)} cannot be written to a YAML model file")


def _axis_text(v):
    for i, c in enumerate("xyz"):
        for sign in (1, -1):
            e = [0.0] * 3
            e[i] = float(sign)
            if np.allclose(v, e, rtol=0, atol=1e-12):
                return ("-" if sign < 0 else "") + c
    return "[" + ", ".join(f"{x:.4g}" for x in v) + "]"


# --- the model -----------------------------------------------------------------------------------

def _model_text(m, fds_path, yaml_path, profile, header, blocks):
    from . import __version__
    from .joints import PRESETS

    _unknown("model", m, MODEL_KEYS)
    _require_idents("measures", m["measures"])
    _require_idents("parameters", m["parameters"])
    _require_idents("splines", m["splines"])
    body_names = _rename(list(m["bodies"]), "body")
    joint_names = _rename([j["name"] for j in m["joints"]], "joint")
    force_names = _rename([f["name"] for f in m["forces"]], "force")
    driver_names = _rename(list(m["drivers"]), "driver")
    model_name = _rename([m["model"]], "model")[m["model"]]
    body = lambda n: "ground" if n == "ground" else body_names[n]

    w = _Writer()
    w.comment(f"{model_name}: converted from {Path(fds_path).name} by freedyn2 convert (FreeDyn 2 "
              f"{__version__}), profile {profile}.")
    if header:
        w.comment("From the header of the .fds:")
        for h in header:
            w.comment("  " + h)
    w.comment("SI units (m, kg, s, N, rad); points and directions global at t = 0; elements in the order of the "
              ".fds,\nthe comments name the .fds blocks.")
    w.key("model", model_name)
    w.key("gravity", m["gravity"], "Force Gravity" if blocks["gravity"] else "no Force Gravity block")

    if m["parameters"]:
        labels = {parameter(b)[0]: b[2].get("Label") or b[2].get("Name") for b in blocks["parameters"]}
        w.lines.append("parameters:")
        for name, v in m["parameters"].items():
            w.key(name, v, f"DataObject Parameter {labels.get(name, '')}".strip(), "  ")

    if m["splines"]:
        sources = _spline_sources(blocks["splines"], Path(fds_path).parent)
        w.lines.append("splines:")
        for name, sp in m["splines"].items():
            _unknown(f"spline {name}", sp, ("x", "y", "type"))
            label = name[2:].rsplit("_", 1)[0]
            src = sources.get(label)
            how = {"akima": "Akima", "zcc": "zero clamped cubic"}[sp["type"]]
            if src and src.get("file"):
                entries = [("file", _relative(src["file"], yaml_path), ""), ("header_lines", src["header_lines"], ""),
                           ("columns", src["columns"], ""), ("type", sp["type"], "")]
            else:
                entries = [("type", sp["type"], ""), ("x", sp["x"], ""), ("y", sp["y"], "")]
            w.element(name, entries, f"DataObject Spline_1D {src['label'] if src else label}, {how}")

    w.lines.append("bodies:")
    for (name, b), blk in zip(m["bodies"].items(), blocks["bodies"]):
        _unknown(f"body {name}", b, BODY_KEYS)
        entries = [(k, b[k], "") for k in BODY_KEYS if k in b and not (k in BODY_DEFAULTS and b[k] == BODY_DEFAULTS[k])]
        note = _ids(blk, "BodyID", "ID")
        if body_names[name] != name:
            note = f"BodyName {name!r}, " + note
        w.element(body_names[name], entries, note.rstrip(", "))

    if m["joints"]:
        w.lines.append("joints:")
    for j, blk in zip(m["joints"], blocks["joints"]):
        _unknown(f"joint {j['name']}", j, JOINT_KEYS)
        preset = next((t for t, fix in PRESETS.items() if fix == j["fix"]), None)
        K = np.asarray(j["frame"], dtype=float)
        q = euler_parameters(j["frame"])
        entries = [("type", preset or "generic", ""), ("body1", body(j["body1"]), "")]
        if j["body2"] != "ground":
            entries.append(("body2", body(j["body2"]), ""))
        entries.append(("location", j["location"], ""))
        if q != [1.0, 0.0, 0.0, 0.0]:
            entries.append(("orientation", q, f"joint z = global {_axis_text(K[:, 2])}"))
        if not preset:
            entries.append(("fix", j["fix"], ""))
        fixed = " ".join(j["fix"]) or "nothing"
        w.element(joint_names[j["name"]], entries,
                  f"Constraint Generic, {_ids(blk, 'Name', 'ConstraintID', 'ID')}: {fixed} fixed")

    if m["measures"]:
        w.lines.append("measures:")
        by_name = {measure_name(blk): blk for blk in blocks["measures"]}
        for name, ms in m["measures"].items():
            _unknown(f"measure {name}", ms, MEASURE_KEYS)
            entries = []
            for k in MEASURE_KEYS:
                if k not in ms or (k == "body2" and ms[k] == "ground") or MEASURE_DEFAULTS.get(k, ()) == ms[k]:
                    continue
                v = ms[k]
                if k in ("body1", "body2") or (k == "frame" and v != "global"):
                    v = body(v)
                entries.append((k, v, ""))
            blk = by_name[name]
            label = blk[2].get("MeasureName") or blk[2].get("Name") or blk[2].get("Label") or ""
            w.element(name, entries, f"Measure {blk[1]} {label}, {_ids(blk, 'MeasureMode', 'MeasureCoordinate')}"
                      .replace(" ,", ",").rstrip(", "))

    if m["forces"]:
        w.lines.append("forces:")
    for f, blk in zip(m["forces"], blocks["forces"]):
        _unknown(f"force {f['name']}", {k: v for k, v in f.items() if k != "name"}, FORCE_KEYS)
        entries = []
        for k in FORCE_KEYS:
            if k not in f or (k == "body2" and f[k] == "ground"):
                continue
            v = body(f[k]) if k in ("body1", "body2") else f[k]
            note = ""
            if k == "force" and f.get("direction") == "line_of_sight" and f["body2"] == "ground":
                note = "legacy In-line-of-sight body-ground points from the ground: law negated"
            entries.append((k, v, note))
        label = blk[2].get("Label") or blk[2].get("ForceName") or blk[2].get("Name") or ""
        w.element(force_names[f["name"]], entries,
                  f"Force {blk[1]} {label}, {_ids(blk, 'ForceID', 'ForceDirectionType', 'AxisType')}".replace("  ", " "))

    if m["drivers"]:
        w.lines.append("drivers:")
        motions = {ident("", b[2].get("Name") or b[2].get("ConstraintName") or b[2].get("Label")
                         or ("motion" + b[2].get("ConstraintID", ""))): b for b in blocks["motions"]}
        for name, d in m["drivers"].items():
            blk = motions[name]
            _unknown(f"driver {name}", d, DRIVER_KEYS)
            entries = [(k, d[k], "") for k in DRIVER_KEYS if k in d]
            w.element(driver_names[name], entries,
                      f"Constraint TimeMotion, {_ids(blk, 'Name', 'ConstraintID')}; ramp = ActivationPeriod")

    s = m["solver"]
    _unknown("solver", s, SOLVER_KEYS)
    w.lines.append(f"solver:  # Simulation Dynamic, Solver HHT; profile {profile}:")
    w.comment(PROFILE_NOTE[profile], "  ")
    w.key("type", "HHT", "", "  ")
    for k in SOLVER_KEYS:
        if k in s and not (k in SOLVER_DEFAULTS and _same(s[k], SOLVER_DEFAULTS[k])):
            w.key(k, s[k], "", "  ")
    renames = {"bodies": body_names, "joints": joint_names, "forces": force_names, "drivers": driver_names,
               "model": {m["model"]: model_name}}
    return w.text(), {kind: {a: b for a, b in r.items() if a != b} for kind, r in renames.items()}


def _relative(path, yaml_path):
    """path (absolute) relative to the directory of the YAML file, with / (absolute if on another drive)."""
    try:
        return Path(os.path.relpath(Path(path).resolve(), Path(yaml_path).resolve().parent)).as_posix()
    except ValueError:  # another drive (Windows)
        return Path(path).resolve().as_posix()


# --- the check: the YAML reads back as the imported model ----------------------------------------

def _same(a, b):
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b or a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return float(a) == float(b) and math.copysign(1, a) == math.copysign(1, b)
    return a == b


def _compare(fds, y, renames):
    """Differences between the imported model and the model the YAML file reads back as."""
    body = lambda n: renames["bodies"].get(n, n)
    diffs = []

    def need(owner, ok):
        if not ok:
            diffs.append(owner)

    need("gravity", _same(fds["gravity"], y["gravity"]))
    need("parameters", _same(fds["parameters"], y["parameters"]))
    need("splines", fds["splines"].keys() == y["splines"].keys() and all(
        _same(sp, {k: y["splines"][n][k] for k in ("x", "y", "type")}) for n, sp in fds["splines"].items()))
    need("bodies", [body(n) for n in fds["bodies"]] == list(y["bodies"]))
    for name, b in fds["bodies"].items():
        yb = y["bodies"].get(body(name), {})
        for k, v in b.items():
            ok = (np.allclose(v, yb.get(k), rtol=0, atol=4e-16) if k == "orientation"
                  else _same(v, yb.get(k, BODY_DEFAULTS.get(k))))
            need(f"body {name}: {k}", ok)
    need("joints", len(fds["joints"]) == len(y["joints"]))
    for j, yj in zip(fds["joints"], y["joints"]):
        need(f"joint {j['name']}", renames["joints"].get(j["name"], j["name"]) == yj["name"]
             and body(j["body1"]) == yj["body1"] and body(j["body2"]) == yj["body2"]
             and _same(j["location"], yj["location"]) and j["fix"] == yj["fix"]
             and np.allclose(j["frame"], yj["frame"], rtol=0, atol=1e-15))
    need("forces", len(fds["forces"]) == len(y["forces"]))
    for f, yf in zip(fds["forces"], y["forces"]):
        want = dict(f, name=renames["forces"].get(f["name"], f["name"]), body1=body(f["body1"]), body2=body(f["body2"]))
        need(f"force {f['name']}", _same(want, yf))
    need("measures", list(fds["measures"]) == list(y["measures"]))
    for name, ms in fds["measures"].items():
        want = {k: (body(v) if k in ("body1", "body2") or (k == "frame" and v != "global") else v) for k, v in ms.items()}
        need(f"measure {name}", _same(want, {**{k: v for k, v in MEASURE_DEFAULTS.items() if k in want},
                                             **y["measures"].get(name, {})}))
    need("drivers", [renames["drivers"].get(n, n) for n in fds["drivers"]] == list(y["drivers"]))
    for name, d in fds["drivers"].items():
        need(f"driver {name}", _same(d, y["drivers"].get(renames["drivers"].get(name, name))))
    # as the runtime reads them: a key left out is its default
    effective = lambda s: {k: s.get(k, SOLVER_DEFAULTS.get(k)) for k in (*SOLVER_KEYS, "newton_iterations")}
    need("solver", _same(effective(fds["solver"]), effective(y["solver"])))
    for key in ("inputs", "sensors", "outputs"):
        need(key, not y.get(key))
    return diffs


# --- entry points --------------------------------------------------------------------------------

def fds_to_yaml(fds_path, yaml_path, profile="v2"):
    """(YAML text, renames) of a .fds model for a YAML file at yaml_path (spline files are
    referenced relative to it); checked: the text reads back as the imported model."""
    from .model import validate

    fds_path, yaml_path = Path(fds_path), Path(yaml_path)
    m = load_fds(fds_path, profile)
    header, blocks = _blocks(fds_path)
    text, renames = _model_text(m, fds_path, yaml_path, profile, header, blocks)
    try:
        back = validate(yaml.safe_load(text), yaml_path.resolve().parent)
    except ModelError as e:
        raise ModelError(f"{fds_path.name}: the converted model is not valid YAML model: {e}") from None
    diffs = _compare(m, back, renames)
    if diffs:
        raise ModelError(f"{fds_path.name}: the YAML file would not reproduce the .fds model in: {', '.join(diffs)}")
    return text, renames


def convert(fds_path, output=None, profile="v2", force=False):
    """Write the .fds model fds_path as a YAML model file (default: next to it, .yaml); its path.
    profile: the solver settings (and bodies) of `freedyn2 run --profile`; force: overwrite."""
    fds_path = Path(fds_path)
    if fds_path.suffix.lower() != ".fds":
        raise ModelError(f"{fds_path.name}: convert reads legacy FreeDyn models (.fds)")
    out = Path(output) if output else fds_path.with_suffix(".yaml")
    if out.is_dir():
        out = out / fds_path.with_suffix(".yaml").name
    if out.exists() and not force:
        raise ModelError(f"{out} exists (--force / force=True overwrites it)")
    text, _ = fds_to_yaml(fds_path, out, profile)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8", newline="\n")
    return out


def renames(fds_path, profile="v2"):
    """{kind: {.fds name: YAML name}} of the elements convert renames (their result channels
    are renamed with them)."""
    return fds_to_yaml(fds_path, Path(fds_path).with_suffix(".yaml"), profile)[1]
