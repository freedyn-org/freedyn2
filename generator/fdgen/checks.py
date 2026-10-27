"""Checks of a model and its results: plausibility, and agreement with references.

  check_model(model) -> report        before simulating
  check_results(model, results) -> report    results: columns of the result CSV
  check_references(model, results) -> list   the model's `checks` section

A model built by someone else (or by an assistant) has to be checkable faster than it can be
built: these checks find what is physically impossible (inertia), likely a slip (units, a joint
far from its bodies, a spring of length zero, a tire in the air at the start), and what the
reader should know (degrees of freedom, bodies not connected to the ground, mass). References
compare result channels with given values and measured curves.

A finding: {"severity": "error" | "warning" | "info", "code", "message", "elements": [names]}.
"""

import csv
import re
from pathlib import Path

import numpy as np

from . import extensions
from .shapes import inertia_box

G = 9.81


def finding(severity, code, message, elements=()):
    return {"severity": severity, "code": code, "message": message, "elements": list(elements)}


def _connections(model):
    """Body pairs coupled by joints, forces and drivers (with the element names)."""
    pairs = [((j["body1"], j["body2"]), j["name"]) for j in model.get("joints", [])]
    pairs += [((f["body1"], f.get("body2", "ground")), f["name"]) for f in model.get("forces", [])]
    measures = model.get("measures", {})
    for name, d in model.get("drivers", {}).items():
        m = measures.get(d["measure"], {})
        pairs.append(((m.get("body1", "ground"), m.get("body2", "ground")), name))
    return pairs


def degrees_of_freedom(model):
    """Grübler count: 6 per body minus the fixed directions of the joints and the drivers. Equal
    to the mobility unless constraints are redundant (then the count is lower)."""
    n = 6 * len(model["bodies"])
    n -= sum(len(j["fix"]) for j in model.get("joints", []))
    return n - len(model.get("drivers", {}))


def _connectivity(model):
    """Bodies that no chain of joints, forces or drivers connects to the ground; bodies with no
    element at all."""
    adj = {b: set() for b in model["bodies"]}
    adj["ground"] = set()
    for (a, b), _ in _connections(model):
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    grounded, todo = {"ground"}, ["ground"]
    while todo:
        for nb in adj[todo.pop()]:
            if nb not in grounded:
                grounded.add(nb)
                todo.append(nb)
    free = [b for b in model["bodies"] if b not in grounded]
    lonely = [b for b in model["bodies"] if not adj[b]]
    return free, lonely


def _extent(model):
    pts = [b["position"] for b in model["bodies"].values()] + [j["location"] for j in model.get("joints", [])]
    return float(np.ptp(np.array(pts), axis=0).max()) if len(pts) > 1 else 0.0


def check_model(model):
    """Findings and a summary of a validated model (no simulation needed)."""
    out = []
    bodies = model["bodies"]
    dof = degrees_of_freedom(model)
    mass = sum(b["mass"] for b in bodies.values())
    summary = {"bodies": len(bodies), "joints": len(model.get("joints", [])), "forces": len(model.get("forces", [])),
               "drivers": len(model.get("drivers", {})), "degrees_of_freedom": dof, "total_mass": mass,
               "extent": _extent(model)}
    # inertia: the principal moments of a real body satisfy the triangle inequality
    for name, b in bodies.items():
        I1, I2, I3 = b["inertia"]
        worst = min(I1 + I2 - I3, I1 + I3 - I2, I2 + I3 - I1)
        if worst < -1e-9 * max(b["inertia"]):
            out.append(finding("error", "inertia_impossible",
                               f"body {name}: inertia {b['inertia']} violates I1 + I2 >= I3 (no real body has it)",
                               [name]))
    # units: SI expected
    g = np.linalg.norm(model.get("gravity", [0, 0, 0]))
    if g and not 9.0 < g < 10.5:
        out.append(finding("warning", "units_gravity", f"|gravity| = {g:g}: expected 9.81 m/s^2 (SI units)"))
    if summary["extent"] > 100 and not any(f["type"] == "tire" for f in model.get("forces", [])):
        out.append(finding("warning", "units_length",
                           f"the model spans {summary['extent']:.0f} m: lengths in mm? (SI: m)"))
    for name, b in bodies.items():
        if b["mass"] > 1e5 or b["mass"] < 1e-6:
            out.append(finding("warning", "units_mass", f"body {name}: mass {b['mass']:g} kg (SI units?)", [name]))
    # topology
    if dof < 0:
        out.append(finding("warning", "overconstrained",
                           f"{-dof} more constraints than degrees of freedom: redundant constraints "
                           "(the solver may fail; replace joints by ones that fix fewer directions)"))
    elif dof == 0 and bodies:
        out.append(finding("info", "no_motion", "0 degrees of freedom: the motion is prescribed entirely"))
    free, lonely = _connectivity(model)
    if lonely:
        out.append(finding("info", "unconnected", f"no joint, force or driver on {', '.join(lonely)}: "
                                                  "they move freely", lonely))
    rest = [b for b in free if b not in lonely]
    if rest:
        out.append(finding("info", "not_grounded", f"not connected to the ground: {', '.join(rest)}", rest))
    # joints far from both bodies (a slip in a coordinate)
    size = {n: float(np.linalg.norm(inertia_box(b["mass"], b["inertia"]))) for n, b in bodies.items()}
    for j in model.get("joints", []):
        loc = np.asarray(j["location"], dtype=float)
        dist = [(np.linalg.norm(loc - bodies[b]["position"]), size[b]) for b in (j["body1"], j["body2"]) if b != "ground"]
        if dist and all(d > 3 * s + 0.05 for d, s in dist):
            out.append(finding("warning", "joint_far",
                               f"joint {j['name']}: {min(d for d, _ in dist):.3g} m from its bodies' centers of mass "
                               "(several times their size)", [j["name"], j["body1"], j["body2"]]))
    # springs and tires at t = 0
    for f in model.get("forces", []):
        if f["type"] == "spring_damper":
            p1 = _global(model, f["body1"], f.get("point1"))
            p2 = _global(model, f.get("body2", "ground"), f.get("point2"))
            L = float(np.linalg.norm(p1 - p2))
            if L < 1e-9:
                out.append(finding("error", "spring_zero_length",
                                   f"spring {f['name']}: its points coincide (no direction)", [f["name"]]))
            elif "length" in f and abs(f["length"] - L) > 0.5 * L:
                out.append(finding("info", "spring_preload", f"spring {f['name']}: free length {f['length']:g} m, "
                                                             f"initial length {L:.4g} m (large preload)", [f["name"]]))
        elif f["type"] == "tire" and "crg" not in (model.get("road") or {}):
            out += _tire_start(model, f)
    unused_points = sorted(name for name, r in (model.get("point_refs") or {}).items() if not r["users"])
    if unused_points:
        out.append(finding("info", "unused_points", f"points and frames not used: {', '.join(unused_points)}",
                           unused_points))
    unused = unused_parameters(model)
    if unused:
        out.append(finding("info", "unused_parameters", f"parameters not used in any expression: {', '.join(unused)} "
                                                         "(changing them changes nothing)", unused))
    references = [r for r in check_references(model, None) if r["kind"] == "mass"]
    return {"summary": summary, "findings": out, "references": references}


def unused_parameters(model):
    """Names of the model parameters that no expression (forces, drivers, measures) and no
    formula in a number field (=..., fdgen.formulas) mentions."""
    texts = []

    def collect(x):
        if isinstance(x, str):
            texts.append(x)
        elif isinstance(x, dict):
            for v in x.values():
                collect(v)
        elif isinstance(x, (list, tuple)):
            for v in x:
                collect(v)

    collect({k: v for k, v in model.items() if k not in ("parameters", "parameter_refs", "point_refs")})
    texts += model.get("parameter_refs", [])
    words = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", " ".join(texts)))
    return [p for p in model.get("parameters", {}) if p not in words]


def _global(model, body, point):
    if point is not None:
        return np.asarray(point, dtype=float)
    return np.zeros(3) if body == "ground" else np.asarray(model["bodies"][body]["position"], dtype=float)


def _tire_start(model, f):
    """Height of the wheel center above the analytic road at t = 0 against the tire radius."""
    road = model.get("road") or {}
    x, y, z = model["bodies"][f["body1"]]["position"]
    gx, gy = road.get("slope", [0.0, 0.0])
    h = gx * x + gy * y + sum(w["amplitude"] * np.sin(2 * np.pi * (x * np.cos(w.get("heading", 0.0)) + y * np.sin(
        w.get("heading", 0.0))) / w["wavelength"] + w.get("phase", 0.0)) for w in road.get("harmonics", []))
    r = (extensions.wheel(f) or (f.get("radius", 0.3),))[0]
    gap = z - h - r
    if gap > 0.01 * r:
        return [finding("info", "tire_in_air", f"tire {f['name']}: starts {gap:.3g} m above the road", [f["name"]])]
    if gap < -0.1 * r:
        return [finding("warning", "tire_deep", f"tire {f['name']}: starts {-gap:.3g} m into the road "
                                                f"({-gap / r:.0%} of its radius)", [f["name"]])]
    return []


def check_results(model, results):
    """Findings on the results: non-finite values, Euler parameter drift, the motion at the start
    (not in equilibrium), and the reference checks of the model."""
    out = []
    bad = [k for k, v in results.items() if not np.all(np.isfinite(v))]
    if bad:
        out.append(finding("error", "not_finite", f"non-finite values in {len(bad)} channels, e.g. {bad[0]}",
                           sorted({k.split('.')[0] for k in bad})))
    t = results.get("t")
    for name in model["bodies"]:
        norm = results.get(f"{name}.theta_norm")
        if norm is not None and np.all(np.isfinite(norm)) and np.max(np.abs(norm - 1)) > 1e-6:
            out.append(finding("warning", "rotation_drift",
                               f"body {name}: |theta| deviates from 1 by {np.max(np.abs(norm - 1)):.2g}", [name]))
    if t is not None and len(t) > 3 and np.linalg.norm(model.get("gravity", [0, 0, 0])) > 0:
        k = max(2, min(len(t) - 1, int(np.searchsorted(t, t[0] + 0.02 * (t[-1] - t[0])))))
        moving = []
        for name in model["bodies"]:
            v = [results.get(f"{name}.Rdot_{c}") for c in "xyz"]
            if any(c is None for c in v):
                continue
            a = np.linalg.norm([(c[k] - c[0]) / (t[k] - t[0]) for c in v])
            if a > 0.5 * G:
                moving.append((name, a))
        if moving:
            names = [n for n, _ in moving]
            out.append(finding("info", "start_transient",
                               f"{len(names)} bodies accelerate by more than g/2 at the start (not in equilibrium "
                               f"at t = 0), e.g. {names[0]} {moving[0][1]:.3g} m/s^2", names))
    return {"findings": out, "references": check_references(model, results)}


def _read_curve(path, x, y):
    with open(path, newline="", encoding="utf-8-sig") as f:  # -sig: CSV from Excel
        rows = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith("#")]
    header = None
    try:
        float(rows[0][0])
    except ValueError:
        header, rows = rows[0], rows[1:]
    col = lambda c: header.index(c) if isinstance(c, str) else int(c)
    data = np.array([[float(r[col(x)]), float(r[col(y)])] for r in rows])
    return data[:, 0], data[:, 1]


def check_references(model, results):
    """The model's `checks` section against the results: values at times and curves (over time
    or over another channel, e.g. toe over wheel travel); total mass."""
    spec = model.get("checks") or {}
    out = []
    if "mass" in spec:
        m, ref = sum(b["mass"] for b in model["bodies"].values()), spec["mass"]
        dev = m - ref["total"]
        out.append({"name": "total mass", "kind": "mass", "value": m, "reference": ref["total"], "deviation": dev,
                    "tolerance": ref["tolerance"], "passed": abs(dev) <= ref["tolerance"]})
    t = results.get("t") if results else None
    for c in spec.get("values", []):
        entry = {"name": c.get("name", f"{c['channel']} at t = {c.get('time', 0.0):g}"), "kind": "value",
                 "channel": c["channel"], "reference": c["value"], "tolerance": c["tolerance"]}
        if t is None or c["channel"] not in results:
            entry.update(passed=False, error=f"channel {c['channel']} not in the results")
        else:
            v = float(np.interp(c.get("time", 0.0), t, results[c["channel"]]))
            entry.update(value=v, deviation=v - c["value"], passed=abs(v - c["value"]) <= c["tolerance"])
        out.append(entry)
    for c in spec.get("curves", []):
        xc = c.get("x_channel", "t")
        entry = {"name": c.get("name", c["channel"] + ("" if xc == "t" else f" over {xc}")), "kind": "curve",
                 "channel": c["channel"], "x_channel": xc, "tolerance": c["tolerance"]}
        try:
            rx, ry = _read_curve(c["file"], c.get("x", 0), c.get("y", 1))
        except (OSError, ValueError, IndexError) as e:
            entry.update(passed=False, error=f"{Path(c['file']).name}: {e}")
            out.append(entry)
            continue
        entry.update(ref_x=rx.tolist(), ref_y=ry.tolist())
        if t is None or c["channel"] not in results or xc not in results:
            entry.update(passed=False, error=f"channel {c['channel']} or {xc} not in the results")
            out.append(entry)
            continue
        mx, my = results[xc], results[c["channel"]]
        steps = np.diff(mx)
        if xc != "t" and np.any(steps > 0) and np.any(steps < 0):
            entry["note"] = (f"{xc} is not monotonic over time: the results are no function of it "
                             "(a curve over a channel expects a sweep)")
        order = np.argsort(mx, kind="stable")
        mx, my = mx[order], my[order]
        inside = (rx >= mx[0]) & (rx <= mx[-1])
        if not inside.any():
            entry.update(passed=False, error="the reference lies outside the range of the results")
        else:
            d = np.interp(rx[inside], mx, my) - ry[inside]
            k = int(np.argmax(np.abs(d)))
            entry.update(max_deviation=float(np.abs(d).max()), at_x=float(rx[inside][k]),
                         rms=float(np.sqrt(np.mean(d * d))), points=int(inside.sum()),
                         passed=bool(np.abs(d).max() <= c["tolerance"]))
        out.append(entry)
    return out


def report_text(model_report, results_report=None):
    """The reports as plain text (command line, assistants)."""
    s = model_report["summary"]
    lines = [f"{s['bodies']} bodies, {s['joints']} joints, {s['forces']} forces, {s['drivers']} drivers; "
             f"{s['degrees_of_freedom']} degrees of freedom; total mass {s['total_mass']:g} kg"]
    findings = model_report["findings"] + (results_report["findings"] if results_report else [])
    lines += [f"{f['severity'].upper()}: {f['message']}" for f in findings] or ["no findings"]
    for r in (results_report or {}).get("references", []) or (model_report.get("references") or []):
        if "error" in r:
            lines.append(f"REFERENCE {r['name']}: not checked: {r['error']}")
        elif r["kind"] == "curve":
            lines.append(f"REFERENCE {r['name']}: {'passed' if r['passed'] else 'FAILED'}, max deviation "
                         f"{r['max_deviation']:.4g} at {r['x_channel']} = {r['at_x']:.4g} (tolerance {r['tolerance']:g}), "
                         f"rms {r['rms']:.4g}" + (f"; note: {r['note']}" if "note" in r else ""))
        else:
            lines.append(f"REFERENCE {r['name']}: {'passed' if r['passed'] else 'FAILED'}, {r['value']:.6g} "
                         f"against {r['reference']:.6g} (tolerance {r['tolerance']:g})")
    return "\n".join(lines)
