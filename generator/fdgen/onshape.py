"""freedyn2 import-onshape: an Onshape assembly as a freedyn2 model file (YAML).

The assembly is read with the Onshape REST API (API keys: ONSHAPE_ACCESS_KEY and
ONSHAPE_SECRET_KEY, HTTP Basic authentication) and written as YAML in the format of the
hand-written examples:

- Parts become bodies. Parts that are rigidly connected (Fastened mates, mate groups) are one
  body: masses, centers of mass and inertia tensors are added up (parallel axis theorem) and the
  principal axes of the sum are the body frame. Fixed parts, and everything rigidly connected to
  them, are the ground.
- Mass properties are Onshape's (computed on the exact geometry, SI units). A body whose parts
  have no mass at all (no material assigned) is an error naming them; massless parts in a body
  with mass (screws, standard content) are fine.
- Mates become joints, in the frame of a mated entity (joint axis z, as Onshape's mate
  connectors): Revolute, Slider, Cylindrical, Ball, Planar, Parallel and Pin Slot. Fastened mates
  and mate groups merge bodies. Other mates (Tangent, Width) and mate relations (gears, screws,
  rack and pinion) are not imported: they are listed in the file and returned as warnings, so
  nothing is dropped silently.
- Mates of sub-assemblies are imported as joints of the parts they connect (the sub-assembly is
  flattened, as in Onshape with "flexible" sub-assemblies).
- Geometry for the viewer: an STL file per part (in meters, placed with the part's transform);
  the simulation does not use it. The Onshape reference of every part (document, microversion,
  element, part id, configuration) is written to <model>.onshape.json next to the model file, so
  the exact geometry (B-rep, STEP) can be fetched again later.

The API responses are read through a client object with get_json(path, params) and
get_bytes(path, params) (OnshapeClient, or a stand-in for tests). The written model is read back
with the YAML loader (schema and model rules) before it is saved.
"""

import base64
import hashlib
import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .convert import _Writer, value
from .errors import ModelError
from .shapes import euler_parameters

API = "/api/v6"
URL = re.compile(r"^(https?://[^/]+)/documents/([0-9a-f]{24})/(w|v|m)/([0-9a-f]{24})/e/([0-9a-f]{24})")
IDENT = re.compile(r"[^A-Za-z0-9_]+")

# Onshape mate type -> (freedyn2 joint type, fixed directions for generic joints)
JOINTS = {
    "REVOLUTE": ("revolute", None),
    "SLIDER": ("prismatic", None),
    "CYLINDRICAL": ("cylindrical", None),
    "BALL": ("spherical", None),
    "PLANAR": ("generic", ["dz", "ax", "ay"]),
    "PARALLEL": ("generic", ["ax", "ay"]),
    "PIN_SLOT": ("generic", ["dy", "dz", "ax", "ay"]),
}
RIGID = "FASTENED"
DEFAULT_SOLVER = {"type": "HHT", "alpha": -0.1, "end_time": 5.0, "step": 1.0e-3, "output_step": 1.0e-2}


# --- the API -------------------------------------------------------------------------------------

def parse_url(url):
    """(server, document id, w|v|m, workspace/version/microversion id, element id, configuration)
    of an Onshape document URL (the address of the assembly tab in the browser)."""
    m = URL.match(url.strip())
    if not m:
        raise ModelError(f"{url}: not an Onshape assembly URL "
                         "(https://cad.onshape.com/documents/<document>/w/<workspace>/e/<element>)")
    query = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)
    return (*m.groups(), query.get("configuration", [""])[0])


class OnshapeClient:
    """The Onshape REST API with API keys (https://dev-portal.onshape.com/keys), HTTP Basic."""

    def __init__(self, access_key, secret_key, server="https://cad.onshape.com", timeout=120):
        if not access_key or not secret_key:
            raise ModelError("Onshape API keys missing: set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY "
                             "(create them at https://dev-portal.onshape.com/keys, read access is enough)")
        token = base64.b64encode(f"{access_key}:{secret_key}".encode()).decode()
        self.server, self.timeout = server.rstrip("/"), timeout
        self.headers = {"Authorization": f"Basic {token}", "User-Agent": "freedyn2"}

    @classmethod
    def from_env(cls, server="https://cad.onshape.com"):
        return cls(os.environ.get("ONSHAPE_ACCESS_KEY"), os.environ.get("ONSHAPE_SECRET_KEY"), server)

    def _get(self, path, params, accept):
        query = urllib.parse.urlencode({k: v for k, v in (params or {}).items() if v not in (None, "")})
        url = f"{self.server}{API}{path}" + (f"?{query}" if query else "")
        req = urllib.request.Request(url, headers={**self.headers, "Accept": accept})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", "replace")[:300]
            hint = {401: " (API keys wrong or expired)", 403: " (no access to this document with these keys)",
                    404: " (document, element or part not found)"}.get(e.code, "")
            raise ModelError(f"Onshape API {path}: HTTP {e.code}{hint}: {detail}") from None
        except urllib.error.URLError as e:
            raise ModelError(f"Onshape API {path}: {e.reason}") from None

    def get_json(self, path, params=None):
        return json.loads(self._get(path, params, "application/json"))

    def get_bytes(self, path, params=None):
        return self._get(path, params, "application/octet-stream")


# --- the assembly ------------------------------------------------------------------------------

@dataclass
class Part:
    """A part occurrence: its instance path, name, definition and placement in the assembly."""
    path: tuple
    name: str
    ref: dict  # documentId, documentMicroversion, elementId, partId, fullConfiguration
    T: np.ndarray  # 4x4, part studio -> assembly coordinates
    fixed: bool
    mass: float = 0.0
    centroid: np.ndarray = None  # part studio coordinates
    inertia: np.ndarray = None  # 3x3 about the centroid, part studio axes


@dataclass
class Mate:
    name: str
    mate_type: str
    entities: list  # [(occurrence path, 4x4 frame in the occurrence's coordinates)]


@dataclass
class Assembly:
    parts: dict = field(default_factory=dict)  # path -> Part
    mates: list = field(default_factory=list)
    groups: list = field(default_factory=list)  # (name, [occurrence paths])
    unsupported: list = field(default_factory=list)  # (name, why)


def _ref_key(x):
    return (x.get("documentId"), x.get("documentMicroversion") or x.get("documentVersion"), x.get("elementId"),
            x.get("partId") or "", x.get("fullConfiguration", x.get("configuration", "")))


def _matrix(transform):
    return np.asarray(transform, dtype=float).reshape(4, 4)  # row-major, meters


def _cs(cs):
    """4x4 of a coordinate system {origin, xAxis, yAxis, zAxis}."""
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = cs["xAxis"], cs["yAxis"], cs["zAxis"], cs["origin"]
    return T


def _part_path(ref, version="m"):
    return (f"/d/{ref['documentId']}/{version}/{ref.get('documentMicroversion') or ref.get('documentVersion')}"
            f"/e/{ref['elementId']}")


def read_assembly(client, did, wvm, wvmid, eid, configuration=""):
    """The parts (with their mass properties), mates and mate groups of an assembly."""
    d = client.get_json(f"/assemblies/d/{did}/{wvm}/{wvmid}/e/{eid}", {
        "includeMateFeatures": "true", "includeMateConnectors": "true", "includeNonSolids": "false",
        "excludeSuppressed": "true", "configuration": configuration})
    root = d["rootAssembly"]
    subs = {_ref_key(s): s for s in d.get("subAssemblies", [])}
    instances = {}
    for a in [root, *subs.values()]:
        for i in a.get("instances", []):
            instances[i["id"]] = i
    asm = Assembly()
    names = {}
    fixed_paths = set()
    for occ in root.get("occurrences", []):
        path = tuple(occ["path"])
        inst = instances.get(path[-1])
        if inst is None:
            continue
        names[path] = inst.get("name", path[-1])
        if occ.get("fixed"):
            fixed_paths.add(path)
        if inst.get("suppressed"):
            continue
        if inst["type"] == "Part":
            full = " / ".join(names.get(path[:k + 1], path[k]) for k in range(len(path)))
            fixed = any(path[:k + 1] in fixed_paths for k in range(len(path)))
            asm.parts[path] = Part(path, full, {k: inst.get(k) for k in (
                "documentId", "documentMicroversion", "documentVersion", "elementId", "partId", "fullConfiguration")},
                _matrix(occ["transform"]), fixed)
        elif inst["type"] == "Assembly":
            sub = subs.get(_ref_key(inst))
            if sub is not None:
                _features(asm, sub.get("features", []), path, names)
    _features(asm, root.get("features", []), (), names)
    done = {}  # one request per part definition (Onshape limits the API calls per account)
    for part in asm.parts.values():
        key = _ref_key(part.ref)
        if key not in done:
            _mass_properties(client, part, did)
            done[key] = part
        else:
            part.mass, part.centroid, part.inertia = done[key].mass, done[key].centroid, done[key].inertia
    return asm


def _features(asm, features, prefix, names):
    for f in features:
        if f.get("suppressed"):
            continue
        data, kind = f.get("featureData", {}), f.get("featureType")
        name = " / ".join([*(names.get(prefix[:k + 1], prefix[k]) for k in range(len(prefix))),
                           data.get("name", f.get("id", "?"))])
        if kind == "mate":
            ents = [(tuple(prefix) + tuple(e["matedOccurrence"]), _cs(e["matedCS"]))
                    for e in data.get("matedEntities", [])]
            if len(ents) != 2:
                asm.unsupported.append((name, f"mate with {len(ents)} entities"))
            else:
                asm.mates.append(Mate(name, data.get("mateType", "?"), ents))
        elif kind == "mateGroup":
            occs = [tuple(prefix) + tuple(o.get("occurrence", [])) for o in data.get("occurrences", [])]
            asm.groups.append((name, [o for o in occs if len(o) > len(prefix)]))
        elif kind == "mateRelation":
            asm.unsupported.append((name, f"mate relation {data.get('relationType', '')}".strip()))
        elif kind != "mateConnector":
            asm.unsupported.append((name, f"feature type {kind}"))


def _mass_properties(client, part, link_document):
    r = part.ref
    d = client.get_json(f"/parts{_part_path(r)}/partid/{urllib.parse.quote(r['partId'], safe='')}/massproperties", {
        "linkDocumentId": link_document, "configuration": r.get("fullConfiguration")})
    body = (d.get("bodies") or {}).get(r["partId"]) or next(iter((d.get("bodies") or {}).values()), {})
    mass = float((body.get("mass") or [0.0])[0])
    if not body.get("hasMass", mass > 0) or mass <= 0:
        part.mass, part.centroid, part.inertia = 0.0, np.zeros(3), np.zeros((3, 3))
        return
    part.mass = mass
    part.centroid = np.asarray(body["centroid"][:3], dtype=float)
    if "principalAxes" in body and "principalInertia" in body:
        P = np.array([[a["x"], a["y"], a["z"]] for a in body["principalAxes"]], dtype=float).T  # axes as columns
        part.inertia = P @ np.diag(np.asarray(body["principalInertia"][:3], dtype=float)) @ P.T
    else:  # the tensor about the centroid, row-major (its first 9 values)
        part.inertia = np.asarray(body["inertia"][:9], dtype=float).reshape(3, 3)


# --- rigid clusters ------------------------------------------------------------------------------

def _clusters(asm):
    """{part path: cluster root}: parts joined by Fastened mates and mate groups; fixed parts in
    the cluster 'ground'."""
    parent = {p: p for p in asm.parts}

    def find(p):
        while parent[p] != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    def parts_under(occ):
        return [p for p in asm.parts if p[:len(occ)] == tuple(occ)]

    for mate in asm.mates:
        if mate.mate_type == RIGID:
            a, b = (parts_under(e[0]) for e in mate.entities)
            for p in a[1:] + b:
                union((a + b)[0], p)
    for _, occs in asm.groups:
        members = [p for o in occs for p in parts_under(o)]
        for p in members[1:]:
            union(members[0], p)
    fixed = [p for p, part in asm.parts.items() if part.fixed]
    for p in fixed[1:]:
        union(fixed[0], p)
    ground = find(fixed[0]) if fixed else None
    return {p: ("ground" if find(p) == ground else find(p)) for p in asm.parts}


def _ident(name, taken):
    base = IDENT.sub("_", name.split(" / ")[-1]).strip("_").lower() or "part"
    if base[0].isdigit():
        base = f"part_{base}"
    out, k = base, 2
    while out in taken or out == "ground":
        out, k = f"{base}_{k}", k + 1
    taken.add(out)
    return out


def _combine(parts):
    """(mass, center of mass, principal moments, principal axes as columns) of rigidly connected
    parts, in assembly coordinates."""
    M = sum(p.mass for p in parts)
    c = sum(p.mass * (p.T[:3, :3] @ p.centroid + p.T[:3, 3]) for p in parts) / M
    J = np.zeros((3, 3))
    for p in parts:
        if p.mass <= 0:
            continue
        R = p.T[:3, :3]
        d = R @ p.centroid + p.T[:3, 3] - c
        J += R @ p.inertia @ R.T + p.mass * (d @ d * np.eye(3) - np.outer(d, d))
    moments, axes = np.linalg.eigh(0.5 * (J + J.T))
    if np.linalg.det(axes) < 0:
        axes[:, 2] = -axes[:, 2]
    return M, c, moments, axes


# --- the model ---------------------------------------------------------------------------------

def build_model(asm, name="onshape_model", mesh_dir=None):
    """(model dict, {body: [part paths]}, warnings) of an assembly; mesh_dir (relative to the
    model file): STL file names of the parts' shapes, None: no shapes."""
    if not asm.parts:
        raise ModelError("the assembly has no parts")
    cluster = _clusters(asm)
    members = {}
    for p, c in cluster.items():
        members.setdefault(c, []).append(p)
    taken, body_of, bodies, groups = set(), {}, {}, {}
    warnings = [f"{n}: {why}, not imported" for n, why in asm.unsupported]
    massless = []
    for c, paths in sorted(members.items(), key=lambda kv: (kv[0] != "ground", min(kv[1]))):
        parts = [asm.parts[p] for p in paths]
        bname = "ground" if c == "ground" else _ident(max(parts, key=lambda p: p.mass).name, taken)
        body_of.update((p, bname) for p in paths)
        groups[bname] = paths
        body = bodies[bname] = {}
        if bname != "ground":
            if sum(p.mass for p in parts) <= 0:
                massless.append(" + ".join(p.name for p in parts))
                continue
            M, com, moments, axes = _combine(parts)
            body.update(mass=M, inertia=[max(float(x), 1e-12) for x in moments], position=com.tolist(),
                        orientation=euler_parameters(axes).tolist())
        if mesh_dir is not None:
            body["shapes"] = [{"type": "mesh", "file": f"{mesh_dir}/{mesh_name(p)}", "position": p.T[:3, 3].tolist(),
                               "orientation": euler_parameters(p.T[:3, :3]).tolist()} for p in parts]
    if massless:
        raise ModelError("no mass (assign a material in Onshape, or fasten them to a part with mass): "
                         + "; ".join(massless))
    joints, jtaken = {}, set()
    for mate in asm.mates:
        if mate.mate_type == RIGID:
            continue
        if mate.mate_type not in JOINTS:
            warnings.append(f"{mate.name}: mate type {mate.mate_type}, not imported")
            continue
        jtype, fix = JOINTS[mate.mate_type]
        if any(occ not in asm.parts for occ, _ in mate.entities):
            warnings.append(f"{mate.name}: a mated part is not in the assembly, not imported")
            continue
        # matedCS is in the coordinates of the mated part: the frames at t = 0, globally
        ends = [(body_of[occ], asm.parts[occ].T @ cs) for occ, cs in mate.entities]
        if ends[0][0] == ends[1][0]:
            warnings.append(f"{mate.name}: both sides are the same rigid body ({ends[0][0]}), not imported")
            continue
        # the joint frame: Pin Slot in the frame of the slot (second entity), else the first entity;
        # body1 carries the joint frame (its free translation of the Pin Slot)
        k = 1 if mate.mate_type == "PIN_SLOT" else 0
        F, body1, body2 = ends[k][1], ends[k][0], ends[1 - k][0]
        if body1 == "ground":
            body1, body2 = body2, body1
        j = {"type": jtype, "body1": body1}
        if body2 != "ground":
            j["body2"] = body2
        j["location"] = F[:3, 3].tolist()
        j["orientation"] = euler_parameters(F[:3, :3]).tolist()
        if fix:
            j["fix"] = fix
        joints[_ident(mate.name, jtaken)] = j
    ground = bodies.pop("ground", {})
    model = {"model": name, "gravity": [0.0, 0.0, -9.81], "bodies": bodies}
    if ground.get("shapes"):
        model["ground"] = {"shapes": ground["shapes"]}
    if joints:
        model["joints"] = joints
    model["solver"] = dict(DEFAULT_SOLVER)
    return model, {b: [asm.parts[p] for p in ps] for b, ps in groups.items()}, warnings


def mesh_name(part):
    """STL file of a part definition (instances of the same part share it)."""
    base = IDENT.sub("_", re.sub(r"\s*<\d+>$", "", part.name.split(" / ")[-1])).strip("_").lower() or "part"
    return f"{base}_{hashlib.sha1(repr(_ref_key(part.ref)).encode()).hexdigest()[:6]}.stl"


def model_text(model, groups, source, warnings):
    """The model as YAML, with comments naming the Onshape parts and mates behind it."""
    w = _Writer()
    w.comment(f"Imported from Onshape by freedyn2 import-onshape:\n{source}\n"
              "Bodies are the rigid groups of parts (Fastened mates, mate groups); mass properties from Onshape.")
    if warnings:
        w.comment("Not imported:\n" + "\n".join(f"  {x}" for x in warnings))
    w.key("model", model["model"])
    w.key("gravity", model["gravity"])
    if "ground" in model:
        w.lines.append("ground:")
        w.comment("parts: " + ", ".join(p.name for p in groups.get("ground", [])), "  ")
        w.lines.append("  shapes:")
        for s in model["ground"]["shapes"]:
            w.lines.append(f"    - {value(s)}")
    w.lines.append("bodies:")
    for b, d in model["bodies"].items():
        w.lines.append(f"  {b}:")
        w.comment("parts: " + ", ".join(p.name for p in groups[b]), "    ")
        for k in ("mass", "inertia", "position", "orientation"):
            w.key(k, d[k], indent="    ")
        if d.get("shapes"):
            w.lines.append("    shapes:  # for the viewer (freedyn2 view) only")
            for s in d["shapes"]:
                w.lines.append(f"      - {value(s)}")
    if model.get("joints"):
        w.lines.append("joints:")
        for j, d in model["joints"].items():
            w.element(j, [(k, v, "") for k, v in d.items()])
    w.key("solver", model["solver"])
    return w.text()


def import_onshape(url, output=None, client=None, meshes=True, force=False, name=None):
    """Write the Onshape assembly at url as a YAML model file; (its path, warnings).
    output: the YAML file (default: <assembly name>.yaml in the current directory); meshes: STL
    files of the parts in <model>_meshes/ for the viewer; client: OnshapeClient (default: from the
    environment variables ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY)."""
    from .model import validate

    server, did, wvm, wvmid, eid, configuration = parse_url(url)
    client = client or OnshapeClient.from_env(server)
    if name is None:
        try:
            el = client.get_json(f"/documents/d/{did}/{wvm}/{wvmid}/elements", {"elementId": eid})
            name = (el[0] if isinstance(el, list) and el else {}).get("name") or "onshape_model"
        except ModelError:
            name = "onshape_model"
    model_name = IDENT.sub("_", name).strip("_").lower() or "onshape_model"
    out = Path(output) if output else Path(f"{model_name}.yaml")
    if out.is_dir():
        out = out / f"{model_name}.yaml"
    if out.exists() and not force:
        raise ModelError(f"{out} exists (--force / force=True overwrites it)")
    asm = read_assembly(client, did, wvm, wvmid, eid, configuration)
    mesh_dir = f"{out.stem}_meshes" if meshes else None
    model, groups, warnings = build_model(asm, model_name, mesh_dir)
    text = model_text(model, groups, url, warnings)
    out.parent.mkdir(parents=True, exist_ok=True)
    if meshes:
        d = out.parent / mesh_dir
        d.mkdir(exist_ok=True)
        done = set()
        for part in asm.parts.values():
            if mesh_name(part) in done:
                continue
            done.add(mesh_name(part))
            r = part.ref
            data = client.get_bytes(f"/partstudios{_part_path(r)}/stl", {
                "partIds": r["partId"], "mode": "binary", "units": "meter", "grouping": "true",
                "configuration": r.get("fullConfiguration"), "linkDocumentId": did})
            (d / mesh_name(part)).write_bytes(data)
    try:
        validate(yaml.safe_load(text), out.resolve().parent)
    except ModelError as e:
        raise ModelError(f"the imported model is not a valid model: {e}") from None
    out.write_text(text, encoding="utf-8", newline="\n")
    refs = {"source": url, "document": did, wvm: wvmid, "element": eid, "configuration": configuration,
            "bodies": {b: [{"name": p.name, "occurrence": list(p.path), **{k: v for k, v in p.ref.items() if v}}
                           for p in ps] for b, ps in groups.items()}}
    out.with_suffix(".onshape.json").write_text(json.dumps(refs, indent=1) + "\n", encoding="utf-8")
    return out, warnings
