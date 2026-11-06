"""The model file in the viewer's editor: read it, save it, and check a text before it is saved.

  read(path)              {text, stamp, editable}: stamp is a hash of the file as read
  save(path, text, stamp) write the text if the file is still the one read (else Conflict)
  lint(text, base_dir)    diagnostics [{line, end_line, severity, message}] of a text: YAML
                          syntax, the schema (all errors, at their line) and the model rules
                          (fdgen.model.validate), each at the line of the element it names

Only YAML model files are edited; a legacy .fds file is shown read-only. Saving writes a
temporary file next to the model and moves it over the model (a reader never sees half a file);
the viewer's file watch then rebuilds and simulates as after any other save.
"""

import copy
import hashlib
import os
import re
import tempfile
from pathlib import Path

import yaml

EDITABLE = (".yaml", ".yml")
MAX_DIAGNOSTICS = 50
# "joint arm: ..." -> the section of the element a message of fdgen.model names
_KINDS = {"body": "bodies", "joint": "joints", "force": "forces", "sensor": "sensors", "measure": "measures",
          "spline": "splines", "driver": "drivers", "point": "points", "frame": "frames"}


class Conflict(Exception):
    """The model file changed since the editor read it (another editor, the assistant)."""


def stamp(data):
    return hashlib.sha256(data).hexdigest()[:16]


def read(path):
    path = Path(path)
    data = path.read_bytes() if path.exists() else b""
    return {"text": data.decode("utf-8", errors="replace").replace("\r\n", "\n"), "stamp": stamp(data),
            "editable": path.suffix.lower() in EDITABLE, "name": path.name}


def save(path, text, expected):
    """Write text to the model file; expected: the stamp of the file the text was made from."""
    path = Path(path)
    if path.suffix.lower() not in EDITABLE:
        raise ValueError(f"{path.name}: only YAML model files are edited in the viewer (freedyn2 convert makes one)")
    old = path.read_bytes() if path.exists() else b""
    if expected is not None and stamp(old) != expected:
        raise Conflict(f"{path.name} was changed meanwhile (by another editor or the assistant)")
    text = str(text)
    if b"\r\n" in old:  # keep the line ends of the file
        text = text.replace("\r\n", "\n").replace("\n", "\r\n")
    data = text.encode("utf-8")
    fd, tmp = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
    return stamp(data)


def _diag(line, message, severity="error", end_line=None):
    return {"line": max(1, int(line)), "end_line": max(1, int(end_line or line)), "severity": severity, "message": message}


def _node_at(root, keys):
    """(node of the key, node of the value) at a path of keys and list indices; the deepest found."""
    key, node = None, root
    for k in keys:
        if isinstance(node, yaml.MappingNode):
            hit = next(((kn, vn) for kn, vn in node.value if kn.value == str(k)), None)
            if hit is None:
                break
            key, node = hit
        elif isinstance(node, yaml.SequenceNode) and isinstance(k, int) and k < len(node.value):
            key, node = None, node.value[k]
        else:
            break
    return key, node


def _schema_errors(raw, root):
    from .. import formulas, hardpoints
    from ..model import _validator

    resolved, _ = formulas.resolve(raw)
    resolved, _ = hardpoints.resolve(resolved)
    out = []
    for e in sorted(_validator().iter_errors(resolved), key=lambda e: list(map(str, e.absolute_path))):
        keys = list(e.absolute_path)
        if e.validator == "additionalProperties":  # point at the key that is too much
            extra = re.findall(r"'([^']+)' (?:was|were) unexpected", e.message)
            names = re.findall(r"'([^']+)'", e.message.split("(")[-1]) if extra else []
            for name in names or [None]:
                key, node = _node_at(root, keys + ([name] if name else []))
                line = (key or node).start_mark.line + 1
                out.append(_diag(line, f"unknown key {name}" + _where(keys) if name else e.message))
            continue
        if e.validator == "required":
            key, node = _node_at(root, keys)
            out.append(_diag((key or node).start_mark.line + 1, e.message + _where(keys)))
            continue
        key, node = _node_at(root, keys)
        hint = ' (empty: a "#" starts a comment; formulas start with "=")' if e.instance is None else ""
        out.append(_diag((key or node).start_mark.line + 1, e.message + _where(keys) + hint))
    return out


def _where(keys):
    return f" (at {'.'.join(map(str, keys))})" if keys else ""


def _scalar(node, value):
    """The first scalar node with this value below node."""
    if isinstance(node, yaml.ScalarNode):
        return node if node.value == value else None
    for item in node.value if isinstance(node, (yaml.MappingNode, yaml.SequenceNode)) else []:
        for n in item if isinstance(item, tuple) else (item,):
            hit = _scalar(n, value)
            if hit is not None:
                return hit
    return None


def _element_line(message, root):
    """The line of the element a model error names ("joint arm: ...", "forces.spring.k: ...")."""
    head = message.split(":", 1)[0].strip()
    words = head.split()
    if len(words) == 2 and words[0] in _KINDS:
        key, node = _node_at(root, [_KINDS[words[0]], words[1]])
        if key is not None:
            named = re.search(r"unknown \w+ '?([A-Za-z_][A-Za-z0-9_]*)", message)
            hit = _scalar(node, named.group(1)) if named else None
            return (hit or key).start_mark.line + 1
    if "." in head and " " not in head:
        key, node = _node_at(root, head.split("."))
        if node is not root:
            return (key or node).start_mark.line + 1
    first = words[0] if words else ""
    key, node = _node_at(root, [first])
    if key is not None:
        return key.start_mark.line + 1
    return 1


def lint(text, base_dir=None):
    """Diagnostics of a model text, the most basic first: a YAML error hides the rest."""
    from ..errors import ModelError
    from ..model import LOADER, validate

    try:
        root = yaml.compose(text, Loader=LOADER)
        raw = yaml.load(text, Loader=LOADER)
    except yaml.MarkedYAMLError as e:
        mark = e.problem_mark or e.context_mark
        return [_diag(mark.line + 1 if mark else 1, f"YAML: {e.problem or e}" + (f" ({e.context})" if e.context else ""))]
    if not isinstance(raw, dict) or root is None:
        return [_diag(1, "a model file is a map: model, bodies, joints, forces, solver, ...")]
    try:
        out = _schema_errors(raw, root)
    except ModelError as e:  # a formula or a hardpoint that cannot be resolved
        return [_diag(_element_line(str(e), root), str(e))]
    if out:
        return out[:MAX_DIAGNOSTICS]
    try:
        validate(copy.deepcopy(raw), base_dir)
    except (ModelError, ValueError, OSError) as e:
        return [_diag(_element_line(str(e), root), str(e))]
    return []
