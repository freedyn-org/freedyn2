"""freedyn2 import-onshape --update: an Onshape assembly imported again into a model file that
was imported before and changed since, keeping the changes.

The elements of the file and of the new import are matched by their ids (fdgen.model element
ids): a body by onshape:body:<occurrence> or, when its heaviest part changed, by the parts of
its shapes (onshape:part:<occurrence>); a joint by onshape:mate:<feature>. Then, in the text of
the file, only where it is needed:

- a matched body: id, mass, inertia, position, orientation and the shapes from Onshape (its own
  shapes, without an onshape id, stay); a matched joint: type, body1, body2, location,
  orientation, fix. Name, other keys, comments and formatting stay: a body renamed in freedyn2
  keeps its name, and so does one renamed in Onshape.
- new bodies and joints: added at the end of their sections, named as by a first import (and
  unlike any name of the file).
- elements that are no longer in the assembly: removed, unless an element of the file still
  refers to them (a measure, a spring): then they stay, with a warning.
- everything without an onshape id (measures, drivers, forces, solver, ...): as it is.

A file without onshape ids (imported before there were ids) cannot be matched: an error.
"""

import re

import yaml

from .convert import value
from .errors import ModelError
from .rename import SECTIONS, _Edits, _get, _items, _references

BODY_KEYS = ("id", "mass", "inertia", "position", "orientation")
JOINT_KEYS = ("id", "type", "body1", "body2", "location", "orientation", "fix")


def _imported(e):
    return isinstance(e, dict) and str(e.get("id", "")).startswith("onshape:")


def _end(node):
    """The end of a node's content in the text (a block collection's end mark is the next token)."""
    if isinstance(node, yaml.ScalarNode) or node.flow_style or not node.value:
        return node.end_mark.index
    last = node.value[-1]
    return _end(last[1] if isinstance(node, yaml.MappingNode) else last)


class _Text:
    def __init__(self, text):
        self.text, self.edits = text, []

    def eol(self, i):
        """The end of the line of position i (before its newline)."""
        k = self.text.find("\n", i)
        return len(self.text) if k < 0 else k

    def bol(self, i):
        return self.text.rfind("\n", 0, i) + 1

    def after(self, i, lines):
        """Insert whole lines after the line of position i (at the start of the next line)."""
        pos = self.eol(i)
        if pos < len(self.text):
            self.replace(pos + 1, pos + 1, "".join(f"{x}\n" for x in lines))
        else:
            self.replace(pos, pos, "".join(f"\n{x}" for x in lines) + "\n")

    def replace(self, a, b, s):
        self.edits.append((a, b, s))

    def apply(self):
        out, last = [], 0
        for a, b, s in sorted(self.edits, key=lambda e: (e[0], e[1])):
            if a < last:
                raise ModelError("the model file could not be updated (overlapping changes); import it with --force")
            out += [self.text[last:a], s]
            last = b
        return "".join(out + [self.text[last:]])


def _set(t, element, key, v, block_shapes=False):
    """Set key of an element mapping node to the value v (None: remove the key)."""
    k, node = _get(element, key)
    flow = element.flow_style
    if node is not None:
        if v is None:  # remove the key
            if flow:
                a, b = k.start_mark.index, _end(node)
                rest = t.text[b:]
                m = re.match(r"\s*,\s*", rest)
                if m:
                    b += m.end()
                else:
                    a = t.text.rfind(",", 0, a)
            else:
                a, b = t.bol(k.start_mark.index), t.eol(_end(node)) + 1
            t.replace(a, b, "")
        elif block_shapes and not flow:
            items = [f"- {value(s)}" for s in v]
            if isinstance(node, yaml.SequenceNode) and not node.flow_style:  # the items where they are
                col = node.start_mark.column
                t.replace(node.start_mark.index, _end(node), ("\n" + " " * col).join(items))
            else:  # a flow list or nothing after the key: the items below it
                col = k.start_mark.column + 2
                t.replace(k.end_mark.index, _end(node), ":" + "".join(f"\n{' ' * col}{i}" for i in items))
        else:
            t.replace(node.start_mark.index, _end(node), value(v))
        return
    if v is None:
        return
    if flow:
        close = element.end_mark.index - 1
        t.replace(close, close, f", {key}: {value(v)}")
        return
    col = element.value[0][0].start_mark.column if element.value else 4
    lines = [f"{' ' * col}{key}:"] + [f"{' ' * col}  - {value(s)}" for s in v] if block_shapes \
        else [f"{' ' * col}{key}: {value(v)}"]
    t.after(_end(element), lines)


def _append(t, root, section, lines, flow_items):
    """Add elements at the end of a section (lines: written with an indent of 2 spaces)."""
    k, node = _get(root, section)
    if node is None or (isinstance(node, yaml.ScalarNode) and node.value in ("", "~", "null")):
        body = "\n".join(lines)
        if node is None:
            t.replace(len(t.text), len(t.text), ("" if t.text.endswith("\n") else "\n") + f"{section}:\n{body}\n")
        else:
            t.replace(node.start_mark.index, node.end_mark.index, "\n" + body)
        return
    if node.flow_style:
        if node.value:
            close = node.end_mark.index - 1
            t.replace(close, close, "".join(f", {n}: {value(d)}" for n, d in flow_items))
        else:
            t.replace(node.start_mark.index, node.end_mark.index, "\n" + "\n".join(lines))
        return
    col = node.value[0][0].start_mark.column
    t.after(_end(node), [" " * col + line[2:] if line.startswith("  ") else line for line in lines])


def _remove(t, key, node):
    a, b = t.bol(key.start_mark.index), t.eol(_end(node)) + 1
    t.replace(a, min(b, len(t.text)), "")
    return a, b


def _referenced(text, root, section, name, removed_spans):
    """Places outside the removed elements that refer to the element."""
    e = _Edits(text, name, name)
    _references(root, e, section)
    return [a for a, _, _ in e.spans if not any(x <= a < y for x, y in removed_spans)]


def update_text(text, model, groups):
    """(the model file's text with the new import merged in, report {updated (values changed),
    unchanged, added, removed, kept, names},
    warnings). model, groups: of fdgen.onshape.build_model."""
    from .onshape import body_lines, element_id, joint_lines

    try:
        root, data = yaml.compose(text), yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ModelError(f"the model file is not valid YAML: {e}") from None
    data = data if isinstance(data, dict) else {}
    old_bodies = {n: b for n, b in (data.get("bodies") or {}).items() if isinstance(b, dict)}
    old_joints = {n: j for n, j in (data.get("joints") or {}).items() if isinstance(j, dict)}
    ground_shapes = (data.get("ground") or {}).get("shapes") or []
    if not any(_imported(e) for e in [*old_bodies.values(), *old_joints.values(), *ground_shapes]):
        raise ModelError("the model file has no Onshape ids (imported before there were ids): --update cannot "
                         "match its elements; import it once with --force and copy your additions over")
    taken = {n for s in SECTIONS for n in (data.get(s) or {})} | {"ground"}
    warnings, report = [], {"updated": [], "unchanged": [], "added": [], "removed": [], "kept": [], "names": {}}

    # bodies: by the body id, else by the parts of the shapes
    by_id = {b["id"]: n for n, b in old_bodies.items() if _imported(b)}
    by_part = {s["id"]: n for n, b in old_bodies.items() for s in b.get("shapes") or []
               if _imported(s) and s["id"].startswith("onshape:part:")}
    name_of, used = {"ground": "ground"}, set()
    for new, b in model["bodies"].items():
        old = by_id.get(b["id"])
        if old is None or old in used:
            parts = [element_id("part", p.path) for p in groups[new]]
            old = next((by_part[p] for p in parts if p in by_part and by_part[p] not in used), None)
        if old is None:
            name, k = new, 2
            while name in taken:
                name, k = f"{new}_{k}", k + 1
            taken.add(name)
        else:
            name = old
            used.add(old)
        name_of[new] = name
    report["names"] = dict(name_of)
    joints = {}
    for new, j in model.get("joints", {}).items():
        j = dict(j, body1=name_of[j["body1"]])
        if "body2" in j:
            j["body2"] = name_of[j["body2"]]
        joints[new] = j
    old_joint_by_id = {j["id"]: n for n, j in old_joints.items() if _imported(j)}

    t = _Text(text)
    nodes = {s: {k.value: (k, v) for k, v in _items(_get(root, s)[1])} for s in ("bodies", "joints")}

    # removed: imported elements of the file that are not in the assembly any more
    gone = [("bodies", n) for n, b in old_bodies.items() if _imported(b) and n not in used]
    gone += [("joints", n) for n, j in old_joints.items()
             if _imported(j) and j["id"] not in {x["id"] for x in joints.values()}]
    spans = {}
    for section, n in gone:
        k, v = nodes[section][n]
        spans[(section, n)] = (t.bol(k.start_mark.index), t.eol(_end(v)) + 1)
    for section, n in gone:
        refs = _referenced(text, root, section, n, list(spans.values()))
        if refs:
            line = text.count("\n", 0, refs[0]) + 1
            warnings.append(f"{ {'bodies': 'body', 'joints': 'joint'}[section]} {n} is no longer in the Onshape assembly, but the model refers to it "
                            f"(line {line}): kept; remove it and the references by hand")
            report["kept"].append(n)
            del spans[(section, n)]
    for (section, n), (a, b) in spans.items():
        t.replace(a, min(b, len(text)), "")
        report["removed"].append(n)

    # matched bodies: the values from Onshape; new ones at the end
    added = []
    for new, b in model["bodies"].items():
        name = name_of[new]
        if name not in nodes["bodies"]:
            added.append((name, b, groups[new]))
            continue
        element = nodes["bodies"][name][1]
        if not isinstance(element, yaml.MappingNode):
            raise ModelError(f"body {name}: not a mapping in the model file")
        changed = False
        for key in BODY_KEYS:
            if old_bodies[name].get(key) != b[key]:
                _set(t, element, key, b[key])
                changed = True
        if "shapes" in b:
            own = [s for s in old_bodies[name].get("shapes") or [] if not _imported(s)]
            shapes = own + b["shapes"]
            if (old_bodies[name].get("shapes") or []) != shapes:
                _set(t, element, "shapes", shapes, block_shapes=True)
                changed = True
        report["updated" if changed else "unchanged"].append(name)
    if added:
        _append(t, root, "bodies", [line for n, b, parts in added for line in body_lines(n, b, parts)],
                [(n, b) for n, b, _ in added])
        report["added"] += [n for n, _, _ in added]

    # joints: matched by the mate id
    added = []
    for new, j in joints.items():
        old = old_joint_by_id.get(j["id"])
        if old is None:
            name, k = new, 2
            while name in taken:
                name, k = f"{new}_{k}", k + 1
            taken.add(name)
            added.append((name, j))
            continue
        element = nodes["joints"][old][1]
        if not isinstance(element, yaml.MappingNode):
            raise ModelError(f"joint {old}: not a mapping in the model file")
        changed = False
        for key in JOINT_KEYS:
            if old_joints[old].get(key) != j.get(key):
                _set(t, element, key, j.get(key))
                changed = True
        report["updated" if changed else "unchanged"].append(old)
    if added:
        _append(t, root, "joints", [line for n, j in added for line in joint_lines(n, j)], added)
        report["added"] += [n for n, _ in added]

    # the ground's shapes
    if "ground" in model:
        own = [s for s in ground_shapes if not _imported(s)]
        shapes = own + model["ground"]["shapes"]
        if shapes != ground_shapes:
            gk, ground = _get(root, "ground")
            if ground is None or not isinstance(ground, yaml.MappingNode):
                lines = ["  shapes:"] + [f"    - {value(s)}" for s in shapes]
                if ground is None:
                    t.replace(len(text), len(text), ("" if text.endswith("\n") else "\n") + "ground:\n"
                              + "\n".join(lines) + "\n")
                else:
                    t.replace(ground.start_mark.index, ground.end_mark.index, "\n" + "\n".join(lines))
            else:
                _set(t, ground, "shapes", shapes, block_shapes=True)
    return t.apply(), report, warnings
