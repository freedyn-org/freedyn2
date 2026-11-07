"""freedyn2 rename: rename an element of a model file and every reference to it.

The file is changed only where the name stands: the element's key and the places that refer to
it, found in the YAML node tree with their exact positions in the text. Comments, formatting
and the order of the file stay as they are; a word in a comment that happens to be the name is
not touched. What refers to what:

- bodies: body1, body2 (joints, forces, measures, sensors), frame of measures, in of points and
  frames; the channels of checks (rod.R_z)
- joints, forces, sensors, drivers, outputs: the channels of checks
- points, frames: the point fields (position, location, point, point1, point2, origin), mirror
  and from of points, x and z of frames; frames also orientation fields and in
- measures: measure of drivers, and like parameters, inputs and splines: the expressions of
  forces (force, torque), drivers (motion) and outputs; parameters also in every formula (=...)

The result is read back with the model loader before it is written: a rename that would make
the model invalid is an error and changes nothing.
"""

import re
from pathlib import Path

import yaml

from .errors import ModelError

IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
SECTIONS = ("bodies", "joints", "forces", "sensors", "drivers", "outputs", "measures", "parameters", "inputs",
            "splines", "points", "frames")
CHANNEL_SECTIONS = {"bodies", "joints", "forces", "sensors", "drivers", "outputs"}
EXPRESSION_SECTIONS = {"measures", "parameters", "inputs", "splines"}
POINT_FIELDS = {"bodies": ("position",), "joints": ("location",), "forces": ("point", "point1", "point2", "location"),
                "sensors": ("point",), "measures": ("point1", "point2")}
ORIENTATION_SECTIONS = ("bodies", "joints", "forces", "sensors", "measures")
EXPRESSION_FIELDS = {"forces": ("force", "torque"), "drivers": ("motion",)}
RESERVED = {"ground", "global", "t", "pi"}


def _get(node, key):
    if isinstance(node, yaml.MappingNode):
        for k, v in node.value:
            if isinstance(k, yaml.ScalarNode) and k.value == key:
                return k, v
    return None, None


def _items(node):
    return [(k, v) for k, v in node.value if isinstance(k, yaml.ScalarNode)] if isinstance(node, yaml.MappingNode) else []


class _Edits:
    def __init__(self, text, old, new):
        self.text, self.old, self.new, self.spans = text, old, new, []
        self.word = re.compile(rf"(?<![A-Za-z0-9_.]){re.escape(old)}(?![A-Za-z0-9_])")

    def _inner(self, node):
        """(start, end) of a scalar's value in the text: without quotes."""
        a, b = node.start_mark.index, node.end_mark.index
        if node.style in ("'", '"'):
            a, b = a + 1, b - 1
        return a, b

    def name(self, node):
        """A scalar that is the name itself (a key or a reference)."""
        if isinstance(node, yaml.ScalarNode) and node.value == self.old:
            a, b = self._inner(node)
            if self.text[a:b] == self.old:
                self.spans.append((a, b, self.new))

    def words(self, node):
        """The name as a word in an expression or formula."""
        if isinstance(node, yaml.ScalarNode) and isinstance(node.value, str):
            a, b = node.start_mark.index, node.end_mark.index
            if self.word.search(node.value):
                for m in self.word.finditer(self.text, a, b):
                    self.spans.append((m.start(), m.end(), self.new))

    def channel(self, node):
        """A result channel <element>.<quantity>."""
        if isinstance(node, yaml.ScalarNode) and node.value.startswith(self.old + "."):
            a, _ = self._inner(node)
            if self.text.startswith(self.old + ".", a):
                self.spans.append((a, a + len(self.old), self.new))

    def apply(self):
        out, last = [], 0
        for a, b, new in sorted(set(self.spans)):
            if a < last:
                continue
            out += [self.text[last:a], new]
            last = b
        return "".join(out + [self.text[last:]])


def _formulas(node, e):
    """Every formula (=...) below node."""
    if isinstance(node, yaml.ScalarNode):
        if isinstance(node.value, str) and node.value.lstrip().startswith("="):
            e.words(node)
    elif isinstance(node, yaml.MappingNode):
        for _, v in node.value:
            _formulas(v, e)
    elif isinstance(node, yaml.SequenceNode):
        for v in node.value:
            _formulas(v, e)


def defined_in(root, name):
    """The sections that define name."""
    return [s for s in SECTIONS if any(k.value == name for k, _ in _items(_get(root, s)[1]))]


def _references(root, e, section):
    """The places that refer to the element e.old of section, into e."""
    if section == "bodies":
        for s in ("joints", "forces", "measures", "sensors"):
            for _, el in _items(_get(root, s)[1]):
                for key in ("body1", "body2") + (("frame",) if s == "measures" else ()):
                    e.name(_get(el, key)[1])
    if section in ("bodies", "frames"):
        for s in ("points", "frames"):
            for _, el in _items(_get(root, s)[1]):
                e.name(_get(el, "in")[1])
    if section in ("points", "frames"):
        for s, keys in POINT_FIELDS.items():
            for _, el in _items(_get(root, s)[1]):
                for key in keys:
                    e.name(_get(el, key)[1])
        for _, el in _items(_get(root, "frames")[1]):
            e.name(_get(el, "origin")[1])
    if section == "points":
        for _, el in _items(_get(root, "points")[1]):
            for key in ("mirror", "from"):
                e.name(_get(el, key)[1])
        for _, el in _items(_get(root, "frames")[1]):
            for key in ("x", "z"):
                v = _get(el, key)[1]
                for item in v.value if isinstance(v, yaml.SequenceNode) else []:
                    e.name(item)
    if section == "frames":
        for s in ORIENTATION_SECTIONS:
            for _, el in _items(_get(root, s)[1]):
                e.name(_get(el, "orientation")[1])
    if section == "measures":
        for _, el in _items(_get(root, "drivers")[1]):
            e.name(_get(el, "measure")[1])
    if section in EXPRESSION_SECTIONS:
        for s, keys in EXPRESSION_FIELDS.items():
            for _, el in _items(_get(root, s)[1]):
                for key in keys:
                    v = _get(el, key)[1]
                    for item in v.value if isinstance(v, yaml.SequenceNode) else [v]:
                        e.words(item)
        for _, v in _items(_get(root, "outputs")[1]):
            e.words(v)
    if section == "parameters":
        _formulas(root, e)
    if section in CHANNEL_SECTIONS:
        checks = _get(root, "checks")[1]
        for kind in ("values", "curves"):
            items = _get(checks, kind)[1]
            for item in items.value if isinstance(items, yaml.SequenceNode) else []:
                for key in ("channel", "x_channel"):
                    e.channel(_get(item, key)[1])


def _spans(text, old, new, section=None):
    """The edits (an _Edits with .section) of renaming the element old to new."""
    if not IDENT.match(new or "") or new in RESERVED:
        raise ModelError(f"{new!r} is not a name for an element (letters, digits, _; not {', '.join(sorted(RESERVED))})")
    root = _compose(text)
    sections = defined_in(root, old)
    if section is not None:
        if section not in sections:
            raise ModelError(f"no {section} element named {old}")
        sections = [section]
    if not sections:
        raise ModelError(f"no element named {old}")
    if len(sections) > 1:
        raise ModelError(f"{old} names elements in {', '.join(sections)}: say which (section)")
    section = sections[0]
    taken = defined_in(root, new)
    if taken:
        raise ModelError(f"{new} is already used in {', '.join(taken)}")
    e = _Edits(text, old, new)
    e.section = section
    for k, _ in _items(_get(root, section)[1]):
        if k.value == old:
            e.name(k)
    _references(root, e, section)
    return e


def rename_text(text, old, new, section=None):
    """(text with the element old renamed to new and every reference to it, number of places
    changed, section of the element). section: bodies, joints, ... where old is defined in
    several."""
    e = _spans(text, old, new, section)
    return e.apply(), len(set(e.spans)), e.section


def _compose(text):
    try:
        return yaml.compose(text)
    except yaml.YAMLError as e:
        raise ModelError(f"the model file is not valid YAML: {e}") from None


def changes(text, old, new, section=None, references_only=False):
    """The edits of a rename as [{from, to, insert}] (character offsets in text), number of
    places, section: for the editor, which applies them to its text (undo, cursor). With
    references_only, the element is already called new (renamed by hand) and only the places
    that still say old change."""
    if not references_only:
        e = _spans(text, old, new, section)
    else:
        root = _compose(text)
        if section not in SECTIONS or new not in [k.value for k, _ in _items(_get(root, section)[1])]:
            raise ModelError(f"no {section} element named {new}")
        e = _Edits(text, old, new)
        e.section = section
        _references(root, e, section)
    spans = sorted(set(e.spans))
    return [{"from": a, "to": b, "insert": ins} for a, b, ins in spans], len(spans), e.section


def apply_changes(text, edits):
    """text with the edits of changes() made."""
    for c in sorted(edits, key=lambda c: c["from"], reverse=True):
        text = text[:c["from"]] + c["insert"] + text[c["to"]:]
    return text


def renamed(before, after):
    """Elements renamed by hand between two texts of a model file: their id is the same, the
    name not, and references to the old name are left. [{section, id, old, new, references}]."""
    from .model import ID_SECTIONS

    try:
        r0, r1 = yaml.compose(before), yaml.compose(after)
    except yaml.YAMLError:
        return []
    out = []
    for section in ID_SECTIONS:
        def ids(root):
            return {v.value: k.value for k, el in _items(_get(root, section)[1])
                    for kk, v in _items(el) if kk.value == "id" and isinstance(v, yaml.ScalarNode)}
        i0, i1 = ids(r0), ids(r1)
        names1 = {k.value for k, _ in _items(_get(r1, section)[1])}
        for i, old in i0.items():
            new = i1.get(i)
            if new is None or new == old or old in names1:
                continue
            e = _Edits(after, old, new)
            _references(r1, e, section)
            if e.spans:
                out.append({"section": section, "id": i, "old": old, "new": new, "references": len(set(e.spans))})
    return out


def rename(path, old, new, section=None):
    """Rename the element old of the model file at path to new, with every reference to it;
    (number of places changed, section). The file is written only if the result is a valid model."""
    from .model import validate

    path = Path(path)
    text = path.read_bytes().decode("utf-8")  # line ends as they are (Windows files stay CRLF)
    out, n, section = rename_text(text, old, new, section)
    try:
        validate(yaml.safe_load(out), path.resolve().parent)
    except ModelError as err:
        raise ModelError(f"renaming {old} to {new} would make the model invalid: {err}") from None
    path.write_bytes(out.encode("utf-8"))
    return n, section
