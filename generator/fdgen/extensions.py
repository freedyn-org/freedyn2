"""Extensions: packages that add to FreeDyn 2.0 without changing it (force element types,
schema keys, text for the assistant), installed next to it. A guide with a complete example:
docs/extensions.md, examples/extension.

An extension is a Python package that declares an entry point in the group
"freedyn2.extensions" pointing to an Extension instance:

  [project.entry-points."freedyn2.extensions"]
  my-tires = "my_tires:extension"

FREEDYN2_EXTENSIONS (module names, comma separated, each with an attribute `extension`) adds
extensions not installed as packages, e.g. while developing one (with PYTHONPATH).

What an extension can do (all optional; the defaults do nothing):

  schema(schema)                  add keys and types to the JSON schema of the model file
  validate_force(f, base_dir)     check / complete a force after the core rules (read its data files)
  force_element(force, bodies)    take over a force: (element type, [bodies], p) instead of the core's
  element_types                   {element type: build(rep)} of its element types, rep "lie" or
                                  "euler"; they are generated and compiled into the model plugin
                                  (a C compiler is needed, as for expression forces)
  road_offsets                    {element type: offset of the runtime road plane in its p}
  wheel(force)                    (radius, width) of the wheel of a tire force it handles
  conventions                     text for the assistant's guide (MCP)

The core's own element types are compiled into the runtime; an extension's are compiled per
model, cached like expression forces (fdgen.plugin).
"""

import functools
import importlib
import os
from importlib import metadata


class Extension:
    """Base of an extension: override what it adds."""

    name = "extension"
    element_types = {}
    road_offsets = {}
    conventions = ""

    def schema(self, schema):
        pass

    def validate_force(self, f, base_dir):
        return f

    def force_element(self, force, bodies):
        return None

    def wheel(self, force):
        return None


@functools.lru_cache(maxsize=None)
def extensions():
    """The extensions found (entry points, then FREEDYN2_EXTENSIONS), each once."""
    found = []
    try:
        points = metadata.entry_points(group="freedyn2.extensions")
    except TypeError:  # Python < 3.10
        points = metadata.entry_points().get("freedyn2.extensions", [])
    for p in points:
        found.append(p.load())
    for name in filter(None, (s.strip() for s in os.environ.get("FREEDYN2_EXTENSIONS", "").split(","))):
        found.append(importlib.import_module(name).extension)
    out = []
    for ext in found:
        if all(ext is not e for e in out):
            out.append(ext)
    return tuple(out)


def element_type(kind):
    """(extension, build) of an extension element type, or None."""
    base = kind.removesuffix("_euler")
    for ext in extensions():
        if base in ext.element_types:
            return ext, ext.element_types[base]
    return None


@functools.lru_cache(maxsize=None)
def build_element(kind):
    """ElementFunctions of an extension element type (kept per process: the symbolic work and the
    generated code are the same for every model)."""
    ext, build = element_type(kind)
    elem = build("euler" if kind.endswith("_euler") else "lie")
    return elem


def road_offset(kind):
    """Offset of the runtime road plane in the parameters of an element type, or None."""
    base = kind.removesuffix("_euler")
    for ext in extensions():
        if base in ext.road_offsets:
            return ext.road_offsets[base]
    return None


def wheel(force):
    """(radius, width) of the wheel of a tire force according to an extension, or None."""
    for ext in extensions():
        w = ext.wheel(force)
        if w:
            return w
    return None
