"""MkDocs hook: key tables from the JSON Schema of the model file, so the reference pages of the
documentation never disagree with what the generator accepts.

  <!-- keys: force -->       the keys of $defs/force (one row per key: type, description)
  <!-- keys: model -->       the top-level keys of the model file
  <!-- keys: road -->        the keys of a top-level section given as an object (road, checks, ...)

and LIVE/ in a page: the relative path to the live examples (docs/live, tools/docs_examples.py),
right for directory URLs (the site) and flat ones (a preview) alike.
"""

import json
import re
from pathlib import Path

SCHEMA = Path(__file__).resolve().parents[1] / "schema" / "model.schema.json"


def load_schema():
    """The schema with what active extensions add (fdgen.extensions), else the core's file."""
    import sys

    sys.path.insert(0, str(SCHEMA.parents[1] / "generator"))
    try:
        from fdgen.model import load_schema as load
    except ImportError:
        return json.loads(SCHEMA.read_text(encoding="utf-8"))
    return load()
MARKER = re.compile(r"<!--\s*keys:\s*([\w.]+)\s*-->")


def _type(spec):
    if "const" in spec:
        return f"`{spec['const']}`"
    if "enum" in spec:
        return " \\| ".join(f"`{v}`" for v in spec["enum"])
    if "$ref" in spec:
        return {"vec3": "[x, y, z]", "point": "[x, y, z] or a point name",
                "orientation": "list of 4 or a frame name"}.get(spec["$ref"].rsplit("/", 1)[1], spec["$ref"].rsplit("/", 1)[1])
    t = spec.get("type")
    if t == "array":
        n = spec.get("minItems")
        return f"list of {n}" if n and n == spec.get("maxItems") else "list"
    if isinstance(t, list):
        return " or ".join(t)
    if t:
        return t
    if "anyOf" in spec or "oneOf" in spec:
        return "number or list" if "number" in json.dumps(spec) else "see text"
    return "number or `=formula`" if not spec else "see text"


def table(name, schema=None):
    schema = schema or load_schema()
    if name == "model":
        node, required = schema, set(schema.get("required", []))
    elif name in schema.get("$defs", {}):
        node = schema["$defs"][name]
        required = set(node.get("required", []))
    else:
        node = schema["properties"][name]
        node = node.get("additionalProperties", node) if "properties" not in node else node
        if "$ref" in node:
            node = schema["$defs"][node["$ref"].rsplit("/", 1)[1]]
        required = set(node.get("required", []))
    rows = ["| key | value | |", "|---|---|---|"]
    for key, spec in node.get("properties", {}).items():
        desc = spec.get("description", "").replace("|", "\\|").replace("\n", " ")
        if "default" in spec and "Default" not in desc:
            desc = f"{desc} Default `{spec['default']}`.".strip()
        rows.append(f"| `{key}`{' *(required)*' if key in required else ''} | {_type(spec)} | {desc} |")
    return "\n".join(rows)


def on_page_markdown(markdown, page, config, files):
    schema = load_schema()
    markdown = MARKER.sub(lambda m: table(m.group(1), schema), markdown)
    return markdown.replace('src="LIVE/', f'src="{"../" * page.url.count("/")}live/')
