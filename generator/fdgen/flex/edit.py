"""Edits of the model file for flexible bodies (the viewer's inspector): an explicit interface in
bodies.<body>.flex.interfaces, set or removed.

The flex block of the body is written anew in its style (flow {...} or block); the rest of the
file stays as it is, comments inside that block are lost.
"""

import numpy as np
import yaml

from ..errors import ModelError

LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


class _Dumper(yaml.SafeDumper):
    """Block style, lists of numbers in flow style ([x, y, z])."""


def _list(dumper, data):
    flow = all(isinstance(x, (int, float)) for x in data)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow)


_Dumper.add_representer(list, _list)


def _child(node, key):
    if not isinstance(node, yaml.MappingNode):
        return None
    for k, v in node.value:
        if isinstance(k, yaml.ScalarNode) and k.value == key:
            return v
    return None


def set_interface(text, body, point, radius=None, kind=None, axis=None, remove=False):
    """text with the interface at point (global, t = 0) of the flexible body set: an entry in its
    flex.interfaces with the given radius, type (rbe2 / rbe3) and axis (None: not given, the
    default), or, with remove, the entry taken out (the interface is made as by default again)."""
    root = yaml.compose(text, Loader=LOADER)
    node = _child(_child(_child(root, "bodies"), body), "flex")
    if node is None or not isinstance(node, yaml.MappingNode):
        raise ModelError(f"body {body} has no flex block")
    flex = yaml.load(text, Loader=LOADER)["bodies"][body]["flex"]
    if not ("material" in flex):
        raise ModelError(f"body {body}: interfaces can be set for a body meshed from its STL (flex.material); "
                         "for an FE model change its RBE2 / RBE3")
    point = [float(x) for x in point]
    items = [i for i in flex.get("interfaces") or [] if isinstance(i, dict)]
    size = max(1e-12, max(abs(x) for x in point))
    same = [k for k, i in enumerate(items) if np.allclose(np.asarray(i.get("point", [np.nan] * 3), float), point,
                                                          rtol=0, atol=1e-9 * size + 1e-12)]
    entry = {"point": point}
    if radius is not None:
        entry["radius"] = float(radius)
    if kind is not None:
        if kind not in ("rbe2", "rbe3"):
            raise ModelError(f"interface type {kind}: rbe2 or rbe3")
        entry["type"] = kind
    if axis is not None:
        entry["axis"] = [float(x) for x in axis]
    if same:
        if remove:
            items.pop(same[0])
        else:
            items[same[0]] = entry
    elif not remove:
        items.append(entry)
    if items:
        flex["interfaces"] = items
    else:
        flex.pop("interfaces", None)
    start, end = node.start_mark.index, node.end_mark.index
    if node.flow_style:
        new = yaml.safe_dump(flex, default_flow_style=True, sort_keys=False, width=10 ** 9).strip()
    else:
        col = node.start_mark.column
        lines = yaml.dump(flex, Dumper=_Dumper, default_flow_style=False, sort_keys=False,
                          width=10 ** 9).rstrip("\n").split("\n")
        old = text[start:end]
        new = ("\n" + " " * col).join(lines) + old[len(old.rstrip()):]  # keep the line end / blank lines after it
    return text[:start] + new + text[end:]
