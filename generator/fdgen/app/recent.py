"""Recently opened models: data_dir()/recent.json, the newest first, at most MAX."""

import json
import os
import time
from pathlib import Path

from .. import userdirs

MAX = 15


def _file():
    return userdirs.data_dir() / "recent.json"


def _load():
    try:
        data = json.loads(_file().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    return [e for e in data if isinstance(e, dict) and isinstance(e.get("path"), str)] if isinstance(data, list) else []


def _save(entries):
    f = _file()
    tmp = f.with_suffix(".tmp")
    tmp.write_text(json.dumps(entries[:MAX], indent=1) + "\n", encoding="utf-8")
    os.replace(tmp, f)


def _same(a, b):
    return os.path.normcase(os.path.normpath(a)) == os.path.normcase(os.path.normpath(b))


def add(path):
    path = str(Path(path).resolve())
    entries = [e for e in _load() if not _same(e["path"], path)]
    _save([{"path": path, "opened": time.time()}] + entries)


def remove(path):
    _save([e for e in _load() if not _same(e["path"], str(path))])


def clear():
    _save([])


def items():
    """[{path, name, folder, opened, exists}], the newest first."""
    out = []
    for e in _load()[:MAX]:
        p = Path(e["path"])
        out.append({"path": str(p), "name": p.name, "folder": str(p.parent), "opened": e.get("opened", 0),
                    "exists": p.is_file()})
    return out
