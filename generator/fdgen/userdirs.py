"""Where freedyn2 keeps per-user files, and the user configuration.

  config_dir()  settings (config.json)      Windows %APPDATA%\\freedyn2, else $XDG_CONFIG_HOME/freedyn2
  data_dir()    state (recent models, ...)  Windows %LOCALAPPDATA%\\freedyn2, else $XDG_DATA_HOME/freedyn2
  cache_dir()   caches (model plugins, ...) Windows %LOCALAPPDATA%\\freedyn2\\cache, else $XDG_CACHE_HOME/freedyn2
  log_dir()     log files of the app        data_dir()/logs

FREEDYN2_HOME, when set, puts all of them below one directory (tests, portable installs).
The directories are created on first use. config.json is a flat JSON object; get() reads one
key with a default, set() writes one (unknown keys are kept). Known keys:

  compiler            C compiler for model plugins (a path or a name on the PATH), like CC
  update_check        false: never ask GitHub for a newer release
  last_update_check   time of the last check (seconds since the epoch), set by fdgen.update
"""

import json
import os
import sys
from pathlib import Path


def _base(win_var, xdg_var, xdg_default):
    home = os.environ.get("FREEDYN2_HOME")
    if home:
        return Path(home)
    if sys.platform == "win32":
        root = os.environ.get(win_var) or Path.home() / "AppData" / ("Roaming" if win_var == "APPDATA" else "Local")
        return Path(root) / "freedyn2"
    return Path(os.environ.get(xdg_var) or Path.home() / xdg_default) / "freedyn2"


def _made(p):
    p.mkdir(parents=True, exist_ok=True)
    return p


def config_dir():
    return _made(_base("APPDATA", "XDG_CONFIG_HOME", ".config"))


def data_dir():
    return _made(_base("LOCALAPPDATA", "XDG_DATA_HOME", ".local/share"))


def cache_dir():
    if os.environ.get("FREEDYN2_HOME") or sys.platform == "win32":
        return _made(_base("LOCALAPPDATA", "XDG_CACHE_HOME", ".cache") / "cache")
    return _made(_base("LOCALAPPDATA", "XDG_CACHE_HOME", ".cache"))


def log_dir():
    return _made(data_dir() / "logs")


def _config_file():
    return config_dir() / "config.json"


def load_config():
    try:
        data = json.loads(_config_file().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def get(key, default=None):
    return load_config().get(key, default)


def set(key, value):  # noqa: A001  (fdgen.userdirs.set reads well)
    data = load_config()
    if value is None:
        data.pop(key, None)
    else:
        data[key] = value
    f = _config_file()
    tmp = f.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, f)
