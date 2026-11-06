"""freedyn2 config and freedyn2 doctor.

  freedyn2 config                  the configuration (userdirs config.json) and where it is
  freedyn2 config KEY              one value
  freedyn2 config KEY VALUE        set it (JSON values: true, 3, ...; anything else is text)
  freedyn2 config KEY --unset      remove it
  freedyn2 doctor                  what this installation finds: version, Python, runtime,
                                   compiler, directories, MCP registration, pywebview (for support)
"""

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

from . import userdirs

KNOWN = {
    "compiler": 'C compiler for model plugins: a path or command, "zig", "msvc" or "auto"',
    "update_check": "false: never ask GitHub for a newer release",
}


def _value(text):
    try:
        return json.loads(text)
    except ValueError:
        return text


def config(key=None, value=None, unset=False, out=print):
    if key is None:
        data = userdirs.load_config()
        out(f"# {userdirs.config_dir() / 'config.json'}")
        for k in sorted(data):
            out(f"{k} = {json.dumps(data[k])}")
        for k in sorted(set(KNOWN) - set(data)):
            out(f"# {k}: {KNOWN[k]}")
        return 0
    if unset:
        userdirs.set(key, None)
        return 0
    if value is None:
        data = userdirs.load_config()
        if key not in data:
            out(f"{key} is not set" + (f" ({KNOWN[key]})" if key in KNOWN else ""))
            return 1
        out(json.dumps(data[key]) if not isinstance(data[key], str) else data[key])
        return 0
    v = _value(value)
    if key == "compiler" and isinstance(v, str) and v.lower() == "auto":
        v = None  # the default: no entry
    userdirs.set(key, v)
    out(f"{key} = {json.dumps(v)}" if v is not None else f"{key}: default")
    return 0


def _version_line(cmd, env=None, msvc=False):
    """(first line of the compiler's version, whether it runs)"""
    try:
        r = subprocess.run(cmd if msvc else [*cmd, "--version"], capture_output=True, text=True, env=env,
                           timeout=60, errors="replace")
    except (OSError, subprocess.SubprocessError) as e:
        return f"(does not run: {e})", False
    text = (r.stderr if msvc else r.stdout or r.stderr).strip()  # cl prints its version to stderr
    return (text.splitlines()[0] if text else f"(exit {r.returncode})"), msvc or r.returncode == 0


def compiler_report():
    """(line, ok)"""
    from . import plugin

    try:
        c = plugin.find_compiler()
    except RuntimeError as e:
        return f"none ({e})", False
    shown = c.cmd[0] if c.kind == "msvc" else " ".join(c.cmd)
    version, runs = _version_line(c.cmd, c.env, c.kind == "msvc")
    return f"{shown} [{c.source}] {version}", runs


def doctor(out=print):
    from . import __version__, cli, mcp_setup, plugin

    ok = True

    def line(name, text, good=True):
        nonlocal ok
        ok = ok and good
        out(f"{name:16s} {text}" if good else f"{name:16s} {text}   <-- problem")

    line("freedyn2", __version__)
    line("python", f"{sys.version.split()[0]} ({sys.executable})")
    line("installed at", str(Path(__file__).resolve().parent))
    rt = cli.find_tool("freedyn2_run", env="FREEDYN2_RUN")
    line("runtime", str(rt) if rt else "freedyn2_run not found (build it or set FREEDYN2_RUN)", rt is not None)
    text, good = compiler_report()
    line("compiler", text, good)
    cache = plugin._cache_dir()
    line("plugin cache", str(cache) if cache else "off (FREEDYN2_PLUGIN_CACHE=off)")
    line("config", str(userdirs.config_dir() / "config.json"))
    line("data", str(userdirs.data_dir()))
    line("logs", str(userdirs.log_dir()))
    if os.environ.get("FREEDYN2_HOME"):
        line("FREEDYN2_HOME", os.environ["FREEDYN2_HOME"])
    path, entry = mcp_setup.desktop_status()
    if entry:
        line("Claude Desktop", f"registered: {' '.join([entry.get('command', '?'), *entry.get('args', [])])}")
    else:
        line("Claude Desktop", f"not registered ({path}; freedyn2 setup-mcp --client claude-desktop)")
    code = mcp_setup.claude_code_status()
    line("Claude Code", {None: "claude not on the PATH", True: "registered",
                         False: "not registered (freedyn2 setup-mcp --client claude-code)"}[code])
    line("mcp", "importable" if importlib.util.find_spec("mcp") else "not installed (pip install mcp)",
         importlib.util.find_spec("mcp") is not None)
    line("pywebview", "importable" if importlib.util.find_spec("webview") else "not installed (pip install pywebview)")
    return 0 if ok else 1
