"""freedyn2 setup-mcp: register the MCP server (freedyn2 mcp) with Claude Desktop and Claude Code.

  Claude Desktop  claude_desktop_config.json (Windows %APPDATA%\\Claude, macOS
                  ~/Library/Application Support/Claude, Linux ~/.config/Claude): the entry
                  mcpServers.freedyn2 is set (or removed); everything else in the file is kept,
                  the previous file is kept as claude_desktop_config.json.bak. Claude Desktop
                  from the Microsoft Store keeps its file below %LOCALAPPDATA%\\Packages\\Claude_*;
                  an existing one there is updated too.
  Claude Code     claude mcp add --scope user freedyn2 -- <command> mcp (remove: claude mcp
                  remove --scope user freedyn2), if `claude` is on the PATH; else the command is
                  printed.

The command is the freedyn2 program this Python runs (absolute path), else this Python with
-m fdgen.cli. Idempotent: an entry that is already there is not written again. The Windows
installer calls `freedyn2 setup-mcp --client claude-desktop` (--remove when uninstalling).
Exit status 0: done (or nothing to do), 1: failed (e.g. a config file that is not JSON).
"""

import json
import os
import shlex
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

NAME = "freedyn2"
CLIENTS = ("claude-desktop", "claude-code")


def server_entry():
    """{"command": ..., "args": [...]} (and "env" if fdgen is not installed) for this installation."""
    exe = freedyn2_executable()
    if exe:
        return {"command": str(exe), "args": ["mcp"]}
    entry = {"command": sys.executable, "args": ["-m", "fdgen.cli", "mcp"]}
    pkg_parent = str(Path(__file__).resolve().parents[1])  # a source checkout: generator/ on the path
    if not _importable_from_sys_path(pkg_parent):
        entry["env"] = {"PYTHONPATH": pkg_parent}
    return entry


def _importable_from_sys_path(directory):
    import site

    dirs = set(site.getsitepackages()) | {site.getusersitepackages(), sysconfig.get_path("purelib"),
                                         sysconfig.get_path("platlib")}
    return any(d and Path(d).resolve() == Path(directory) for d in dirs)


def freedyn2_executable():
    """Absolute path of the freedyn2 program of this installation, None if there is none."""
    suffixes = (".exe", "") if sys.platform == "win32" else ("",)
    candidates = []
    if sys.argv and sys.argv[0] and Path(sys.argv[0]).stem.lower() == NAME:
        candidates.append(Path(sys.argv[0]))
    candidates.append(Path(sysconfig.get_path("scripts")) / NAME)
    candidates.append(Path(sys.executable).parent / NAME)  # venvs on Windows: Scripts beside python.exe
    for c in candidates:
        for suffix in suffixes:
            p = c.with_name(c.stem + suffix) if suffix else c
            if p.is_file():
                return p.resolve()
    return None


def desktop_config_paths(environ=None, platform=None):
    """claude_desktop_config.json of Claude Desktop (the first), and of its Store version if present."""
    environ = os.environ if environ is None else environ
    platform = platform or sys.platform
    home = Path(environ.get("HOME") or environ.get("USERPROFILE") or Path.home())
    if platform == "win32":
        appdata = Path(environ.get("APPDATA") or home / "AppData" / "Roaming")
        paths = [appdata / "Claude" / "claude_desktop_config.json"]
        local = Path(environ.get("LOCALAPPDATA") or home / "AppData" / "Local")
        paths += sorted((local / "Packages").glob("Claude_*/LocalCache/Roaming/Claude/claude_desktop_config.json"))
        return paths
    if platform == "darwin":
        return [home / "Library" / "Application Support" / "Claude" / "claude_desktop_config.json"]
    config = Path(environ.get("XDG_CONFIG_HOME") or home / ".config")
    return [config / "Claude" / "claude_desktop_config.json"]


def update_desktop_config(path, entry, remove=False, dry_run=False):
    """Set (or remove) mcpServers.freedyn2 in the file. Returns "added", "updated", "unchanged",
    "removed" or "absent". Raises ValueError if the file is not a JSON object."""
    path = Path(path)
    text = path.read_text(encoding="utf-8") if path.is_file() else ""
    try:
        data = json.loads(text) if text.strip() else {}
    except ValueError as e:
        raise ValueError(f"{path} is not valid JSON ({e}); not changed") from None
    if not isinstance(data, dict):
        raise ValueError(f"{path} is not a JSON object; not changed")
    servers = data.get("mcpServers")
    if servers is None:
        servers = {}
    if not isinstance(servers, dict):
        raise ValueError(f"{path}: mcpServers is not an object; not changed")
    if remove:
        if NAME not in servers:
            return "absent"
        del servers[NAME]
        status = "removed"
    else:
        if servers.get(NAME) == entry:
            return "unchanged"
        status = "updated" if NAME in servers else "added"
        servers[NAME] = entry
    data["mcpServers"] = servers
    if not dry_run:
        path.parent.mkdir(parents=True, exist_ok=True)
        if text:
            shutil.copyfile(path, path.with_name(path.name + ".bak"))
        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        os.replace(tmp, path)
    return status


def desktop_status(environ=None, platform=None):
    """(path, entry or None) of the Claude Desktop configuration."""
    path = desktop_config_paths(environ, platform)[0]
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return path, (data.get("mcpServers") or {}).get(NAME)
    except (OSError, ValueError, AttributeError):
        return path, None


def claude_code_commands(entry, remove=False):
    if remove:
        return [["claude", "mcp", "remove", "--scope", "user", NAME]]
    return [["claude", "mcp", "add", "--scope", "user", NAME, *_env_options(entry), "--",
             entry["command"], *entry["args"]]]


def _env_options(entry):
    return [o for k, v in entry.get("env", {}).items() for o in ("-e", f"{k}={v}")]


def _quote(args):
    return subprocess.list2cmdline(args) if sys.platform == "win32" else shlex.join(args)


def claude_code_status(timeout=20):
    """True / False if `claude mcp get freedyn2` knows the server, None if claude is not on the PATH."""
    claude = shutil.which("claude")
    if not claude:
        return None
    try:
        r = subprocess.run([claude, "mcp", "get", NAME], capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.returncode == 0


def setup_claude_code(entry, remove=False, dry_run=False, out=print):
    """Returns the exit status (0 also when claude is not on the PATH: the command is printed)."""
    claude = shutil.which("claude")
    (cmd,) = claude_code_commands(entry, remove)
    if not claude:
        out(f"Claude Code: claude is not on the PATH; run:\n  {_quote(cmd)}")
        return 0
    if dry_run:
        out(f"Claude Code: would run {_quote(cmd)}")
        return 0
    if not remove:  # idempotent: the server's entry is replaced by the current one
        subprocess.run([claude, "mcp", "remove", "--scope", "user", NAME], capture_output=True, text=True)
    r = subprocess.run([claude, *cmd[1:]], capture_output=True, text=True)
    if r.returncode != 0:
        if remove and "not found" in (r.stdout + r.stderr).lower():
            out("Claude Code: freedyn2 was not registered")
            return 0
        out(f"Claude Code: {_quote(cmd)} failed:\n{(r.stderr or r.stdout).strip()}")
        return 1
    out(f"Claude Code: {'removed' if remove else 'registered'} freedyn2 (user scope)")
    return 0


def setup_desktop(entry, remove=False, dry_run=False, out=print, paths=None):
    status = 0
    for path in paths or desktop_config_paths():
        try:
            result = update_desktop_config(path, entry, remove, dry_run)
        except (ValueError, OSError) as e:
            out(f"Claude Desktop: {e}")
            status = 1
            continue
        prefix = "would be " if dry_run and result in ("added", "updated", "removed") else ""
        text = {"unchanged": "already registered", "absent": "not registered"}.get(result, prefix + result)
        out(f"Claude Desktop: {text} ({path})")
    if status == 0 and not remove and not dry_run:
        out("Claude Desktop: restart it to load the server")
    return status


def setup_mcp(client="all", remove=False, dry_run=False, out=print):
    entry = server_entry()
    if dry_run and not remove:
        out(f"server: {_quote([entry['command'], *entry['args']])}")
    status = 0
    if client in ("claude-desktop", "all"):
        status |= setup_desktop(entry, remove, dry_run, out)
    if client in ("claude-code", "all"):
        status |= setup_claude_code(entry, remove, dry_run, out)
    return status
