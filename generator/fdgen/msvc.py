"""MSVC without a developer prompt (Windows): find Visual Studio with vswhere, run its
vcvars64.bat once and keep the environment it sets (fdgen.plugin runs cl with it).

  vswhere.exe   %ProgramFiles(x86)%\\Microsoft Visual Studio\\Installer\\vswhere.exe (comes
                with Visual Studio 2017 and later, also with the Build Tools)
  vcvars64.bat  <installation>\\VC\\Auxiliary\\Build\\vcvars64.bat
  cache         userdirs.cache_dir()/msvc_env.json: the environment after vcvars64.bat, kept as
                long as that file and the cl.exe it puts on the PATH exist

The parsing (vswhere output, `set` output) is plain Python and tested with fakes; running
vswhere and cmd.exe only happens on Windows. Not yet run on a real Windows machine: the
cmd.exe command line of capture_env and the encoding of its output are unverified.
"""

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

from . import userdirs

VC_COMPONENT = "Microsoft.VisualStudio.Component.VC.Tools.x86.x64"
CACHE_NAME = "msvc_env.json"
MARKER = "__FREEDYN2_ENV__"


def vswhere_path(environ=None):
    environ = os.environ if environ is None else environ
    root = environ.get("ProgramFiles(x86)") or environ.get("ProgramFiles") or r"C:\Program Files (x86)"
    return Path(root) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"


def parse_vswhere(text):
    """Installation paths in the output of vswhere -property installationPath (one per line)."""
    return [line.strip() for line in text.splitlines() if line.strip()]


def vcvars_of(installation):
    return Path(installation) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"


def parse_set(text):
    """The environment printed by cmd's `set` after MARKER (lines NAME=value; lines before the
    marker are vcvars' own output)."""
    lines = text.splitlines()
    if MARKER in lines:
        lines = lines[lines.index(MARKER) + 1:]
    env = {}
    for line in lines:
        name, sep, value = line.partition("=")
        if sep and name and not name.startswith("="):  # cmd's =C:=C:\... entries
            env[name] = value
    return env


def _run(cmd):
    """Output of cmd (a list, or a command line string passed to Windows as it is)."""
    r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    if r.returncode != 0:
        name = cmd if isinstance(cmd, str) else cmd[0]
        raise RuntimeError(f"{name} failed (exit {r.returncode}): {r.stderr.strip() or r.stdout.strip()}")
    return r.stdout


def find_vcvars(run=_run, environ=None):
    """vcvars64.bat of the newest Visual Studio (or Build Tools) with the C++ tools, None if there
    is none. run: runs a command and returns its output (a fake in the tests)."""
    vswhere = vswhere_path(environ)
    if not vswhere.is_file():
        return None
    out = run([str(vswhere), "-latest", "-products", "*", "-requires", VC_COMPONENT, "-property",
               "installationPath", "-utf8"])
    for inst in parse_vswhere(out):
        bat = vcvars_of(inst)
        if bat.is_file():
            return bat
    return None


def capture_env(vcvars, run=_run):
    """The environment after running vcvars (cmd.exe). A command line string: cmd.exe does not
    take the quoting of subprocess' argument lists; `&`, not `&&`, since some installations end
    vcvars with an error level although they set the environment (cl_of checks it)."""
    out = run(f'cmd.exe /d /s /c ""{vcvars}" >nul 2>&1 & echo {MARKER}& set"')
    env = parse_set(out)
    if not env:
        raise RuntimeError(f"{vcvars} set no environment")
    return env


def cl_of(env):
    """cl.exe on the PATH of env, None if not there."""
    path = next((v for k, v in env.items() if k.upper() == "PATH"), "")
    return shutil.which("cl", path=path)


def environment(run=_run, environ=None, cache=True):
    """(cl.exe, environment) of MSVC; None if Visual Studio with the C++ tools is not installed.
    The environment is kept in the cache directory: vcvars64.bat takes seconds."""
    f = userdirs.cache_dir() / CACHE_NAME
    if cache:
        try:
            data = json.loads(f.read_text(encoding="utf-8"))
            if Path(data["vcvars"]).is_file() and data["cl"] and Path(data["cl"]).is_file():
                return data["cl"], data["env"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
    vcvars = find_vcvars(run, environ)
    if vcvars is None:
        return None
    env = capture_env(vcvars, run)
    cl = cl_of(env)
    if cl is None:
        raise RuntimeError(f"no cl.exe on the PATH after {vcvars}")
    if cache:
        tmp = f.with_suffix(".tmp")
        tmp.write_text(json.dumps({"vcvars": str(vcvars), "cl": cl, "env": env}), encoding="utf-8")
        os.replace(tmp, f)
    return cl, env


def available():
    return sys.platform == "win32"
