"""Compiler selection (CC, configuration, PATH, MSVC via vswhere, ziglang), the plugin cache
directory, freedyn2 setup-mcp, config and doctor. Fakes for paths, configs and programs;
FREEDYN2_HOME isolates the user directories."""

import ctypes
import json
import os
import stat
import sys

import pytest

from fdgen import cli, doctor, mcp_setup, msvc, plugin, userdirs


@pytest.fixture(autouse=True)
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("FREEDYN2_HOME", str(tmp_path / "home"))
    monkeypatch.delenv("CC", raising=False)
    monkeypatch.delenv("FREEDYN2_PLUGIN_CACHE", raising=False)
    return tmp_path / "home"


def no_path(monkeypatch, found=()):
    monkeypatch.setattr(plugin.shutil, "which", lambda name, *a, **k: f"/usr/bin/{name}" if name in found else None)


# --- compiler selection -------------------------------------------------------------------

def test_cc_wins(monkeypatch):
    monkeypatch.setenv("CC", "mycc")
    userdirs.set("compiler", "othercc")
    c = plugin.find_compiler()
    assert (c.cmd, c.kind, c.source) == (["mycc"], "gcc", "CC")


def test_cc_command_line(monkeypatch):
    monkeypatch.setenv("CC", "zig cc")
    c = plugin.find_compiler()
    assert (c.cmd, c.kind) == (["zig", "cc"], "zig")


def test_config_before_path(monkeypatch):
    no_path(monkeypatch, found=("gcc",))
    userdirs.set("compiler", "/opt/tools/clang")
    c = plugin.find_compiler()
    assert (c.cmd, c.source) == (["/opt/tools/clang"], "config")


def test_auto_and_path_order(monkeypatch):
    userdirs.set("compiler", "auto")
    no_path(monkeypatch, found=("gcc", "clang"))
    assert plugin.find_compiler().cmd == ["gcc"]
    no_path(monkeypatch, found=("cl",))
    c = plugin.find_compiler()
    assert (c.cmd, c.kind) == (["cl"], "msvc")


def test_ziglang_after_path(monkeypatch):
    no_path(monkeypatch)
    monkeypatch.setattr(plugin, "ziglang_available", lambda: True)
    c = plugin.find_compiler()
    assert (c.cmd, c.kind) == ([sys.executable, "-m", "ziglang", "cc"], "zig")
    monkeypatch.setattr(plugin, "ziglang_available", lambda: False)
    with pytest.raises(RuntimeError, match="ziglang"):
        plugin.find_compiler()


def test_config_zig(monkeypatch):
    userdirs.set("compiler", "zig")
    monkeypatch.setattr(plugin, "ziglang_available", lambda: False)
    no_path(monkeypatch, found=("zig",))
    assert plugin.find_compiler().cmd == ["/usr/bin/zig", "cc"]
    no_path(monkeypatch)
    with pytest.raises(RuntimeError, match="pip install ziglang"):
        plugin.find_compiler()


def test_config_msvc(monkeypatch):
    userdirs.set("compiler", "msvc")
    if sys.platform != "win32":
        with pytest.raises(RuntimeError, match="Windows only"):
            plugin.find_compiler()
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(msvc, "environment", lambda: (r"C:\VS\cl.exe", {"PATH": r"C:\VS"}))
    c = plugin.find_compiler()
    assert (c.cmd, c.kind, c.env) == ([r"C:\VS\cl.exe"], "msvc", {"PATH": r"C:\VS"})
    monkeypatch.setattr(msvc, "environment", lambda: None)
    with pytest.raises(RuntimeError, match="vswhere"):
        plugin.find_compiler()


def test_msvc_auto_on_windows_after_path(monkeypatch):
    no_path(monkeypatch)
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(msvc, "environment", lambda: (r"C:\VS\cl.exe", {"PATH": r"C:\VS"}))
    monkeypatch.setattr(plugin, "ziglang_available", lambda: True)
    assert plugin.find_compiler().kind == "msvc"
    monkeypatch.setattr(msvc, "environment", lambda: None)
    assert plugin.find_compiler().kind == "zig"


def record_runs(monkeypatch):
    runs = []
    monkeypatch.setattr(plugin, "_run", lambda cmd, env=None: runs.append((cmd, env)))
    return runs


def test_command_list_and_env(tmp_path, monkeypatch):
    src = tmp_path / "f.c"
    src.write_text("double f(double x) { return x; }\n")
    runs = record_runs(monkeypatch)
    c = plugin.Compiler(["python", "-m", "ziglang", "cc"], env={"X": "1"})
    plugin.compile_library([src], tmp_path / "f.so", c, cache=False)
    assert c.kind == "zig"
    assert all(cmd[:4] == c.cmd and env == {"X": "1"} for cmd, env in runs)
    assert "-mcpu=baseline" in runs[0][0] and "-shared" in runs[1][0]


def test_zig_windows_target_and_def(tmp_path, monkeypatch):
    src = tmp_path / "f.c"
    src.write_text("int f(void) { return 1; }\n")
    runs = record_runs(monkeypatch)
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(plugin.platform, "machine", lambda: "AMD64")
    plugin.compile_library([src], tmp_path / "f.dll", plugin.Compiler(["zig", "cc"]), exports=plugin.PLUGIN_EXPORTS,
                           cache=False)
    compile_cmd, link_cmd = runs[0][0], runs[1][0]
    for cmd in (compile_cmd, link_cmd):
        assert cmd[cmd.index("-target") + 1] == "x86_64-windows-gnu"
    assert "-fPIC" not in compile_cmd and "-Wl,--export-all-symbols" not in link_cmd
    assert (tmp_path / "f.def").read_text().split() == [
        "EXPORTS", "fd_plugin_abi", "fd_plugin_element_types", "DATA", "fd_plugin_n_element_types", "DATA"]
    assert link_cmd[-1] == str(tmp_path / "f.def")


def test_cache_key_has_the_command(tmp_path, monkeypatch):
    src = tmp_path / "f.c"
    src.write_text("int f(void) { return 1; }\n")

    def fake(cmd, env=None):
        if "-shared" in cmd:
            (tmp_path / "f.so").write_bytes(" ".join(cmd[:2]).encode())

    monkeypatch.setattr(plugin, "_run", fake)
    plugin.compile_library([src], tmp_path / "f.so", plugin.Compiler(["gcc"]))
    plugin.compile_library([src], tmp_path / "f.so", plugin.Compiler(["zig", "cc"]))
    assert len(list((userdirs.cache_dir() / "plugins").glob("*.so"))) == 2


def test_plugin_cache_dir(tmp_path, monkeypatch, home):
    assert plugin._cache_dir() == home / "cache" / "plugins"
    monkeypatch.setenv("FREEDYN2_PLUGIN_CACHE", str(tmp_path / "c"))
    assert plugin._cache_dir() == tmp_path / "c"
    monkeypatch.setenv("FREEDYN2_PLUGIN_CACHE", "off")
    assert plugin._cache_dir() is None


@pytest.mark.skipif(not plugin.ziglang_available(), reason="ziglang not installed (pip install ziglang)")
def test_zig_builds_a_library(tmp_path, monkeypatch):
    """A library from zig (python -m ziglang cc) that loads; a Windows DLL cross-compiled with the
    registry exports (it cannot be run here)."""
    monkeypatch.setenv("FREEDYN2_PLUGIN_CACHE", "off")
    monkeypatch.setenv("CC", "zig")
    src = tmp_path / "f.c"
    src.write_text("double twice(double x) { return 2 * x; }\nconst int fd_plugin_n_element_types = 3;\n"
                   "const char* fd_plugin_abi(void) { return \"x\"; }\nconst int fd_plugin_element_types[1] = {0};\n")
    if sys.platform != "win32":
        plugin.compile_library([src], tmp_path / "f.so")
        lib = ctypes.CDLL(str(tmp_path / "f.so"))
        lib.twice.restype, lib.twice.argtypes = ctypes.c_double, [ctypes.c_double]
        assert lib.twice(1.5) == 3.0
        monkeypatch.setattr(sys, "platform", "win32")
    plugin.compile_library([src], tmp_path / "f.dll", exports=plugin.PLUGIN_EXPORTS)
    dll = (tmp_path / "f.dll").read_bytes()
    assert dll[:2] == b"MZ"
    for name in (b"fd_plugin_abi", b"fd_plugin_element_types", b"fd_plugin_n_element_types"):
        assert name + b"\0" in dll


# --- MSVC via vswhere (fakes: no Windows needed) ---------------------------------------------

def test_parse_vswhere_and_set():
    assert msvc.parse_vswhere("C:\\VS\\2022\\BuildTools\r\n\r\n") == ["C:\\VS\\2022\\BuildTools"]
    out = f"[vcvarsall.bat] Environment initialized\n{msvc.MARKER}\n=C:=C:\\x\nPath=C:\\VC\\bin;C:\\Windows\nINCLUDE=C:\\VC\\inc\nA=b=c\n"
    assert msvc.parse_set(out) == {"Path": "C:\\VC\\bin;C:\\Windows", "INCLUDE": "C:\\VC\\inc", "A": "b=c"}


def fake_visual_studio(tmp_path):
    pf = tmp_path / "pf86"
    vswhere = pf / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    vswhere.parent.mkdir(parents=True)
    vswhere.write_text("")
    inst = tmp_path / "VS"
    bat = msvc.vcvars_of(inst)
    bat.parent.mkdir(parents=True)
    bat.write_text("")
    bindir = tmp_path / "VC" / "bin"
    bindir.mkdir(parents=True)
    for name in ("cl", "cl.exe"):
        (bindir / name).write_text("")
        (bindir / name).chmod(stat.S_IRWXU)
    calls = []

    def run(cmd):
        calls.append(cmd)
        if isinstance(cmd, list):
            assert cmd[0] == str(vswhere) and msvc.VC_COMPONENT in cmd
            return f"{inst}\n"
        assert str(bat) in cmd
        return f"noise\n{msvc.MARKER}\nPATH={bindir}{os.pathsep}/usr/bin\nINCLUDE=inc\n"

    return {"ProgramFiles(x86)": str(pf)}, bat, bindir, run, calls


def test_find_vcvars_and_environment(tmp_path):
    environ, bat, bindir, run, calls = fake_visual_studio(tmp_path)
    assert msvc.find_vcvars(run, environ) == bat
    assert msvc.find_vcvars(run, {"ProgramFiles(x86)": str(tmp_path / "none")}) is None
    cl, env = msvc.environment(run, environ)
    assert os.path.dirname(cl) == str(bindir) and env["INCLUDE"] == "inc"
    n = len(calls)
    assert msvc.environment(run, environ) == (cl, env)  # from the cache
    assert len(calls) == n
    assert json.loads((userdirs.cache_dir() / msvc.CACHE_NAME).read_text())["vcvars"] == str(bat)


def test_environment_without_cl(tmp_path):
    environ, bat, bindir, run, calls = fake_visual_studio(tmp_path)
    with pytest.raises(RuntimeError, match="no cl.exe"):
        msvc.environment(lambda cmd: run(cmd) if isinstance(cmd, list) else f"{msvc.MARKER}\nPATH=/nowhere\n",
                         environ, cache=False)


# --- setup-mcp --------------------------------------------------------------------------------

ENTRY = {"command": "/opt/freedyn2/bin/freedyn2", "args": ["mcp"]}


def test_desktop_config_merge(tmp_path):
    f = tmp_path / "Claude" / "claude_desktop_config.json"
    assert mcp_setup.update_desktop_config(f, ENTRY) == "added"  # created
    assert json.loads(f.read_text()) == {"mcpServers": {"freedyn2": ENTRY}}
    f.write_text(json.dumps({"theme": "dark", "mcpServers": {"other": {"command": "x"}}}))
    assert mcp_setup.update_desktop_config(f, ENTRY) == "added"
    data = json.loads(f.read_text())
    assert data == {"theme": "dark", "mcpServers": {"other": {"command": "x"}, "freedyn2": ENTRY}}
    assert json.loads((tmp_path / "Claude" / "claude_desktop_config.json.bak").read_text())["theme"] == "dark"
    text = f.read_text()
    assert mcp_setup.update_desktop_config(f, ENTRY) == "unchanged"
    assert f.read_text() == text
    assert mcp_setup.update_desktop_config(f, dict(ENTRY, command="/new")) == "updated"
    assert mcp_setup.update_desktop_config(f, ENTRY, dry_run=True) == "updated"
    assert json.loads(f.read_text())["mcpServers"]["freedyn2"]["command"] == "/new"
    assert mcp_setup.update_desktop_config(f, ENTRY, remove=True) == "removed"
    assert json.loads(f.read_text()) == {"theme": "dark", "mcpServers": {"other": {"command": "x"}}}
    assert mcp_setup.update_desktop_config(f, ENTRY, remove=True) == "absent"


def test_desktop_config_not_json(tmp_path):
    f = tmp_path / "claude_desktop_config.json"
    f.write_text("{not json")
    with pytest.raises(ValueError, match="not valid JSON"):
        mcp_setup.update_desktop_config(f, ENTRY)
    assert f.read_text() == "{not json"
    assert mcp_setup.setup_desktop(ENTRY, paths=[f], out=lambda s: None) == 1


def test_desktop_config_paths(tmp_path):
    win = {"APPDATA": str(tmp_path / "Roaming"), "LOCALAPPDATA": str(tmp_path / "Local"), "USERPROFILE": str(tmp_path)}
    store = tmp_path / "Local" / "Packages" / "Claude_abc" / "LocalCache" / "Roaming" / "Claude"
    store.mkdir(parents=True)
    (store / "claude_desktop_config.json").write_text("{}")
    assert mcp_setup.desktop_config_paths(win, "win32") == [
        tmp_path / "Roaming" / "Claude" / "claude_desktop_config.json", store / "claude_desktop_config.json"]
    assert mcp_setup.desktop_config_paths({"HOME": str(tmp_path)}, "darwin") == [
        tmp_path / "Library" / "Application Support" / "Claude" / "claude_desktop_config.json"]
    assert mcp_setup.desktop_config_paths({"HOME": str(tmp_path)}, "linux") == [
        tmp_path / ".config" / "Claude" / "claude_desktop_config.json"]


def test_server_entry_fallback(monkeypatch):
    monkeypatch.setattr(mcp_setup, "freedyn2_executable", lambda: None)
    e = mcp_setup.server_entry()
    assert e["command"] == sys.executable and e["args"] == ["-m", "fdgen.cli", "mcp"]
    monkeypatch.setattr(mcp_setup, "freedyn2_executable", lambda: "/x/freedyn2")
    assert mcp_setup.server_entry() == {"command": "/x/freedyn2", "args": ["mcp"]}


def test_claude_code(monkeypatch):
    lines, calls = [], []
    monkeypatch.setattr(mcp_setup.shutil, "which", lambda name: None)
    assert mcp_setup.setup_claude_code(ENTRY, out=lines.append) == 0
    assert "claude mcp add --scope user freedyn2 -- /opt/freedyn2/bin/freedyn2 mcp" in lines[0]

    class R:
        returncode, stdout, stderr = 0, "", ""

    monkeypatch.setattr(mcp_setup.shutil, "which", lambda name: "/bin/claude")
    monkeypatch.setattr(mcp_setup.subprocess, "run", lambda cmd, **k: calls.append(cmd) or R())
    assert mcp_setup.setup_claude_code(ENTRY, out=lines.append) == 0
    assert calls == [["/bin/claude", "mcp", "remove", "--scope", "user", "freedyn2"],
                     ["/bin/claude", "mcp", "add", "--scope", "user", "freedyn2", "--", ENTRY["command"], "mcp"]]
    calls.clear()
    assert mcp_setup.setup_claude_code(ENTRY, remove=True, out=lines.append) == 0
    assert calls == [["/bin/claude", "mcp", "remove", "--scope", "user", "freedyn2"]]
    calls.clear()
    assert mcp_setup.setup_claude_code(ENTRY, dry_run=True, out=lines.append) == 0
    assert calls == []
    R.returncode, R.stderr = 1, "boom"
    assert mcp_setup.setup_claude_code(ENTRY, out=lines.append) == 1


def test_cli_setup_mcp_desktop(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    monkeypatch.setattr(mcp_setup, "server_entry", lambda: ENTRY)
    f = tmp_path / "cfg" / "Claude" / "claude_desktop_config.json"
    assert cli.main(["setup-mcp", "--client", "claude-desktop", "--dry-run"]) == 0
    assert not f.exists()
    assert cli.main(["setup-mcp", "--client", "claude-desktop"]) == 0
    assert json.loads(f.read_text())["mcpServers"]["freedyn2"] == ENTRY
    assert cli.main(["setup-mcp", "--client", "claude-desktop"]) == 0
    assert "already registered" in capsys.readouterr().out
    assert cli.main(["setup-mcp", "--client", "claude-desktop", "--remove"]) == 0
    assert json.loads(f.read_text())["mcpServers"] == {}


# --- config and doctor ------------------------------------------------------------------------

def test_cli_config(capsys, home):
    assert cli.main(["config", "compiler", "zig"]) == 0
    assert userdirs.get("compiler") == "zig"
    assert cli.main(["config", "update_check", "false"]) == 0
    assert userdirs.get("update_check") is False
    capsys.readouterr()
    assert cli.main(["config", "compiler"]) == 0
    assert capsys.readouterr().out.strip() == "zig"
    assert cli.main(["config"]) == 0
    out = capsys.readouterr().out
    assert 'compiler = "zig"' in out and str(home) in out
    assert cli.main(["config", "compiler", "auto"]) == 0
    assert userdirs.get("compiler") is None
    assert cli.main(["config", "compiler"]) == 1
    userdirs.set("x", 1)
    assert cli.main(["config", "x", "--unset"]) == 0
    assert "x" not in userdirs.load_config()


def test_doctor(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    monkeypatch.setattr(mcp_setup, "claude_code_status", lambda: None)
    monkeypatch.setenv("CC", "no-such-compiler-xyz")
    lines = []
    assert doctor.doctor(out=lines.append) == 1  # a compiler that does not run: reported
    text = "\n".join(lines)
    for key in ("freedyn2", "python", "runtime", "compiler", "plugin cache", "Claude Desktop", "Claude Code",
                "pywebview"):
        assert key in text
    assert "no-such-compiler-xyz" in text
