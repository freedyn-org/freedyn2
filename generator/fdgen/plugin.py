"""Model plugin: the model-specific element types of a model, compiled into a shared library.

Expression forces (fdgen.expression_forces), drivers (fdgen.drivers) and outputs
(fdgen.outputs) cannot be library element types: their laws are part of the model. The element types of extensions
(fdgen.extensions) are not in the runtime either and are compiled here too. The generator builds one element type per
force and driver, generates its C code with CasADi and compiles all of them into
`<model>_plugin.so` next to the .fdm file, which the runtime loads at model load (registry
layout checked with FD_REGISTRY_ABI). Derivatives are exact, as for the library types.

The C files are compiled in parallel, and the library is cached by the hash of its sources
and the compiler command (directory FREEDYN2_PLUGIN_CACHE, default userdirs.cache_dir()/plugins:
~/.cache/freedyn2/plugins, on Windows %LOCALAPPDATA%\\freedyn2\\cache\\plugins; "off" disables
the cache): constants are parameters, so a model whose parameters change
reuses its plugin. Within a process, the elements themselves (their derivatives and generated
code) are kept too (fdgen.custom_element.ELEMENT_CACHE): rebuilding a model with other
parameter values, as the viewer does for every tuned value, skips the symbolic work.

Compiler (find_compiler): CC, else the configured one (userdirs key `compiler`), else the first
of cc, gcc, clang, cl on the PATH, else (Windows) MSVC found with vswhere (fdgen.msvc), else
zig if the Python package ziglang is installed (pip install freedyn2[compiler]). CC and the
configuration take a path or a command ("zig cc"), or "zig" (ziglang, else zig on the PATH),
"msvc" (Visual Studio without a developer prompt) or "auto". On Windows the library is a .dll;
with MSVC (cl) the registry symbols are exported explicitly, with MinGW gcc all symbols are;
zig targets the MinGW ABI (<arch>-windows-gnu) and is linked like MinGW gcc (cross-compiled
and checked for its exports on Linux; a Windows run of a zig-built plugin is not verified yet).
"""

import hashlib
import importlib.util
import os
import platform
import shlex
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from . import extensions, userdirs
from .codegen import code_generator, write_element, write_registry
from .custom_element import PluginElement
from .drivers import driver_elements
from .expression_forces import expression_elements
from .outputs import output_elements


def extension_elements(model):
    """[(force name, element type, [bodies], p values, ElementFunctions)] of the forces an
    extension takes over (fdgen.extensions); its element types go into the model plugin."""
    out = []
    for force in model.get("forces", []):
        for ext in extensions.extensions():
            r = ext.force_element(force, model["bodies"])
            if r:
                kind, names, p = r
                if all(model["bodies"][b].get("element") == "rigid_body_euler" for b in names):
                    kind += "_euler"
                out.append((force["name"], kind, names, list(p), extensions.build_element(kind)))
                break
    return out


def plugin_elements(model):
    """[(name, element type, [bodies], p values, ElementFunctions)] of all plugin elements
    (PluginElement: .inputs maps the runtime inputs an element uses to their offsets in p).
    Elements whose generated code is the same up to the type name (the same law on other
    bodies, other parameter values) share one element type: a model with many copies of a
    subsystem compiles each law once, and the runtime runs the same code for all of them."""
    out, types = [], {}
    for pe in (expression_elements(model) + driver_elements(model) + extension_elements(model)
               + output_elements(model)):
        name, tname, bodies, p, elem = pe
        inputs = getattr(pe, "inputs", {})
        key = elem.__dict__.get("code_key")  # elements are kept between builds (custom_element)
        if key is None:
            key = hashlib.sha256(code_generator("x", elem, header=False).dump().replace(tname, "@").encode()).hexdigest()
            elem.code_key = key
        tname, elem = types.setdefault(key, (tname, elem))
        out.append(PluginElement(name, tname, bodies, p, elem, inputs))
    return out


LIB_SUFFIX = ".dll" if sys.platform == "win32" else ".so"
PLUGIN_EXPORTS = ("fd_plugin_abi", "fd_plugin_element_types,DATA", "fd_plugin_n_element_types,DATA")


def plugin_name(model):
    return f"{model['model']}_plugin"


def generate_plugin(model, outdir, cc=None, elements=None):
    """Generate and compile <model>_plugin.so in outdir for the plugin elements (default:
    plugin_elements(model)). Returns its path, None if the model has no expression forces or
    drivers."""
    elems = plugin_elements(model) if elements is None else elements
    if not elems:
        return None
    outdir = Path(outdir)
    src = outdir / f"{plugin_name(model)}_src"
    if src.exists():
        shutil.rmtree(src)
    src.mkdir(parents=True)
    names = list({tname: elem for _, tname, _, _, elem in elems}.items())  # each type once
    for tname, elem in names:
        write_element(tname, elem, src)
    write_registry(src, names, plugin=True)
    lib = outdir / (plugin_name(model) + LIB_SUFFIX)
    compile_library(sorted(src.glob("*.c")), lib, cc, exports=PLUGIN_EXPORTS)
    return lib


def _cache_dir():
    d = os.environ.get("FREEDYN2_PLUGIN_CACHE")
    if d == "off":
        return None
    return Path(d) if d else userdirs.cache_dir() / "plugins"


def _run(cmd, env=None):
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f"compiling the model plugin failed ({' '.join(cmd)}):\n{r.stderr}")


class Compiler:
    """A C compiler: cmd, the command (a list: ["gcc"], [python, "-m", "ziglang", "cc"]); kind,
    "gcc" (gcc-like: gcc, clang, cc), "zig" or "msvc"; env, the environment to run it in (None:
    this process'); source, where it was found (for freedyn2 doctor)."""

    def __init__(self, cmd, kind=None, env=None, source=""):
        self.cmd, self.env, self.source = list(cmd), env, source
        self.kind = kind or _kind(self.cmd)

    def __repr__(self):
        return f"Compiler({self.cmd!r}, {self.kind!r}, source={self.source!r})"


def _kind(cmd):
    if "ziglang" in cmd or Path(cmd[0]).stem.lower() == "zig":
        return "zig"
    return "msvc" if _is_msvc(cmd[0]) else "gcc"


def ziglang_available():
    return importlib.util.find_spec("ziglang") is not None


def _zig():
    if ziglang_available():
        return Compiler([sys.executable, "-m", "ziglang", "cc"], "zig", source="ziglang")
    zig = shutil.which("zig")
    if zig:
        return Compiler([zig, "cc"], "zig", source="zig on the PATH")
    raise RuntimeError("compiler zig: install it with pip install ziglang (or pip install freedyn2[compiler])")


def _msvc():
    from . import msvc

    if sys.platform != "win32":
        raise RuntimeError("compiler msvc: Visual Studio is Windows only")
    found = msvc.environment()
    if found is None:
        raise RuntimeError("compiler msvc: no Visual Studio (or Build Tools) with the C++ tools found by vswhere")
    cl, env = found
    return Compiler([cl], "msvc", env, source="Visual Studio (vcvars64.bat)")


def _parse(value, source):
    """A Compiler from a value of CC or the configuration; None for "auto" / empty."""
    if isinstance(value, Compiler):
        return value
    if isinstance(value, (list, tuple)):
        return Compiler(value, source=source)
    value = str(value).strip()
    if value.lower() in ("", "auto"):
        return None
    if value.lower() == "zig":
        c = _zig()
        c.source = f"{source}: {c.source}"
        return c
    if value.lower() == "msvc":
        c = _msvc()
        c.source = f"{source}: {c.source}"
        return c
    if Path(value).is_file() or not any(ch.isspace() for ch in value):
        return Compiler([value], source=source)
    return Compiler(shlex.split(value, posix=sys.platform != "win32"), source=source)  # "zig cc"


def find_compiler(cc=None):
    """The Compiler for model plugins: cc (a name, path, command list or Compiler), else CC, else
    the configuration (key `compiler`), else the PATH, else MSVC (Windows, vswhere), else ziglang."""
    for value, source in ((cc, "argument"), (os.environ.get("CC"), "CC"), (userdirs.get("compiler"), "config")):
        if value:
            c = _parse(value, source)
            if c is not None:
                return c
    for name in ("cc", "gcc", "clang", "cl"):
        if shutil.which(name):
            return Compiler([name], source="PATH")
    if sys.platform == "win32":
        try:
            return _msvc()
        except RuntimeError:
            pass
    if ziglang_available():
        return _zig()
    raise RuntimeError("no C compiler for the model plugin (expression forces, drivers): pip install ziglang "
                       "(or freedyn2[compiler]), or install gcc or clang (Windows: MinGW-w64, or Visual Studio "
                       "Build Tools), or set CC or freedyn2 config compiler")


# zig cc beyond the flags of gcc: code for any CPU of the architecture, as gcc makes by default (zig
# compiles for the CPU it runs on, a cached plugin would not run on an older one)
ZIG_FLAGS = ["-mcpu=baseline"]


def _is_msvc(cc):
    return Path(cc).stem.lower() in ("cl", "clang-cl")


def _zig_target():
    """zig's target on Windows: the MinGW ABI, whose DLLs export like MinGW gcc's."""
    arch = {"amd64": "x86_64", "x86_64": "x86_64", "arm64": "aarch64", "aarch64": "aarch64"}.get(
        platform.machine().lower(), "x86_64")
    return f"{arch}-windows-gnu"


def _write_def(path, exports):
    """A module definition file exporting exports (",DATA": data), for zig's MinGW link. The
    CasADi functions are __declspec(dllexport) and exported anyway; the registry symbols are
    not, and lld (unlike GNU ld) does not take --export-all-symbols from zig cc."""
    lines = ["EXPORTS", *(f"  {e.split(',')[0]}" + (" DATA" if e.endswith(",DATA") else "") for e in exports)]
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def compile_library(sources, lib, cc=None, exports=(), cache=True):
    """Compile C sources into the shared library lib (one compiler process per file), or copy
    it from the cache. exports: symbols the library must export (MSVC; ",DATA" for data)."""
    compiler = find_compiler(cc)
    cc, env = compiler.cmd, compiler.env
    lib = Path(lib)
    windows = sys.platform == "win32"
    if compiler.kind == "msvc":
        flags = ["/nologo", "/O2"]
        compile_cmd = lambda f, o: [*cc, *flags, "/c", f"/Fo{o}", str(f)]
        link = lambda objs: [*cc, "/nologo", "/LD", *objs, f"/Fe{lib}", "/link", *(f"/EXPORT:{e}" for e in exports)]
        obj_suffix = ".obj"
    else:
        target = (["-target", _zig_target()] if windows else []) + ZIG_FLAGS if compiler.kind == "zig" else []
        flags = (["-O2"] if windows else ["-O2", "-fPIC"]) + target
        compile_cmd = lambda f, o: [*cc, *flags, "-c", "-o", str(o), str(f)]
        exported = "-Wl,--export-all-symbols" if windows else "-lm"
        if windows and compiler.kind == "zig":  # zig's lld drops --export-all-symbols: a .def file
            exported = str(_write_def(sources[0].parent / (lib.stem + ".def"), exports))
        link = lambda objs: [*cc, *target, "-shared", "-o", str(lib), *objs, exported]
        obj_suffix = ".o"
    key = hashlib.sha256(" ".join([*cc, *flags]).encode())
    for f in sources:
        key.update(f.name.encode() + b"\0" + f.read_bytes())
    cache = _cache_dir() if cache else None
    cached = cache / f"{key.hexdigest()}{lib.suffix}" if cache else None
    if cached and cached.is_file():
        shutil.copyfile(cached, lib)
        return
    objects = [f.with_suffix(obj_suffix) for f in sources]
    with ThreadPoolExecutor(os.cpu_count() or 1) as pool:
        list(pool.map(lambda cmd: _run(cmd, env), [compile_cmd(f, o) for f, o in zip(sources, objects)]))
    _run(link([str(o) for o in objects]), env)
    if cached:
        cache.mkdir(parents=True, exist_ok=True)
        tmp = cached.with_suffix(f".{os.getpid()}.tmp")
        shutil.copyfile(lib, tmp)
        os.replace(tmp, cached)  # atomic: concurrent builds of the same plugin are safe
