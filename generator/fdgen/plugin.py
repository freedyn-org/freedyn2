"""Model plugin: the model-specific element types of a model, compiled into a shared library.

Expression forces (fdgen.expression_forces) and drivers (fdgen.drivers) cannot be library
element types: their laws are part of the model. The element types of extensions
(fdgen.extensions) are not in the runtime either and are compiled here too. The generator builds one element type per
force and driver, generates its C code with CasADi and compiles all of them into
`<model>_plugin.so` next to the .fdm file, which the runtime loads at model load (registry
layout checked with FD_REGISTRY_ABI). Derivatives are exact, as for the library types.

The C files are compiled in parallel, and the library is cached by the hash of its sources
and the compiler command (directory FREEDYN2_PLUGIN_CACHE, default ~/.cache/freedyn2/plugins;
"off" disables the cache): constants are parameters, so a model whose parameters change
reuses its plugin. Within a process, the elements themselves (their derivatives and generated
code) are kept too (fdgen.custom_element.ELEMENT_CACHE): rebuilding a model with other
parameter values, as the viewer does for every tuned value, skips the symbolic work.

Compiler: CC, else the first of cc, gcc, clang, cl on the PATH. On Windows the library is a
.dll; with MSVC (cl, from a developer prompt) the registry symbols are exported explicitly,
with MinGW gcc all symbols are.
"""

import hashlib
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from . import extensions
from .codegen import code_generator, write_element, write_registry
from .drivers import driver_elements
from .expression_forces import expression_elements


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
    """[(name, element type, [bodies], p values, ElementFunctions)] of all plugin elements.
    Elements whose generated code is the same up to the type name (the same law on other
    bodies, other parameter values) share one element type: a model with many copies of a
    subsystem compiles each law once, and the runtime runs the same code for all of them."""
    out, types = [], {}
    for name, tname, bodies, p, elem in expression_elements(model) + driver_elements(model) + extension_elements(model):
        key = elem.__dict__.get("code_key")  # elements are kept between builds (custom_element)
        if key is None:
            key = hashlib.sha256(code_generator("x", elem, header=False).dump().replace(tname, "@").encode()).hexdigest()
            elem.code_key = key
        tname, elem = types.setdefault(key, (tname, elem))
        out.append((name, tname, bodies, p, elem))
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
    base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "freedyn2" / "plugins"
    return Path(d) if d else base


def _run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"compiling the model plugin failed ({' '.join(cmd)}):\n{r.stderr}")


def find_compiler(cc=None):
    cc = cc or os.environ.get("CC")
    if cc:
        return cc
    for name in ("cc", "gcc", "clang", "cl"):
        if shutil.which(name):
            return name
    raise RuntimeError("no C compiler for the model plugin (expression forces, drivers): install gcc or clang "
                       "(Windows: MinGW-w64, or Visual Studio Build Tools and a developer prompt) or set CC")


def _is_msvc(cc):
    return Path(cc).stem.lower() in ("cl", "clang-cl")


def compile_library(sources, lib, cc=None, exports=(), cache=True):
    """Compile C sources into the shared library lib (one compiler process per file), or copy
    it from the cache. exports: symbols the library must export (MSVC; ",DATA" for data)."""
    cc = find_compiler(cc)
    lib = Path(lib)
    windows = sys.platform == "win32"
    if _is_msvc(cc):
        flags = ["/nologo", "/O2"]
        compile_cmd = lambda f, o: [cc, *flags, "/c", f"/Fo{o}", str(f)]
        link = lambda objs: [cc, "/nologo", "/LD", *objs, f"/Fe{lib}", "/link", *(f"/EXPORT:{e}" for e in exports)]
        obj_suffix = ".obj"
    else:
        flags = ["-O2"] if windows else ["-O2", "-fPIC"]
        compile_cmd = lambda f, o: [cc, *flags, "-c", "-o", str(o), str(f)]
        link = lambda objs: [cc, "-shared", "-o", str(lib), *objs, "-Wl,--export-all-symbols" if windows else "-lm"]
        obj_suffix = ".o"
    key = hashlib.sha256(" ".join([cc, *flags]).encode())
    for f in sources:
        key.update(f.name.encode() + b"\0" + f.read_bytes())
    cache = _cache_dir() if cache else None
    cached = cache / f"{key.hexdigest()}{lib.suffix}" if cache else None
    if cached and cached.is_file():
        shutil.copyfile(cached, lib)
        return
    objects = [f.with_suffix(obj_suffix) for f in sources]
    with ThreadPoolExecutor(os.cpu_count() or 1) as pool:
        list(pool.map(_run, [compile_cmd(f, o) for f, o in zip(sources, objects)]))
    _run(link([str(o) for o in objects]))
    if cached:
        cache.mkdir(parents=True, exist_ok=True)
        tmp = cached.with_suffix(f".{os.getpid()}.tmp")
        shutil.copyfile(lib, tmp)
        os.replace(tmp, cached)  # atomic: concurrent builds of the same plugin are safe
