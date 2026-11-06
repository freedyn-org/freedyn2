"""Build the freedyn2 runtime as WebAssembly (freedyn2.wasm) with Zig (pip install ziglang).

    python tools/build_wasm.py build/generated build-wasm

The first argument is a directory with the generated library element code (fd_registry.c and the
element .c files: the CMake build writes them to build/generated, or python -m fdgen -o DIR).
Eigen is taken from /usr/include/eigen3 or EIGEN3_INCLUDE_DIR.

The module is a WASI reactor without a file system (runtime/wasm/fd_wasm.mjs provides the few
WASI functions it calls). Library elements only: no model plugins, no KLU (Eigen SparseLU).

Zig's C++ library for WASI comes without exception support; the runtime reports errors with
exceptions, so the exception parts of libc++abi and libunwind that Zig ships as sources are
compiled here with WebAssembly exception handling (supported by all current browsers and Node).
"""

import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET = ["-target", "wasm32-wasi", "-O2", "-fwasm-exceptions", "-mexception-handling"]


def zig(*args):
    subprocess.run([sys.executable, "-m", "ziglang", *map(str, args)], check=True)


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    gen, out = Path(argv[0]).resolve(), Path(argv[1]).resolve()
    if not (gen / "fd_registry.c").exists():
        sys.exit(f"{gen}: no fd_registry.c (generated element code)")
    import ziglang

    zl = Path(ziglang.__file__).parent / "lib"
    eigen = os.environ.get("EIGEN3_INCLUDE_DIR", "/usr/include/eigen3")
    rt, crg = ROOT / "runtime", ROOT / "runtime/third_party/opencrg"
    out.mkdir(parents=True, exist_ok=True)
    obj = Path(tempfile.mkdtemp(prefix="fd-wasm-"))

    jobs = []  # (output object, compiler, flags, source)
    abi = [f"-I{zl}/libcxxabi/include", f"-I{zl}/libcxxabi/src", f"-I{zl}/libunwind/include", f"-I{zl}/libcxx/src",
           "-D_LIBCXXABI_HAS_NO_THREADS", "-D_LIBCPP_BUILDING_LIBRARY", "-D_LIBCXXABI_BUILDING_LIBRARY",
           "-DLIBCXXABI_USE_LLVM_UNWINDER", "-std=c++23"]
    for f in ("cxa_exception", "cxa_personality"):
        jobs.append((obj / f"0eh_{f}.o", "c++", abi, zl / f"libcxxabi/src/{f}.cpp"))
    jobs.append((obj / "0eh_unwind.o", "cc", ["-fdeclspec", "-D_LIBUNWIND_HAS_NO_THREADS", f"-I{zl}/libunwind/include",
                                             f"-I{zl}/libunwind/src"], zl / "libunwind/src/Unwind-wasm.c"))
    for f in sorted(gen.glob("*.c")):
        jobs.append((obj / f"gen_{f.stem}.o", "cc", [f"-I{gen}"], f))
    for f in sorted((crg / "src").glob("*.c")):
        jobs.append((obj / f"crg_{f.stem}.o", "cc", ["-w", f"-I{crg}/inc"], f))
    for f in [*sorted((rt / "src").glob("*.cpp")), rt / "wasm/fd_wasm.cpp"]:
        jobs.append((obj / f"rt_{f.stem}.o", "c++", ["-std=c++17", f"-I{rt}/include", f"-I{gen}", f"-I{crg}/inc",
                                                    f"-I{eigen}"], f))
    with ThreadPoolExecutor(os.cpu_count()) as ex:
        list(ex.map(lambda j: zig(j[1], *TARGET, *j[2], "-c", j[3], "-o", j[0]), jobs))

    # The C++ exception tag, which Emscripten's runtime would define.
    tag = obj / "0eh_tag.s"
    tag.write_text("\t.globl __cpp_exception\n\t.tagtype __cpp_exception i32\n__cpp_exception:\n")
    zig("cc", "-target", "wasm32-wasi", "-mexception-handling", "-c", tag, "-o", obj / "0eh_tag.o")

    # The exception objects first: before Zig's libc++abi, whose no-exception stubs would clash.
    objects = sorted(obj.glob("0eh_*.o")) + sorted(p for p in obj.glob("*.o") if not p.name.startswith("0eh_"))
    wasm = out / "freedyn2.wasm"
    zig("c++", *TARGET, "-s", "-mexec-model=reactor", *objects, "-o", wasm)
    print(f"wrote {wasm} ({wasm.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
