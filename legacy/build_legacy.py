"""Build the legacy FreeDyn command-line solver (freedyn-org/me) on Linux for comparisons.

The legacy repository is not modified: its sources are copied into a build directory,
include paths are rewritten (backslashes, Windows case-insensitive file names), and a
CMake project is generated from the Visual Studio project files (Release|x64).

Usage: python legacy/build_legacy.py /path/to/me [-o build/legacy]
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
MSB = "{http://schemas.microsoft.com/developer/msbuild/2003}"
CONFIG = "Release|x64"

# Projects needed by FreeDyn.exe, in link order (dependents first).
PROJECTS = [
    "FreeDyn", "SIM_Dll", "SOL_Dll", "MBS_Dll", "TIR_Dll", "PCM_Dll", "CFO_Lib",
    "FE_Lib", "DAT_Dll", "FUN_Lib", "exprTk_Dll", "MBM_Lib", "IO_Lib",
]
INCLUDE_RE = re.compile(r'^(\s*#\s*include\s*)"([^"]+)"', re.M)

# Source patches applied to the copy (never to the legacy repository): Windows path
# separators in file name handling, and a hard-coded Windows timing file.
PATCHES = [
    (r'\+ "\\\\" \+', '+ "/" +'),                              # dir + "\\" + name
    (r'find_last_of\("\\\\"\)', 'find_last_of("/\\\\")'),       # file name from path
    (r"rfind\('\\\\'\)", 'find_last_of("/\\\\")'),
    (r'R"\(C:\\\\temp\\\\timing1\.txt\)"', '"/dev/null"'),
]


def condition_matches(elem):
    cond = elem.get("Condition")
    return cond is None or CONFIG in cond


def parse_project(path, me_root):
    """Sources, include directories and defines of one vcxproj for CONFIG."""
    root = ET.parse(path).getroot()
    srcs = []
    for e in root.iter(MSB + "ClCompile"):
        inc = e.get("Include")
        if not inc:
            continue
        excluded = any(
            x.tag == MSB + "ExcludedFromBuild" and condition_matches(x) and x.text.strip().lower() == "true"
            for x in e
        )
        if not excluded:
            srcs.append(inc.replace("\\", "/"))
    incs, defs = [], []
    for grp in root.iter(MSB + "ItemDefinitionGroup"):
        if not condition_matches(grp):
            continue
        for e in grp.iter(MSB + "AdditionalIncludeDirectories"):
            incs += [x for x in (e.text or "").split(";") if x and not x.startswith("%")]
        for e in grp.iter(MSB + "PreprocessorDefinitions"):
            defs += [x for x in (e.text or "").split(";") if x and not x.startswith("%")]

    def to_rel(d):
        d = d.replace("\\", "/")
        d = re.sub(r"^[A-Za-z]:/me/?", "", d)
        d = d.replace("$(SolutionDir)", "").replace("$(ProjectDir)", "")
        return d.rstrip("/")

    incs = [to_rel(d) for d in incs]
    defs = [d for d in defs if d not in ("WIN32", "_WINDOWS", "_DEBUG")]
    return srcs, incs, defs


class CaseResolver:
    """Resolve Windows-style paths against the real (case-sensitive) file tree."""

    def __init__(self, root):
        self.root = root
        self.index = {}
        for p in root.rglob("*"):
            if p.is_file():
                self.index.setdefault(str(p.relative_to(root)).lower(), str(p.relative_to(root)))

    def resolve(self, rel):
        return self.index.get(os.path.normpath(rel).lower())


def rewrite_includes(src_root, resolver, search_dirs):
    """Rewrite every quoted #include to a path that exists with correct case."""
    unresolved = set()
    for path in src_root.rglob("*"):
        if path.suffix.lower() not in (".cpp", ".h", ".hpp", ".c", ".inl") or not path.is_file():
            continue
        text = path.read_text(encoding="latin-1")
        rel_dir = str(path.parent.relative_to(src_root))

        def fix(m):
            inc = re.sub(r"^[A-Za-z]:/me/", "", m.group(2).replace("\\", "/"))
            if inc != m.group(2).replace("\\", "/"):  # absolute C:/me/... include
                hit = resolver.resolve(inc)
                if hit:
                    return f'{m.group(1)}"{os.path.relpath(hit, rel_dir)}"'
            for base in [rel_dir] + search_dirs:
                hit = resolver.resolve(os.path.join(base, inc))
                if hit:
                    new = os.path.relpath(hit, rel_dir)
                    return f'{m.group(1)}"{new}"'
            unresolved.add(inc)
            return f'{m.group(1)}"{inc}"'

        new_text = INCLUDE_RE.sub(fix, text)
        if new_text != text:
            path.write_text(new_text, encoding="latin-1")
    return unresolved


def apply_patches(src_root):
    count = 0
    for path in src_root.rglob("*"):
        if path.suffix.lower() not in (".cpp", ".h") or not path.is_file():
            continue
        text = path.read_text(encoding="latin-1")
        new = text
        for pat, rep in PATCHES:
            new = re.sub(pat, lambda m: rep, new)
        if new != text:
            path.write_text(new, encoding="latin-1")
            count += 1
    return count


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("me_root", type=Path)
    ap.add_argument("-o", "--outdir", type=Path, default=Path("build/legacy"))
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--mkl", choices=("parallel", "sequential"), default="parallel",
                    help="MKL threading layer; the Windows release uses parallel MKL")
    args = ap.parse_args(argv)
    me = args.me_root.resolve()
    out = args.outdir.resolve()
    src = out / "src"

    vcx = {p.stem: p for p in me.rglob("*.vcxproj")}
    projects = {}
    for name in PROJECTS:
        if name not in vcx:
            sys.exit(f"project {name} not found in {me}")
        srcs, incs, defs = parse_project(vcx[name], me)
        projects[name] = (str(vcx[name].parent.relative_to(me)), srcs, incs, defs)

    print(f"copying sources to {src}")
    if src.exists():
        shutil.rmtree(src)
    shutil.copytree(me, src, ignore=shutil.ignore_patterns(".git", "bin", "*.lib", "*.dll", "*.exe"))

    # Version header normally generated by batchbuild/10_UpdateVersionFromGitTag.bat.
    gen = src / "batchbuild" / "generated"
    gen.mkdir(parents=True, exist_ok=True)
    sha = subprocess.run(["git", "-C", str(me), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    (gen / "FreeDynVersion.generated.h").write_text(
        "#pragma once\n#define FREEDYN_VERSION_TAG \"linux-compare\"\n#define FREEDYN_VERSION_MAJOR \"0\"\n"
        "#define FREEDYN_VERSION_MINOR \"0\"\n#define FREEDYN_VERSION_PATCH \"0\"\n"
        "#define FREEDYN_VERSION_IS_HEAD_TAGGED \"0\"\n#define FREEDYN_BUILD_DATE \"\"\n"
        f"#define FREEDYN_GIT_SHA \"{sha}\"\n"
    )

    print(f"patched {apply_patches(src)} files (path separators)")
    resolver = CaseResolver(src)
    all_incs = sorted({d for _, _, incs, _ in projects.values() for d in incs})
    unresolved = rewrite_includes(src, resolver, all_incs + [p for p, *_ in projects.values()])
    if unresolved:
        print("unresolved quoted includes (system or missing):", ", ".join(sorted(unresolved)[:20]))

    # CMake project: one static library per vcxproj, FreeDyn executable.
    lines = [
        "cmake_minimum_required(VERSION 3.16)",
        "project(freedyn_legacy CXX C)",
        "set(CMAKE_CXX_STANDARD 17)",
        "set(CMAKE_POSITION_INDEPENDENT_CODE ON)",
        f'set(COMPAT "{HERE / "compat"}")',
        "set(MKL_ROOT \"$ENV{MKL_ROOT}\" CACHE PATH \"MKL prefix (include/, lib/)\")",
        "find_package(Threads REQUIRED)",
        "find_package(OpenMP REQUIRED)",
        # The Windows release: Intel C++ 2025, MaxSpeed, precise FP model, OpenMP enabled.
        "add_compile_options(-O2 -w -fpermissive -fopenmp -include ${COMPAT}/legacy_compat.h)",
    ]
    for name in PROJECTS:
        pdir, srcs, incs, defs = projects[name]
        files = []
        for s in srcs:
            hit = resolver.resolve(os.path.join(pdir, s))
            if hit is None:
                print(f"warning: {name}: source {s} not found")
                continue
            files.append(f'"${{CMAKE_SOURCE_DIR}}/src/{hit}"')
        kind = "add_executable" if name == "FreeDyn" else "add_library"
        lines.append(f"{kind}({name} {'' if name == 'FreeDyn' else 'STATIC'}\n  " + "\n  ".join(files) + ")")
        inc_dirs = [f'"${{CMAKE_SOURCE_DIR}}/src/{d}"' for d in incs] + [f'"${{CMAKE_SOURCE_DIR}}/src/{pdir}"']
        inc_dirs += [f'"${{CMAKE_SOURCE_DIR}}/src/{p}/hdr"' for p, *_ in projects.values()]
        lines.append(f"target_include_directories({name} PRIVATE ${{COMPAT}} " + " ".join(inc_dirs) + " ${MKL_ROOT}/include /usr/include/eigen3)")
        if defs:
            lines.append(f"target_compile_definitions({name} PRIVATE " + " ".join(defs) + ")")
    libs = " ".join(p for p in PROJECTS if p != "FreeDyn")
    lines.append(
        f"target_link_libraries(FreeDyn PRIVATE -Wl,--start-group {libs} -Wl,--end-group "
        "${MKL_LIBS} OpenMP::OpenMP_CXX Threads::Threads dl m)",
    )
    lines.append('set_target_properties(FreeDyn PROPERTIES BUILD_RPATH "${MKL_ROOT}/lib")')
    (out / "CMakeLists.txt").write_text("\n".join(lines) + "\n")

    mkl_root = os.environ.get("MKL_ROOT", sys.prefix if Path(sys.prefix, "include/mkl.h").exists() else "/usr/local")
    # pip wheels (mkl-devel) ship only versioned shared objects, e.g. libmkl_core.so.2.
    mkl_libs = []
    layer = "mkl_gnu_thread" if args.mkl == "parallel" else "mkl_sequential"
    for lib in ("mkl_intel_lp64", layer, "mkl_core"):
        cands = sorted(Path(mkl_root, "lib").glob(f"lib{lib}.so*"))
        if not cands:
            sys.exit(f"MKL library {lib} not found in {mkl_root}/lib (pip install mkl-devel)")
        mkl_libs.append(str(cands[0]))
    subprocess.run(["cmake", "-S", str(out), "-B", str(out / "build"), f"-DMKL_ROOT={mkl_root}",
                    "-DMKL_LIBS=" + ";".join(mkl_libs)], check=True)
    r = subprocess.run(["cmake", "--build", str(out / "build"), "-j", str(args.jobs), "--", "-k"])
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
