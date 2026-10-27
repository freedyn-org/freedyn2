"""Build the C/C++ dependencies of the runtime into a prefix (for CMAKE_PREFIX_PATH):

  python tools/build_deps.py PREFIX

- Eigen 3.4.0 (header-only, MPL-2.0),
- SuiteSparse 7.14.1 with only what KLU needs: SuiteSparse_config, AMD, COLAMD, BTF, KLU, as
  shared libraries (KLU and BTF are LGPL-2.1: linked dynamically), no BLAS and no CHOLMOD.

Used by the Windows CI job and the wheel builds (cibuildwheel), on Windows and Linux alike; a
Linux developer can take libeigen3-dev and libsuitesparse-dev instead. Needs git, CMake and a C/C++
compiler. Does nothing if PREFIX already holds a complete build (a marker file).
"""

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

EIGEN = ("https://gitlab.com/libeigen/eigen.git", "3.4.0")
SUITESPARSE = ("https://github.com/DrTimothyAldenDavis/SuiteSparse.git", "v7.14.1")
SUITESPARSE_PARTS = ["SuiteSparse_config", "AMD", "COLAMD", "BTF", "KLU"]
MARKER = "freedyn2-deps-eigen-3.4.0-suitesparse-7.14.1"


def run(*cmd, cwd=None):
    print("+", " ".join(map(str, cmd)), flush=True)
    subprocess.run([str(c) for c in cmd], cwd=cwd, check=True)


def cmake_install(src, build, prefix, *options):
    run("cmake", "-S", src, "-B", build, "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_INSTALL_PREFIX={prefix}", *options)
    run("cmake", "--build", build, "--config", "Release", "--parallel")
    run("cmake", "--install", build, "--config", "Release")


def main(prefix):
    prefix = Path(prefix).resolve()
    if (prefix / MARKER).is_file():
        print(f"dependencies already in {prefix}")
        return 0
    prefix.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="freedyn2_deps_"))
    try:
        url, tag = EIGEN
        run("git", "clone", "-q", "--depth", "1", "--branch", tag, url, work / "eigen")
        cmake_install(work / "eigen", work / "eigen-build", prefix, "-DBUILD_TESTING=OFF", "-DEIGEN_BUILD_DOC=OFF",
                      "-DEIGEN_BUILD_PKGCONFIG=OFF")
        url, tag = SUITESPARSE
        run("git", "clone", "-q", "--depth", "1", "--branch", tag, "--filter=blob:none", "--sparse", url, work / "ss")
        run("git", "-C", work / "ss", "sparse-checkout", "set", *SUITESPARSE_PARTS)
        cmake_install(work / "ss", work / "ss-build", prefix,
                      "-DSUITESPARSE_ENABLE_PROJECTS=suitesparse_config;amd;colamd;btf;klu",
                      "-DSUITESPARSE_REQUIRE_BLAS=OFF", "-DBLA_VENDOR=Generic",  # no BLAS: its checks need a name
                      "-DKLU_USE_CHOLMOD=OFF", "-DSUITESPARSE_USE_FORTRAN=OFF", "-DSUITESPARSE_USE_OPENMP=OFF",
                      "-DBUILD_STATIC_LIBS=OFF", "-DSUITESPARSE_DEMOS=OFF", "-DBUILD_TESTING=OFF",
                      "-DCMAKE_INSTALL_LIBDIR=lib")
        # the license texts go along with the libraries (into the wheel: fdgen/licenses)
        lic = prefix / "share" / "licenses" / "suitesparse"
        lic.mkdir(parents=True, exist_ok=True)
        for part in ("AMD", "COLAMD", "BTF", "KLU"):
            shutil.copyfile(work / "ss" / part / "Doc" / "License.txt", lic / f"{part}.txt")
        shutil.copyfile(work / "ss" / "LICENSE.txt", lic / "SuiteSparse_all_packages.txt")  # incl. SuiteSparse_config
        (prefix / MARKER).write_text("complete\n")
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
