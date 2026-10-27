# Legacy FreeDyn on Linux (reference for comparisons)

`build_legacy.py` builds the command-line solver `FreeDyn` of the legacy kernel
(`freedyn-org/me`) with GCC on Linux, so every FreeDyn 2.0 result can be compared with it
on the same machine. The legacy repository is never modified:

1. its sources are copied to `<outdir>/src`,
2. quoted `#include`s are rewritten (backslashes, `C:\me\...`, Windows case-insensitive
   file names),
3. a few Windows path separators in file name handling are patched (`PATCHES` in the script),
4. a CMake project is generated from the Visual Studio projects (`Release|x64`: sources,
   include directories, defines) and built.

`compat/` holds the force-included `legacy_compat.h` and small stand-ins for `windows.h`,
`io.h`, `concurrent_queue.h` (MSVC PPL) and `tbb/concurrent_vector.h`.

```sh
pip install mkl-devel                         # MKL headers and libraries
git clone --depth 1 https://github.com/freedyn-org/me ../me
python legacy/build_legacy.py ../me -o build/legacy [--mkl parallel|sequential]
build/legacy/build/FreeDyn model.fds screen   # writes model.srf, ... next to the .fds
```

Differences to the Windows release: GCC 13 `-O2` instead of Intel C++ 2025 (MaxSpeed,
precise FP), GNU OpenMP and `mkl_gnu_thread` instead of the Intel runtime. Results should
agree to round-off; performance numbers from this build are indicative only and are
complemented by runs of the Windows binaries.
