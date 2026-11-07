# freedyn2 in WebAssembly (spike)

The runtime as a WebAssembly module, for simulating in a browser (e.g. an Onshape app without
a server). Status: a spike; not part of the CMake build, the wheels or CI.

```sh
cmake -S . -B build && cmake --build build -j          # generated element code: build/generated
pip install ziglang
python tools/build_wasm.py build/generated build-wasm   # build-wasm/freedyn2.wasm
node runtime/wasm/compare.mjs build-wasm/freedyn2.wasm build/freedyn2_run build/generated
```

```js
import { loadFreedyn2 } from "./fd_wasm.mjs";
const fd = await loadFreedyn2("freedyn2.wasm");
const r = fd.run(fdmText);  // { names, rows, cols, data (Float64Array, row major), steps, rejected }
```

- `fd_wasm.cpp`: the C interface (`fd_run` on the text of a `.fdm` file, results in memory).
- `fd_wasm.mjs`: loader, the WASI functions the module calls (no file system), `run`.
- `compare.mjs`: the same models with `freedyn2.wasm` and the native `freedyn2_run`.
- `demo/`: a page that runs everything in the browser (below).

## The browser demo

```sh
python tools/build_web_demo.py build-wasm/freedyn2.wasm build-web
python tools/serve_web_demo.py build-web             # http://localhost:8000/ (right content types)
```

A static site (2.1 MB plus Pyodide from its CDN; any static host, e.g. GitHub Pages): pick or
edit a YAML model, *simulate*, and the freedyn2 viewer shows the results, as `freedyn2 view`
does. The YAML is translated by `fdgen.web` in Pyodide (fdgen without CasADi: `fdgen._casadi`
imports it on first use), simulated by `freedyn2.wasm`, and shown by the viewer's static page
(`fdgen.view.server.static_page`, the same as `--export`). Nothing leaves the browser.

Meshes: *open files* (or drop onto the left panel) takes a YAML model with the STL and OBJ files
it reads, or one zip with all of them (also with a folder around them, as an export); meshes
alone are added to the model in the editor. They go to Pyodide's file system next to the model
(`fdgen.web.add_file`), so paths in the YAML are relative to it as on disk. *download* gives the
YAML, or a zip with the model and its meshes. The example `two_links` (`demo/examples`) has two
parts as a CAD system exports them: STL in millimeters, in assembly coordinates.

Measured in headless Chromium (double pendulum, 10 s, 10 000 steps): start 3.3 to 3.8 s (Pyodide,
numpy, pyyaml, jsonschema; cached afterwards), YAML to `.fdm` 0.15 to 0.18 s, simulation 0.4 to
0.5 s, viewer page 0.4 to 0.5 s.

## Results (November 2026, Zig 0.17.1, Node 22)

- Size 1.35 MB, 0.44 MB gzip; imports 13 WASI functions.
- All example models without a plugin: largest relative difference to native (both with Eigen
  SparseLU) 5.6e-14, most 1e-16; the same steps and rows. Time about the native one.
- Windows (MSVC runtime): the same, except the models that amplify last-bit differences of the
  math library: double pendulum 3.4e-5, pendulum 4.5e-9 (same steps and rows). The module's
  results are bit-identical on Linux and Windows (fingerprints of `compare.mjs`): the
  difference is the native MSVC `sin`/`cos`/`atan`, not WebAssembly.
- Errors (model file, missing plugin) reach JavaScript as exceptions with the runtime's
  message; the module keeps working.
- YAML to `.fdm` (`fdgen.model`) needs CasADi only at import time for library-element models:
  with a stand-in module that fails on any use after the imports, the `.fdm` files of all
  library examples are byte-identical. Expression forces and drivers need CasADi
  (`fdgen.expressions`). The CasADi imports are lazy now (`fdgen._casadi`): in Pyodide the
  `.fdm` file of the double pendulum is identical to the native build's.

## Limits and next steps

- No model plugins (expression forces, drivers): they are shared libraries loaded at run time.
- No files: splines from data files and OpenCRG roads would need a file system (or the data
  passed in).
- No KLU yet (SuiteSparse builds with the same toolchain; SparseLU is fine for small models).
- Exceptions use WebAssembly exception handling (legacy encoding); Zig's libc++abi for WASI has
  none, so `tools/build_wasm.py` compiles the parts from the sources Zig ships.
