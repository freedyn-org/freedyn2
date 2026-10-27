# freedyn2
FreeDyn 2.0: multibody dynamics with CasADi-generated models (offline HHT and real-time export)

Models are described declaratively in YAML (or imported from legacy FreeDyn `.fds` files). A
Python generator (`generator/fdgen`) builds the Lagrange formulation in CasADi, derives all
equations and Jacobians symbolically and emits plain C. A slim C++ runtime (`runtime/`)
assembles the generated element functions via precomputed scatter maps and integrates with HHT.
The generated C needs no CasADi library.

Contents: rigid bodies (Lie group R³×SO(3)), joints (fixed, spherical, revolute, cylindrical,
prismatic, generic), gravity, library forces (spring-damper, torsion spring, bushing, constant
force and torque), model-specific force laws (expressions of measures, parameters, splines and
time, compiled into a model plugin), drivers (prescribed motion), splines from data files, tires
(a Magic Formula tire on an analytic or OpenCRG road), sensors, hardpoints, linearization about
a steady motion and eigenvalues (stability analysis), extensions.

**User documentation:** https://freedyn-org.github.io/freedyn2/ (getting started, a tutorial,
the viewer, the model reference, working with an assistant, coming from Adams; sources in
[`docs/`](docs/index.md), `mkdocs serve` with `requirements-docs.txt`). `freedyn2 examples` lists
the example models and copies one.

**Developer documentation:** [`docs/architecture.md`](docs/architecture.md) (data flow, reading guide,
module map, design, conventions), [`docs/findings.md`](docs/findings.md) (numerical findings,
including on the legacy kernel), [`legacy/README.md`](legacy/README.md) (legacy build).

## Install

Wheels for Windows (x64) and Linux (x86_64) contain everything: the Python package, the
compiled runtime (`freedyn2_run`, `freedyn2_road`) and its KLU libraries. One wheel per
platform serves Python 3.10 and newer.

```sh
uv tool install freedyn2          # with uv (https://docs.astral.sh/uv/): also installs a Python if needed
pip install freedyn2              # or with pip, into the current environment
freedyn2 run model.yaml           # the command line: run, view, check, examples, mcp, compare
```

For assistants, without installing first (the MCP client starts it): `uvx freedyn2 mcp`, e.g.
`claude mcp add freedyn2 -- uvx freedyn2 mcp`. Until the first release on PyPI, the wheels are
the artifacts of the `Wheels` workflow (`pip install freedyn2-*.whl`).

Models with expression forces or drivers are compiled into a model plugin: they need a C
compiler (Linux: gcc or clang; Windows: Visual Studio Build Tools in a developer prompt, or
MinGW-w64 gcc on the PATH). Library elements (bodies, joints, springs, bushings, tires, ...) need
none.

## Build and test

Requirements: CMake >= 3.16, a C/C++17 compiler, Eigen >= 3.3, optionally SuiteSparse KLU (the
default linear solver when found), Python >= 3.10.
- Linux: `apt install libeigen3-dev libsuitesparse-dev`.
- Windows (or Linux without the packages): `python tools/build_deps.py deps` builds Eigen and KLU
  (only KLU and its parts of SuiteSparse, no BLAS) into `deps`; then pass
  `-DCMAKE_PREFIX_PATH=<path of deps>` to CMake. With MSVC, build in a developer prompt.

```sh
python -m pip install -r requirements-dev.txt   # casadi, numpy, pyyaml, jsonschema, pytest, mcp, pyflakes
cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
python -m pytest                                # generator and end-to-end tests (uses build/)
python -m fdgen.cli run examples/pendulum.yaml  # the command line from the sources (PYTHONPATH=generator)
```

`ctest -V` prints a `RESULT` line with the achieved error and its tolerance for every check.
The CMake build runs the generator itself (needs a Python with `casadi`; pass
`-DPython3_EXECUTABLE=...` if CMake picks another one). `pip wheel .` builds a wheel as the
release does (scikit-build-core).

## Run a model

```sh
freedyn2 run model.fds [-o result.csv] [--profile v2|legacy] [-- runtime options]
freedyn2 run examples/driven_rotor.yaml -- --end-time 2 --step 1e-4
```

`freedyn2 run` imports the model (`.yaml` or `.fds`), writes the runtime model file and, for
models with expression forces or drivers, compiles the model plugin (cached), then runs
`build/freedyn2_run` and reports build and simulation time. The steps can also be run
separately:

```sh
python -m fdgen examples/free_body.yaml -o out --models-only      # out/free_body.fdm
build/freedyn2_run out/free_body.fdm -o free_body.csv [--alpha A] [--step H] [--tolerance E]
                   [--end-time T] [--linear-solver klu|sparselu|dense] [--step-log FILE]
```

## Viewer

```sh
freedyn2 view examples/double_pendulum.yaml                 # simulate, show in the browser
freedyn2 view model.yaml result.csv                         # show these results
freedyn2 view model.yaml --no-run                           # the model at t = 0
freedyn2 view model.yaml --export page.html -- --end-time 2 # one HTML file to pass on
```

`freedyn2 view` serves the viewer on `http://127.0.0.1:8765/` (`--port`) and opens the
browser. It shows the bodies, joints, springs, bushings, tire contact points and the road (analytic
or OpenCRG, colored by friction); a timeline plays the results, the channels of the result CSV
are plotted (click a name; channels of very different size get their own lanes; a click in the
plot sets the time). Every save of the model file (or of a mesh or the given results) rebuilds
and re-simulates; an invalid model is reported over the last valid one, so the model file can
be edited by hand (or by an assistant) with the viewer open. A model that drives away is
followed by the camera. The viewer works offline (three.js is included).

Geometry is optional and for the viewer only; it does not change the simulation. Without
shapes a body is drawn as the box of its mass and inertia, the wheel of a tire as a cylinder of
the tire's radius and width. Shapes are given like all model input, globally at t = 0
(`frame: body` for the body frame):

```yaml
bodies:
  arm:
    mass: 2.0
    inertia: [0.01, 0.2, 0.2]
    position: [0.5, 0, 0]
    shapes:
      - {type: box, size: [1.0, 0.05, 0.05]}                          # at the body frame
      - {type: cylinder, radius: 0.04, length: 0.1, position: [1, 0, 0],
         orientation: [0.7071068, -0.7071068, 0, 0], color: "#e15759"} # axis along global y
      - {type: mesh, file: arm.stl, scale: 0.001, position: [0, 0, 0]} # CAD export of the assembly, mm
ground:
  shapes:
    - {type: box, size: [2, 2, 0.02], position: [0, 0, -0.5], opacity: 0.5}
```

Shape types: `box` (`size`), `cylinder` (`radius`, `length`, axis z of the shape frame),
`sphere` (`radius`), `mesh` (`file`: binary or ASCII STL, OBJ; relative to the model file;
`scale`). A mesh exported from a CAD assembly has its vertices in global coordinates: give it
`position: [0, 0, 0]` (and the default orientation), it then sits where it was in the assembly.

### Plots

Below the 3D view (drag the border to resize, ⤢ for large plots), any number of plots:
- **over time**, all with one time axis: mouse wheel zooms, dragging zooms to a range,
  shift+drag pans, double click resets; the values of all channels under the mouse; a click
  sets the time of the animation. Channels of very different size get their own lanes.
- **x-y**: a channel over another one (e.g. toe over wheel travel), with a marker at the
  current time; **spectrum**: amplitude spectrum (FFT, Hann window) of the visible time range,
  with its peak.
- **A|B cursors**: values at A and B, the difference, Δt and 1/Δt, min, max, mean and rms
  between them, for every plotted channel.
- **ƒ formula**: channels from formulas, e.g. `v_kmh = hypot(chassis1.Rdot_x, chassis1.Rdot_y) * 3.6`
  or `der(spring.length)`; functions sin, cos, tan, asin, acos, atan, atan2, sqrt, abs, exp,
  log, log10, min, max, hypot, sign, deg, rad, der (time derivative), int (time integral).
- Units from the channel names, other runs dashed, references as points, PNG and CSV
  export (the visible range). The plots and formulas of a model are kept in the browser.

Channels are added by a click in the list (to the active plot, marked at its left edge) or
dragged onto a plot; a click on a channel's label hides it, × removes it.

### Checking a model

A model built by someone else (or by an assistant) has to be checkable faster than it can be
built. `freedyn2 check model.yaml [result.csv | --run]` and the viewer's Checks panel report:
- errors: inertia no real body has (I1 + I2 < I3), springs whose points coincide, non-finite results;
- likely slips: units (gravity, lengths, masses not in SI), a joint far from both its bodies,
  redundant constraints, a tire deep in the road at the start, drift of the Euler parameters;
- what the reader should know: degrees of freedom, total mass, bodies not connected to the
  ground, bodies not at rest at the start;
- references in the model's `checks` section: total mass, values at times, measured curves over
  time or over another channel (e.g. toe over wheel travel of a K&C rig), with tolerances.

```yaml
checks:
  mass: {total: 1430, tolerance: 5}                       # from the weight list
  values:
    - {channel: chassis1.R_z, time: 0.0, value: 0.536, tolerance: 0.002}
  curves:
    - {name: toe fl, channel: toe_fl, x_channel: travel_fl, file: kc_rig.csv, x: travel, y: toe, tolerance: 0.001}
```

In the viewer a click on a body, joint or force shows its data, its result channels, its
findings and its YAML text with the line in the model file; what it is connected to stays,
the rest fades. A reference curve over another channel is plotted over that channel.

### Runs: computing, comparing

Every simulation of a viewer session is a run, shown in the run strip above the plots with
what differs from the model file (tuned values, the version of the file). A click on a run
shows it in the plots (dashed, its dash on the card); ★ makes it the reference: grey bodies
in the 3D view, and **Δ reference** plots every run minus the reference, which shows the
small differences an overlay hides. ⟲ takes a run's values back into Tune, × removes it.

The header shows whether the results belong to what is set:

| state | |
|---|---|
| up to date (green) | the results are those of the values shown |
| changes not computed (amber) | values changed: the 3D view and the curves fade, **Compute ⏎** or Enter computes, discard goes back |
| computing (blue) | progress; the new curves grow over the faded ones while the simulation runs |
| error (red) | the message over the last good results |

When to compute (Tune): *adaptive* (by itself when the last run took less than 1.5 s, else on
Enter), *always*, or *on Enter*. A new computation stops the one still running.

### Parameters and formulas

Every number of a model may be a formula of the model parameters, written with `=` (as in a
spreadsheet; no quotes needed in YAML):

```yaml
parameters: {k_front: 30000, k_rear: =0.8 * k_front, h_mount: 0.55}
forces:
  susp_fl:
    type: spring_damper
    stiffness: =k_front
    point2: [2.6, 0.8, =h_mount]
```

Formulas use the syntax of the force expressions (`+ - * / ^`, `sqrt`, `sin`, `min`, `pi`, ...)
with the parameters as names, and are numbers fixed when the model is read (functions of time
are force expressions, written without `=`). The viewer's sliders set the parameters, so
everything written as a formula of them follows. A `#` starts a comment in YAML: `=k`, not `#k`.

### Tuning, eigenmodes, force arrows, notes, report

- **Tune**: the model's `parameters` get sliders; any number of an element joins them with a
  click in the inspector (`tune: mass stiffness point1.z …`). Changed values are computed as
  above (runs); the model file is not changed (the values are kept by the server until
  `reset all`). Parameters no expression uses are greyed (and reported by the checks).
- **Eigenmodes**: linearized about the state at the current time (`fdgen.linearize`):
  frequencies and damping ratios; a click animates a mode (amplitude slider, not to time
  scale). Meaningful in an equilibrium or a steady motion. **live**: the eigenvalues follow
  the sliders at once, before simulating (▲▼ the change against the computed values), about
  the positions and velocities of the current run with the constraint forces of the changed
  model (exact as long as the change does not move the equilibrium much).
- **Fan (parameter sweep)**: ≋ at a slider simulates 7 values over its range at once (in
  parallel, the other values as computed): a fan of curves in the time plots (blue: first value,
  red: last) and a **key figure** plot (max, min, max |x|, peak-peak, mean, rms, final of the
  channels over the swept value), taken over the visible time range: zoom to an event to get
  its key figures. The fan is a card in the run strip (show / hide, ×). MCP: `sweep` runs the
  same study and returns the key figures as a table; each value becomes a run.
- **Root locus**: ⌁ at a slider linearizes for 11 values over its range and plots the
  eigenvalues (real part over frequency, the values as a color scale, lines of equal damping
  ratio); the view starts on the eigenvalues that move. MCP: `locus` gives the same as a table.
- **Force arrows**: forces red, torques purple, at their points of application, from the
  force outputs (springs, bushings, tires, applied loads); the largest of the run is 0.3 model sizes.
- **Notes to the assistant**: a text with its context (time, selected element, plotted
  channels, tuned values, the eigenmode shown) and a picture of the view, stored next to the
  model (`<model>.notes/notes.json`, `note_<id>.png`). The assistant reads them (MCP `notes`,
  or the files), changes the model and answers; the answer appears under the note.
- **Report**: one HTML file (print to PDF from the browser) with the view, the plots, the
  checks, tuned values, eigenmodes and notes.

## Assistants (MCP)

`freedyn2 mcp` is an MCP server (stdio; the MCP SDK comes with the package) through
which an assistant builds and examines models: it writes the YAML file and calls

| tool | |
|---|---|
| `guide`, `schema` | conventions, element keys, example; the JSON Schema |
| `validate`, `check` | model and result checks (as `freedyn2 check`) |
| `simulate` | build and run; a run id per run, the solver log, the checks of the results |
| `channels`, `results` | channel names; statistics and a thinned table |
| `compare` | two runs, largest differences first (round-off ignored) |
| `view` | the viewer in the person's browser, reloading on every save |
| `screenshot` | a picture of the viewer for the assistant (`pip install playwright`, `playwright install chromium`) |
| `modes` | eigenfrequencies, damping and the bodies moving in each mode, about the state of a run |
| `locus` | eigenfrequencies and damping for several values of one number (root locus as a table) |
| `sweep` | variant study: one number over several values, simulated in parallel; a key figure of channels per value |
| `notes`, `note_image`, `answer_note` | what the person wrote in the viewer (with context and picture); the answer |

Claude Code: `claude mcp add freedyn2 -- uvx freedyn2 mcp` (or `-- freedyn2 mcp` if installed).
Claude Desktop (`claude_desktop_config.json`):

```json
{"mcpServers": {"freedyn2": {"command": "uvx", "args": ["freedyn2", "mcp"]}}}
```

Paths in tool calls are absolute or relative to the directory the server was started in.

## Legacy FreeDyn against FreeDyn 2.0

```sh
freedyn2 compare examples/compare/*.fds --legacy build/legacy/build/FreeDyn [--markdown report.md]
```

Both programs read the same `.fds` and run it as given (FreeDyn 2.0 with its own methods). The
accuracy of both is measured against a converged solution that the programs compute themselves
(step size / 16, tolerance / 256; FreeDyn 2.0 at two refinements estimates the uncertainty,
legacy FreeDyn cross-checks), so any `.fds` can be compared without model-specific reference
code. For chaotic motion the report gives how long each program stays accurate. Times: legacy
FreeDyn as one program run, FreeDyn 2.0 split into build (once per model) and simulation.
Details in `legacy/benchmark.py`. Result on the comparison models (single thread, this build):
FreeDyn 2.0 equal or more accurate on all 14 models, simulation 2.5 to 19x faster.

## Scaling: a car on a chassis dynamometer

`examples/scaling/make_car.py` writes a fictitious car as a legacy `.fds` (13 bodies: chassis,
4 wheel carriers on vertical prismatic joints with suspension springs, 4 wheels with vertical
tire springs on a road profile, driveline with engine, gearbox, propeller shaft, open
differential, half shafts and roller load; 60 constraint equations, 16 expression forces and
torques, 30 measures), `examples/scaling/scaling.py` runs N independent cars in one model with
both programs.

```sh
freedyn2 compare examples/scaling/car_1.fds --legacy build/legacy/build/FreeDyn
python examples/scaling/scaling.py --legacy build/legacy/build/FreeDyn --cars 1 2 4 8
```

One car, 5 s, against the converged reference: error legacy 9.7e-5 (position), 2.0 (rotation),
16 % (velocity), accurate until 0.92 s; FreeDyn 2.0 1.3e-5, 5.3e-5, 1e-3, accurate to the end;
13.8x faster. The engine settles at 214.29 rad/s (power balance of engine torque, damping and
roller load): FreeDyn 2.0 214.18 rad/s at 5 s, legacy FreeDyn 180.4 rad/s, the spin loss of
Euler parameters under HHT damping (`docs/findings.md`).

| cars | bodies | constraints | legacy [s] | v2 build cold + simulation [s] | v2 per step [us] | speed-up simulation |
|---|---|---|---|---|---|---|
| 1 | 13 | 60 | 13.9 | 2.8 + 0.9 | 181 | 15.4x |
| 2 | 26 | 120 | 29.5 | 3.9 + 2.1 | 428 | 13.8x |
| 4 | 52 | 240 | 64.6 | 5.2 + 5.1 | 1025 | 12.6x |
| 8 | 104 | 480 | 134.5 | 8.4 + 10.2 | 2040 | 13.2x |

Both grow linearly in the work per step (FreeDyn 2.0: the same instruction count per car and
step for 1 and 8 cars); the remaining excess of the FreeDyn 2.0 times over linear (181 -> 255
us per car and step) is the memory hierarchy. The plugin build grows slowly: copies of a law
share one compiled element type.

## Extensions

Packages can add element types, model keys, checks and text for the assistant without changing
FreeDyn (`generator/fdgen/extensions.py`): an `Extension` object registered under the entry
point group `freedyn2.extensions` (or named in `FREEDYN2_EXTENSIONS`). Its element types are
generated and compiled into the model plugin, like expression forces. A guide with a complete
example (a new force type, air drag; `examples/extension`): [`docs/extensions.md`](docs/extensions.md).

## Roads from OpenCRG files

```yaml
road:
  crg: track.crg              # ASAM OpenCRG height grid (relative to the model file)
  crg_friction: track_mu.crg  # optional: OpenCRG file whose grid values are the friction coefficient
  friction: 1.0               # else: constant friction
  position: [0.0, 0.0, 0.0]   # optional placement of the CRG frame in the world
  heading: 0.0
```

The built-in tire (and the tire elements of extensions) run on it. The road files are read and
evaluated with the official ASAM OpenCRG C library (`runtime/third_party/opencrg`, Apache-2.0),
so everything the standard defines applies:
- all data formats;
- a curved or straight reference line, slope and banking;
- NaN values;
- the file's options (border modes) and modifiers;
- include files.

The runtime (`fd::Road`) evaluates the road under each wheel center before every evaluation of
the tire and hands it the local tangent plane and friction. A second CRG file can give the
friction coefficient. Without a modifiers section in the file, FreeDyn keeps the file's
coordinates (the library's default would move the road to the origin) and places the road with
`position` and `heading`.

`freedyn2_road road.crg [--place x y z phi] [--uv] < points` prints height, gradient and
friction at points, to check a road and its placement (`--grid cell n`: a grid over the whole
road, for the viewer). `fdgen.crg.write_crg` writes CRG files.

On all sample roads of the OpenCRG repository, FreeDyn's heights equal the library's.
`tests/python/test_crg.py` checks:
- all four formats against the analytic surface;
- (x, y) against (u, v) evaluation;
- the gradient on a curved, sloped and banked road;
- a vehicle on a CRG road (waves in the grid, slope and banking as channels) against the same
  road given analytically: road height under the tires within 3e-5 m, vertical tire force
  within 0.5 %.

Cost (vehicle with 4 full tires, 10 s at h = 1 ms, median per step):

| | CRG road | analytic road (16 harmonics) |
|---|---|---|
| converged Newton | 219 us | 238 us |
| `newton fixed` 2 | 204 us | 222 us |

The road evaluation is 1.5 % of the instructions of a step (height plus central differences for
the gradient, and friction, for 4 wheels). It does not depend on the size of the road: the
library keeps a search history per wheel.

A 2 km x 12 m road on a 2 cm grid (48 million values, KRBI, 194 MB) loads in 1.4 s (with an empty file cache), takes 375 MB
of memory, and costs the same per step as a small one.

## Real-time mode

`solver.newton: fixed` (or `freedyn2_run --newton-fixed N`): exactly N Newton corrections per
step (default 2), fixed step size, fixed pivot order, so every step does the same work. At
h = 1 ms the Newton error is 50 (N = 1) to 10^7 (N = 2) times smaller than the discretization
error. Median per step, N = 2 (N = 1): car 164 (81) us, vehicle with tires 231 (125) us.
Details, accuracy and determinism: [`docs/architecture.md`](docs/architecture.md#real-time-mode).

## Comparison with legacy FreeDyn

**Rule: compatibility with the legacy kernel lives only at the edges.** FreeDyn 2.0 makes its
own architecture and method decisions on their merits. Legacy compatibility is limited to
(a) the `.fds` importer and (b) the `legacy` profile, which selects the legacy variants of
algorithms that exist in FreeDyn 2.0 anyway. No core interface exists only to make the
comparison possible, and features the legacy kernel lacks are verified against analytic or
high-accuracy references instead.

The `.fds` importer (`python -m fdgen model.fds --profile legacy|v2`) translates a legacy model
into the internal model; new models are written in YAML and need no `.fds`. The comparison has
two levels (`legacy/compare.py --level exact|physics`):

| level | FreeDyn 2.0 profile | checks | purpose |
|---|---|---|---|
| exact | `legacy`: Euler-parameter bodies, Negrut HHT form, legacy Newton rule and step size control, Jacobian and ModifyNewmark options | same steps and Newton iterations, results equal to round-off | verifies generator, assembly and solver implementation |
| physics | `v2`: Lie group bodies, Arnold–Bruls HHT form, velocity-level Newton criterion, step size control keeping the model's tolerance | both programs against an independent reference (`legacy/references`: RK4 on Euler's equations for free bodies, on the minimal-coordinate (joint angle) equations of motion for revolute joint chains, on the Newton–Euler equations with independently written kinematics and loads for free bodies under expression forces; analytic for the forced oscillator) | FreeDyn 2.0 may use other methods, must solve the same physics at least as accurately |

```sh
pip install mkl-devel && python legacy/build_legacy.py ../me -o build/legacy   # see legacy/README.md
python legacy/compare.py examples/compare/*.fds --level exact
python legacy/compare.py examples/compare/*.fds --level physics
python -m pytest tests/python/test_legacy_compare.py tests/python/test_legacy_physics.py
```

If keeping the exact level costs an architecture decision, the exact level is frozen for the
affected models, not the architecture adapted. CI runs both levels when the repository secret
`FREEDYN_ME_TOKEN` (read access to `freedyn-org/me`) is set.

## Layout

| path | content |
|---|---|
| `generator/fdgen/` | generator, `.fds` importer (`fds/`), `freedyn2` command line (`cli.py`), viewer (`view/`) |
| `runtime/` | C++ runtime and `freedyn2_run` |
| `schema/model.schema.json` | JSON Schema of the YAML input |
| `examples/` | YAML models; `compare/*.fds` models compared with legacy FreeDyn; `data/` spline tables |
| `legacy/` | Linux build of legacy FreeDyn, result readers, comparisons, independent references |
| `tests/python`, `tests/cpp` | pytest and ctest suites |
| `docs/` | architecture and reading guide, numerical findings |

## Citing and acknowledgements

To cite FreeDyn 2.0, use the metadata in `CITATION.cff` (GitHub: "Cite this repository").
FreeDyn 2.0 succeeds FreeDyn, developed at the University of Applied Sciences Upper Austria,
Campus Wels, by Wolfgang Steiner, Wolfgang Witteveen, Karin Nachbagauer, Karim Sherif, Thomas
Lauss and Stefan Oberpeilsteiner, among others. Its solver served as the reference for every
step and is reproduced in the legacy profile. Publications on FreeDyn:

> Witteveen, W., Lauss, T., Oberpeilsteiner, S. (2021). FreeDyn: a free, flexible and state of
> the art multibody simulation package for education, research and industrial applications.
> In: *Nonlinear Structures & Systems, Volume 1*, Conference Proceedings of the Society for
> Experimental Mechanics Series. Springer, Cham. https://doi.org/10.1007/978-3-030-47626-7_8
>
> Nachbagauer, K., Sherif, K., Witteveen, W. (2014). FreeDyn: a multibody simulation research
> code. 11th World Congress on Computational Mechanics (WCCM XI) and 5th European Conference on
> Computational Mechanics (ECCM V), Barcelona.
>
> Oberpeilsteiner, S., Lauss, T., Eichmeir, P., Steiner, W. (2019). FreeDyn: a software for
> advanced multibody simulation: applications in the field of vehicle dynamics. ECCOMAS
> Thematic Conference on Multibody Dynamics 2019, Duisburg.
>
> Sherif, K., Nachbagauer, K., Steiner, W., Lauss, T. (2019). A modified HHT method for the
> numerical simulation of rigid body rotations with Euler parameters. *Multibody System
> Dynamics* 46(2), 181-202. https://doi.org/10.1007/s11044-019-09672-6

The methods and their sources are in [`docs/theory.md`](docs/theory.md).

## License

Apache License 2.0 (`LICENSE`, `NOTICE`). Code that FreeDyn 2.0 generates from your model
belongs to you.
