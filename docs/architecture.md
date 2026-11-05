# FreeDyn 2.0 architecture and reading guide

FreeDyn 2.0 splits multibody simulation into two programs with one narrow interface between
them:

- the **generator** (Python, `generator/fdgen`) knows the physics: it derives every equation
  and every derivative symbolically with CasADi and writes plain C;
- the **runtime** (C++, `runtime/`) knows nothing about physics: it assembles generated element
  functions into a sparse system and integrates it with HHT.

Everything model-specific is generated; everything the runtime does is generic. That is why the
whole program is small (generator 3,500 lines including two tire models and the linearization,
runtime 1,650 lines, plus 1,400 lines of legacy tooling and 2,700 lines of tests) and why a new element type is a few lines of physics, not a
change to the solver.

## Data flow

```
 model.yaml ──┐                         generator (Python, CasADi)                runtime (C++)
              ├─> internal model ──┬──> library element types ──> C ──────┐
 model.fds ───┘   (dict, validated)│    (rigid body, joints, forces)       ├──> freedyn2_run ──> result.csv
   fdgen.fds      fdgen.model      │                                       │    model.cpp: read .fdm
                                   ├──> model plugin: expression forces,   │    system.cpp: elements,
                                   │    drivers ──> C ──> <model>_plugin.so┤      scatter assembly
                                   │    (fdgen.plugin)                     │    hht.cpp: HHT, Newton,
                                   └──> <model>.fdm (plain text: elements, ┘      step size control
                                        indices, parameters, solver)
```

- **Library element types** (rigid bodies, joint primitives, library forces) are generated
  once, at build time (CMake runs `python -m fdgen`), and compiled into `freedyn2_run`.
- **Model plugin**: force laws and prescribed motions written as expressions belong to the
  model; they become element types of their own, compiled into `<model>_plugin.so` and loaded
  with `dlopen`. Constants stay parameters, so changing a value needs no recompilation.
- **`.fdm`** lists element instances: element type, global indices (coordinates, velocity
  dofs, constraint rows), parameters `p`, initial state, solver settings.
- `freedyn2 run model.fds` (`fdgen/cli.py`) does all three steps in one call.

## Reading guide

Read in this order; each stop builds on the previous one.

1. **One element, end to end.** `generator/fdgen/lagrange.py`: `ElementFunctions` (the set of
   functions every element type provides) and `lagrange_element` (residual, iteration matrix
   and initial-condition functions from `T`, `V`, `Q`, `Phi`, all by CasADi). Then
   `rigid_body.py`, `build()`: the Lie group rigid body via `group_element`.
2. **From element functions to C.** `codegen.py`: `ELEMENT_BUILDERS` (all library types),
   `generate_elements` and `write_registry` (the table the runtime looks types up in).
3. **The runtime.** `runtime/include/freedyn2/system.hpp` (state layout, element instances),
   `runtime/src/system.cpp`: `residual` and `iteration_matrix` (gather local inputs, call the
   generated function, scatter-add), `scatter.cpp` (CSR pattern and value map built once),
   `hht.cpp`: `step` (predictor, Newton loop, update on the Lie group), `LegacyTimeStepper`
   (step size control). `runtime/app/main.cpp` is the command line.
4. **Connecting bodies.** `kinematics.py`: `Body`/`Ground` (points, axes and their velocities of
   a body) and `Connector` (an element without coordinates of its own; generalized forces by
   virtual power, constraint rows, exact time derivatives `rate`). Then `joints.py`
   (constraint primitives and the joint presets) and `forces.py` (`loads()`: each library
   force law in a few lines).
5. **Model-specific laws.** `expressions.py` (safe expression language to CasADi),
   `measures.py` (measures as expressions; continuous angles with element state),
   `custom_element.py` (an element from expressions), `expression_forces.py`, `drivers.py`,
   `plugin.py` (compile and cache the plugin), `runtime/src/plugin.cpp` (load it).
6. **Model input.** `model.py` (`validate` the YAML against `schema/model.schema.json`,
   `write_fdm`), `fds/` (legacy `.fds` importer: `blocks` parser, `bodies`, `loads`,
   `solver` settings and profiles).
7. **Verification.** `tests/python`, `tests/cpp` and `legacy/` (next section).
8. **Viewer.** `shapes.py` (geometry in the body frames, default shapes), `view/scene.py` (the
   scene: bodies, joint frames, force points, road grid), `view/server.py` (`freedyn2 view`:
   local server, rebuild on save, export to one HTML file), `view/web/` (three.js page).

## Module map

| module | role |
|---|---|
| `fdgen/lagrange.py` | element function set; Lagrange elements (vector space) and Lie group elements |
| `fdgen/rigid_body.py` | rigid body: Lie group R³×SO(3) (default), Euler parameters (legacy profile), Shabana Eq. 2.13 |
| `fdgen/kinematics.py` | `Body`, `Ground`, `Connector`: kinematics of connected bodies, virtual power, exact rates |
| `fdgen/joints.py` | translational / rotational constraint primitives; presets fixed, spherical, revolute, cylindrical, prismatic |
| `fdgen/forces.py` | library forces: spring-damper, force, torque, torsion spring, bushing, tires (one function per law) |
| `fdgen/tire.py` | Magic Formula tire on an analytic road (harmonics, slope, friction patch) |
| `fdgen/extensions.py` | extensions: element types, schema keys, checks from other packages (entry points) |
| `fdgen/crg.py` | write OpenCRG road files (examples, tests) |
| `runtime/third_party/opencrg` | ASAM OpenCRG C library (reads and evaluates road files) |
| `fdgen/geometry.py` | numpy geometry of the initial configuration (rotation matrix, local points and axes) |
| `fdgen/expressions.py` | expression language (parsed with `ast`, nothing executed), legacy `step` |
| `fdgen/sweep.py` | parameter sweeps: one number of a model over several values, built one after the other, simulated in parallel; key figures |
| `fdgen/formulas.py` | formulas in number fields (`=k_susp`): evaluated with the parameters before the schema check |
| `fdgen/splines.py` | Akima and zero-clamped cubic splines with the legacy coefficients; table files |
| `fdgen/measures.py` | measures (displacement, distance, angle, rates, ...) as CasADi expressions |
| `fdgen/custom_element.py`, `expression_forces.py`, `drivers.py` | element types built from model expressions |
| `fdgen/plugin.py` | generate, compile (parallel, cached) the model plugin |
| `fdgen/codegen.py` | C code of the library element types and the registry |
| `fdgen/model.py`, `schema/` | YAML validation, `.fdm` writer |
| `fdgen/fds/` | legacy `.fds` importer and solver profiles |
| `fdgen/cli.py` | `freedyn2 run` (import, build, simulate), `view`, `check`, `mcp` |
| `fdgen/checks.py` | checks of a model and its results; references (`checks` section) |
| `fdgen/mcp_server.py` | `freedyn2 mcp`: tools for assistants (`Session`), MCP registration |
| `fdgen/resources.py` | data files (schema, examples, runtime programs) in the installed package or the repository |
| `tools/build_deps.py`, `pyproject.toml` | Eigen and KLU for a build without system packages; the wheel (scikit-build-core, cibuildwheel) |
| `fdgen/shapes.py` | shapes of bodies and the ground for the viewer, in the body frame; default shapes |
| `fdgen/view/` | viewer: `scene.py` (model and results as viewer data), `server.py` (local server, reload, runs, cancelling, export, tuned values, eigenmodes), `notes.py` (notes to the assistant), `web/` (page: `viewer.js` 3D, checks, force arrows, eigenmode animation; `plot.js` plots; `channels.js` units and formulas; `panels.js` tuning and compute state, notes, eigenmode list, report; `runs.js` run strip; three.js r170 vendored) |
| `fdgen/linearize.py` | linearization about a steady motion from the iteration matrices; eigenvalues |
| `runtime/` | `model` (.fdm reader), `system` (state, elements, assembly), `scatter`, `linear_solver`, `hht`, `plugin`, `generated_function`, `crg` (OpenCRG roads); tools `freedyn2_run`, `freedyn2_road` |
| `legacy/` | Linux build of legacy FreeDyn, result readers, comparisons (`compare.py`, `benchmark.py`), independent references (`references/`) |

## Adding an element type

A new library force is a function of the connector `c` (the connected bodies and the
parameters `c.p`) in `fdgen/forces.py`, entered in `_LAWS`: it returns the loads
`(body, F, u, M)` (force `F` at body point `u`, torque `M`) or `(None, s, rate)` (generalized
force `s` conjugate to a scalar measure with this rate), and named outputs. Add its parameter
count to `KINDS`, its YAML parametrization (a function in `_ELEMENTS` that maps the YAML force
to the parameters from the geometry at t = 0) and the schema, and its type name to
`FD_ELEMENT_TYPES` in `CMakeLists.txt`. Generalized forces, all derivatives for the
iteration matrix, the Lie group and Euler-parameter variants and the C code follow. A joint
works the same way with constraint rows (`joints.constraint`).

## Verification

- `tests/python`: generator (symbolic properties: Shabana matrix, Lie group derivatives against
  finite differences, conservation, every force law against independent numpy code), importer,
  schema, command line.
- `tests/cpp` (`fd_tests`): runtime against analytic solutions (free body: momentum, energy,
  precession; pendulum period from the elliptic integral; forces against closed-form
  solutions), with the achieved error and its tolerance printed per check.
- `legacy/compare.py` with two levels: **exact** (legacy profile: same steps, same Newton
  iterations, results equal to round-off with legacy FreeDyn) and **physics** (both programs
  against independent references in `legacy/references`).
- `freedyn2 compare` (`legacy/benchmark.py`): both programs on the same `.fds`, accuracy against
  a converged solution both programs compute themselves, timing split into build and simulation.

## Design

**Element functions.** One set of generated functions per element type; instances differ only by
parameters `p` and their global indices. Local unknowns are `x = [vd; lam]` (accelerations of the
velocity dofs), local residual rows are `[dynamics; constraints]`, so rows and columns share one
local-to-global map; stored coordinates have their own map (`coords`). Vector-space element types
are built by `fdgen.lagrange.lagrange_element`, Lie group types by `group_element`, which derives
all derivatives with respect to configuration perturbations `q o exp(delta)` and the tangent
operator of the exponential map.

| function | inputs | outputs |
|---|---|---|
| `fd_<type>_res` | q, qd, qdd, lam, p, coef, incr, t | `r` (scaled HHT residual), `f = h + Phi_q^T lam - Q` |
| `fd_<type>_iter` | q, qd, qdd, lam, p, coef, incr, t | `J = [dr/dqdd + gamma h dr/dqd + beta h^2 dr/dq, dr/dlam]` |
| `fd_<type>_iter_noqv` | q, qd, qdd, lam, p, coef, incr, t | same without the derivatives of `h` (legacy option) |
| `fd_<type>_ic` | q, qd, …, p, …, t | `[M Phi_q^T; Phi_q 0]`, `[Q - h; -(Phi_q qd)_q qd]`, `Phi`, `Phi_q qd` |
| `fd_<type>_update` | q_n, incr | `q_n o exp(incr)` (vector space: `q_n + incr`) |
| `fd_<type>_scale` | q | magnitude per velocity dof, relative part of the error weights (step size control) |
| `fd_<type>_mn_qd`, `_jac_qd` | … | legacy modified Newmark formulas and `dr/dqd` (`rigid_body_euler`) |
| `fd_<type>_init` | q, u, p | `qd` from user velocities `u` |
| `fd_<type>_post` | q, qd, p, t | new parameters `p` after each accepted step (element state, e.g. the reference of a continuous angle) |
| `fd_<type>_out` | q, qd, p, t | named output channels |

`coef = [alpha, beta, gamma, h]`. `M = d2T/dqd2` and all derivatives by CasADi. For the rigid
body the quadratic velocity vector is the legacy form `h_theta = 2 Gbar^T (omega_bar x J omega_bar)`
(otherwise `h = (d2L/dqd dq) qd - dL/dq`); on `|theta| = 1`, `theta.thetadot = 0` both differ only
along `theta`, but the discrete solutions differ by ~5e-7 in the example, so the legacy form is
required for identical results. The Lie group body uses Euler's equations directly:
`M = diag(m I, J)`, `h = [0; omega_bar x J omega_bar]`, iteration matrix
`J = dr/dvd + gamma' h dr/dv + beta' h^2 (dr/ddelta) T(incr)`.
All outputs keep their structural sparsity (8x8 iteration matrix of a rigid body: 27 nonzeros).

**Chain rule through intermediates** (`fdgen.lagrange.Chain`). Elements without coordinates of
their own (joints, forces, plugin elements) are written in intermediate symbols: the rotation
matrix `A_k` of each connected Lie group body, with its known derivative
`dA/ddelta_i = A skew(e_i)`, and, in plugin elements, every measure and the value of the force
law. Derivatives (iteration matrix, constraint Jacobian, rates, initial-condition terms) go
through the chain rule `dr/dS dS/dx`, each intermediate differentiated once, never through the
quaternion-to-matrix map; the symbols are replaced by their expressions at the end. Without it a
law that depends on a few measures is differentiated once per row of the residual. Iteration
matrix of the car model of `examples/scaling`: gear coupling of 5 bodies 40,800 -> 6,000
instructions, rotation primitive between two bodies 3,450 -> 1,400.

**Shared plugin element types.** Plugin elements whose generated code is the same up to the type
name (the same law on other bodies or with other parameter values; constants are parameters)
share one element type (`fdgen.plugin.plugin_elements`): 8 copies of the car compile 11 types
instead of 128 (the 4 tire types differ by their road profile literals).

**Joints** are elements without coordinates of their own (no `init`, `update`, `scale`): they
reference the coordinates and velocity dofs of the connected bodies and add one constraint row
each. Their iteration matrix block is `[B T(incr); B^T lam terms]` with `B = dPhi/ddelta` from the
same configuration perturbation as the bodies, so the scatter assembly needs nothing new. The
velocity projection at the initial conditions (`vproj_tol`) makes `Phi_q qd = 0` consistent.

**Forces** are elements of the same kind without constraint rows (`fdgen.kinematics.Connector`):
their generalized force follows from virtual power, `Q = sum (dv_P/dv)^T F + (domega/dv)^T M`
(or `s (dmdot/dv)^T` for a force conjugate to a scalar measure `m`, the torsion spring), and enters
`r` and, through CasADi, the iteration matrix (`dQ/dv`, `dQ/ddelta`). Library types are fixed force
laws with parameters. **Model-specific force laws** (`expression_force`, `expression_torque`):
`fdgen.expression_forces` builds one element type per force from its expression of measures, parameters and
`t` (`fdgen.expressions`, a safe subset of Python syntax plus the legacy `step`), generates C and
`fdgen.plugin` compiles `<model>_plugin.so` next to the `.fdm` (`plugin` line), which the runtime loads with
`dlopen` after checking the registry layout (`FD_REGISTRY_ABI`). Constants are element parameters,
so other parameter values need no recompilation. Measures: displacement, velocity, distance and its
rate, angle (continuous: it counts revolutions through element state, `angle = r + 2 pi
round((ref - r) / 2 pi)` with `r = atan2(...)` and `ref` the last accepted value, updated by
`post`; or ranges (-pi, pi], [0, 2 pi)), angular rate, angular velocity. Splines (`akima`, `zcc`,
`fdgen.splines`) are called like functions, with the coefficients of the legacy splines. Spline
data come inline (`x`, `y`) or from a text table (`file`, `header_lines`, `columns`; read as the
legacy `IO_TableDataReader`: space-separated, columns from 0, relative to the model file). The
legacy kernel evaluates such expressions with
an interpreter and differentiates them numerically; here they are compiled and exact.

**Drivers** (prescribed motion, `drivers` in YAML, legacy `Constraint TimeMotion`): a
constraint `Phi = m(q) - f(t)` per driver, `m` a position measure (displacement, distance or a
continuous angle), `f` an expression of `t`, parameters and splines, built in the model plugin.
With `ramp = Ta > 0` the legacy activation polynomial (7th order, from the initial measure
value at rest, joining `f` C2 at `Ta`) replaces `f` for `t < Ta`. The time derivatives
`Phi_t`, `Phi_tt` enter the consistent initial conditions and the velocity projection
(`phid = B v + Phi_t`, `gamma = -d/dt(B) v - 2 (B v)_t - Phi_tt`), derived by CasADi for any
element; the multiplier is the driving force or torque.

**HHT**, with `beta = (1-alpha)^2/4` and `gamma = 1/2 - alpha`, in two forms. Default
`arnold_bruls`: equations of motion at `t_{n+1}`, `alpha` only in the Newmark pseudo-acceleration
`a_{n+1} = (1+alpha) qdd_{n+1} - alpha qdd_n` (second order). `negrut` (legacy profile):
`r_dyn = M qdd/(1+alpha) + f_{n+1} - alpha/(1+alpha) f_n`. Constraint rows are scaled by the
inverse of the position sensitivity in both forms. Both forms use the same generated
functions, only with other coefficients: `(alpha, beta, gamma, h)` for Negrut,
`(0, (1+alpha) beta, (1+alpha) gamma, h)` for Arnold–Bruls. Arnold–Bruls runs on the Lie group
(Bruls, Cardona, Arnold 2012): `q_{n+1} = q_n o exp(incr)` with
`incr = h (v_n + h (1/2 - beta) a_n + h beta a_{n+1})`, Newmark on the velocities. Newton stops by the legacy rule
(`newton legacy`, profile `legacy`) or when the last correction changes the velocities by less
than `newton_tol` (`newton velocity`, YAML and profile `v2`): `gamma h |dx| <= tol (1 + |qd|)`. A
criterion directly on `x` cannot go below the round-off floor `eps/(beta h^2)` of the scaled
constraint rows.

**Step size control** (only with `adaptive`, i.e. `.fds` models; YAML models and the planned
real-time use run with a fixed step). The controller is the one of legacy FreeDyn
(`LegacyTimeStepper`: `SOL_SimTimeManagerHHT`, `SIM_SimulationObject`): step factor
`0.9 theta^(-1/6)`, halving after a rejected step, output time handling. The local error is
Negrut et al. (2007) Eq. 37a, the Newmark truncation term `(beta - 1/(6(1+alpha))) h^2 (qdd -
qdd_n)` per velocity dof, in the RMS norm with weights `eps w_i`; `theta <= 1` accepts. The
term belongs to the Newmark formulas, which both HHT forms use (Arnold-Bruls with the
pseudo-acceleration); estimators of this kind for Newmark schemes: Zienkiewicz and Xie (1991),
Zeng, Wiberg, Li and Xie (1993), Earthquake Eng. Struct. Dyn. 21, 555-571. Only the weights
differ by profile (`error_norm`):

- `legacy` (profile `legacy`): `w_i = max(1, largest |s_i| so far)` (Negrut Eq. 35b, as
  legacy FreeDyn): once a coordinate has been large, its tolerance stays loose for the rest
  of the run.
- `mixed` (profile `v2`): absolute and relative tolerance `eps`, `w_i = 1 + max(|s_i,n|,
  |s_i,n+1|)` at the last accepted and the trial state, the standard weights of Hairer,
  Norsett and Wanner, Solving ODEs I, Sec. II.4. `s` is the element `scale` function: `|R|` for
  the translations; rotations of Lie group bodies have no magnitude to be relative to (`s = 0`,
  absolute tolerance only), Euler parameters `|theta|`.

No estimator specific to the Lie group form is used: the error analysis of Arnold, Bruls and
Cardona (2015), Numer. Math. 129, 149-179, gives its leading error term, but an estimator built
on it is not established in the literature. In the examples of this repository both weights
give the same steps (all results byte-identical); they differ when a coordinate has been larger
earlier in the run than it is now (a body that passes far from the origin and comes back).

**Scatter assembly.** At setup the global CSR pattern is built from the generated sparsity
patterns, plus the CSR value index of every local nonzero of every element. Per update the value
array is zeroed and local values are added through that map.

**Linear solver** behind `fd::LinearSolver` (`factorize`, `solve`). SuiteSparse KLU (default when
found: symbolic analysis once, then numerical refactorization with the kept pivot order, full
factorization when the pivot quality degrades), Eigen `SparseLU` (pattern analyzed once) and a
dense LU are provided; MKL Pardiso is meant to go behind the same interface. Multibody iteration
matrices are small and very sparse; there KLU is about 2.5x faster than `SparseLU` (pendulum, car model), whose
supernodal kernels pay off only for larger fill-in.

## Roads from OpenCRG files

A road given as data cannot be part of the generated code (the generated functions have no
table lookups), so the tires on such a road (`tire_local`, and the tire elements of extensions
with a road offset, chosen by the generator when `road.crg` is set) take the road as a local tangent plane in their parameters,
`p[road .. road+5] = (x0, y0, h0, gx, gy, mu)`. `System::update_roads` sets it from
`fd::Road` at the wheel center `(x0, y0)` before every residual, initial-condition and output
evaluation; the iteration matrix is evaluated at the state of the preceding residual. The
residual is exact at the wheel center; the Jacobian has the road gradient but not its curvature,
which Newton tolerates (same iteration counts as on the analytic road). A tire whose contact
point lies beside the wheel center takes the tangent-plane value there.

`fd::Road` (`runtime/src/crg.cpp`) reads and evaluates the files with the official ASAM OpenCRG C
library (`runtime/third_party/opencrg`, the unmodified `c-api/baselib` of OpenCRG 2.0.0,
Apache-2.0): one OpenCRG contact point per wheel (search history), z from `crgEvalxy2z`, the
gradient by central differences over +-1e-4 m (exact inside a grid cell; the bilinear surface
has kinks at cell edges). Modifiers follow section 6.4.6 of the standard: a file with a
modifiers section gets exactly those; otherwise FreeDyn's defaults apply. They keep the file's
coordinates and fill NaN grid values with the last valid one. The library's own defaults would
move the reference point to the origin. A failed `crgCheck` (e.g. reference line curvature
above the file's warning limits) is reported and the data are used, as in the OpenCRG tools.
Relative include-file paths inside a CRG file are resolved by the library against the working
directory.

An own implementation of the format, written first without access to the specification, was
compared with the library on the sample roads of the OpenCRG repository. It matched on straight
and gently curved roads but differed elsewhere:
- the integration of the heading channel (the library uses the heading at the end of each
  segment, blended with a backward integration when the end point is given);
- default lateral positions of numbered long sections;
- the default modifiers;
- the curve mapping.

It was replaced by the library.

## Real-time mode

`solver.newton: fixed` (YAML; `freedyn2_run --newton-fixed N`) makes every step the same work:
exactly `newton_iterations` Newton corrections from the predictor (default 2), no convergence
test, no step size control (fixed step), and the linear solver keeps the pivot order of its
first factorization (KLU: numerical refactorization only, also when the pivot quality estimate
degrades; a new pivot order only if the refactorization is singular). After the last
correction the residual is not evaluated (with the Arnold-Bruls form no `f_n` term needs it),
and the element scales of the error estimate are not computed. `StepResult::correction`, the
velocity change of the last correction relative to `1 + |qd|`, is what to monitor;
`freedyn2_run` prints its largest value.

How far one or two corrections are from converged Newton (`newton velocity`, tolerance 1e-10)
at h = 1 ms, against the discretization error of that step (converged run against h = 0.1 ms);
largest deviation of the angular velocities over the run [rad/s]:

| model | discretization error | N = 1 | N = 2 |
|---|---|---|---|
| car on the dynamometer (`examples/scaling`, 5 s) | 8.0e-4 | 7.0e-9 | 1.1e-11 |
| vehicle with tires on a rough road (10 s) | 2.2e-3 | 2.4e-7 | 1.0e-10 |

The Newton error is 50 (N = 1) to 10^7 (N = 2) times smaller than the error of the step itself:
the predictor is close to the solution at 1 ms. Median time per step (one core, Xeon 2.1 GHz):

| model | converged (2-4 iterations) | N = 1 | N = 2 |
|---|---|---|---|
| car | 175 us | 81 us | 164 us |
| vehicle with tires | 297 us | 125 us | 231 us |

N = 2 is the default: the margin against strongly nonlinear events (tire lift-off, impacts) at
twice the cost of N = 1, which fits budgets that are tight.

**Determinism.** Counted instructions per step of a vehicle model (callgrind) in two phases of
a maneuver (straight running, then cornering) differ by about 3 % with N fixed (4 %
converged), and the whole difference is in the math library (`sin`, `cos`, `atan`: their cost
depends on the argument); the KLU refactorization and all generated code execute the same
number of instructions in every step. The generated element functions are straight-line code
(`if_else`, `step` and spline segments are arithmetic blends of all cases), a time step
allocates no memory (work buffers are allocated with the system; exceptions only on failure),
and the scatter assembly visits the same nonzeros every step.

The tails of the wall-clock distribution (`--timing`: min, median, p99, p99.9, max; `--timing-log`
per step, with the CPU time of the thread and the new pivot orders) come from the machine: on
the shared cloud machine of these measurements the slow steps of two identical runs are
different steps (2 of 88 in common for the car). On the target, pin the thread to a core of its
own and run it at high priority. Not deterministic: result output (CSV formatting) at output
steps, the legacy modified Newmark option (allocates).

## Runtime inputs

A program that runs a model in a loop with the outside world (a simulator, a hardware-in-the-
loop rig) changes the model's [inputs](reference/model-file.md#runtime-inputs) between steps:

```cpp
fd::Model model = fd::load_model("bike.fdm");
fd::System sys(model);
fd::HhtIntegrator hht(sys, model.solver, "klu");
const int steer = sys.input_index("T_steer");   // -1: the model has no such input
hht.initialize();
for (;;) {
  sys.set_input(steer, read_rider_torque());     // held over the next step
  hht.step();
  sys.outputs(y);                                // channels by name: sys.output_index("...")
}
```

The generator gives an input a slot in the parameters p of every element whose law uses it
(`inputs name:offset` in the model file) and writes its default (`input name value`).
`set_input` writes the value into those slots; the generated functions read p on every call,
so the next residual and iteration matrix see it. Nothing is rebuilt and nothing is
allocated: setting an input costs a few stores, and the real-time properties above hold.
The slots survive the element state update of `post_step`, which passes untouched entries
of p through.

## Viewer

The viewer is a page in the browser, served by `freedyn2 view` on localhost. Python prepares
everything that needs the model (`fdgen.view.scene`): shapes and joint frames in body
coordinates, spring and bushing points on their bodies, the road as a grid with friction
(the analytic road with the formulas of `fdgen.tire` around the tire contact points of the
results; an OpenCRG road sampled over its whole length by `freedyn2_road --grid`, so with the
runtime's library and placement). The page only places each body by its `R` and `theta`
columns of the result CSV per output time, so it needs no model knowledge and works on any
result file with these columns. Shapes never reach the `.fdm` file: the golden manifest is
unchanged by them.

The server polls the modification times of the model, its meshes and the given result file;
a change rebuilds (and re-simulates) in the background and raises the version the page polls.
`--export` writes the page, three.js, scene, results and meshes into one HTML file (the
modules as blob URLs of an import map), to pass a result on without FreeDyn.

Every simulation is a run (`View.runs`, the last 20 and the reference): the page loads the
runs it compares (`/api/runs/<id>.csv`), and while a simulation runs it draws
`/api/running.csv`, the rows written so far (the runtime flushes its CSV every 0.25 s). A new
request raises a generation counter; the rebuild of an older one kills its `freedyn2_run` and
leaves no run, so the page never waits for values that are no longer wanted.

The page also talks back (`web/panels.js`; POST requests only from the page of the server
itself): tuned numbers go to `/api/overrides`, which the server applies to the YAML data before
validation (`apply_overrides`, paths like `forces.spring.stiffness`) on every rebuild, without
touching the file; `/api/modes` linearizes about a result row with the iteration matrices of
the elements (`fdgen.linearize.modes`), for other tuned values (live, root locus) with the
constraint forces recomputed from the elements' `ic` functions in that state
(`consistent_lambda`), since those of the run belong to the model before; notes are files next to the model
(`fdgen.view.notes`), so the person and the assistant share them without a running server.
Force arrows are drawn from the force outputs alone (vectors in their frames), no model code
in the page.

## Checks and assistants

The intended way of working: an assistant writes and changes the model file, a person checks
and decides. That only works if checking is cheaper than modelling, hence `fdgen.checks`
(plausibility findings and references, plain functions on the validated model and the result
columns), used by the command line (`freedyn2 check`), the viewer (Checks panel, inspector,
runs to compare) and the MCP server. The MCP tools are methods of `fdgen.mcp_server.Session`,
testable without the SDK; `serve` only registers them. Runs live in the session's work
directory, so an assistant compares a change with the run before (`compare`) the same way the
person does in the viewer.

## Conventions

- Rigid body (`rigid_body`, default): Lie group R^3 x SO(3). Configuration `q = [R; theta]`
  (stored), velocity `v = [Rdot; omega_bar]` (6 dofs: global velocity of the center of mass,
  body-frame angular velocity). `theta = [theta0..theta3]` is a unit quaternion (Euler
  parameters, `theta0` scalar part) that only stores the rotation; the configuration is updated
  by the exponential map, so there is no normalization constraint.
- Legacy rigid body (`rigid_body_euler`, `legacy` profile only): absolute coordinates
  `q = [R; theta]`, `qd = dq/dt`, normalization constraint `theta.theta = 1`, as the legacy kernel.
- Rotation matrix exactly Shabana 2005, Eq. 2.13. `omega = 2 G thetadot` (global),
  `omega_bar = 2 Gbar thetadot` (body frame), `A = G Gbar^T` on `|theta| = 1`.
- Input `velocity` and `angular_velocity` are in **global** components. The initial
  `thetadot = 1/2 G^T omega` satisfies `theta . thetadot = 0`.
- `inertia` are the principal moments about the center of mass in the body frame.
- Output (CSV) per body: `R`, `theta`, `Rdot`, `omega` (global), `omega_body`, `T`, `H`
  (global angular momentum about the center of mass `A J omega_bar`), `theta_norm`, `Rddot`,
  `omegadot`, `omegadot_body` (from the solved accelerations `qdd`: every output function takes
  `(q, qd, qdd, p, t)`, `fdgen.kinematics.OUT_INPUTS`); per sensor (library element `sensor`,
  no load) the motion of a point in a frame fixed in its body; per joint
  primitive `<joint>.<dx|dy|dz|ax|ay|az>.lambda` (Lagrange multiplier of the scaled constraint).
- `gravity` (global acceleration) acts at the center of mass of every body, `Q = [m g; 0]`.
- Joints connect `body1` (a body) with `body2` (a body or `ground`, the default). `location`
  (global) and `orientation` (Euler parameters of the joint frame K, global) are given in the
  initial configuration; the joint frame moves with `body1` and, for body-body joints, its copy
  with `body2`. `type` selects which relative motions are fixed in K: `fixed` (all six),
  `spherical` (dx, dy, dz), `revolute` (all but az: free rotation about the K z axis),
  `cylindrical` (dx, dy, ax, ay), `prismatic` (all but dz) or `generic` with an explicit `fix`
  list, the same set as the legacy `Constraint Generic`.
- Each fixed direction is one scalar primitive constraint element, as in the legacy kernel:
  translation to the ground `n.(R1 + A1 u1) = c` (K axis `n` global), body-body
  `(R2 + A2 u2 - R1).(A1 n1) = c` (K axis carried by body1), rotation `(A1 b1).(A2 b2) = c` for
  the pairs of K axes (ax: y, z; ay: x, z; az: x, y; `A2 = I` for the ground). The constants `c`
  are the initial values, so every primitive is zero in the initial configuration. `transfix_ground`, `rotfix_ground`, `transfix_bb`, `rotfix_bb`
  (Lie group) and `*_euler` (legacy profile).
