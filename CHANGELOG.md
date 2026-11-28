# Changelog

All notable changes of freedyn2. Versions follow [semantic versioning](https://semver.org);
the section of a version becomes the text of its GitHub Release (docs/releasing.md).

## [Unreleased]

### Fixed

- The modal reduction of large FE models is about 6 times faster: an own multifrontal
  supernodal LDL^T (Eigen's dense kernels, all cores; 12 times faster than Eigen's
  SimplicialLDLT, no new dependency), an AVX2 variant of freedyn2_modal taken where the CPU has
  it, Lanczos with one product with M per step, the inertia invariants from two matrix products
  (64 000 dofs: 79 s to 14 s). freedyn2_modal is signed in the Windows installer.
- Flexible bodies, from a review:
  - The Nastran reader no longer leaves out cards silently: elements, point masses (CONM2),
    rigid elements and MPC it does not support are an error; INCLUDE files are read (and are
    in the cache key of the reduction); the case control before BEGIN BULK is skipped.
  - The modal reduction checks the modes by a Sturm count (no eigenvalue below the highest
    missing; kept modes that end inside a multiple eigenvalue warn); modes that did not
    converge are an error, a residual above the tolerance a warning (it was only printed).
  - Forces, measures and sensors on the body frame of a flexible body (not at an interface
    node) warn: they do not see the deformation.
  - `freedyn2 check` warns when a flexible body spins faster than 10 % of its first elastic
    frequency (`spin_stiffening`: no geometric stiffening). The limits of the model are
    documented (docs/reference/bodies.md).
- Splines are data, not code: the runtime looks up the segment (binary search) and the
  compiled law evaluates one cubic. A driver on an Akima spline of 1000 points (legacy
  101_V8Motion) builds in 0.3 s instead of 20 minutes (1.9 MB of C before, 45 kB now), with
  bitwise the same results; other tables reuse the compiled model (the table is runtime data,
  not in p). The post function of element state writes only the state entries of p. Model
  plugins of earlier versions must be built again (registry layout 3).
- Newton (velocity criterion) at the round-off floor: a correction that stops getting smaller
  below 1e3 times the tolerance counts as converged. Models with massless bodies in mm units
  (legacy 106_V8_FlexConRod) broke down with no step size converging.
- Model plugins with very large generated sources (a driver on a spline of 1000 points) took
  20 minutes to compile: sources above 256 kB are compiled without optimization.
- `freedyn2_run --step-log` also writes the log when the run fails.
- The modal reduction finds its modes by Lanczos (shift-invert, full reorthogonalization)
  instead of a subspace iteration, and solves the constraint modes in parallel: 70 000 dofs
  and 20 modes in 26 s instead of 62 s.

### Added

- **Generalized-alpha** (`solver: rho_inf`, Chung and Hulbert, in the Lie group form of Arnold
  and Bruls; `freedyn2_run --rho-inf`): the numerical damping set by the spectral radius at
  infinity, with less damping of the resolved motion than HHT at the same value (pendulum: a
  third). HHT stays the default and gives the same results to the last bit. The spectral radius
  of the runtime is measured in a test against the textbook amplification matrix (to 1e-15).
- **PI step size control** (`adaptive: true`, `controller: pi`, the default of YAML models;
  `freedyn2_run --adaptive pi`): Gustafsson's PI controller, steps that end exactly at the
  output times, 6 to 10 times fewer rejected steps than the legacy controller for the same
  error. The local error estimate of the Arnold-Bruls and generalized-alpha forms uses their own
  Newmark constant. `.fds` models keep the legacy controller.
- The initial state is made consistent before the first step, and the run says what that
  took. **Redundant constraints** (an overconstrained mechanism, such as a planar loop of
  revolute joints) are found in model order and deactivated (multiplier 0, the others carry
  the load) instead of a singular matrix; a redundant constraint that is violated while the
  others hold stops the run with its name. **Positions that violate the constraints** are
  assembled by Newton's method (the smallest correction in the mass metric) instead of
  stopping the run. **`solver: statics: true`** (`freedyn2_run --statics`) starts from the
  static equilibrium at rest, by pseudo-transient continuation with the generated iteration
  matrix. Example `parallelogram.yaml`. Models that were consistent and not overconstrained
  run as before.
- Flexible bodies: `max_frequency` and `exclude_modes` select modes (the selection is a
  `.fdflex` of its own in the user cache); without `damping` the modes are damped as in
  Adams/Flex (1 % below 100 Hz, 10 % to 1000 Hz, 100 % above; `damping: 0` for none, the
  `.fds` import keeps the legacy value); RBE3 spiders of meshed bodies weight their nodes by
  surface area; an interface far from the surface warns.
- Splines as library items: `spring_damper` and `torsion_spring` take a spline name as
  `stiffness` or `damping` (force over the deflection, over the rate), and drivers along a
  curve of time (`motion: <spline>(t)`) on angles, distances and displacements are built into
  the runtime (six types, displacement, distance, angle to the ground or between two bodies;
  frame, component, points and axes of the measure are parameters): no model plugin, no
  compiler (legacy 101_V8Motion runs without one).
- `curve_force` and `curve_torque`: a force or torque that is a spline of a measure,
  s = a curve(b m + c), library types for any measure between its two bodies (frames,
  components and directions are parameters: 42 types). The `.fds` importer makes them of a
  legacy law `a * SPL[MEAVAL[..], ..]`.
- Fatigue output of flexible bodies (modal superposition):
  `freedyn2 modal-stress model.yaml body [result.csv]` and the MCP tool `flex_modal_stress` write the
  FE mesh with the stress tensor of each mode (Universal file) and the modal coordinates of the
  run (CSV). The reduction computes the modal stresses at the nodes and keeps them in the
  `.fdflex` (fdgen.fem.stress; HEX8 with its incompatible modes). Checked against a cantilever
  under a tip load.
- Viewer: the inspector of a joint or force at a spider of a body meshed from its STL sets the
  spider's radius and type (RBE2 / RBE3) or makes it automatic again (`flex.interfaces` in the
  model file, fdgen.flex.edit).
- MCP tools for flexible bodies: `flex_info` (modes, interfaces, spiders, what is attached),
  `flex_convergence` (frequencies over mesh sizes or numbers of modes), `flex_deformation`
  (largest deformation in a run), `flex_export`; `screenshot` with a deformation
  magnification, `modes` names the elastic mode of flexible bodies; `guide` explains them.
- Flexible bodies (floating frame of reference, modal coordinates) from Adams MTX files:
  `bodies.<name>.flex: {modal, damping, rotation_coupling, quadratic_inertia}`; joints at a node
  of a flexible body (`node` of a joint, any joint type). The body is split into element types
  of fixed size (rigid part, one mode, one pair of modes) that the runtime adds up: no model
  plugin, no compiler, any number of modes. The `.fds` importer reads `Body MBS_FlexBody_Full`
  and `Constraint FixRiBoFleBo` (default profile). Checked against the legacy check-in tests
  03_FlyingFleBo and 04_FlexRotUHackerl, free and clamped beam frequencies, energy and momentum
  conservation (docs/theory.md, 3.4).
- Flexible bodies from a Nastran FE model of the part (`flex.fem`, `interfaces`, `modes`): own
  linear FEM (TET4, TET10, HEX8 with incompatible modes, Timoshenko beams; RBE2, RBE3),
  Craig-Bampton reduction orthonormalized as Adams/Flex, inertia invariants with the consistent
  mass; the new program `freedyn2_modal` (Eigen only) does the sparse part. Checked against the
  MSC Nastran / Adams MTX files of the legacy check-in tests (docs/findings.md).
- Rigid to flex: `flex: {material, mesh_size}` on a body with an STL shape meshes it (Netgen,
  quadratic tetrahedra; extra `fem`: `pip install "freedyn2[fem]"`), makes an interface node
  with an RBE2 spider at every joint and force point, reduces and simulates it as a flexible
  body; joints act at the nodes, forces on massless bodies fixed to them.
- `freedyn2 flex-export model.yaml body`: the meshed body as Nastran bulk data, to change its
  spiders or mesh and use it with `flex.fem`. Joints and forces without `node` attach to the
  interface node at their point for every flexible body (also `fem` and `modal`); a torque and a
  torsion spring at a flexible body need their point.
- The CMS result of a reduction is kept next to the model in freedyn2's own modal data file
  (`cms/<body>.fdflex`: a ZIP archive of NumPy arrays with the modal data, the mode shapes on
  the surface and what it was made from), reused while its inputs are the same and usable by
  other models with `flex.modal`; `flex.cms` gives another place. `freedyn2 flex-info` shows a
  modal data file and converts between .fdflex and Adams MTX.
- Spiders in bores: a revolute or cylindrical joint at a body meshed from its STL connects
  to the bore around its axis (`interfaces[].axis` for others); the viewer shows the spiders
  of flexible bodies, a click on one inspects its joint or force with the spider's size.
- Viewer: flexible bodies deform (their STL, or the surface of the FE mesh), magnified with
  *deformation* about their best-fitting rigid motion; eigenmodes with the deformation. Mesh
  shapes count for the size of the scene and *fit view*.

## [0.1.0rc4] - 2026-11-07

Release candidate with the Onshape import, element ids and renaming that follows every
reference. Not signed: Windows SmartScreen warns on the first start; check the download against
SHA256SUMS.

### Added

- `freedyn2 import-onshape <assembly address>` (prototype): an Onshape assembly as a model file,
  read with the Onshape REST API and API keys (`ONSHAPE_ACCESS_KEY`, `ONSHAPE_SECRET_KEY`).
  Parts connected by Fastened mates or mate groups become one body with the added-up mass
  properties from Onshape; fixed parts become the ground; revolute, slider, cylindrical, ball,
  planar, parallel and pin slot mates become joints; other mates are listed, not imported. An
  STL file per part for the viewer, and the Onshape reference of every part in
  `<model>.onshape.json`. See docs/from-onshape.md.
- Element ids: an optional `id` on bodies, joints, forces, measures, sensors, drivers, splines,
  frames and shapes, the element's stable identity for tools (references keep using names);
  parameters, inputs, outputs and points take the long form `{value: ..., id: ...}`.
  Unique in the model. `convert` writes the legacy ids (`fds:body:1`), `import-onshape`
  Onshape's instance and feature ids (`onshape:body:...`).
- `freedyn2 import-onshape ... --update`: the assembly imported again into a model file worked
  on since. Bodies and joints are matched by their ids and get Onshape's new values; names,
  own keys, own shapes, comments and everything without an Onshape id (measures, drivers,
  forces, solver) stay; new parts and mates are added, deleted ones removed unless still
  referred to.
- `freedyn2 rename model.yaml old new` and the assistant's tool `rename`: an element and every
  reference to it (bodies, points, frames, expressions, formulas, channels in checks), changed
  only where the name stands; comments and formatting stay. In the viewer's editor: F2. An
  element with an id renamed by hand (in the editor or another one) is recognized by its id;
  the editor and the viewer's header offer to update the references left.

### Fixed

- The viewer's editor: Ctrl+Z right after opening a file undid the loading and left the editor
  empty (a save then wrote an empty file). A loaded file now starts its own undo history.

## [0.1.0rc3] - 2026-11-06

Release candidate with the model editor in the viewer and a much smaller Windows installer
(core 195 instead of about 390 MB). Not signed: Windows SmartScreen warns on the first start;
check the download against SHA256SUMS.

### Added

- The viewer edits the model file: a click on the file name (or E) opens an editor beside the
  3D view, with syntax highlighting, completion from the schema and the model (keys, types,
  bodies, points, parameters), templates for new bodies, joints and forces, help on hover, and
  errors at their lines while typing. Ctrl+S saves and simulates; a file changed meanwhile by
  the assistant or another editor is reloaded or, with unsaved changes, not overwritten. The
  element under the cursor is selected in the 3D view and the other way round.

### Changed

- The viewer's report (Report) in the brand: ink on paper, JetBrains Mono and the word mark,
  still one HTML file.
- Windows installer much smaller: CasADi without its solver plugins (IPOPT, HiGHS, OpenBLAS, ...,
  which freedyn2 never uses) and Zig with the C libraries for Windows only. The core goes from
  about 380 to 165 MB, the C compiler from about 380 to 320 MB. The build checks that CasADi
  still generates code and that Zig still compiles a DLL.

### Fixed

- Viewer: unsaved changes in the editor showed *up to date* in the header. The header now
  shows *model file not saved* with *Save & compute* and *discard*, and the results fade as for
  tuned values not computed. The compute state in the header is no longer cut off when the path of
  the model is long.
- Windows installer: an upgrade from 0.1.0rc1 or rc2 removes the old start menu folder
  *FreeDyn 2* (with the shortcuts rc2 had put into it) and the old desktop icon; the entries are
  then only in the folder *freedyn2*.

## [0.1.0rc2] - 2026-11-06

Release candidate in the brand (brand/README.md). Not signed: Windows SmartScreen warns on the
first start; check the download against SHA256SUMS.

### Changed

- The name is `freedyn2` everywhere; the installer is `freedyn2-setup.exe`, the winget
  identifier `freedyn-org.freedyn2`.
- App, viewer, installer and documentation in the brand: word mark, icon, colours, fonts
  (JetBrains Mono, Inter in the docs), the claim "ai-first multibody dynamics.".
- The viewer shows whether an assistant is connected (agent status).

### Fixed

- Windows: the viewer served its fonts with a wrong content type.

## [0.1.0rc1] - 2026-11-06

Release candidate: a test of the release process (PyPI, GitHub Release, Windows installer).
Not signed yet. Install it explicitly: `pip install --pre freedyn2` or the installer from this
release.

### Added

- Wheels for Windows (x64) and Linux (x86_64): `pip install freedyn2`, `uv tool install freedyn2`;
  extras `gui` (app window, pywebview) and `compiler` (Zig, a C compiler for model plugins).
- Windows installer `freedyn2-setup.exe`: per user without administrator rights, its own Python,
  the app, the command line, optionally Zig, the .fds association and the Claude Desktop setup.
- The app `freedyn2-gui` / `freedyn2 gui`: start page (open, recent models, examples), a window
  per model, one instance, an update notice for new releases.
- `freedyn2 convert`: legacy .fds models as YAML model files.
- The Python API: `import freedyn2` (load, run, check, sweep, convert).
- `freedyn2 setup-mcp`, `freedyn2 doctor`, `freedyn2 config`, `freedyn2 update-check`.
- Compiler selection: Zig, MSVC without a developer prompt (vswhere), gcc, clang.

## [0.1.0] - unreleased

First release.

### Added

- Wheels for Windows (x64) and Linux (x86_64) on PyPI: `pip install freedyn2`, `uv tool install freedyn2`.
- Windows installer `freedyn2-setup.exe` (per user, no administrator rights): its own Python, the
  app, the viewer, the MCP server and optionally a C compiler (Zig) for model plugins.
- ...
