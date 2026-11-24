# Changelog

All notable changes of freedyn2. Versions follow [semantic versioning](https://semver.org);
the section of a version becomes the text of its GitHub Release (docs/releasing.md).

## [Unreleased]

### Fixed

- Newton (velocity criterion) at the round-off floor: a correction that stops getting smaller
  below 1e3 times the tolerance counts as converged. Models with massless bodies in mm units
  (legacy 106_V8_FlexConRod) broke down with no step size converging.
- Model plugins with very large generated sources (a driver on a spline of 1000 points) took
  20 minutes to compile: sources above 256 kB are compiled without optimization.
- `freedyn2_run --step-log` also writes the log when the run fails.

### Added

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
