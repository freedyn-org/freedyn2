# Changelog

All notable changes of freedyn2. Versions follow [semantic versioning](https://semver.org);
the section of a version becomes the text of its GitHub Release (docs/releasing.md).

## [Unreleased]

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
