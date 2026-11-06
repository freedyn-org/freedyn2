# Releasing

How a version of FreeDyn 2.0 gets to its users, and what a maintainer sets up for it. Everything
runs in the workflow `Wheels` (`.github/workflows/wheels.yml`) when a version tag `vX.Y.Z` is
pushed:

| Job | What it does |
|---|---|
| `version` | the tag must equal `version` in `pyproject.toml` (tag `v0.1.0` for `0.1.0`), else everything stops |
| `wheels`, `sdist` | the wheels for Windows x64 and Linux x86_64 (cibuildwheel, tested), the source distribution |
| `installer` | the Windows installer from the Windows wheel (`installer/windows`), signed if signing is set up, then installed silently, used (`freedyn2 --version`, `doctor`, `run` on an `.fds` and on a model with a plugin compiled by Zig, `setup-mcp --dry-run`) and uninstalled |
| `publish` | the wheels and the sdist to PyPI (trusted publishing, environment `pypi`) |
| `release` | the GitHub Release `vX.Y.Z`: wheels, sdist, `FreeDyn2-X.Y.Z-setup.exe`, the same as `FreeDyn2-setup.exe`, `SHA256SUMS`; text from `CHANGELOG.md` plus generated notes |
| `winget` | a pull request to [microsoft/winget-pkgs](https://github.com/microsoft/winget-pkgs) with the new version (if `WINGET_TOKEN` is set) |

Pull requests that change the packaging (`pyproject.toml`, `CMakeLists.txt`, `tools/`,
`installer/`, the workflow) and a manual run (Actions, Wheels, Run workflow) do the same up to and
including the installer's test, without signing and without publishing. The installer of such a
run is the artifact `installer-windows`.

A version with letters (`0.2.0rc1`, `0.2.0b1`) is a pre-release: on PyPI as such, on GitHub
marked as pre-release and not "latest" (so the stable download link and the app's update notice
stay on the last stable version), and not sent to winget.

## Once: what a maintainer sets up

### The repository

- **Public.** The download links of the releases (and the app's update check through the GitHub
  API) work without login only for a public repository. Also required by the SignPath
  Foundation.
- Releases are **GitHub Releases with tags `vX.Y.Z`**: the app reads
  `https://api.github.com/repos/freedyn-org/freedyn2/releases/latest` for its update notice, the
  stable link
  [`https://github.com/freedyn-org/freedyn2/releases/latest/download/FreeDyn2-setup.exe`](https://github.com/freedyn-org/freedyn2/releases/latest/download/FreeDyn2-setup.exe)
  points to the installer of the latest stable release.
- Settings, Actions, General, Workflow permissions: the default (read) is enough; the release job
  asks for `contents: write` itself.
- Optional: a tag protection rule (Settings, Rules, Rulesets, target tags `v*`) so that only
  maintainers can push version tags.

### PyPI (trusted publishing, no token)

1. An account on [pypi.org](https://pypi.org/account/register/) with two-factor authentication
   (required for publishing). Ideally two maintainers, both owners of the project later.
2. Your account, Publishing, **Add a new pending publisher** (the project `freedyn2` does not
   exist yet; the first upload creates it):
   PyPI project name `freedyn2`, owner `freedyn-org`, repository name `freedyn2`, workflow name
   `wheels.yml`, environment name `pypi`.
3. On GitHub: Settings, Environments, **New environment `pypi`**. Protection rules: "Required
   reviewers" (one or two maintainers) so that every upload to PyPI is approved by hand, and
   "Deployment branches and tags": selected tags, `v*`.
4. Optional, for testing the whole path first: the same on [test.pypi.org](https://test.pypi.org)
   (a pending publisher there; the workflow would need `repository-url:
   https://test.pypi.org/legacy/` in the publish step for such a test run).

### Code signing of the Windows installer (optional, recommended)

Without signing everything works, but Windows SmartScreen warns about an "unknown publisher"
when the installer is downloaded and started ("More info", "Run anyway"), and some company
computers block unsigned programs. A signature names the publisher; SmartScreen's reputation grows
with downloads of signed files.

The workflow signs with **SignPath** when the secret `SIGNPATH_API_TOKEN` exists, and only for
version tags: first FreeDyn 2.0's own programs that go into the installer (`freedyn2.exe`,
`freedyn2-gui.exe`, `freedyn2_run.exe`, `freedyn2_road.exe`), then the installer. Without the
token these steps are skipped and the installer is unsigned. The Inno Setup uninstaller
(`unins000.exe`) remains unsigned in this setup.

**SignPath Foundation** ([signpath.org](https://signpath.org)) signs open-source projects free of
charge with its own certificate; the publisher shown is then "SignPath Foundation", and the
signature attests that the file was built from the project's repository. Requirements (check
the current conditions on [signpath.org](https://signpath.org) before applying), which FreeDyn
2.0 meets as set up here:

- an OSI-approved license (Apache-2.0), no proprietary components;
- a public repository, the software actively maintained and already released or about to be;
- the signed files built in CI from that repository (GitHub-hosted runners; SignPath checks the
  origin of each artifact through the GitHub API);
- a documented release process (this page) and roles: who may submit, and who approves each
  release signing request (manual approval of every release);
- a code signing policy on the project's website (the roles above, what is signed: the artifact
  configurations below, a privacy statement) and the note "Free code signing provided by
  SignPath.io, certificate by SignPath Foundation", e.g. in the README's install section.

Steps:

1. Apply on [signpath.org](https://signpath.org) (the application form for open-source
   projects) with the repository URL. How long the review takes is not stated: unknown, to be
   checked (allow some weeks).
2. After approval SignPath creates the organization and the project; in it:
    - a **trusted build system** "GitHub.com" linked to the repository (the SignPath GitHub app);
    - two **artifact configurations**, contents from the repository:
      [`installer/windows/signpath/programs.xml`](https://github.com/freedyn-org/freedyn2/blob/main/installer/windows/signpath/programs.xml)
      (slug e.g. `programs`: the zip artifact `programs-unsigned` with the paths of the programs
      relative to the installation directory, each Authenticode-signed) and
      [`installer/windows/signpath/installer.xml`](https://github.com/freedyn-org/freedyn2/blob/main/installer/windows/signpath/installer.xml)
      (slug e.g. `installer`: the artifact `installer-unsigned` with `FreeDyn2-*-setup.exe`).
      Two requests are needed because SignPath cannot sign inside an Inno Setup installer: the
      programs are signed before Inno Setup packs them;
    - a **signing policy** (e.g. `release-signing`) with the approvers, origin verification
      ("allowed branches/tags": `refs/tags/v*`);
    - a CI user with an **API token** for submitting requests.
3. On GitHub (Settings, Secrets and variables, Actions):
    - secret `SIGNPATH_API_TOKEN`: the API token of the CI user;
    - variables `SIGNPATH_ORGANIZATION_ID` (a GUID), `SIGNPATH_PROJECT_SLUG` (e.g. `freedyn2`),
      `SIGNPATH_SIGNING_POLICY_SLUG` (e.g. `release-signing`), `SIGNPATH_CONFIG_PROGRAMS`
      (e.g. `programs`), `SIGNPATH_CONFIG_INSTALLER` (e.g. `installer`).
4. At every release an approver approves the two signing requests in SignPath (an e-mail comes;
   the job waits up to two hours for each).

Alternatives (not wired up; they replace the steps "Sign the programs" and "Sign the installer",
which read `build/installer/to-sign` and `dist-installer` and write the signed files back):

- **Azure Artifact Signing** (formerly Trusted Signing, action `azure/artifact-signing-action`):
  about 10 USD per month (Basic), the publisher is the organization itself; needs an Azure
  subscription and an identity validation of the organization (a legal entity with some years of
  history; individuals only in some countries).
- **A classic OV or EV certificate** from a certificate authority (some hundred USD per year; the
  key on a hardware token or in a cloud HSM, so it needs a signing service reachable from CI).

### winget (Windows Package Manager)

Package identifier proposed: **`FreeDynOrg.FreeDyn2`** (to be confirmed before the first
submission: it cannot be changed easily afterwards). Templates in
[`installer/winget`](https://github.com/freedyn-org/freedyn2/tree/main/installer/winget).

1. **First version by hand**, after its GitHub Release exists: on Windows,
   download the released `FreeDyn2-X.Y.Z-setup.exe`, then
   `python installer/winget/render.py X.Y.Z FreeDyn2-X.Y.Z-setup.exe` writes the manifests to
   `build/winget/manifests/f/FreeDynOrg/FreeDyn2/X.Y.Z`;
   `winget validate --manifest <that directory>`, `winget install --manifest <that directory>`
   (needs `winget settings --enable LocalManifestFiles` as administrator) to test; then a pull
   request to microsoft/winget-pkgs with that directory (fork, commit, PR; or
   `wingetcreate submit <directory>`). Microsoft's validation runs on the PR; a moderator merges it.
2. **Later versions automatically**: a GitHub account with a fork of microsoft/winget-pkgs and a
   classic personal access token with scope `public_repo`, as the secret `WINGET_TOKEN`. The job
   `winget` then runs `wingetcreate update FreeDynOrg.FreeDyn2` with the new installer for both
   scopes (user, machine) and opens the pull request.

### Documentation site

The documentation link of the installer's Start menu points to
`https://freedyn-org.github.io/freedyn2/` (GitHub Pages of this repository).

## Per release

1. **Version**: set `version` in `pyproject.toml` (e.g. `0.2.0`; pre-releases `0.2.0rc1`).
2. **Changelog**: in `CHANGELOG.md` move the entries under `## [Unreleased]` to a new section
   `## [0.2.0] - 2026-12-01`. Its text becomes the text of the GitHub Release.
3. Commit, pull request, merge to `main` (the CI and, as the version changed `pyproject.toml`,
   the Wheels workflow with the installer test run on the pull request).
4. **Tag** the merge commit on `main` and push the tag:

    ```sh
    git checkout main && git pull
    git tag -a v0.2.0 -m "FreeDyn 2.0 0.2.0"
    git push origin v0.2.0
    ```

5. **What happens**: the Wheels workflow runs (about an hour): checks the version, builds and
   tests the wheels, builds, signs (approve the two SignPath requests) and tests the installer,
   waits for the approval of the environment `pypi` (approve it under the workflow run), uploads
   to PyPI, creates the GitHub Release, opens the winget pull request.
6. **Verify**:
    - [pypi.org/project/freedyn2](https://pypi.org/project/freedyn2/) shows the version;
      `uvx freedyn2@0.2.0 --version` prints it;
    - the release page lists the wheels, the sdist, both installer names and `SHA256SUMS`;
      `https://github.com/freedyn-org/freedyn2/releases/latest/download/FreeDyn2-setup.exe`
      downloads the new installer (stable releases only);
    - on a Windows computer: download, check the hash (`Get-FileHash FreeDyn2-setup.exe`
      against `SHA256SUMS`), right click, Properties, Digital Signatures (if signed), install
      over the previous version, start "FreeDyn 2.0" from the Start menu, `freedyn2 doctor` in
      the "FreeDyn 2.0 Command Prompt";
    - later: the winget pull request is merged (`winget show FreeDynOrg.FreeDyn2`).

If a job fails after PyPI accepted the upload: a version on PyPI cannot be replaced. Fix the
cause and re-run the failed jobs of the same run (the release and winget jobs can be re-run on
their own); if the code itself must change, release the next patch version.

## The Windows installer in detail

`installer/windows/build.py` builds it from the Windows wheel (locally on Windows with Inno
Setup 6 installed: `python installer/windows/build.py all --wheel-dir wheelhouse`; on any system
`--dry-run` checks the steps without Windows):

- a pinned CPython 3.12 of [python-build-standalone](https://github.com/astral-sh/python-build-standalone)
  (version and SHA256 in `build.py`; to update, take both from the release's `SHA256SUMS`);
- `freedyn2[gui,compiler]` and all dependencies as wheels (`pip wheel`, which also builds the
  few that only have a source distribution), then installed offline from them;
- the launchers `freedyn2.exe`, `freedyn2-gui.exe` made relocatable (their interpreter path
  relative to the launcher), in `bin\` (on the PATH) and in `python\Scripts\`;
- Zig (`ziglang`) separated as the component "C compiler"; tests and caches removed; bytecode
  precompiled; `licenses\python-packages.txt` generated from the packages' metadata.

`installer/windows/freedyn2.iss` (Inno Setup): per user by default
(`%LOCALAPPDATA%\Programs\FreeDyn2`) or for all users (`C:\Program Files\FreeDyn2`), components
core, C compiler, examples; tasks PATH, `.fds` association, "Open with FreeDyn 2.0" for `.yaml`,
Claude Desktop registration (`freedyn2 setup-mcp --client claude-desktop`, undone at uninstall),
desktop icon. A new version installs over the old one. Uninstalling keeps the user data in
`%APPDATA%\freedyn2` and `%LOCALAPPDATA%\freedyn2`. Silent:

```bat
FreeDyn2-setup.exe /VERYSILENT /SUPPRESSMSGBOXES /NORESTART /CURRENTUSER /TASKS="path"
FreeDyn2-setup.exe /VERYSILENT /ALLUSERS /COMPONENTS="core" /TASKS="path,assocfds"
"%LOCALAPPDATA%\Programs\FreeDyn2\unins000.exe" /VERYSILENT /SUPPRESSMSGBOXES /NORESTART
```

Task names: `path`, `assocfds`, `openyaml`, `claudedesktop`, `desktopicon`; components: `core`,
`compiler`, `examples`. `/TASKS` replaces the default selection, `/MERGETASKS` adds to it.
