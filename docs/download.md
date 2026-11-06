# Download

## Windows

The installer contains everything: freedyn2, its own Python, and optionally a C compiler for
models with expression forces or drivers. It installs per user, without administrator rights,
and runs alongside legacy FreeDyn.

**[Download freedyn2-setup.exe](https://github.com/freedyn-org/freedyn2/releases/latest/download/freedyn2-setup.exe)**
(the latest release; all releases, their notes and SHA256 checksums:
[GitHub Releases](https://github.com/freedyn-org/freedyn2/releases))

!!! note "Before the first release"
    The installer is published with the first release. Until then, install with pip or uv.

## Python (Windows and Linux)

```sh
uv tool install "freedyn2[gui,compiler]"    # as a program, with uv
pip install "freedyn2[gui,compiler]"        # into the current Python environment
```

The package is on [PyPI](https://pypi.org/project/freedyn2/). Which way suits you, updating
and uninstalling: [Getting started](getting-started.md#install).

## Source code

[github.com/freedyn-org/freedyn2](https://github.com/freedyn-org/freedyn2), Apache License 2.0.

## Code signing

The Windows installer is **not signed yet**. Windows SmartScreen therefore warns on the first
start ("Windows protected your PC"): choose *More info*, then *Run anyway*.

To check that a download is the one built here, compare its SHA256 checksum with
`SHA256SUMS` of the release:

```powershell
Get-FileHash freedyn2-setup.exe -Algorithm SHA256
```

The installer and everything in it are built from this repository by GitHub Actions
(workflow `.github/workflows/wheels.yml`) from a version tag; nothing is built by hand.

freedyn2 sends no data to anyone except as described in the [privacy policy](privacy.md).
