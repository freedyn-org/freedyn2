# Download

## Windows

The installer contains everything: FreeDyn 2.0, its own Python, and optionally a C compiler for
models with expression forces or drivers. It installs per user, without administrator rights,
and runs alongside legacy FreeDyn.

**[Download FreeDyn2-setup.exe](https://github.com/freedyn-org/freedyn2/releases/latest/download/FreeDyn2-setup.exe)**
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

## Code signing policy

Free code signing provided by [SignPath.io](https://about.signpath.io/), certificate by
[SignPath Foundation](https://signpath.org/).

The Windows installer and the programs in it are signed. They are built from this repository
by GitHub Actions (workflow `.github/workflows/wheels.yml`) from a version tag; nothing is built
or signed by hand. Each signing request is approved by a person.

| Role | Members |
|---|---|
| Committers and reviewers | [members of the freedyn-org organization](https://github.com/orgs/freedyn-org/people) |
| Approvers | ROLE-APPROVERS |

Changes to the main branch go through pull requests with a review. Team members use
multi-factor authentication for GitHub and SignPath.

FreeDyn 2.0 sends no data to anyone except as described in the [privacy policy](privacy.md).
