"""fdgen: freedyn2 model generator (YAML -> CasADi -> C)."""


def _version():
    """The version: of the repository's pyproject.toml when run from the sources, else of the
    installed package."""
    import re
    from pathlib import Path

    project = Path(__file__).resolve().parents[2] / "pyproject.toml"
    if (project.parent / "generator" / "fdgen").is_dir() and project.is_file():
        m = re.search(r'^version\s*=\s*"([^"]+)"', project.read_text(encoding="utf-8"), re.M)
        if m:
            return m.group(1)
    try:
        from importlib.metadata import version

        return version("freedyn2")
    except Exception:  # noqa: BLE001  (not installed, no metadata)
        return "unknown"


__version__ = _version()
