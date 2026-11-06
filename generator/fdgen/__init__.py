"""fdgen: FreeDyn 2.0 model generator (YAML -> CasADi -> C)."""


def _version():
    try:
        from importlib.metadata import version

        return version("freedyn2")
    except Exception:  # noqa: BLE001  (from the sources: pyproject.toml of the repository)
        import re
        from pathlib import Path

        try:
            text = (Path(__file__).resolve().parents[2] / "pyproject.toml").read_text(encoding="utf-8")
        except OSError:
            return "0+unknown"
        m = re.search(r'^version\s*=\s*"([^"]+)"', text, re.M)
        return m.group(1) if m else "0+unknown"


__version__ = _version()
