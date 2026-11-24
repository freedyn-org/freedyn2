"""Flexible bodies: modal data (the own format .fdflex, Adams MTX files as import and export)
and the inertia invariants the flexible body element types are built from (fdgen.flex_body)."""

from .modal import ModalData
from .fdflex import read_fdflex, write_fdflex
from .mtx import read_mtx, write_mtx

__all__ = ["ModalData", "read_mtx", "write_mtx", "read_fdflex", "write_fdflex", "load_modal"]


_CACHE = {}


def load_modal(path):
    """ModalData of a modal data file (.fdflex, or an Adams .mtx), read once per path and
    modification time."""
    from pathlib import Path
    p = Path(path).resolve()
    key = (str(p), p.stat().st_mtime_ns)
    if key not in _CACHE:
        if p.suffix.lower() not in (".mtx", ".fdflex"):
            from ..errors import ModelError
            raise ModelError(f"{path}: modal data are a .fdflex or an Adams .mtx file")
        if len(_CACHE) >= 8:  # large files: keep a few
            _CACHE.pop(next(iter(_CACHE)))
        _CACHE[key] = read_fdflex(p) if p.suffix.lower() == ".fdflex" else read_mtx(p)
    return _CACHE[key]
