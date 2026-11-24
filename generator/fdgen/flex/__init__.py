"""Flexible bodies: modal data (Adams MTX files, later the own linear FEM) and the inertia
invariants the flexible body element types are built from (fdgen.flex_body)."""

from .modal import ModalData
from .mtx import read_mtx

__all__ = ["ModalData", "read_mtx", "load_modal"]


_CACHE = {}


def load_modal(path):
    """ModalData of a modal data file (.mtx), read once per path and modification time."""
    from pathlib import Path
    p = Path(path).resolve()
    key = (str(p), p.stat().st_mtime_ns)
    if key not in _CACHE:
        if p.suffix.lower() != ".mtx":
            from ..errors import ModelError
            raise ModelError(f"{path}: modal data must be an Adams MTX file (.mtx)")
        if len(_CACHE) >= 8:  # large files: keep a few
            _CACHE.pop(next(iter(_CACHE)))
        _CACHE[key] = read_mtx(p)
    return _CACHE[key]
