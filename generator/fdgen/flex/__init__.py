"""Flexible bodies: modal data (the own format .fdflex, Adams MTX files as import and export)
and the inertia invariants the flexible body element types are built from (fdgen.flex_body)."""

from .modal import ModalData
from .fdflex import read_fdflex, write_fdflex
from .mtx import read_mtx, write_mtx

__all__ = ["ModalData", "read_mtx", "write_mtx", "read_fdflex", "write_fdflex", "load_modal", "select_modes",
           "default_damping"]


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


def select_modes(path, max_frequency=None, exclude=()):
    """The modal data file with only the modes up to max_frequency (Hz) and without the modes
    exclude (numbers from 1, in the order of the file): path itself if all are kept, else a
    .fdflex in the user cache (made once; meta: kept_modes, the numbers kept). The selected
    modes are numbered from 1 again."""
    import hashlib
    from pathlib import Path

    from ..errors import ModelError
    from ..userdirs import cache_dir
    d = load_modal(path)
    keep = [i for i in range(d.nm) if (max_frequency is None or d.frequencies[i] <= max_frequency)
            and i + 1 not in set(exclude)]
    bad = [m for m in exclude if not 1 <= m <= d.nm]
    if bad:
        raise ModelError(f"exclude_modes {bad}: the modal data have modes 1 .. {d.nm}")
    if len(keep) == d.nm:
        return Path(path)
    if not keep:
        raise ModelError(f"no mode left (max_frequency {max_frequency} Hz, the lowest is {d.frequencies.min():.4g} Hz)")
    p = Path(path).resolve()
    key = hashlib.sha256(repr((str(p), p.stat().st_mtime_ns, p.stat().st_size, keep)).encode()).hexdigest()[:20]
    out = cache_dir() / "flex" / f"{p.stem}_modes_{key}.fdflex"
    if not out.is_file():
        out.parent.mkdir(parents=True, exist_ok=True)
        sel = d.select(keep)
        meta = dict(sel.meta, kept_modes=[i + 1 for i in keep], selected_from=str(p))
        write_fdflex(sel, out, meta)
    return out


def default_damping(frequencies):
    """Modal damping ratios when none is given, as Adams/Flex: 1 % below 100 Hz, 10 % from 100 Hz
    to 1000 Hz, 100 % from 1000 Hz (high modes, e.g. the constraint modes of the interfaces,
    are critically damped)."""
    import numpy as np
    f = np.asarray(frequencies, float)
    return np.where(f < 100.0, 0.01, np.where(f < 1000.0, 0.1, 1.0)).tolist()
