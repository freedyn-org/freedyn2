"""Linear FE models of flexible bodies: Nastran bulk data (nastran), element matrices
(elements), assembly with RBE2 / RBE3 (assemble), Craig-Bampton reduction and inertia
invariants (reduce, with the freedyn2_modal program)."""

from .assemble import assemble
from .nastran import read_nastran
from .reduce import reduce

__all__ = ["assemble", "read_nastran", "reduce"]


FEM_VERSION = 1  # part of the cache key: raise it when the results of the reduction change


def modal_file(path, interfaces=None, modes=20, log=None):
    """The modal data of a Nastran FE model as an MTX file in the user cache (made once per
    content of the file, interfaces and number of modes): Craig-Bampton with the interface nodes
    (default: all RBE2 master and RBE3 reference nodes), `modes` fixed-interface normal modes,
    mode shapes kept at the interface nodes."""
    import hashlib
    from pathlib import Path

    from ..userdirs import cache_dir
    from ..flex.mtx import write_mtx

    path = Path(path)
    key = hashlib.sha256(path.read_bytes())
    key.update(repr((FEM_VERSION, None if interfaces is None else sorted(int(i) for i in interfaces),
                     int(modes))).encode())
    out = cache_dir() / "flex" / f"{path.stem}_{key.hexdigest()[:16]}.mtx"
    if out.is_file():
        return out
    fe = read_nastran(path)
    a = assemble(fe)
    ifs = sorted(a.masters) if interfaces is None else [int(i) for i in interfaces]
    if log:
        log(f"{path.name}: {a.n} dofs, interfaces {ifs}, {modes} modes")
    data = reduce(a, ifs, int(modes), output_nodes=ifs)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    write_mtx(data, tmp, title=f"{path.name}, freedyn2 FEM, Craig-Bampton, interfaces {' '.join(map(str, ifs))}")
    tmp.replace(out)
    return out
