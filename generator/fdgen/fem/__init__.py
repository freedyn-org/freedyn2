"""Linear FE models of flexible bodies: Nastran bulk data (nastran), element matrices
(elements), assembly with RBE2 / RBE3 (assemble), Craig-Bampton reduction and inertia
invariants (reduce, with the freedyn2_modal program)."""

from .assemble import assemble
from .nastran import read_nastran
from .reduce import reduce

__all__ = ["assemble", "read_nastran", "reduce"]


FEM_VERSION = 6  # part of the cache key: raise it when the results of the reduction change


def cms_file(base_dir, name, flex, key, compute, describe):
    """The CMS (Craig-Bampton) result of a flexible body as a file next to the model, made once:
    <base_dir>/cms/<body>.fdflex (fdgen.flex.fdflex: modal data, mode shapes on the surface for
    the viewer, and in its meta what it was made from: the key of the inputs, frequencies,
    sizes, time). flex.cms: another path (relative to the model). It is reused while the key is
    the same, else made again; other models use it with flex.modal. A model directory that
    cannot be written (installed examples): the user cache instead. compute() -> (ModalData,
    info dict); describe: a line for the meta."""
    import time
    from pathlib import Path

    from ..errors import ModelError
    from ..flex.fdflex import read_meta, write_fdflex
    from ..userdirs import cache_dir

    target = Path(base_dir or ".") / flex.get("cms", f"cms/{name}.fdflex")
    if target.suffix.lower() != ".fdflex":
        raise ModelError(f"body {name}: cms must be a .fdflex file")
    try:
        if target.is_file() and read_meta(target).get("key") == key:
            return target
    except (OSError, ValueError, KeyError, ModelError):
        pass
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        probe = target.with_suffix(".probe")
        probe.write_text("")
        probe.unlink()
    except OSError:
        target = cache_dir() / "flex" / f"{name}_{key[:16]}.fdflex"
        if target.is_file():
            return target
        target.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    data, info = compute()
    record = {"key": key, "body": name, "description": describe, "made": time.strftime("%Y-%m-%d %H:%M:%S"),
              "seconds": round(time.time() - t0, 1), "modes": int(data.nm),
              "frequencies_hz": [float(f) for f in data.frequencies],
              "interface_nodes": [int(i) for i in data.node_labels], "mass": float(data.mass), **info}
    write_fdflex(data, target, record)
    return target


def modal_file(path, interfaces=None, modes=20, log=None, base_dir=None, name=None, flex=None):
    """The modal data of a Nastran FE model as a .fdflex file (cms_file): Craig-Bampton with the
    interface nodes (default: all RBE2 master and RBE3 reference nodes), `modes`
    fixed-interface normal modes, mode shapes kept at the interface nodes."""
    import hashlib
    from pathlib import Path

    from .nastran import bulk_files

    path = Path(path)
    key = hashlib.sha256(bulk_files(path)[0].encode("latin-1"))  # with the INCLUDE files
    key.update(repr((FEM_VERSION, None if interfaces is None else sorted(int(i) for i in interfaces),
                     int(modes))).encode())

    def compute():
        fe = read_nastran(path)
        a = assemble(fe)
        ifs = sorted(a.masters) if interfaces is None else [int(i) for i in interfaces]
        if log:
            log(f"{path.name}: {a.n} dofs, interfaces {ifs}, {modes} modes")
        return reduce(a, ifs, int(modes), output_nodes=ifs), {"source": str(path), "dofs": int(a.n),
                                                              "elements": len(fe.solids) + len(fe.bars)}

    return cms_file(base_dir or path.parent, name or path.stem, flex or {}, key.hexdigest(), compute,
                    f"{path.name}, freedyn2 FEM, Craig-Bampton")
