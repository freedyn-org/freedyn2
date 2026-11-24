"""Linear FE models of flexible bodies: Nastran bulk data (nastran), element matrices
(elements), assembly with RBE2 / RBE3 (assemble), Craig-Bampton reduction and inertia
invariants (reduce, with the freedyn2_modal program)."""

from .assemble import assemble
from .nastran import read_nastran
from .reduce import reduce

__all__ = ["assemble", "read_nastran", "reduce"]


FEM_VERSION = 2  # part of the cache key: raise it when the results of the reduction change


def viz_path(mtx):
    """The mode shapes on the surface of the FE model, next to its MTX file (for the viewer)."""
    from pathlib import Path
    return Path(mtx).with_suffix(".viz.npz")


def save_viz(data, mtx):
    viz = getattr(data, "viz", None)
    if viz is not None:
        import numpy as np
        tmp = viz_path(mtx).with_suffix(".tmp.npz")
        np.savez_compressed(tmp, xyz=viz[0], tri=viz[1], phi=viz[2].astype(np.float32))
        tmp.replace(viz_path(mtx))


def cms_file(base_dir, name, flex, key, compute, describe):
    """The CMS (Craig-Bampton) result of a flexible body as files next to the model, made once:

        <base_dir>/cms/<body>.mtx       modal data (MTX: usable by any model, flex.mtx)
        <base_dir>/cms/<body>.viz.npz   mode shapes on the surface (viewer)
        <base_dir>/cms/<body>.json      what it was made from (key), frequencies, sizes, date

    flex.cms: another path for the .mtx (relative to the model). The files are reused while the
    key of the inputs is the same, else made again. A model directory that cannot be written
    (installed examples): the user cache instead. compute() -> (ModalData, info dict);
    describe: title line of the MTX file."""
    import json
    import time
    from pathlib import Path

    from ..flex.mtx import write_mtx
    from ..userdirs import cache_dir

    target = Path(base_dir or ".") / flex.get("cms", f"cms/{name}.mtx")
    meta = target.with_suffix(".json")
    try:
        if target.is_file() and json.loads(meta.read_text(encoding="utf-8")).get("key") == key:
            return target
    except (OSError, ValueError):
        pass
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        probe = target.with_suffix(".probe")
        probe.write_text("")
        probe.unlink()
    except OSError:
        target = cache_dir() / "flex" / f"{name}_{key[:16]}.mtx"
        meta = target.with_suffix(".json")
        if target.is_file():
            return target
        target.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    data, info = compute()
    tmp = target.with_suffix(".tmp")
    write_mtx(data, tmp, title=describe)
    tmp.replace(target)
    save_viz(data, target)
    record = {"key": key, "body": name, "made": time.strftime("%Y-%m-%d %H:%M:%S"),
              "seconds": round(time.time() - t0, 1), "modes": int(data.nm),
              "frequencies_hz": [float(f) for f in data.frequencies],
              "interface_nodes": [int(i) for i in data.node_labels], "mass": float(data.mass), **info}
    meta.write_text(json.dumps(record, indent=1) + "\n", encoding="utf-8")
    return target


def modal_file(path, interfaces=None, modes=20, log=None, base_dir=None, name=None, flex=None):
    """The modal data of a Nastran FE model as an MTX file (cms_file): Craig-Bampton with the
    interface nodes (default: all RBE2 master and RBE3 reference nodes), `modes`
    fixed-interface normal modes, mode shapes kept at the interface nodes."""
    import hashlib
    from pathlib import Path

    path = Path(path)
    key = hashlib.sha256(path.read_bytes())
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
