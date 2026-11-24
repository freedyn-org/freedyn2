"""The freedyn2 modal data file (.fdflex): everything the flexible body element types and the
viewer need from a reduction, in one file.

Format: a ZIP archive of NumPy arrays (.npy, numpy.savez_compressed), readable with numpy
(numpy.load) or any ZIP and .npy reader. Arrays (nm modes, nn nodes kept, units of the model):

    format        "fdflex 1"
    meta          JSON text: what the data were made from (key of the inputs, source, mesh,
                  material, interfaces, time) and anything a tool adds
    mass          ()            I2 (3)          I3, I4 (3, nm)       I5 (nm, 3, nm)
    I6, K (nm, nm)              I7 (3, 3)       I8 (nm, 3, 3)        I9 (nm, nm, 3, 3)
    frequencies   (nm) Hz       has_I5, has_I9  () bool (full coupling data present)
    node_labels   (nn) int      node_positions (nn, 3)   Phi, Psi (nn, 3, nm)
    viz_xyz, viz_tri, viz_phi   optional: mode shapes on the surface of the FE model (viewer)
    spider_node, spider_type, spider_start, spider_xyz   optional: the RBE2 / RBE3 of the
                  interfaces: node, "rbe2" or "rbe3", the positions of the nodes it connects
                  (spider_xyz[spider_start[k]:spider_start[k + 1]])
    mesh_node, mesh_xyz, mesh_tet4, mesh_tet10, mesh_hex8, stress
                  optional: the solids of the FE model (node labels, positions, elements as
                  indices into them, Nastran node order) and the modal stresses at its nodes
                  (nn_mesh, 6, nm) float32, components xx yy zz xy yz zx, averaged over the
                  elements at a node; for fatigue by modal superposition (freedyn2 femfat)

The inertia invariants are those of fdgen.flex.modal (Adams' definitions).
"""

import json
from pathlib import Path

import numpy as np

from ..errors import ModelError
from .modal import ModalData

FORMAT = "fdflex 1"
_ARRAYS = ("I2", "I3", "I4", "I5", "I6", "I7", "I8", "I9", "K", "frequencies", "node_labels", "node_positions",
           "Phi", "Psi")
_CELLS = {4: "tet4", 10: "tet10", 8: "hex8"}


def write_fdflex(data, path, meta=None):
    """Write ModalData (with its .viz, if any) and meta (dict) to path; atomically."""
    path = Path(path)
    arrays = {k: np.asarray(getattr(data, k)) for k in _ARRAYS}
    arrays.update(format=np.array(FORMAT), meta=np.array(json.dumps(meta or {}, indent=1)),
                  mass=np.array(float(data.mass)), has_I5=np.array(bool(data.has_I5)),
                  has_I9=np.array(bool(data.has_I9)))
    viz = getattr(data, "viz", None)
    if viz is not None:
        arrays.update(viz_xyz=np.asarray(viz[0], float), viz_tri=np.asarray(viz[1], np.int32),
                      viz_phi=np.asarray(viz[2], np.float32))
    spiders = getattr(data, "spiders", None)
    if spiders:
        arrays.update(spider_node=np.array([s[0] for s in spiders], np.int64),
                      spider_type=np.array([s[1] for s in spiders]),
                      spider_start=np.cumsum([0] + [len(s[2]) for s in spiders]),
                      spider_xyz=np.concatenate([np.asarray(s[2], float).reshape(-1, 3) for s in spiders]))
    mesh = getattr(data, "mesh", None)
    if mesh is not None:
        arrays.update(mesh_node=np.asarray(mesh[0], np.int64), mesh_xyz=np.asarray(mesh[1], float),
                      stress=np.asarray(mesh[3], np.float32))
        for nn, cells in mesh[2].items():
            arrays[f"mesh_{_CELLS[nn]}"] = np.asarray(cells, np.int32)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "wb") as f:
        np.savez_compressed(f, **arrays)
    tmp.replace(path)
    return path


def read_meta(path):
    with np.load(path, allow_pickle=False) as z:
        _check(z, path)
        return json.loads(str(z["meta"]))


def _check(z, path):
    if "format" not in z.files or not str(z["format"]).startswith("fdflex "):
        raise ModelError(f"{path}: not a freedyn2 modal data file (.fdflex)")
    if str(z["format"]) != FORMAT:
        raise ModelError(f"{path}: {z['format']}, this freedyn2 reads {FORMAT}")


def read_fdflex(path):
    """ModalData of a .fdflex file (with .viz and .meta)."""
    with np.load(path, allow_pickle=False) as z:
        _check(z, path)
        a = {k: z[k] for k in _ARRAYS}
        data = ModalData(mass=float(z["mass"]), has_I5=bool(z["has_I5"]), has_I9=bool(z["has_I9"]),
                         units=(), source=str(path), **a)
        data.meta = json.loads(str(z["meta"]))
        data.viz = (z["viz_xyz"], z["viz_tri"], z["viz_phi"]) if "viz_xyz" in z.files else None
        data.spiders = []
        if "spider_node" in z.files:
            st = z["spider_start"]
            data.spiders = [(int(n), str(t), z["spider_xyz"][st[k]:st[k + 1]])
                            for k, (n, t) in enumerate(zip(z["spider_node"], z["spider_type"]))]
        data.mesh = None
        if "mesh_node" in z.files:
            cells = {nn: z[f"mesh_{t}"] for nn, t in _CELLS.items() if f"mesh_{t}" in z.files}
            data.mesh = (z["mesh_node"], z["mesh_xyz"], cells, z["stress"])
    return data
