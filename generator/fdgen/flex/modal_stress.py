"""Output of a flexible body for fatigue by modal superposition (fatigue tools that
take modal stresses and load histories): sigma(x, t) = sum_i sigma_i(x) eta_i(t).

  <body>_modes.unv   the solids of the FE model (Universal file: nodes 2411, elements 2412) and,
                     per elastic mode, the stress tensor at the nodes (2414, normal mode, stress,
                     symmetric tensor); in the body frame and the units of the model
  <body>_eta.csv     the modal coordinates eta_i(t) of the run: time, mode1, mode2, ...

The modal stresses come from the reduction (fdgen.fem.stress), so only for bodies with an FE
model (flex.fem or meshed from their STL), not for modal data from an MTX file. The modal
coordinates are those of the body frame of freedyn2 (rigid motion removed, as in Adams).
"""

from pathlib import Path

import numpy as np

from ..errors import ModelError
from ..fem.stress import von_mises
from . import load_modal

# UNV element descriptors and the order of the nodes (index into the Nastran order)
_UNV = {4: (111, [0, 1, 2, 3]), 10: (118, [0, 4, 1, 5, 2, 6, 7, 8, 9, 3]), 8: (115, list(range(8)))}
_TENSOR = [0, 3, 1, 5, 4, 2]  # UNV symmetric tensor xx xy yy xz yz zz from xx yy zz xy yz zx


def _e13(values):
    return "".join(f"{v:13.5E}" for v in values)


def write_unv(path, d, name="body"):
    """Mesh and modal stresses of ModalData d (with .mesh) as a Universal file."""
    labels, xyz, cells, sig = d.mesh
    out = []
    out += ["    -1", "  2411"]
    for n, p in zip(labels, xyz):
        out.append(f"{n:10d}{1:10d}{1:10d}{11:10d}")
        out.append("".join(f"{v:25.16E}" for v in p).replace("E", "D"))
    out += ["    -1"]
    out += ["    -1", "  2412"]
    eid = 0
    for nn in sorted(cells):
        desc, order = _UNV[nn]
        for c in np.asarray(cells[nn])[:, order]:
            eid += 1
            out.append(f"{eid:10d}{desc:10d}{1:10d}{1:10d}{7:10d}{nn:10d}")
            ids = labels[c]
            for k in range(0, nn, 8):
                out.append("".join(f"{i:10d}" for i in ids[k:k + 8]))
    out += ["    -1"]
    f = d.frequencies
    for m in range(sig.shape[2]):
        out += ["    -1", "  2414", f"{m + 1:10d}", f"MODE {m + 1} STRESS", f"{1:10d}",
                f"freedyn2 modal stresses of {name}"[:80], f"mode {m + 1}, {f[m]:.6g} Hz, mass-normalized"[:80],
                "components xx xy yy xz yz zz, body frame, model units", "NONE", "NONE",
                f"{1:10d}{2:10d}{4:10d}{2:10d}{2:10d}{6:10d}",
                f"{0:10d}{0:10d}{1:10d}{0:10d}{0:10d}{m + 1:10d}{0:10d}{m + 1:10d}",
                f"{0:10d}{0:10d}",
                _e13([0.0, f[m], (2 * np.pi * f[m]) ** 2, 1.0, 0.0, 0.0]), _e13([0.0] * 6)]
        s = sig[:, _TENSOR, m]
        for n, v in zip(labels, s):
            out.append(f"{n:10d}")
            out.append(_e13(v))
        out += ["    -1"]
    Path(path).write_text("\n".join(out) + "\n")
    return Path(path)


def modal_coordinates(d, results, name):
    """(t, eta (nt, nm)) of body name in a run."""
    try:
        eta = np.column_stack([results[f"{name}.eta{i + 1}.eta"] for i in range(d.nm)])
    except KeyError:
        raise ModelError(f"the results have no modal coordinates of {name} ({name}.eta1.eta)") from None
    return results["t"], eta


def write_eta(path, t, eta):
    header = ",".join(["time"] + [f"mode{i + 1}" for i in range(eta.shape[1])])
    np.savetxt(path, np.column_stack([t, eta]), delimiter=",", header=header, comments="", fmt="%.10g")
    return Path(path)


def peak(d, t, eta, steps=400):
    """(largest von Mises stress, time, node label) of the superposed stresses, at up to `steps`
    instants (every k-th output step)."""
    labels, _, _, sig = d.mesh
    best = (0.0, 0.0, 0)
    for k in range(0, len(t), max(1, len(t) // steps)):
        vm = von_mises(np.einsum("nsm,m->ns", sig, eta[k]))
        i = int(vm.argmax())
        if vm[i] > best[0]:
            best = (float(vm[i]), float(t[k]), int(labels[i]))
    return best


def export(model, name, results, out_dir):
    """Write <name>_modes.unv and <name>_eta.csv of body name of a validated model and its results
    ({column: array}) to out_dir; returns (paths, text)."""
    b = model["bodies"].get(name)
    if b is None or "flex" not in b:
        raise ModelError(f"{name}: not a flexible body")
    d = load_modal(b["flex"]["modal"])
    if getattr(d, "mesh", None) is None:
        raise ModelError(f"{name}: its modal data have no FE mesh and modal stresses (modal data from an MTX file); "
                         "give the body an FE model (flex.fem) or mesh it from its STL (flex.material)")
    t, eta = modal_coordinates(d, results, name)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    unv = write_unv(out_dir / f"{name}_modes.unv", d, name)
    csv = write_eta(out_dir / f"{name}_eta.csv", t, eta)
    vm, tk, node = peak(d, t, eta)
    labels, _, cells, sig = d.mesh
    text = (f"body {name}: {len(labels)} nodes, {sum(len(c) for c in cells.values())} elements, {d.nm} modes, "
            f"{len(t)} time steps; largest von Mises stress {vm:.4g} at node {node}, t = {tk:.4g} s "
            "(modal superposition, model units)")
    return [unv, csv], text
