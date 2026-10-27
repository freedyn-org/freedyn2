"""Readers for legacy FreeDyn result files (.srf, .vrf, .arf) and the HHT step log (.db1)."""

import csv
import re
from pathlib import Path

import numpy as np


def _blocks(path):
    """Return (topology tokens, list of (t, values)) of a FreeDyn result file.

    Each frame is a line with t followed by one line per entry of the topology (body IDs)."""
    lines = Path(path).read_text().splitlines()
    i = lines.index("Topology") + 2
    topology = lines[i].split()
    i = lines.index("Data", i) + 2
    rows = [l for l in lines[i:] if l.strip()]
    n = len(topology)
    frames = [(float(rows[k]), np.array(" ".join(rows[k + 1:k + 1 + n]).split(), dtype=float))
              for k in range(0, len(rows), n + 1)]
    return topology, frames


def read_rigid_body_results(stem):
    """Results of a model with rigid bodies only, from <stem>.srf/.vrf/.arf.

    Returns {"t": (n,), "<body index>": {"R", "A", "Rdot", "omega_body", "Rddot", "alpha_body"}}.
    A is the rotation matrix (body to global); the .srf stores its columns.
    """
    stem = Path(stem)
    out = {}
    topo, frames = _blocks(stem.with_suffix(".srf"))
    nb = len(topo)
    t = np.array([f[0] for f in frames])
    data = np.array([f[1] for f in frames]).reshape(len(frames), nb, 12)
    out["t"] = t
    bodies = [{"id": topo[b], "R": data[:, b, :3], "A": data[:, b, 3:].reshape(-1, 3, 3).transpose(0, 2, 1)}
              for b in range(nb)]
    for suffix, names in ((".vrf", ("Rdot", "omega_body")), (".arf", ("Rddot", "alpha_body"))):
        p = stem.with_suffix(suffix)
        if not p.exists():
            continue
        _, frames = _blocks(p)
        d = np.array([f[1] for f in frames]).reshape(len(frames), nb, 6)
        for b in range(nb):
            bodies[b][names[0]] = d[:, b, :3]
            bodies[b][names[1]] = d[:, b, 3:]
    out["bodies"] = bodies
    return out


def read_step_log(path):
    """Every HHT step attempt from a DebugLevel >= 1 log (<stem>.db1).

    Returns arrays t, h, accepted (bool) and iterations (number of corrector updates).
    Columns are fixed width: t, h, h/hMax bar, reason for step size reduction (empty if the
    step was accepted), corrector information ("LU+FBS" per factorization and solve).
    """
    t, h, ok, its = [], [], [], []
    for line in Path(path).read_text().splitlines()[1:]:
        if len(line.split()) < 2:
            continue
        t.append(float(line[0:13]))
        h.append(float(line[13:26]))
        ok.append(not line[47:88].strip())
        info = line[88:]
        its.append(info.count("LU+FBS") + sum(int(n) for n in re.findall(r"(\d+)xFBS", info)))
    return np.array(t), np.array(h), np.array(ok), np.array(its)


def rotation_matrix(t):
    """Shabana 2005 Eq. 2.13, for Euler parameters t (n, 4)."""
    t0, t1, t2, t3 = t.T
    return np.stack(
        [
            np.stack([1 - 2 * (t2**2 + t3**2), 2 * (t1 * t2 - t0 * t3), 2 * (t1 * t3 + t0 * t2)], -1),
            np.stack([2 * (t1 * t2 + t0 * t3), 1 - 2 * (t1**2 + t3**2), 2 * (t2 * t3 - t0 * t1)], -1),
            np.stack([2 * (t1 * t3 - t0 * t2), 2 * (t2 * t3 + t0 * t1), 1 - 2 * (t1**2 + t2**2)], -1),
        ],
        -2,
    )


def read_v2_results(csv_path, bodies):
    """FreeDyn 2.0 CSV results in the layout of read_rigid_body_results, for the named bodies."""
    rows = list(csv.DictReader(open(csv_path)))
    col = lambda b, k, sfx: np.array([[float(r[f"{b}.{k}{s}"]) for s in sfx] for r in rows])
    xyz = ("_x", "_y", "_z")
    return {"t": np.array([float(r["t"]) for r in rows]),
            "bodies": [{"id": b, "R": col(b, "R", xyz), "Rdot": col(b, "Rdot", xyz),
                        "omega_body": col(b, "omega_body", xyz), "A": rotation_matrix(col(b, "theta", "0123"))}
                       for b in bodies]}
