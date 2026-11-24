"""Reader of the Nastran bulk data a linear FE model of one part needs (small, large and free
field format, continuations):

    GRID (basic coordinates only), CTETRA (4 or 10 nodes), CHEXA (8 nodes), CBAR (with a PBARL
    ROD section: a 3D Euler-Bernoulli beam), PSOLID, PBARL, MAT1 (E, G or nu, rho), RBE2, RBE3
    (one weight and component set per group).

Other cards are ignored and listed (FeModel.ignored), so that a model is never reduced with
parts silently missing: the caller decides.
"""

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..errors import ModelError


def number(s):
    """A Nastran real: '7.86-9', '1.+5', '210000.', ' 1.0E+3'."""
    s = s.strip().replace("D", "E").replace("d", "e")
    if not s:
        return 0.0
    try:
        return float(s)
    except ValueError:
        for k in range(len(s) - 1, 0, -1):
            if s[k] in "+-" and s[k - 1] not in "eE":
                return float(s[:k] + "E" + s[k:])
        raise ModelError(f"Nastran: not a number '{s}'") from None


def integer(s):
    s = s.strip()
    return int(s) if s else 0


def _fields(line):
    """Key and data fields of one physical line."""
    line = line.rstrip("\n").expandtabs(8)
    if "," in line[:80]:  # free field
        parts = line.split(",")
        return parts[0].strip(), [p.strip() for p in parts[1:]]
    key = line[:8].strip()
    if key.endswith("*") or key.startswith("*"):  # large field: 4 fields of 16 per line
        return key, [line[8 + 16 * i:24 + 16 * i] for i in range(4)]
    return key, [line[8 + 8 * i:16 + 8 * i] for i in range(8)]


def cards(text):
    """[(NAME, [fields])] with the continuations joined."""
    out = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("$"):
            continue
        raw = raw.split("$", 1)[0]
        key, f = _fields(raw)
        if key == "" or key.startswith("+") or key.startswith("*"):
            if not out:
                continue
            out[-1][1].extend(f)
            continue
        name = key.rstrip("*").upper()
        if name in ("BEGIN", "ENDDATA") or name.startswith(("SOL", "CEND", "INCLUDE")):
            continue
        out.append((name, f))
    return out


@dataclass
class FeModel:
    nodes: dict = field(default_factory=dict)          # id -> xyz (basic)
    solids: list = field(default_factory=list)         # (eid, pid, [node ids]) CTETRA / CHEXA
    bars: list = field(default_factory=list)           # (eid, pid, ga, gb, orientation vector)
    psolid: dict = field(default_factory=dict)         # pid -> mid
    pbar: dict = field(default_factory=dict)           # pid -> (mid, A, Iy, Iz, J)
    materials: dict = field(default_factory=dict)      # mid -> (E, nu, rho)
    rbe2: list = field(default_factory=list)           # (eid, master, components, [slaves])
    rbe3: list = field(default_factory=list)           # (eid, ref, ref components, [(weight, components, [nodes])])
    ignored: Counter = field(default_factory=Counter)

    def node_array(self, ids):
        return np.array([self.nodes[i] for i in ids], float)


def _rod(dims):
    r = dims[0]
    A, I = np.pi * r * r, np.pi * r ** 4 / 4
    return A, I, I, 2 * I


def read_nastran(path):
    """FeModel of a Nastran bulk data file (.bdf, .fem, .nas, .dat)."""
    path = Path(path)
    m = FeModel()
    for name, f in cards(path.read_text(encoding="latin-1")):
        if name == "GRID":
            if integer(f[1]) or (len(f) > 5 and integer(f[5])):
                raise ModelError(f"{path}: GRID {f[0].strip()} in a local coordinate system (CP / CD) is not supported")
            m.nodes[integer(f[0])] = (number(f[2]), number(f[3]), number(f[4]))
        elif name in ("CTETRA", "CHEXA"):
            ids = [integer(x) for x in f[2:] if x.strip()]
            if len(ids) not in ((4, 10) if name == "CTETRA" else (8,)):
                raise ModelError(f"{path}: {name} {f[0].strip()} with {len(ids)} nodes is not supported")
            m.solids.append((integer(f[0]), integer(f[1]), ids))
        elif name == "CBAR":
            if f[5].strip() and not f[6].strip():
                raise ModelError(f"{path}: CBAR {f[0].strip()}: orientation by a node is not supported")
            m.bars.append((integer(f[0]), integer(f[1]), integer(f[2]), integer(f[3]),
                           np.array([number(f[4]), number(f[5]), number(f[6])])))
        elif name == "PSOLID":
            m.psolid[integer(f[0])] = integer(f[1])
        elif name == "PBARL":
            if f[3].strip().upper() != "ROD":
                raise ModelError(f"{path}: PBARL {f[0].strip()}: section {f[3].strip()} is not supported (ROD only)")
            dims = [number(x) for x in f[8:] if x.strip()]
            m.pbar[integer(f[0])] = (integer(f[1]), *_rod(dims))
        elif name == "MAT1":
            E, G, nu, rho = number(f[1]), number(f[2]), number(f[3]), number(f[4])
            if not f[3].strip():
                nu = E / (2 * G) - 1
            m.materials[integer(f[0])] = (E, nu, rho)
        elif name == "RBE2":
            m.rbe2.append((integer(f[0]), integer(f[1]), f[2].strip(), [integer(x) for x in f[3:] if x.strip()]))
        elif name == "RBE3":
            eid, ref, refc = integer(f[0]), integer(f[2]), f[3].strip()
            groups, rest, k = [], f[4:], 0
            while k < len(rest):
                tok = rest[k].strip()
                if not tok:
                    k += 1
                    continue
                if tok.upper() in ("UM", "ALPHA", "TREF"):
                    raise ModelError(f"{path}: RBE3 {eid}: {tok} is not supported")
                w, comp, nodes = number(tok), rest[k + 1].strip(), []
                k += 2
                while k < len(rest) and rest[k].strip() and "." not in rest[k] and rest[k].strip().upper() not in ("UM",):
                    nodes.append(integer(rest[k]))
                    k += 1
                groups.append((w, comp, nodes))
            m.rbe3.append((eid, ref, refc, groups))
        else:
            m.ignored[name] += 1
    return m
