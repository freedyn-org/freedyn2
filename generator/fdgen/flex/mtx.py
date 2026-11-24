"""Reader of Adams MTX files (ADAMSMAT2, as written by Adams/Flex or MSC Nastran with the
ADAMSMNF/MTX option), the input of the legacy FreeDyn flexible body (FeLib_AdamsMtxFileReader,
FeLib_MtxConverter).

Layout: line 1 "ADAMSMAT2 <title> <units>", line 2 the number of blocks, then their names, four
per line. Each block starts with a header

    NAME  FULL  RORDER|CORDER  nrow ncol nvalues  ( format )
    NAME  SPARSE  nrow ncol nnz  ( 2I8, E22.14 )

with Fortran fixed-width fields (I8, E22.14). FULL blocks whose format starts with an integer
field carry a label per row (SELNOD: node label and position, SELMOD: mode label and
frequency in Hz); the label counts as a column. RORDER lists the rows, each starting on a new
line; CORDER the columns. SPARSE lists 1-based (row, col, value).

T_MODE / R_MODE (3 x nn*nm): column i*nm + j is node i, mode j (FeLib_MtxContent::setModalData).

Values are used as they are, like the legacy kernel: the unit names in the title line are
informational only (several test files name METER but are in millimeters).
"""

import re
from pathlib import Path

import numpy as np

from ..errors import ModelError
from .modal import ModalData

_HEADER = re.compile(r"^([A-Z_0-9]+)\s+(FULL|SPARSE)\s+(?:(RORDER|CORDER)\s+)?(\d+)\s+(\d+)\s+(\d+)\s*\((.*)\)")
_FIELD = re.compile(r"(\d*)\s*([IE])(\d+)")


def _fields(fmt):
    """[(kind, width)] per value of one line from '( 1I8, 3E22.14 )'."""
    out = []
    for part in fmt.split(","):
        m = _FIELD.search(part.strip())
        if not m:
            raise ModelError(f"MTX: unknown format '{fmt}'")
        out += [(m.group(2), int(m.group(3)))] * int(m.group(1) or 1)
    return out


def _split(line, widths):
    vals, pos = [], 0
    for w in widths:
        vals.append(line[pos:pos + w])
        pos += w
    return vals


def _blocks(lines):
    """{name: (kind, order, nrow, ncol, data)} of all blocks; data: ndarray (labels included as
    column 0 for labeled blocks)."""
    nblocks = int(lines[1].split()[0])
    i = 2 + (nblocks + 3) // 4
    out = {}
    while i < len(lines):
        m = _HEADER.match(lines[i])
        if not m:
            if lines[i].strip():
                raise ModelError(f"MTX: unexpected line {i + 1}: {lines[i][:60]!r}")
            i += 1
            continue
        name, kind, order, nrow, ncol, _nval, fmt = m.groups()
        nrow, ncol = int(nrow), int(ncol)
        fields = _fields(fmt)
        widths = [w for _, w in fields]
        i += 1
        if kind == "SPARSE":
            nnz = int(_nval)
            a = np.zeros((nrow, ncol))
            for k in range(nnz):
                r, c, v = _split(lines[i + k], [8, 8, 22])
                a[int(r) - 1, int(c) - 1] = float(v)
            i += nnz
        else:
            labeled = fields[0][0] == "I"
            if labeled:  # one line per row: label and values
                a = np.zeros((nrow, ncol))
                for k in range(nrow):
                    a[k] = [float(x) for x in _split(lines[i + k], widths)[:ncol]]
                i += nrow
            else:
                # Values per line as written (the stated count is not reliable: several files
                # say 9E22.14 and write three per line); each row (column) starts on a new line.
                outer, inner = (nrow, ncol) if order == "RORDER" else (ncol, nrow)
                w = widths[0]
                flat = np.empty((outer, inner))
                for k in range(outer):
                    vals = []
                    while len(vals) < inner:
                        ln = lines[i].rstrip()
                        vals += [float(ln[p:p + w]) for p in range(0, len(ln), w) if ln[p:p + w].strip()]
                        i += 1
                    flat[k] = vals[:inner]
                a = flat if order == "RORDER" else flat.T
        out[name] = (kind, order, nrow, ncol, a)
    return out


def read_mtx(path, rigid_modes="auto"):
    """ModalData of an MTX file. rigid_modes: 'auto' removes the modes below 1e-3 of the first
    elastic frequency (at most 6, like the legacy automatic mode), an int the first n modes."""
    path = Path(path)
    lines = path.read_text(encoding="latin-1").splitlines()
    if not lines or not lines[0].startswith("ADAMSMAT2"):
        raise ModelError(f"{path}: not an ADAMSMAT2 (MTX) file")
    b = _blocks(lines)
    for need in ("SELNOD", "SELMOD", "INVAR1", "INVAR6", "INVAR7", "GENSTIFF", "T_MODE", "R_MODE"):
        if need not in b:
            raise ModelError(f"{path}: block {need} missing")
    sel = b["SELMOD"][4]
    freqs = sel[:, 1]
    nm = len(freqs)
    nodes = b["SELNOD"][4]
    nn = nodes.shape[0]

    def get(name, shape):
        return b[name][4].reshape(shape) if name in b else np.zeros(shape)

    I5 = get("INVAR5", (3 * nm, nm)).reshape(nm, 3, nm)          # rows 3j..3j+2: I5_j
    I8 = get("INVAR8", (nm, 9)).reshape(nm, 3, 3)                 # row j: I8_j row by row
    I9 = get("INVAR9", (3 * nm * nm, 3)).reshape(nm, nm, 3, 3)    # rows 3(j nm + k): I9_jk
    T = b["T_MODE"][4]
    R = b["R_MODE"][4]
    if T.shape != (3, nn * nm) or R.shape != (3, nn * nm):
        raise ModelError(f"{path}: T_MODE / R_MODE do not match SELNOD and SELMOD")
    units = tuple(u for u in ("MEGAGRAM", "KILOGRAM", "MILLIMETER", "METER", "SECOND", "NEWTON") if re.search(rf"\b{u}\b", lines[0].upper()))
    data = ModalData(
        mass=float(b["INVAR1"][4][0, 0]), I2=get("INVAR2", (3,)), I3=get("INVAR3", (3, nm)), I4=get("INVAR4", (3, nm)),
        I5=I5, I6=b["INVAR6"][4], I7=b["INVAR7"][4], I8=I8, I9=I9, K=b["GENSTIFF"][4], frequencies=freqs,
        node_labels=nodes[:, 0].astype(int), node_positions=nodes[:, 1:4],
        Phi=T.reshape(3, nn, nm).transpose(1, 0, 2), Psi=R.reshape(3, nn, nm).transpose(1, 0, 2),
        has_I5="INVAR5" in b, has_I9="INVAR9" in b, units=units, source=str(path))
    if rigid_modes == "auto":
        f = np.abs(freqs)
        n = 0
        while n < min(6, nm - 1) and f[n] < 1e-3 * max(f[min(6, nm - 1)], 1e-30):
            n += 1
    else:
        n = int(rigid_modes)
    if n:
        data = data.select(range(n, nm))
        data.removed_modes = [int(x) for x in sel[:n, 0]]
    return data


def _fmt(x):
    return f"{x:22.14E}"


def write_mtx(data, path, nodes=None, title="written by freedyn2"):
    """Write modal data as an MTX file (FULL RORDER blocks, three values per line, each row on a
    new line; read back by read_mtx). nodes: the node labels to keep (default all); everything
    else (invariants, stiffness) is independent of them."""
    keep = list(range(len(data.node_labels))) if nodes is None else [data.node_index(n) for n in nodes]
    if np.any(np.abs(data.node_labels[keep]) > 99_999_999):
        raise ValueError("MTX: node labels have at most 8 digits")
    nm = data.nm
    blocks = [
        ("SELNOD", np.column_stack([data.node_labels[keep], data.node_positions[keep]]), True),
        ("SELMOD", np.column_stack([np.arange(7, 7 + nm), data.frequencies]), True),
        ("INVAR1", np.array([[data.mass]]), False),
        ("INVAR2", data.I2.reshape(1, 3), False),
        ("INVAR3", data.I3, False),
        ("INVAR4", data.I4, False),
    ]
    if data.has_I5:
        blocks.append(("INVAR5", data.I5.reshape(3 * nm, nm), False))
    if data.has_I9:
        blocks.append(("INVAR9", data.I9.reshape(3 * nm * nm, 3), False))
    blocks += [("INVAR6", data.I6, False), ("INVAR7", data.I7, False), ("INVAR8", data.I8.reshape(nm, 9), False),
               ("GENSTIFF", data.K, False)]
    T = data.Phi[keep].transpose(1, 0, 2).reshape(3, -1)
    R = data.Psi[keep].transpose(1, 0, 2).reshape(3, -1)
    lines = [f"ADAMSMAT2 {title}  {'  '.join(data.units)}", str(len(blocks) + 2)]
    names = [b[0] for b in blocks] + ["T_MODE", "R_MODE"]
    for k in range(0, len(names), 4):
        lines.append("".join(f"{n:<18}" for n in names[k:k + 4]))
    for name, a, labeled in blocks:
        nrow, ncol = a.shape
        if labeled:
            lines.append(f"{name:<10}FULL    RORDER {nrow:6d} {ncol:4d} {nrow * ncol:4d}   ( 1I8, {ncol - 1}E22.14 )")
            lines += [f"{int(r[0]):8d}" + "".join(_fmt(x) for x in r[1:]) for r in a]
        else:
            lines.append(f"{name:<10}FULL    RORDER {nrow:6d} {ncol:4d} {nrow * ncol:4d}   ( 3E22.14 )")
            for r in a:
                lines += ["".join(_fmt(x) for x in r[c:c + 3]) for c in range(0, ncol, 3)]
    for name, a in (("T_MODE", T), ("R_MODE", R)):
        lines.append(f"{name:<10}FULL    CORDER {3:6d} {a.shape[1]:4d} {a.size:4d}   ( 3E22.14 )")
        lines += ["".join(_fmt(x) for x in a[:, c]) for c in range(a.shape[1])]
    Path(path).write_text("\n".join(lines) + "\n", encoding="latin-1")
