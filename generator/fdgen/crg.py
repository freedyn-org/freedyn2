"""Write ASAM OpenCRG road files (for examples and tests; the runtime reads them, fd::Road).

  write_crg(path, du, v, z, phi=None, slope=None, bank=None, start=(0, 0, 0), fmt="LDFI")

z: heights [nu x nv] of the cross sections u_i = i du at the lateral positions v (ascending,
left positive); phi: heading of the reference line per segment (phi[i] for u_i -> u_i+1,
nu - 1 values; written as the channel value at u_i+1, as the OpenCRG library integrates it,
the value at u_0 repeats the first one); slope and bank per u (dz/du of the reference line,
dz/dv). start: x, y, heading of the reference line start.
REFERENCE_LINE_END_X/Y are written, so a reader can check its reference line. Formats: LRFI,
LDFI (ASCII, 10 / 20 characters per value, 80-character records), KRBI, KDBI (big-endian
float32 / float64). The reference line points follow x_i+1 = x_i + du cos(phi_i).
"""

import math

import numpy as np

_WIDTH = {"LRFI": 10, "LDFI": 20}


def reference_line(du, phi, start=(0.0, 0.0, 0.0), n=None):
    """Points (x, y) of the reference line with segment headings phi (or straight: phi None)."""
    x0, y0, phi0 = start
    if phi is None:
        phi = np.full(n - 1, phi0)
    x, y = [x0], [y0]
    for a in phi:
        x.append(x[-1] + du * math.cos(a))
        y.append(y[-1] + du * math.sin(a))
    return np.array(x), np.array(y)


def write_crg(path, du, v, z, phi=None, slope=None, bank=None, start=(0.0, 0.0, 0.0), fmt="LDFI", comment=""):
    z = np.asarray(z, dtype=float)
    nu, nv = z.shape
    v = np.asarray(v, dtype=float)
    if len(v) != nv:
        raise ValueError("one lateral position per column of z")
    x, y = reference_line(du, phi, start, nu)
    phi0 = start[2] if phi is None else phi[0]
    cols, names = [], []
    if phi is not None:
        cols.append(np.r_[phi[0], phi])
        names.append("D:reference line phi,rad")
    if slope is not None:
        cols.append(np.asarray(slope, dtype=float))
        names.append("D:reference line slope,m/m")
    if bank is not None:
        cols.append(np.asarray(bank, dtype=float))
        names.append("D:reference line banking,m/m")
    for j in range(nv):
        cols.append(z[:, j])
        names.append(f"D:long section at v = {v[j]:.4f},m")
    data = np.column_stack(cols)
    head = ["$CT", comment or "written by fdgen.crg", "$", "$ROAD_CRG",
            f"REFERENCE_LINE_START_U = {0.0:.6f}", f"REFERENCE_LINE_END_U = {du * (nu - 1):.6f}",
            f"REFERENCE_LINE_INCREMENT = {du:.9f}",
            f"REFERENCE_LINE_START_X = {start[0]:.9f}", f"REFERENCE_LINE_START_Y = {start[1]:.9f}",
            f"REFERENCE_LINE_START_PHI = {phi0:.12f}",
            f"REFERENCE_LINE_END_X = {x[-1]:.9f}", f"REFERENCE_LINE_END_Y = {y[-1]:.9f}",
            f"LONG_SECTION_V_RIGHT = {v[0]:.6f}", f"LONG_SECTION_V_LEFT = {v[-1]:.6f}", "$",
            "$KD_DEFINITION", f"#:{fmt}", *names, "$", "$" * 72]
    for line in head:
        if len(line) > 72:
            raise ValueError(f"header record longer than 72 characters: {line}")
    with open(path, "wb") as f:
        f.write(("\n".join(head) + "\n").encode("ascii"))
        if fmt in _WIDTH:
            w = _WIDTH[fmt]
            per = 80 // w
            spec = "{:10.4f}" if fmt == "LRFI" else "{:20.12e}"
            nan = "**unused**".rjust(w)
            for row in data:
                fields = [nan if np.isnan(x) else spec.format(x) for x in row]
                for k in range(0, len(fields), per):
                    f.write(("".join(fields[k:k + per]) + "\n").encode("ascii"))
        elif fmt in ("KRBI", "KDBI"):
            f.write(data.astype(">f4" if fmt == "KRBI" else ">f8").tobytes())
        else:
            raise ValueError(f"format {fmt!r}: LRFI, LDFI, KRBI or KDBI")
    return x, y
