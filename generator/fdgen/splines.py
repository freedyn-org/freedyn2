"""Cubic splines y(x) for force laws, with the coefficients of the legacy FreeDyn splines.

  akima  cubic Akima spline (FL_XYCubicAkimaSpline): slopes from the Akima weights, end slopes
         by repeating the first / last secant twice; linear beyond the last point.
  zcc    zero clamped cubic spline (FL_XYZeroClampedCubicSpline): C2 cubic with zero slope at
         both ends (tridiagonal system); quadratic beyond the last point.

Both: s(x) = y_i + b_i dx + c_i dx^2 + d_i dx^3, dx = x - x_i, on the segment of the largest
i with x_i <= x (i = 0 below the first point: the first cubic is extended). The coefficients
are computed here with the operation order of the legacy code, so the values agree to the bit.
"""

import re

import casadi as ca

TYPES = ("akima", "zcc")

_LEADING_FLOAT = re.compile(r"[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")


def _to_double(token):
    """As std::stringstream >> double: the leading number of the token, 0 if there is none."""
    m = _LEADING_FLOAT.match(token)
    return float(m.group(0)) if m else 0.0


def read_table(path, header_lines=0, col_x=0, col_y=1):
    """x, y columns of a text table, as the legacy IO_TableDataReader: header_lines skipped;
    columns separated by spaces (not tabs), each entry stripped of spaces and tabs; the first
    data line must not be empty, later empty lines are ignored, every line needs the same
    number of columns; column indices start at 0."""
    lines = open(path, encoding="latin-1").read().split("\n")
    if len(lines) - 1 < header_lines:
        raise ValueError(f"{path}: unexpected end of file in the header")
    rows = lines[header_lines:]
    if rows and rows[-1] == "":
        rows = rows[:-1]  # the newline after the last line
    strip = lambda x: x.strip(" \t")
    table = []
    for k, line in enumerate(rows):
        line = strip(line)
        if not line:
            if k == 0:
                raise ValueError(f"{path}: unexpected empty line after the header block")
            continue
        fields = [_to_double(strip(f)) for f in line.split(" ") if strip(f)]
        if table and len(fields) != len(table[0]):
            raise ValueError(f"{path}: line {header_lines + k + 1}: number of columns does not match")
        table.append(fields)
    if not table or len(table[0]) < 2:
        raise ValueError(f"{path}: less than two columns")
    if max(col_x, col_y) >= len(table[0]):
        raise ValueError(f"{path}: column index larger than the last column ({len(table[0]) - 1})")
    return [r[col_x] for r in table], [r[col_y] for r in table]


def coefficients(x, y, kind):
    """b, c, d (one per data point) of the legacy spline of type kind."""
    x, y = [float(v) for v in x], [float(v) for v in y]
    n = len(x)
    if n < 3:
        raise ValueError("a spline needs at least 3 points")
    if any(x[i + 1] <= x[i] for i in range(n - 1)):
        raise ValueError("spline x values must be strictly increasing")
    if kind == "akima":
        h = [x[i + 1] - x[i] for i in range(n - 1)]
        m = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]
        m = [m[0], m[0]] + m + [m[-1], m[-1]]
        b = []
        for i in range(n):
            if m[i + 1] == m[i + 2]:
                b.append(m[i + 2])
            elif m[i + 1] == m[i] and m[i + 3] != m[i + 2]:
                b.append(m[i + 1])
            elif m[i + 1] != m[i] and m[i + 3] == m[i + 2]:
                b.append(m[i + 2])
            elif m[i + 1] == m[i] and m[i + 3] == m[i + 2]:
                b.append((m[i + 1] + m[i + 2]) * 0.5)
            else:
                b.append((m[i + 1] * abs(m[i + 3] - m[i + 2]) + m[i + 2] * abs(m[i + 1] - m[i]))
                         / (abs(m[i + 3] - m[i + 2]) + abs(m[i + 1] - m[i])))
        c = [(3. * m[i + 2] - 2. * b[i] - b[i + 1]) / h[i] for i in range(n - 1)] + [0.0]
        d = [(b[i] + b[i + 1] - 2. * m[i + 2]) / (h[i] * h[i]) for i in range(n - 1)] + [0.0]
        return b, c, d
    if kind == "zcc":
        ne = n - 1
        h = [x[i + 1] - x[i] for i in range(ne)]
        b = [(y[i + 1] - y[i]) / h[i] for i in range(ne)]
        inv = [0.0] * (ne + 1)
        inv[0] = 0.5 / h[0]
        c = [3.0 * b[0]]
        for i in range(1, ne):
            factor = h[i - 1] * inv[i - 1]
            diag = 2.0 * h[i] + (2.0 - factor) * h[i - 1]
            inv[i] = 1.0 / diag
            c.append(3.0 * (b[i] - b[i - 1]) - factor * c[i - 1])
        factor = h[ne - 1] * inv[ne - 1]
        diag = (2.0 - factor) * h[ne - 1]
        inv[ne] = 1.0 / diag
        c.append(-3.0 * b[ne - 1] - factor * c[ne - 1])
        c[ne] *= inv[ne]
        for i in range(ne - 1, -1, -1):
            c[i] = (c[i] - h[i] * c[i + 1]) * inv[i]
        d = []
        for i in range(ne):
            d.append((c[i + 1] - c[i]) / (3.0 * h[i]))
            b[i] -= h[i] * (c[i] + h[i] * d[i])
        return b + [0.0], c, d + [0.0]
    raise ValueError(f"spline type must be one of {TYPES}, not {kind!r}")


def evaluate(xv, knots, y, b, c, d):
    """CasADi expression of the spline at xv; knots, y, b, c, d are SX vectors (or numbers)."""
    n = knots.numel() if hasattr(knots, "numel") else len(knots)

    def seg(i):
        dx = xv - knots[i]
        dx2 = dx * dx
        return y[i] + b[i] * dx + c[i] * dx2 + d[i] * (dx2 * dx)

    value = seg(n - 1)
    for i in range(n - 2, -1, -1):
        value = ca.if_else(xv < knots[i + 1], seg(i), value)
    return value


def numeric(x, y, kind):
    """Plain Python evaluator f(xv) (as FL_XY*Spline::eval), coefficients computed once."""
    import bisect

    b, c, d = coefficients(x, y, kind)
    x, y = [float(v) for v in x], [float(v) for v in y]

    def f(xv):
        i = max(bisect.bisect_right(x, xv) - 1, 0)
        dx = xv - x[i]
        dx2 = dx * dx
        return y[i] + b[i] * dx + c[i] * dx2 + d[i] * (dx2 * dx)

    return f
