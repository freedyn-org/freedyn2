"""Linearization of a model about a steady motion, eigenvalues (stability analysis).

  python -m fdgen.linearize model.yaml model.fdm result.csv [--time T]

The reference state (positions, orientations, velocities, constraint forces) is a row of a
freedyn2_run result. The system matrices follow exactly from the generated iteration matrices
of the elements: with alpha = 0, h = 1 and incr = 0 every element gives
J(beta, gamma) = [M + gamma C + beta K, B^T; B, 0], so three evaluations separate the mass
matrix M, the velocity Jacobian C (damping, gyroscopic terms) and the stiffness K (with the
constraint force terms B^T lambda) for the perturbation q o exp(delta) of the configuration.

Linearized equations about a motion with constant velocities v_ref (in the velocity dofs:
global velocity, body-frame angular velocity of each Lie group body):

  d(delta)/dt = dv + D delta,   D = -[omega_ref x] on the rotation dofs (q o exp(delta))
  M d(dv)/dt + C dv + K delta + B^T dlambda = 0,   B delta = 0

Axisymmetric bodies spinning about their axis (wheels) are described in a frame that does
not spin with them (spin_vectors, kinematic_coupling), so that the equations do not depend on
the wheel angle. The constraints are eliminated with a basis N of the null space of B (delta = N xi, dv =
N xi' - D N xi), which leaves M_r xi'' + C_r xi' + K_r xi = 0 with M_r = N^T M N,
C_r = N^T (C - M D) N, K_r = N^T (K - C D) N. The motion must be steady in the sense that
these matrices do not change along it (straight running on a flat road, steady turning):
cyclic coordinates (position, heading, wheel angles) give eigenvalues 0.
"""

import argparse
import csv
from pathlib import Path

import numpy as np

from . import codegen


def read_fdm(path):
    """Elements of a .fdm file: [{type, name, coords, dofs, cons, p}]."""
    elems, cur = [], None
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        key, _, rest = line.partition(" ")
        if key == "element":
            t, name = rest.split()
            cur = {"type": t, "name": name}
        elif cur is not None and key in ("coords", "dofs", "cons"):
            cur[key] = [int(x) for x in rest.split()]
        elif cur is not None and key == "p":
            cur["p"] = [float(x) for x in rest.split()]
        elif key == "end" and cur is not None:
            elems.append(cur)
            cur = None
    return elems


def element_functions(elems, model=None):
    """ElementFunctions per element type: library types, plugin types from the model."""
    funs = {}
    plugin = {}
    if model is not None:
        from .plugin import plugin_elements

        plugin = {tname: elem for _, tname, _, _, elem in plugin_elements(model)}
    for e in elems:
        t = e["type"]
        if t not in funs:
            funs[t] = plugin[t] if t in plugin else codegen.ELEMENT_BUILDERS[t]()
    return funs


def reference_state(elems, row):
    """Global q, v, lam of a result row (bodies: R, theta, Rdot, omega_body; constraint
    elements: <name>.lambda; vector-space elements are not supported here)."""
    nq = 1 + max(i for e in elems for i in e.get("coords", []))
    nv = 1 + max(i for e in elems for i in e.get("dofs", []))
    nc = 1 + max([i for e in elems for i in e.get("cons", [])] or [-1])
    q, v, lam = np.zeros(nq), np.zeros(nv), np.zeros(nc)
    get = lambda k: float(row[k])
    for e in elems:
        if e["type"] == "rigid_body":
            n = e["name"]
            q[e["coords"]] = [get(f"{n}.R_{c}") for c in "xyz"] + [get(f"{n}.theta{k}") for k in range(4)]
            v[e["dofs"]] = [get(f"{n}.Rdot_{c}") for c in "xyz"] + [get(f"{n}.omega_body_{c}") for c in "xyz"]
        elif e["type"].endswith("_euler") or e["type"] == "rigid_body_euler":
            raise ValueError("linearize: Euler-parameter bodies (legacy profile) are not supported")
        if e.get("cons") and f"{e['name']}.lambda" in row:
            lam[e["cons"]] = get(f"{e['name']}.lambda")
    return q, v, lam


def consistent_lambda(elems, funs, q, v, t=0.0):
    """Constraint forces that belong to the state q, v of this model: [M B^T; B 0] [a; lam] =
    [Q - h; gamma] assembled from the elements' ic functions (as the runtime starts a run). For a
    model changed after the run (other parameter values), lam of the run would be those of the
    model before; with these the linearization is that of the changed model in the same state."""
    nv, nc = len(v), 1 + max([i for e in elems for i in e.get("cons", [])] or [-1])
    A, b = np.zeros((nv + nc, nv + nc)), np.zeros(nv + nc)
    for e in elems:
        f = funs[e["type"]]
        idx = e.get("dofs", []) + [nv + c for c in e.get("cons", [])]
        nl = len(e.get("dofs", []))
        Al, bl, _, _ = f.ic(q[e["coords"]], v[e["dofs"]], np.zeros(nl), np.zeros(len(e.get("cons", []))), e["p"],
                            np.zeros(4), np.zeros(nl), t)
        A[np.ix_(idx, idx)] += np.array(Al)
        b[idx] += np.array(bl).ravel()
    return np.linalg.lstsq(A, b, rcond=None)[0][nv:]


def matrices(elems, funs, q, v, lam, t=0.0):
    """M, C, K (nv x nv) and B (nc x nv) at the state q, v, lam (accelerations 0)."""
    nv, nc = len(v), len(lam)

    def J(beta, gamma):
        out = np.zeros((nv + nc, nv + nc))
        for e in elems:
            f = funs[e["type"]]
            idx = e.get("dofs", []) + [nv + c for c in e.get("cons", [])]
            nl_v = len(e.get("dofs", []))
            Jl = np.array(f.iter(q[e["coords"]], v[e["dofs"]], np.zeros(nl_v), lam[e.get("cons", [])],
                                 e["p"], [0.0, beta, gamma, 1.0], np.zeros(nl_v), t))
            out[np.ix_(idx, idx)] += Jl
        return out

    J10, J20, J11 = J(1.0, 0.0), J(2.0, 0.0), J(1.0, 1.0)
    K = (J20 - J10)[:nv, :nv]
    C = (J11 - J10)[:nv, :nv]
    M = J10[:nv, :nv] - K
    B = J10[nv:, :nv]
    BT = J10[:nv, nv:]
    if nc and np.max(np.abs(BT - B.T)) > 1e-8 * max(1.0, np.max(np.abs(B))):
        raise RuntimeError("linearize: constraint Jacobian and its transpose in the dynamics differ")
    return M, C, K, B


def spin_vectors(elems, v, rtol=1e-9):
    """Spin Omega_s (body frame) of every Lie group body that is axisymmetric (two equal
    principal moments) and turns about its symmetry axis: wheels. Its rotation is described in
    a frame that does not take part in the spin, so that the linearized equations do not
    depend on the wheel angle (they would in the body frame: every external load turns in it)."""
    spin = {}
    for e in elems:
        if e["type"] != "rigid_body":
            continue
        J = e["p"][1:4]
        w = v[e["dofs"][3:6]]
        for k in range(3):
            i, j = [x for x in range(3) if x != k]
            if abs(J[i] - J[j]) <= rtol * max(J) and abs(w[k]) > 0:
                s = np.zeros(3)
                s[k] = w[k]
                spin[e["name"]] = s
                break
    return spin


def kinematic_coupling(elems, v, M=None, spin=None):
    """D (d(delta)/dt = dv + D delta for q o exp(delta)) and the correction of C for the
    non-spinning frames of the bodies in spin: there delta_b = S^T delta, dv_b = S^T dv with
    the spin rotation S, which gives D = -[(omega - Omega_s) x] and C += -M [Omega_s x]."""
    spin = spin or {}
    D = np.zeros((len(v), len(v)))
    dC = np.zeros((len(v), len(v)))
    skew = lambda w: np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    for e in elems:
        if e["type"] == "rigid_body":
            r = e["dofs"][3:6]
            s = spin.get(e["name"], np.zeros(3))
            D[np.ix_(r, r)] = -skew(v[r] - s)
            if M is not None and np.any(s):
                dC[:, r] = -M[:, r] @ skew(s)
    return D, dC


def null_space(B, tol=1e-10):
    if B.shape[0] == 0:
        return np.eye(B.shape[1])
    _, s, Vt = np.linalg.svd(B)
    rank = int(np.sum(s > tol * max(1.0, s[0])))
    return Vt[rank:].T


def cyclic_directions(elems, nv):
    """Global translations of all rigid bodies together (x, y, z) as directions in the velocity
    dofs: candidates for cyclic coordinates (eigen keeps those the equations do not depend on)."""
    out = []
    for k in range(3):
        n = np.zeros(nv)
        for e in elems:
            if e["type"] == "rigid_body":
                n[e["dofs"][k]] = 1.0
        if np.any(n):
            out.append(n / np.linalg.norm(n))
    return out


def eigen(M, C, K, B, D, dC=None, cyclic=(), tol=1e-9):
    """Eigenvalues and eigenvectors (in delta) of the linearized system.

    cyclic: directions n in the velocity dofs (cyclic_directions); those with B n = 0 and
    K n = 0 (a free translation, e.g. the lateral position on a flat road) are coordinates the
    equations do not depend on. Their position states are left out: they only add eigenvalues 0,
    and with a coupled heading (the lateral position grows with a heading change at speed) a
    defective double eigenvalue 0 that round-off splits into +-sqrt(eps) and mixes with slow modes."""
    if dC is not None:
        C = C + dC
    N = null_space(B)
    Mr = N.T @ M @ N
    Cr = N.T @ (C - M @ D) @ N
    Kr = N.T @ (K - C @ D) @ N
    n = Mr.shape[0]
    scale = max(1.0, np.max(np.abs(Kr)))
    cs = []
    for d in cyclic:
        c = N.T @ d
        if np.linalg.norm(N @ c - d) < 1e-8 and np.linalg.norm(Kr @ c) < tol * scale:
            cs.append(c)
    k = len(cs)
    Q = np.linalg.qr(np.column_stack(cs + [np.eye(n)]))[0][:, :n] if k else np.eye(n)
    Mq, Cq, Kq = (Q.T @ X @ Q for X in (Mr, Cr, Kr))
    Mi = np.linalg.inv(Mq)
    A = np.block([[np.zeros((n, n)), np.eye(n)], [-Mi @ Kq, -Mi @ Cq]])
    keep = list(range(k, 2 * n))  # without the positions along the cyclic directions
    lam, Y = np.linalg.eig(A[np.ix_(keep, keep)])
    pos = np.vstack([np.zeros((k, len(lam))), Y[:n - k]])
    vel = Y[n - k:]
    big = np.abs(lam) > 1e-12
    pos[:, big] = vel[:, big] / lam[big]  # xi' = lambda xi
    return lam, N @ Q @ pos


def _row(csv_path, time=None):
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[0] if time is None else min(rows, key=lambda r: abs(float(r["t"]) - time))


def linearize(model, fdm, row, consistent=False):
    """Eigenvalues and eigenvectors (in the velocity dofs) of the model (validated, and its .fdm)
    about a result row; also the elements of the .fdm. consistent: the constraint forces from the
    model in the state of the row (consistent_lambda), not those of the row: for a model changed
    since the run."""
    elems = read_fdm(fdm)
    funs = element_functions(elems, model)
    q, v, lam = reference_state(elems, row)
    if consistent and len(lam):
        lam = consistent_lambda(elems, funs, q, v, float(row["t"]))
    M, C, K, B = matrices(elems, funs, q, v, lam, float(row["t"]))
    D, dC = kinematic_coupling(elems, v, M, spin_vectors(elems, v))
    return (*eigen(M, C, K, B, D, dC, cyclic_directions(elems, len(v))), elems)


def analyze(model_path, fdm, csv_path, time=None):
    """Eigenvalues of the model (its .fdm) about the result row at time (default: the first row)."""
    from .cli import load

    lam, X, _ = linearize(load(model_path, "v2"), fdm, _row(csv_path, time))
    return lam, X


def modes(model, fdm, csv_path, time=None, count=24, consistent=False):
    """Eigenmodes for the viewer about the result row at time: the oscillating ones by frequency,
    then the real ones (without the eigenvalues 0); per body the complex amplitudes of its translation
    (global) and rotation (body frame), [re, im] each, the largest component 1 and real."""
    row = _row(csv_path, time)
    lam, X, elems = linearize(model, fdm, row, consistent)
    tiny = max(1e-6, 1e-5 * np.max(np.abs(lam), initial=0.0))  # eigenvalues 0 (cyclic coordinates) in round-off
    keep = [k for k in range(len(lam)) if abs(lam[k]) > tiny and lam[k].imag >= -1e-12]
    keep.sort(key=lambda k: (abs(lam[k].imag) < 1e-9, abs(lam[k].imag) if abs(lam[k].imag) >= 1e-9 else abs(lam[k])))
    bodies = [e for e in elems if e["type"] == "rigid_body"]
    out = []
    for k in keep[:count]:
        x = X[:, k]
        big = x[np.argmax(np.abs(x))]
        if abs(big) == 0:
            continue
        x = x / big
        z = lam[k]
        out.append({"eigenvalue": [z.real, z.imag], "frequency": abs(z.imag) / (2 * np.pi),
                    "natural_frequency": abs(z) / (2 * np.pi), "damping": -z.real / abs(z),
                    "bodies": {e["name"]: [[float(c.real), float(c.imag)] for c in x[e["dofs"][:6]]] for e in bodies}})
    return {"time": float(row["t"]), "modes": out}


def main(argv=None):
    ap = argparse.ArgumentParser(description="eigenvalues of a model about a steady motion")
    ap.add_argument("model")
    ap.add_argument("fdm", help="the model file the result was computed with")
    ap.add_argument("result", help="freedyn2_run CSV")
    ap.add_argument("--time", type=float)
    args = ap.parse_args(argv)
    lam, _ = analyze(args.model, args.fdm, args.result, args.time)
    for x in sorted(lam, key=lambda z: (abs(z.imag) < 1e-9, -z.real)):
        if abs(x) > 1e-6:
            f = abs(x.imag) / (2 * np.pi)
            print(f"{x.real:+12.5f} {x.imag:+12.5f}i   f = {f:7.3f} Hz   zeta = {-x.real / abs(x):+.3f}")


if __name__ == "__main__":
    main()
