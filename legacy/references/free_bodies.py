"""Free bodies under gravity and expression forces / torques: RK4 on the Newton-Euler equations
with kinematics, measures and loads written here in numpy (only the expression parser is
shared with the generator)."""

import math

import casadi as ca
import numpy as np

from .common import rotation

def free_bodies_reference(model, times, h=2e-4):
    """[(omega_body, A, R)] for free rigid bodies under gravity and expression forces / torques,
    RK4 on the Newton-Euler equations (Euler parameters, theta' = 1/2 Gbar^T omega_bar).

    Kinematics, measures and loads are evaluated here in numpy from the model description
    (fdgen.plugin definitions); only the expression parser is shared with the generator.
    h = 2e-4 agrees with h = 1e-4 to 1.4e-12 on examples/compare/spring_chain.fds."""
    from fdgen.expressions import translate

    if model["joints"]:
        raise ValueError("free_bodies_reference: no joints")
    names = list(model["bodies"])
    bodies = [model["bodies"][n] for n in names]
    g = np.array(model["gravity"], dtype=float)
    R0 = {n: np.array(b["position"], dtype=float) for n, b in zip(names, bodies)}
    A0 = {n: rotation(b["orientation"]) for n, b in zip(names, bodies)}

    def local(body, r):
        return np.asarray(r, dtype=float) if body == "ground" else A0[body].T @ (np.asarray(r, dtype=float) - R0[body])

    def axis_local(body, v):
        v = np.asarray(v, dtype=float) / np.linalg.norm(v)
        return v if body == "ground" else A0[body].T @ v

    def kin(state, body):
        """position, velocity of body points and rotation of a body (or the ground)."""
        if body == "ground":
            return (lambda u: u), (lambda u: np.zeros(3)), np.eye(3), np.zeros(3)
        i = names.index(body)
        R, Rd, th, wb = state[i]
        A = rotation(th)
        return (lambda u: R + A @ u), (lambda u: Rd + A @ np.cross(wb, u)), A, A @ wb

    def measure(spec, state):
        P1, V1, A1, w1 = kin(state, spec["body1"])
        P2, V2, A2, w2 = kin(state, spec.get("body2", "ground"))
        u1 = local(spec["body1"], spec.get("point1", R0[spec["body1"]]))
        u2 = local(spec.get("body2", "ground"), spec.get("point2", [0, 0, 0] if spec.get("body2", "ground") == "ground" else R0[spec["body2"]]))
        d, w = P1(u1) - P2(u2), V1(u1) - V2(u2)
        t = spec["type"]
        if t == "distance":
            val = np.linalg.norm(d)
        elif t == "distance_rate":
            return d @ w / np.linalg.norm(d)
        elif t in ("displacement", "velocity"):
            vec = d if t == "displacement" else w
            if spec.get("frame", "global") != "global":
                vec = kin(state, spec["frame"])[2].T @ vec
            val = vec["xyz".index(spec["component"])]
            if t == "velocity":
                return val
        else:
            raise ValueError(f"free_bodies_reference: measure {t} not supported")
        off = spec.get("offset", "none")
        if off == "initial":
            val -= measure(dict(spec, offset="none"), initial)
        elif not isinstance(off, str):
            val -= off
        return val

    # Each force law once as a numeric function of (t, its measures); parameters as constants.
    from fdgen.expressions import names_in

    laws = []
    for f in model["forces"]:
        law = f["force"] if f["type"] == "expression_force" else f["torque"]
        used = sorted(names_in(law) & set(model["measures"]))
        tt = ca.SX.sym("t")
        ms = [ca.SX.sym(n) for n in used]
        sym = dict(model.get("parameters", {}), t=tt, **dict(zip(used, ms)))
        laws.append((ca.Function("law", [tt, *ms], [translate(law, sym)]), used))

    def loads(state, t):
        F = [m * g for m in (b["mass"] for b in bodies)]
        M = [np.zeros(3) for _ in bodies]  # global torque about the center of mass
        for f, (law, used) in zip(model["forces"], laws):
            s = float(law(t, *[measure(model["measures"][n], state) for n in used]))
            i1 = names.index(f["body1"])
            i2 = None if f["body2"] == "ground" else names.index(f["body2"])
            P1, _, A1, _ = kin(state, f["body1"])
            if f["type"] == "expression_force":
                u1 = local(f["body1"], f.get("point1", R0[f["body1"]]))
                u2 = local(f["body2"], f.get("point2", [0, 0, 0] if f["body2"] == "ground" else R0[f["body2"]]))
                P2, _, A2, _ = kin(state, f["body2"])
                if f.get("direction", "line_of_sight") == "line_of_sight":
                    e = P2(u2) - P1(u1)
                    e = e / np.linalg.norm(e)
                elif f.get("frame", "global") == "global":
                    e = axis_local("ground", f["direction"])
                else:
                    e = A1 @ axis_local(f["body1"], f["direction"])
                F[i1] = F[i1] + s * e
                M[i1] = M[i1] + np.cross(P1(u1) - state[i1][0], s * e)
                if i2 is not None:
                    F[i2] = F[i2] - s * e
                    M[i2] = M[i2] - np.cross(P2(u2) - state[i2][0], s * e)
            else:
                e = axis_local("ground", f["axis"]) if f.get("frame", "global") == "global" else A1 @ axis_local(f["body1"], f["axis"])
                M[i1] = M[i1] + s * e
                if i2 is not None:
                    M[i2] = M[i2] - s * e
        return F, M

    def rhs(state, t):
        F, M = loads(state, t)
        out = []
        for i, b in enumerate(bodies):
            R, Rd, th, wb = state[i]
            J = np.array(b["inertia"], dtype=float)
            dwb = (rotation(th).T @ M[i] - np.cross(wb, J * wb)) / J
            G = np.array([[-th[1], th[0], th[3], -th[2]], [-th[2], -th[3], th[0], th[1]], [-th[3], th[2], -th[1], th[0]]])
            out.append((Rd, F[i] / b["mass"], 0.5 * G.T @ wb, dwb))
        return out

    add = lambda s, k, c: [tuple(x + c * y for x, y in zip(si, ki)) for si, ki in zip(s, k)]
    initial = [(R0[n], np.array(b["velocity"], dtype=float), np.array(b["orientation"], dtype=float),
                A0[n].T @ np.array(b["angular_velocity"], dtype=float)) for n, b in zip(names, bodies)]
    state, t, out = initial, 0.0, []
    for target in times:
        n = max(1, math.ceil((target - t) / h - 1e-9)) if target > t else 0
        dt = (target - t) / n if n else 0.0
        for _ in range(n):
            k1 = rhs(state, t)
            k2 = rhs(add(state, k1, dt / 2), t + dt / 2)
            k3 = rhs(add(state, k2, dt / 2), t + dt / 2)
            k4 = rhs(add(state, k3, dt), t + dt)
            state = [tuple(x + dt / 6 * (a + 2 * b + 2 * c + d) for x, a, b, c, d in zip(*z))
                     for z in zip(state, k1, k2, k3, k4)]
            t += dt
        t = target
        out.append([(s[3].copy(), rotation(s[2] / np.linalg.norm(s[2])), s[0].copy()) for s in state])
    return [tuple(np.array([o[i][j] for o in out]) for j in range(3)) for i in range(len(names))]
