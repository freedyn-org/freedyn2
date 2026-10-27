"""Models with prescribed motion or a single translational dof: RK4 on the free dofs."""

import math

import casadi as ca
import numpy as np

from .common import rk4

def slider_reference(model, times, h=1e-4):
    """[(omega_body, A, R)] for one body on a prismatic joint along global z under gravity and
    expression forces along z (examples/compare/spline_spring.fds): RK4 on
    m z'' = m g_z + sum F(t, z, z'). Measures z / vz of the body point at the center of mass;
    the spline coefficients are the data definition shared with the generator (fdgen.splines),
    the integration is independent."""
    from fdgen import splines
    from fdgen.expressions import names_in, translate

    (name, body), = model["bodies"].items()
    m, gz = body["mass"], model["gravity"][2]
    fns = {k: splines.numeric(s["x"], s["y"], s["type"]) for k, s in model["splines"].items()}
    laws = []
    for f in model["forces"]:
        if f["type"] != "expression_force" or f.get("frame", "global") != "global" or \
                list(np.asarray(f["direction"], float) / np.linalg.norm(f["direction"])) != [0.0, 0.0, 1.0]:
            raise ValueError("slider_reference: global z expression forces only")
        laws.append((f["force"], sorted(names_in(f["force"]) & set(model["measures"]))))
    z0 = body["position"][2]

    def measure(n, z, zd):
        s = model["measures"][n]
        if s["type"] == "displacement" and s["component"] == "z":
            return z - (z0 if s.get("offset") == "initial" else 0.0)
        if s["type"] == "velocity" and s["component"] == "z":
            return zd
        raise ValueError(f"slider_reference: measure {n} not supported")

    def acc(t, z, zd):
        F = m * gz
        for law, used in laws:
            sym = dict(model.get("parameters", {}), t=t, **{n: measure(n, z, zd) for n in used})
            F += float(translate(law, sym, {k: (lambda xv, f=f: f(float(xv))) for k, f in fns.items()}))
        return F / m

    y, t, out = np.array([z0, body["velocity"][2]], dtype=float), 0.0, []
    for target in times:
        n = max(1, math.ceil((target - t) / h - 1e-9)) if target > t else 0
        dt = (target - t) / n if n else 0.0
        for _ in range(n):
            f = lambda tt, yy: np.array([yy[1], acc(tt, yy[0], yy[1])])
            k1 = f(t, y)
            k2 = f(t + dt / 2, y + dt / 2 * k1)
            k3 = f(t + dt / 2, y + dt / 2 * k2)
            k4 = f(t + dt, y + dt * k3)
            y = y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            t += dt
        t = target
        out.append(y.copy())
    out = np.array(out)
    R = np.zeros((len(out), 3))
    R[:, :2] = body["position"][:2]
    R[:, 2] = out[:, 0]
    return [(np.zeros((len(out), 3)), np.repeat(np.eye(3)[None], len(out), 0), R)]


def _prescribed(model, name):
    """f(t), f'(t), f''(t) of a driver's motion including the activation polynomial, numeric
    (time derivatives exact through CasADi; the ramp joins f C2 at Ta)."""
    from fdgen.expressions import translate
    from fdgen.drivers import ramp

    d = model["drivers"][name]
    t = ca.SX.sym("t")
    f = translate(d["motion"], dict(model.get("parameters", {}), t=t))
    ft = ca.jacobian(f, t)
    F = ca.Function("f", [t], [f, ft, ca.jacobian(ft, t)])
    Ta = d.get("ramp", 0.0)
    if Ta > 0:
        f1, f1d, f1dd = (float(v) for v in F(Ta))
        A0 = 0.0  # both comparison models start their driven measure at 0
        g = ramp(t, Ta, A0, f1, f1d, f1dd)
        gt = ca.jacobian(g, t)
        G = ca.Function("g", [t], [g, gt, ca.jacobian(gt, t)])
        return lambda tt: tuple(float(v) for v in (G(tt) if tt < Ta else F(tt)))
    return lambda tt: tuple(float(v) for v in F(tt))


def driven_base_reference(model, times, h=1e-4):
    """[(omega_body, A, R)] of examples/compare/driven_base.fds: base z_b(t) prescribed (driver
    shaker, with its activation polynomial), bob on a prismatic joint below it:
    m z'' = -m g + k ((z_b - z) - 1) + c (z_b' - z') + 0.5 g (spring force from the model)."""
    zb = _prescribed(model, "shaker")
    m = model["bodies"]["bob"]["mass"]
    p = model["parameters"]
    k, c, g = p["p_k"], p["p_c"], -model["gravity"][2]

    def rhs(t, y):
        b, bd, _ = zb(t)
        return np.array([y[1], (-m * g + k * ((b - y[0]) - 1.0) + c * (bd - y[1]) + 0.5 * 9.81) / m])

    Y = rk4(rhs, [-1.0, 0.0], times, h)
    I = np.repeat(np.eye(3)[None], len(times), 0)
    Rb = np.zeros((len(times), 3))
    Rb[:, 2] = [zb(t)[0] for t in times]
    Rr = np.zeros((len(times), 3))
    Rr[:, 2] = Y[:, 0]
    zero = np.zeros((len(times), 3))
    return [(zero, I, Rb), (zero, I, Rr)]


def driven_rotor_reference(model, times, h=1e-4):
    """[(omega_body, A, R)] of examples/compare/driven_rotor.fds: rotor angle th1(t) prescribed
    (driver motor with activation), flywheel J2 th2'' = -k (th2 - th1) about the common z axis."""
    th1 = _prescribed(model, "motor")
    J2 = model["bodies"]["flywheel"]["inertia"][2]
    k = model["parameters"]["p_k"]
    Y = rk4(lambda t, y: np.array([y[1], -k * (y[0] - th1(t)[0]) / J2]), [0.0, 0.0], times, h)
    out = []
    for th, om, name in (([th1(t)[0] for t in times], [th1(t)[1] for t in times], "rotor"), (Y[:, 0], Y[:, 1], "flywheel")):
        th, om = np.asarray(th), np.asarray(om)
        A = np.zeros((len(times), 3, 3))
        A[:, 0, 0], A[:, 0, 1], A[:, 1, 0], A[:, 1, 1], A[:, 2, 2] = np.cos(th), -np.sin(th), np.sin(th), np.cos(th), 1
        W = np.zeros((len(times), 3))
        W[:, 2] = om
        out.append((W, A, np.tile(np.array(model["bodies"][name]["position"], dtype=float), (len(times), 1))))
    return out
