"""Tire on a road surface: library force element "tire" (wheel body - ground).

Road (the same for every tire of a model, element parameters):
  height    h(x, y) = sum_k A_k sin(2 pi (x cos(psi_k) + y sin(psi_k)) / L_k + phi_k) + g_x x + g_y y
            (NH harmonics: amplitude A, wavelength L, heading psi, phase phi; mean slope g):
            roughness, waves, cross slope; analytic, so the contact normal and every
            derivative are exact and the cost does not depend on the position.
  friction  mu(x, y) = mu0 + (mu_patch - mu0) b(x; x0, x1) b(y; y0, y1), b a smooth box
            (tanh edges of width w): e.g. a split-mu patch under the wheels of one side.

Contact (point contact under the wheel center C, axle a = wheel body y axis):
  n road normal at (C_x, C_y); e_x = a x n / |a x n| (longitudinal), e_y = n x e_x (lateral);
  d = (C - (C_x, C_y, h)) . n distance of C from the road tangent plane; the loaded radius is
  r = d / (d_w . n) along d_w = -(n - (n.a) a) / |.| (from C to the road in the wheel plane),
  penetration delta = r0 - r, contact point P = C + r d_w.
  Normal force Fz = smooth max(0, kz delta + cz delta') (lift-off; softplus width eps_z).
  Slip from the contact point velocity V_P = V_C + omega x (P - C):
  kappa = -V_P.e_x / v_ref, tan(alpha) = -V_P.e_y / v_ref, v_ref = sqrt((V_C.e_x)^2 + v_eps^2).

Forces (Magic Formula, load dependent, combined slip by cosine weighting functions):
  dfz = (Fz - Fz0) / Fz0
  Fx0 = Dx sin(Cx atan(Bx kappa - Ex (Bx kappa - atan(Bx kappa)))), Dx = mu (pDx1 + pDx2 dfz) Fz,
        Kx = Fz pKx1 exp(pKx3 dfz), Bx = Kx / (Cx Dx)
  Fy0 = Dy sin(Cy atan(By alpha - Ey (By alpha - atan(By alpha)))), Dy = mu (pDy1 + pDy2 dfz) Fz,
        Ky = pKy1 Fz0 sin(2 atan(Fz / (pKy2 Fz0))), By = Ky / (Cy Dy)
  Fx = Fx0 cos(atan(rBx1 alpha)), Fy = Fy0 cos(atan(rBy1 kappa))
  Mz = -t Fy, t = t0 cos(atan(qBz alpha)) (pneumatic trail); My = -qsy1 Fz r0 tanh(omega_a / w_eps)
  (rolling resistance, omega_a the wheel spin about its axle).
The force F = Fx e_x + Fy e_y + Fz n acts at P: on the wheel as F at C and the moment
(P - C) x F + Mz n + My a (same virtual power). No relaxation lengths (no tire states).
"""

import casadi as ca
import numpy as np

NH = 16  # road harmonics
TIRE_KEYS = ("r0", "kz", "cz", "Fz0", "Cx", "pDx1", "pDx2", "pEx1", "pKx1", "pKx3", "Cy", "pDy1", "pDy2",
             "pEy1", "pKy1", "pKy2", "rBx1", "rBy1", "t0", "qBz", "qsy1", "v_eps", "w_eps", "eps_z")
TIRE_DEFAULTS = {  # a passenger car tire, roughly 205/55 R16 (Pacejka 2006, simplified)
    "r0": 0.3, "kz": 250000.0, "cz": 300.0, "Fz0": 4000.0,
    "Cx": 1.65, "pDx1": 1.2, "pDx2": -0.08, "pEx1": 0.3, "pKx1": 22.0, "pKx3": 0.2,
    "Cy": 1.3, "pDy1": 1.1, "pDy2": -0.1, "pEy1": -0.8, "pKy1": 18.0, "pKy2": 1.8,
    "rBx1": 10.0, "rBy1": 8.0, "t0": 0.03, "qBz": 8.0, "qsy1": 0.012,
    "v_eps": 0.5, "w_eps": 0.5, "eps_z": 20.0,
}
ROAD_KEYS = ("g_x", "g_y", "mu0", "mu_patch", "x0", "x1", "y0", "y1", "w")
N_ROAD = 4 * NH + len(ROAD_KEYS)  # harmonics, then ROAD_KEYS
N_P = len(TIRE_KEYS) + N_ROAD  # tire, then road
# Road given by the runtime (OpenCRG roads, fd::Road): the tangent plane h = h0 + gx (x - x0) +
# gy (y - y0) at the wheel center (x0, y0) and the friction there, updated before every
# evaluation of the element (*_local element types).
LOCAL_KEYS = ("x0", "y0", "h0", "gx", "gy", "mu")
N_LOCAL = len(LOCAL_KEYS)
N_P_LOCAL = len(TIRE_KEYS) + N_LOCAL


def road_parameters(road):
    """Element parameters of a road (dict, see the module docstring): NH harmonics, then ROAD_KEYS."""
    harmonics = list(road.get("harmonics", []))
    if len(harmonics) > NH:
        raise ValueError(f"at most {NH} road harmonics")
    harm = np.zeros((NH, 4))
    harm[:, 1] = 1.0  # unused harmonics: amplitude 0, wavelength 1
    for k, hk in enumerate(harmonics):
        harm[k] = [hk["amplitude"], hk["wavelength"], hk.get("heading", 0.0), hk.get("phase", 0.0)]
    patch = road.get("patch", {})
    slope = road.get("slope", [0.0, 0.0])
    r = {"g_x": slope[0], "g_y": slope[1], "mu0": road.get("friction", 1.0),
         "mu_patch": patch.get("friction", road.get("friction", 1.0)),
         "x0": patch.get("x", [0.0, 0.0])[0], "x1": patch.get("x", [0.0, 0.0])[1],
         "y0": patch.get("y", [0.0, 0.0])[0], "y1": patch.get("y", [0.0, 0.0])[1],
         "w": patch.get("edge", 0.5)}
    return list(harm.ravel()) + [float(r[k]) for k in ROAD_KEYS]


def split_road(p):
    """(harmonics, road dict) of road parameters p (road_parameters layout)."""
    harm = [ca.vertsplit(p[4 * k:4 * k + 4]) for k in range(NH)]
    r = {k: p[4 * NH + i] for i, k in enumerate(ROAD_KEYS)}
    return harm, r


def local_road_parameters(road):
    """Initial local road parameters (LOCAL_KEYS; the runtime sets them before use)."""
    return [0.0, 0.0, 0.0, 0.0, 0.0, float(road.get("friction", 1.0))]


def parameters(tire, road, local=False):
    """Element parameters p of a tire (dict, defaults TIRE_DEFAULTS) on a road (dict); local:
    the road is given by the runtime (tire_local)."""
    unknown = set(tire) - set(TIRE_KEYS)
    if unknown:
        raise ValueError(f"unknown tire parameters {sorted(unknown)}")
    t = dict(TIRE_DEFAULTS, **tire)
    return [float(t[k]) for k in TIRE_KEYS] + (local_road_parameters(road) if local else road_parameters(road))


class AnalyticRoad:
    """The road of the element parameters (harmonics, slope, friction patch)."""

    def __init__(self, p):
        self.harm, self.r = split_road(p)

    def contact(self, C):
        return road_contact(C, self.harm, self.r)

    def friction(self, x, y):
        return friction(x, y, self.r)


class LocalRoad:
    """The road as the tangent plane the runtime gives at the wheel center (LOCAL_KEYS)."""

    def __init__(self, p):
        self.x0, self.y0, self.h0, self.gx, self.gy, self.mu = ca.vertsplit(p[0:N_LOCAL])

    def contact(self, C):
        h = self.h0 + self.gx * (C[0] - self.x0) + self.gy * (C[1] - self.y0)
        g = ca.vertcat(self.gx, self.gy)
        n = ca.vertcat(-self.gx, -self.gy, 1.0) / ca.sqrt(1.0 + self.gx ** 2 + self.gy ** 2)
        return h, g, n

    def friction(self, x, y):
        return self.mu


def road_contact(C, harm, r):
    """Road height h, height gradient g and unit normal n below the point C (x, y of C)."""
    xs, ys = ca.SX.sym("xs"), ca.SX.sym("ys")
    hs = road_height(xs, ys, harm, r)
    hg = ca.substitute(ca.vertcat(hs, ca.gradient(hs, ca.vertcat(xs, ys))), ca.vertcat(xs, ys), C[0:2])
    h, g = hg[0], hg[1:3]
    n = ca.vertcat(-g[0], -g[1], 1.0) / ca.sqrt(1.0 + g[0] ** 2 + g[1] ** 2)
    return h, g, n


def road_height(x, y, harm, r):
    h = r["g_x"] * x + r["g_y"] * y
    for A, L, psi, phi in harm:
        h += A * ca.sin(2 * np.pi * (x * ca.cos(psi) + y * ca.sin(psi)) / L + phi)
    return h


def friction(x, y, r):
    box = lambda s, a, b: 0.25 * (1 + ca.tanh((s - a) / r["w"])) * (1 - ca.tanh((s - b) / r["w"]))
    return r["mu0"] + (r["mu_patch"] - r["mu0"]) * box(x, r["x0"], r["x1"]) * box(y, r["y0"], r["y1"])


def _magic(B, C, D, E, s):
    Bs = B * s
    return D * ca.sin(C * ca.atan(Bs - E * (Bs - ca.atan(Bs))))


def loads(c, road_type=AnalyticRoad):
    """Loads and outputs of the tire element on connector c (one wheel body, ground)."""
    t = {k: c.p[i] for i, k in enumerate(TIRE_KEYS)}
    road = road_type(c.p[len(TIRE_KEYS):])
    w = c.bodies[0]
    C = w.R
    a = w.axis(ca.DM([0.0, 1.0, 0.0]))
    h, g, n = road.contact(C)
    ex = ca.cross(a, n)
    ex = ex / ca.norm_2(ex)
    ey = ca.cross(n, ex)
    dw = -(n - ca.dot(n, a) * a)
    dw = dw / ca.norm_2(dw)
    d = (C[2] - h) * n[2]  # distance from the road tangent plane at (C_x, C_y)
    rl = d / -ca.dot(dw, n)
    delta = t["r0"] - rl
    f = t["kz"] * delta + t["cz"] * c.rate(delta)
    Fz = 0.5 * (f + ca.sqrt(f * f + t["eps_z"] ** 2))
    arm = rl * dw  # P - C
    VC = w.point_velocity(ca.DM.zeros(3))
    VP = VC + ca.cross(w.omega, arm)
    vref = ca.sqrt(ca.dot(VC, ex) ** 2 + t["v_eps"] ** 2)
    kappa = -ca.dot(VP, ex) / vref
    alpha = ca.atan(-ca.dot(VP, ey) / vref)
    mu = road.friction(C[0], C[1])
    dfz = (Fz - t["Fz0"]) / t["Fz0"]
    Dx = mu * (t["pDx1"] + t["pDx2"] * dfz) * Fz
    Kx = Fz * t["pKx1"] * ca.exp(t["pKx3"] * dfz)
    Fx0 = _magic(Kx / (t["Cx"] * Dx), t["Cx"], Dx, t["pEx1"], kappa)
    Dy = mu * (t["pDy1"] + t["pDy2"] * dfz) * Fz
    Ky = t["pKy1"] * t["Fz0"] * ca.sin(2 * ca.atan(Fz / (t["pKy2"] * t["Fz0"])))
    Fy0 = _magic(Ky / (t["Cy"] * Dy), t["Cy"], Dy, t["pEy1"], alpha)
    Fx = Fx0 * ca.cos(ca.atan(t["rBx1"] * alpha))
    Fy = Fy0 * ca.cos(ca.atan(t["rBy1"] * kappa))
    Mz = -t["t0"] * ca.cos(ca.atan(t["qBz"] * alpha)) * Fy
    My = -t["qsy1"] * Fz * t["r0"] * ca.tanh(ca.dot(w.omega, a) / t["w_eps"])
    F = Fx * ex + Fy * ey + Fz * n
    M = ca.cross(arm, F) + Mz * n + My * a
    out = {"force": ca.vertcat(Fx, Fy, Fz), "slip": ca.vertcat(kappa, alpha), "friction": mu,
           "penetration": delta, "contact": C + arm, "road_height": h}
    return [(0, F, ca.DM.zeros(3), M)], out
