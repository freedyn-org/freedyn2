"""Expression forces: expression language, legacy translation, measures, plugin elements."""

import math

import casadi as ca
import numpy as np
import pytest

from fdgen import drivers, expression_forces, rigid_body
from fdgen.expressions import ExpressionError, names_in, step, translate
from fdgen.fds import legacy_expression
from fdgen.model import ModelError, validate

LIE = rigid_body.build()


def num(expr, **values):
    syms = {k: ca.SX.sym(k) for k in values}
    e = translate(expr, syms)
    return float(ca.Function("f", list(syms.values()), [e])(*values.values()))


def test_expression_language():
    assert num("-k * z - d * vz + F0 * sin(w * t)", k=2, z=0.5, d=3, vz=-1, F0=4, w=2, t=0.3) == pytest.approx(
        -1 + 3 + 4 * math.sin(0.6))
    assert num("x^2 + x**3 - atan2(y, x) + abs(-y) + max(x, y)", x=2, y=-1) == pytest.approx(
        4 + 8 - math.atan2(-1, 2) + 1 + 2)
    assert num("if_else(x < 1, 10, 20) + pi", x=0.5) == pytest.approx(10 + math.pi)
    assert names_in("k * sin(z) + step(t, 0, 0, 1, 1)") == {"k", "z", "t"}
    for bad in ("x +", "__import__('os')", "x.real", "unknown(x)", "y"):
        with pytest.raises(ExpressionError):
            translate(bad, {"x": ca.SX.sym("x")})


@pytest.mark.parametrize("mode", ["polynomial", "cosine", "hard"])
def test_step_matches_legacy_formulas(mode):
    """FL_Step: y0 + (y1 - y0) (70 tau^9 - 315 tau^8 + 540 tau^7 - 420 tau^6 + 126 tau^5) or
    y0 + (y1 - y0) (1 - cos(pi tau)) / 2 between x0 and x1; hard: y0 for x < x0, else y1."""
    x0, y0, x1, y1 = 0.5, -2.0, 1.5, 3.0
    xs = ca.SX.sym("x")
    f = ca.Function("s", [xs], [step(xs, x0, y0, x1, y1, mode)])
    for x in (0.0, 0.5, 0.7, 1.0, 1.4999, 1.5, 2.0):
        tau = (x - x0) / (x1 - x0)
        if mode == "hard":
            ref = y0 if x < x0 else y1
        elif x >= x1:
            ref = y1
        elif x >= x0:
            s = (70 * tau**9 - 315 * tau**8 + 540 * tau**7 - 420 * tau**6 + 126 * tau**5 if mode == "polynomial"
                 else (1 - math.cos(math.pi * tau)) / 2)
            ref = y0 + (y1 - y0) * s
        else:
            ref = y0
        # Cancellation in the polynomial near tau = 1: coefficients up to 540 times |y1 - y0| = 5.
        assert float(f(x)) == pytest.approx(ref, abs=1e-11), x


def test_legacy_expression_translation():
    m, p = {"mea_y": "m_mea_y", "vy": "m_vy"}, {"c1": "p_c1"}
    assert legacy_expression("-PARVAL[c1]*MEAVAL[mea_y]-0.5*MEAVAL[ vy ]+sin(SIMTIME[])*10.0", m, p) == \
        "-p_c1*m_mea_y-0.5*m_vy+sin(t)*10.0"
    s = legacy_expression("STEP[SIMTIME[],0,0,0.5,1000,polynomial]+STEP[SIMTIME[],0.5,1000,1,0]", m, p)
    assert s == 'step(t, 0, 0, 0.5, 1000, "polynomial")+step(t, 0.5, 1000, 1, 0, "hard")'
    for bad in ("MEAVAL[nope]", "PARVAL[x]", "SPL[t,data]"):
        with pytest.raises(ModelError):
            legacy_expression(bad, m, p)


def _model(forces, measures=None, parameters=None, n_bodies=2, rng=None):
    rng = rng or np.random.default_rng(0)
    bodies = {}
    for i in range(n_bodies):
        t = rng.normal(size=4)
        bodies[f"b{i}"] = {"mass": 1.0 + i, "inertia": [0.1, 0.2, 0.3], "position": list(rng.normal(size=3)),
                           "orientation": list(t / np.linalg.norm(t))}
    return validate({"model": "m", "bodies": bodies, "forces": forces, "measures": measures or {},
                     "parameters": parameters or {}, "solver": {"type": "HHT", "end_time": 1, "step": 0.01}})


MEASURES = {
    "dx": {"type": "displacement", "body1": "b0", "body2": "b1", "point1": [0.1, 0.2, 0.3], "component": "x",
           "frame": "b1", "offset": "initial"},
    "vy": {"type": "velocity", "body1": "b0", "point1": [0.0, 0.1, 0.0], "component": "y"},
    "dist": {"type": "distance", "body1": "b0", "body2": "b1", "offset": "initial"},
    "rate": {"type": "distance_rate", "body1": "b0", "body2": "b1"},
    "phi": {"type": "angle", "body1": "b0", "body2": "b1", "orientation": [0.9, 0.1, -0.3, 0.2]},
    "phid": {"type": "angular_rate", "body1": "b0", "body2": "b1", "orientation": [0.9, 0.1, -0.3, 0.2]},
}
FORCES = {
    "f_los": {"type": "expression_force", "body1": "b0", "body2": "b1", "point1": [0.2, 0, 0],
              "force": "-k * dist - 0.3 * rate + sin(2 * t) * dx"},
    "f_body": {"type": "expression_force", "body1": "b1", "direction": [0, 1, 1], "frame": "body",
               "force": "-k * vy * phi + step(t, 0.1, 0, 0.4, 2, 'cosine')"},
    "m_global": {"type": "expression_torque", "body1": "b0", "body2": "b1", "axis": [1, 0, 0],
                 "torque": "-2 * phi - 0.1 * phid + dx^2"},
}


def _elements():
    return expression_forces.expression_elements(_model(FORCES, MEASURES, {"k": 50.0}))


def test_measures_and_offsets_at_initial_configuration():
    """Offset measures are zero initially; the angle of a frame fixed relative to body2 too."""
    model = _model({"probe": {"type": "expression_force", "body1": "b0", "body2": "b1",
                              "force": "dx + dist + phi"}}, MEASURES)
    (name, _, bodies, p, elem), = expression_forces.expression_elements(model)
    q = np.concatenate([np.r_[model["bodies"][b]["position"], model["bodies"][b]["orientation"]] for b in bodies])
    value = float(elem.out(q, np.zeros(6 * len(bodies)), np.zeros(6 * len(bodies)), p, 0.0)[0])
    assert value == pytest.approx(0.0, abs=1e-13)


@pytest.mark.parametrize("index", range(len(FORCES)))
def test_plugin_iteration_matrix_vs_fd(index):
    """J of a generated expression element vs FD through the bodies' exponential update."""
    name, _, bodies, p, e = _elements()[index]
    rng = np.random.default_rng(index)
    n = len(bodies)
    beta, gamma, h = 0.2722, 0.54, 0.05
    coef = np.array([0.0, beta, gamma, h])
    q_n = [np.r_[rng.normal(size=3), rng.normal(size=4) / 2] for _ in range(n)]
    q_n = [np.r_[q[:3], q[3:] / np.linalg.norm(q[3:])] for q in q_n]
    v0, x0, incr0 = rng.normal(size=6 * n), rng.normal(size=6 * n), 0.3 * rng.normal(size=6 * n)

    def st(x):
        incr = incr0 + beta * h * h * x
        q = np.concatenate([np.array(LIE.update(q_n[k], incr[6 * k:6 * k + 6])).ravel() for k in range(n)])
        return q, v0 + gamma * h * x, incr

    def r(x):
        q, v, incr = st(x)
        return np.array(e.res(q, v, x, np.zeros(0), p, coef, incr, 0.37)[0]).ravel()

    q, v, incr = st(x0)
    J = np.array(e.iter(q, v, x0, np.zeros(0), p, coef, incr, 0.37))
    eps = 1e-5
    J_fd = np.column_stack([(r(x0 + eps * d) - r(x0 - eps * d)) / (2 * eps) for d in np.eye(len(x0))])
    assert np.max(np.abs(J - J_fd)) / max(1, np.max(np.abs(J_fd))) < 1e-8, name


def test_body_body_force_is_internal():
    """Loads of a body-body expression force on the two bodies cancel (zero resultant force)."""
    model = _model(FORCES, MEASURES, {"k": 50.0})
    name, _, bodies, p, e = expression_forces.expression_elements(model)[0]  # f_los, line of sight
    rng = np.random.default_rng(3)
    q = np.concatenate([np.r_[model["bodies"][b]["position"], model["bodies"][b]["orientation"]] for b in bodies])
    v = rng.normal(size=12)
    f = -np.array(e.res(q, v, np.zeros(12), np.zeros(0), p, np.r_[0, .25, .5, .01], 0 * v, 0.2)[1]).ravel()
    F1, F2 = f[0:3], f[6:9]
    np.testing.assert_allclose(F1 + F2, 0, atol=1e-12)


def test_yaml_checks():
    with pytest.raises(ModelError):
        _model({"f": {"type": "expression_force", "body1": "b0"}})  # no force law
    with pytest.raises(ModelError):
        _model({}, {"t": {"type": "distance", "body1": "b0"}})  # reserved name
    with pytest.raises(ValueError):
        expression_forces.expression_elements(_model({"f": {"type": "expression_force", "body1": "b0", "force": "q1"}}))


# --- splines -----------------------------------------------------------------------------

from fdgen import splines  # noqa: E402

XS, YS = [-0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2], [600.0, 150.0, 50.0, 0.0, -40.0, -120.0, -500.0]


def _spline_fun(kind):
    xs = ca.SX.sym("x")
    b, c, d = splines.coefficients(XS, YS, kind)
    return ca.Function("s", [xs], [splines.evaluate(xs, XS, YS, b, c, d)])


@pytest.mark.parametrize("kind", splines.TYPES)
def test_spline_interpolates_and_is_smooth(kind):
    f = _spline_fun(kind)
    df = f.jacobian()
    for x, y in zip(XS, YS):
        assert float(f(x)) == pytest.approx(y, abs=1e-12)
    for x in XS[1:-1]:  # C1 at the knots (both types); zcc also C2
        e = 1e-7
        assert float(df(x - e, 0)) == pytest.approx(float(df(x + e, 0)), rel=1e-5, abs=1e-3)
    if kind == "zcc":  # zero slope at both ends, quadratic beyond the last point
        assert abs(float(df(XS[0] + 1e-12, 0))) < 1e-6 and abs(float(df(XS[-1], 0))) < 1e-9
    else:  # Akima: linear beyond the last point
        assert float(df(0.3, 0)) == pytest.approx(float(df(0.5, 0)), rel=1e-12)


@pytest.mark.parametrize("kind", splines.TYPES)
def test_spline_casadi_matches_numeric_evaluation(kind):
    f, g = _spline_fun(kind), splines.numeric(XS, YS, kind)
    for x in np.linspace(-0.3, 0.3, 61):
        assert float(f(x)) == g(x)  # same operations: bitwise equal


def test_legacy_spline_translation():
    data = {"stiff": ([0, 1, 2], [0, 1, 4], "akima"), "drive": ([0, 1, 2], [0, 1, 0], "zcc")}
    used = {}
    s = legacy_expression("SPL[MEAVAL[z],stiff]+AKISPL[SIMTIME[],drive]+ZCCSPL[SIMTIME[],stiff]", {"z": "m_z"}, {},
                          data, used)
    # AKISPL is an alias of SPL in the legacy kernel: the data object's interpolation type.
    assert s == "s_stiff_akima(m_z)+s_drive_zcc(t)+s_stiff_zcc(t)"
    assert set(used) == {"s_stiff_akima", "s_drive_zcc", "s_stiff_zcc"}
    with pytest.raises(ModelError):
        legacy_expression("SPL[t, nope]", {}, {}, data, used)


# --- continuous angle (element state) -----------------------------------------------------

def test_continuous_angle_counts_revolutions():
    """angle = r + 2 pi round((ref - r) / 2 pi) with ref the last accepted value (post)."""
    model = _model({"m": {"type": "expression_torque", "body1": "b0", "axis": [0, 0, 1], "torque": "phi"}},
                   {"phi": {"type": "angle", "body1": "b0", "offset": "initial"}}, n_bodies=1)
    model["bodies"]["b0"]["orientation"] = [1.0, 0, 0, 0]
    (_, _, bodies, p, e), = expression_forces.expression_elements(model)
    p = np.array(p, dtype=float)
    R = np.array(model["bodies"]["b0"]["position"])
    angles, values = np.linspace(0, 4.5 * np.pi, 200), []
    for a in angles:  # rotate about global z in small steps, accepting each
        q = np.r_[R, np.cos(a / 2), 0, 0, np.sin(a / 2)]
        values.append(float(e.out(q, np.zeros(6), np.zeros(6), p, 0.0)[0]))
        p = np.array(e.post(q, np.zeros(6), p, 0.0)).ravel()
    np.testing.assert_allclose(values, angles, atol=1e-12)


# --- drivers ---------------------------------------------------------------------------------

def test_ramp_starts_at_rest_and_joins_c2():
    t = ca.SX.sym("t")
    Ta, A = 0.3, 0.2
    f = 0.5 * ca.sin(3 * t) + t**2
    ft, ftt = ca.jacobian(f, t), ca.jacobian(ca.jacobian(f, t), t)
    F = ca.Function("F", [t], [f, ft, ftt])
    g = drivers.ramp(t, Ta, A, *(float(v) for v in F(Ta)))
    gt = ca.jacobian(g, t)
    G = ca.Function("G", [t], [g, gt, ca.jacobian(gt, t)])
    np.testing.assert_allclose([float(v) for v in G(0.0)], [A, 0, 0], atol=1e-14)
    np.testing.assert_allclose([float(v) for v in G(Ta)], [float(v) for v in F(Ta)], rtol=1e-12, atol=1e-12)


def _driver_model(motion, ramp=0.0):
    m = _model({}, {"x": {"type": "displacement", "body1": "b0", "component": "x", "offset": "initial"}}, n_bodies=1)
    m["drivers"] = {"d": {"measure": "x", "motion": motion, "ramp": ramp}}
    return m


def test_driver_constraint_and_time_derivatives():
    """Phi = x - f(t): Phi(q0, 0) = 0, phid = B v + Phi_t, gamma includes -Phi_tt."""
    model = _driver_model("0.1 * sin(2 * t)")
    (_, _, bodies, p, e), = drivers.driver_elements(model)
    b = model["bodies"]["b0"]
    q = np.r_[b["position"], b["orientation"]]
    for v in (np.zeros(6), np.r_[0.2, 0, 0, 0, 0, 0]):
        A, gam, phi, phid = (np.array(x).ravel() for x in e.ic(q, v, 0, 0, p, 0, 0, 0.0))
        assert phi[-1] == pytest.approx(0.0, abs=1e-15)
        assert phid[-1] == pytest.approx(v[0] - 0.2, abs=1e-14)  # x' - f'(0), f'(0) = 0.2
    t = 0.7
    A, gam, phi, phid = (np.array(x).ravel() for x in e.ic(q, np.zeros(6), 0, 0, p, 0, 0, t))
    assert gam[-1] == pytest.approx(-0.4 * math.sin(2 * t), rel=1e-12)  # -Phi_tt = f''(t)
    with pytest.raises(ModelError):
        validate({"model": "m", "bodies": {"a": {"mass": 1, "inertia": [1, 1, 1]}},
                  "measures": {"v": {"type": "velocity", "body1": "a", "component": "x"}},
                  "drivers": {"d": {"measure": "v", "motion": "t"}},
                  "solver": {"type": "HHT", "end_time": 1, "step": 0.01}})


def test_fds_time_motion_import():
    from conftest import ROOT
    from fdgen.fds import load_fds

    m = load_fds(ROOT / "examples" / "compare" / "driven_base.fds")
    (name, d), = m["drivers"].items()
    assert d["ramp"] == 0.1 and d["measure"] == "m_zb" and "sin(" in d["motion"]
    m = load_fds(ROOT / "examples" / "compare" / "driven_rotor.fds")
    assert m["drivers"]["motor"]["ramp"] == 0.5 and m["measures"]["m_phi"]["type"] == "angle"


# --- spline data files ------------------------------------------------------------------------

def test_read_table_legacy_semantics(tmp_path):
    f = tmp_path / "t.txt"
    f.write_text("header 1\nheader 2\n  0.0   10  1.5\n1.0 20 2.5e0 \n\n2.0 30 4.0x\n")
    x, y = splines.read_table(f, 2, 0, 2)
    assert x == [0.0, 1.0, 2.0] and y == [1.5, 2.5, 4.0]  # empty line skipped, trailing garbage ignored
    assert splines.read_table(f, 2, 1, 0)[0] == [10.0, 20.0, 30.0]
    bad = {
        "a\nb\n\n1 2\n": 2,          # empty first data line
        "1 2\n1 2 3\n": 0,           # column count changes
        "1\t2\n3\t4\n": 0,           # tabs are not separators (legacy): one column
    }
    for text, header in bad.items():
        f.write_text(text)
        with pytest.raises(ValueError):
            splines.read_table(f, header, 0, 1)
    f.write_text("1 2\n3 4\n")
    with pytest.raises(ValueError):
        splines.read_table(f, 0, 0, 2)  # column index out of range


def test_yaml_spline_from_file(tmp_path):
    (tmp_path / "law.txt").write_text("x F\n-1 100\n0 0\n1 -100\n2 -300\n")
    m = validate({"model": "m", "bodies": {"a": {"mass": 1, "inertia": [1, 1, 1]}},
                  "splines": {"law": {"file": "law.txt", "header_lines": 1, "type": "zcc"}},
                  "solver": {"type": "HHT", "end_time": 1, "step": 0.01}}, base_dir=tmp_path)
    assert m["splines"]["law"]["x"] == [-1.0, 0.0, 1.0, 2.0] and m["splines"]["law"]["y"][-1] == -300.0
    with pytest.raises(ModelError):
        validate({"model": "m", "bodies": {"a": {"mass": 1, "inertia": [1, 1, 1]}},
                  "splines": {"law": {"file": "missing.txt"}},
                  "solver": {"type": "HHT", "end_time": 1, "step": 0.01}}, base_dir=tmp_path)


def test_fds_spline_from_file():
    from conftest import ROOT
    from fdgen.fds import load_fds

    m = load_fds(ROOT / "examples" / "compare" / "spline_spring.fds")
    sp = m["splines"]["s_stiff_akima"]
    assert sp["x"] == [-0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2] and sp["y"][0] == 600.0


def test_elements_are_kept_for_other_parameter_values(tmp_path):
    """The derivatives depend on the expressions only: the same model with other parameter
    values reuses its elements (custom_element.ELEMENT_CACHE) and its generated code, with the
    new values in p; another law builds a new element."""
    from fdgen import custom_element
    from fdgen.plugin import plugin_elements

    def build(k, law="-k * z"):
        m = {"model": "cache", "solver": {"type": "HHT", "end_time": 0.1, "step": 1e-3},
             "parameters": {"k": k},
             "bodies": {"b": {"mass": 1.0, "inertia": [1, 1, 1]}},
             "measures": {"z": {"type": "displacement", "body1": "b", "component": "z"}},
             "forces": {"f": {"type": "expression_force", "body1": "b", "force": law, "direction": [0, 0, 1]}}}
        return plugin_elements(validate(m))

    custom_element.ELEMENT_CACHE.clear()
    (a,), (b,) = build(10.0), build(20.0)
    assert b[4] is a[4] and len(custom_element.ELEMENT_CACHE) == 1  # the same ElementFunctions
    assert 10.0 in a[3] and 20.0 in b[3] and 10.0 not in b[3]  # the values are in p
    (c,) = build(10.0, "-k * z * z")
    assert c[4] is not a[4] and len(custom_element.ELEMENT_CACHE) == 2


def _input_model(**extra):
    m = {"model": "inp", "solver": {"type": "HHT", "end_time": 0.1, "step": 1e-3},
         "parameters": {"k": 10.0}, "inputs": {"F": 2.5, "x_set": 0.0, "unused": 1.0},
         "bodies": {"b": {"mass": 1.0, "inertia": [1, 1, 1]},
                    "c": {"mass": 1.0, "inertia": [1, 1, 1], "position": [1, 0, 0]}},
         "measures": {"z": {"type": "displacement", "body1": "b", "component": "z"},
                      "zc": {"type": "displacement", "body1": "c", "component": "z"}},
         "forces": {"f": {"type": "expression_force", "body1": "b", "force": "-k * z + F", "direction": [0, 0, 1]}},
         "drivers": {"d": {"measure": "zc", "motion": "x_set"}}}
    m.update(extra)
    return m


def test_runtime_inputs_are_parameters_with_slots():
    """An input enters p like a parameter, at its default value; the plugin element records its
    offset, which the runtime overwrites (fd::System::set_input)."""
    from fdgen.plugin import plugin_elements

    elems = {pe[0]: pe for pe in plugin_elements(validate(_input_model()))}
    f, d = elems["f"], elems["d"]
    assert set(f.inputs) == {"F"} and set(d.inputs) == {"x_set"}
    assert f[3][f.inputs["F"]] == 2.5 and d[3][d.inputs["x_set"]] == 0.0
    # the generated law reads the input from p: Q_z = -k z + F at z = 0 is the value in p
    p = list(f[3])
    p[f.inputs["F"]] = 7.0
    q = np.r_[0, 0, 0, 1, 0, 0, 0]
    Q = f[4].res(q, np.zeros(6), np.zeros(6), np.zeros(0), p, [0, 0, 0, 1e-3], np.zeros(6), 0.0)
    assert float(np.asarray(Q[1]).ravel()[2]) == pytest.approx(-7.0)  # h - Q: Q enters with a minus


def test_runtime_inputs_in_the_model_file(tmp_path):
    from fdgen.model import write_fdm

    path = tmp_path / "inp.fdm"
    write_fdm(validate(_input_model()), path)
    lines = path.read_text().splitlines()
    assert "input F 2.5" in lines and "input x_set 0.0" in lines and "input unused 1.0" in lines
    assert any(ln.startswith("inputs F:") for ln in lines) and any(ln.startswith("inputs x_set:") for ln in lines)


def test_runtime_input_names_must_be_unique_and_not_in_formulas():
    with pytest.raises(ModelError, match="used twice"):
        validate(_input_model(inputs={"k": 1.0}))
    with pytest.raises(ModelError, match="used twice"):
        validate(_input_model(inputs={"z": 1.0}))
    with pytest.raises(ModelError, match="reserved"):
        validate(_input_model(inputs={"t": 1.0}))
    m = _input_model()
    m["bodies"]["b"]["mass"] = "=F"  # formulas are evaluated at generation time: parameters only
    with pytest.raises(Exception, match="F"):
        validate(m)


def test_outputs_are_channels_without_loads():
    from fdgen.plugin import plugin_elements

    m = _input_model(outputs={"z_mm": "z * 1000", "push": "F - k * z"})
    elems = {pe[0]: pe for pe in plugin_elements(validate(m))}
    o = elems["z_mm"]
    q = np.r_[0, 0, 0.02, 1, 0, 0, 0]
    f = o[4].out(q, np.zeros(6), np.zeros(6), o[3], 0.0)
    assert float(f) == pytest.approx(20.0)
    Q = o[4].res(q, np.zeros(6), np.zeros(6), np.zeros(0), o[3], [0, 0, 0, 1e-3], np.zeros(6), 0.0)
    assert np.allclose(np.asarray(Q[1]).ravel(), 0.0)  # no load
    assert set(elems["push"].inputs) == {"F"}
    with pytest.raises(ValueError, match="no measure"):
        plugin_elements(validate(_input_model(outputs={"const": "k * 2"})))
    with pytest.raises(ModelError, match="already used"):
        validate(_input_model(outputs={"f": "z"}))
