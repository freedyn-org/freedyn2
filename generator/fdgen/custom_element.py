"""Common part of model-specific elements (expression forces, drivers) of the model plugin.

A custom element connects the bodies its force law or motion and their measures refer to. Its
parameters p are collected while the expressions are built (Consts): model parameters, points,
directions, offsets, spline data and element state, so the generated C code has no literals
and the same plugin serves other parameter values.

The derivatives of an element (its ElementFunctions) are the expensive part; they depend only
on its symbolic expressions, not on the values of p, so they are kept per process, keyed by
the expressions (ELEMENT_CACHE): building the same model again with other parameter values
(the viewer's sliders) reuses them.
"""

import hashlib
from collections import OrderedDict

import casadi as ca
import numpy as np

from . import splines as spl
from .expressions import FUNCTIONS as BUILTIN_FUNCTIONS
from .expressions import calls_in, names_in, translate
from .kinematics import Connector, Ground
from .lagrange import ElementFunctions, expand
from .measures import Consts, measure


ELEMENT_CACHE = OrderedDict()  # key of the expressions -> ElementFunctions (least recently used first)
MAX_CACHED_ELEMENTS = 512


class PluginElement(tuple):
    """(name, element type, [bodies], p values, ElementFunctions) of one plugin element, with
    .inputs: runtime input name -> offset in p (empty for elements without inputs), .splines:
    [(table offset, number of points, segment offset)] of its spline calls (the runtime selects
    their segments)."""

    def __new__(cls, name, tname, bodies, p, elem, inputs=None, splines=()):
        self = super().__new__(cls, (name, tname, bodies, p, elem))
        self.inputs = dict(inputs or {})
        self.splines = list(splines)
        return self


class CustomElement:
    """Symbols of one model-specific element: connected bodies, parameters p, measures, splines."""

    def __init__(self, model, owner, texts, bodies, allow_measures=True):
        params, measures = model.get("parameters", {}), model.get("measures", {})
        inputs = model.get("inputs") or {}
        model_splines = model.get("splines", {})
        used = set().union(*[names_in(x) for x in texts])
        calls = set().union(*[calls_in(x) for x in texts])
        unknown_calls = calls - set(model_splines) - set(BUILTIN_FUNCTIONS)
        if unknown_calls:
            raise ValueError(f"{owner}: unknown functions {sorted(unknown_calls)} in {texts}")
        allowed = set(params) | set(inputs) | {"t", "pi"} | (set(measures) if allow_measures else set())
        if used - allowed:
            raise ValueError(f"{owner}: unknown names {sorted(used - allowed)} in {texts}")
        self.measure_names = sorted(used & set(measures))
        bodies = list(bodies)
        for m in self.measure_names:
            for key in ("body1", "body2", "frame"):
                b = measures[m].get(key)
                if b and b not in ("ground", "global") and b not in bodies:
                    bodies.append(b)
        types = {model["bodies"][b].get("element", "rigid_body") for b in bodies}
        if len(types) != 1:
            raise ValueError(f"{owner}: bodies of different element types")
        self.rep = "euler" if types == {"rigid_body_euler"} else "lie"
        self.bodies = bodies
        self.model = model
        c = self.c = Connector(self.rep, len(bodies), 0)
        self.consts = Consts()
        self.q0 = np.concatenate([np.r_[model["bodies"][b]["position"], model["bodies"][b]["orientation"]]
                                  for b in bodies])
        self.symbols = {"t": c.t}
        for name in sorted(used & set(params)):
            self.symbols[name] = self.consts.add(params[name], name)
        for name in sorted(used & set(inputs)):  # runtime inputs: parameters the runtime overwrites
            self.symbols[name] = self.consts.add_input(inputs[name], name)
        # Measures enter the laws as intermediate symbols (chain rule in the derivatives).
        self.intermediates = []
        for name in self.measure_names:
            self.symbols[name] = self.intermediate(self.measure(measures[name]), name)
        # splines: the table is data in p, the runtime selects the segment of each call (the
        # generated code is one cubic per call, whatever the number of points)
        self.functions = {}
        for name in sorted(calls & set(model_splines)):
            sp = model_splines[name]
            b_, c_, d_ = spl.coefficients(sp["x"], sp["y"], sp.get("type", "akima"))
            table = len(self.consts.values)
            self.consts.add(np.concatenate([sp["x"], sp["y"], b_, c_, d_]), f"{name}_table")
            self.functions[name] = lambda xv, table=table, n=len(sp["x"]): self._spline_call(xv, table, n)

    def spline_slots(self):
        """[(table offset, number of points, segment offset)] of the spline calls (fdm: splines)."""
        return [(table, n, seg) for _, table, n, seg in self.consts.splines]

    def _spline_call(self, xv, table, n):
        if isinstance(xv, (int, float)) or (hasattr(xv, "is_constant") and xv.is_constant()):
            # a constant argument: the value (no runtime lookup needed)
            v = self.consts.values
            x0 = float(xv)
            tab = [v[table + k * n: table + (k + 1) * n] for k in range(5)]
            i = spl.segment(tab[0], x0)
            dx = x0 - tab[0][i]
            return tab[1][i] + tab[2][i] * dx + tab[3][i] * dx ** 2 + tab[4][i] * dx ** 3
        r = self.consts.add_spline_call(xv, table, n)
        self.select_segments(self.q0, 0.0, self.consts.values)  # p consistent with the initial state
        return r

    def intermediate(self, expr, name):
        """Symbol standing for the scalar expr in loads and constraints (see group_element)."""
        if expr.is_constant():
            return expr
        S = ca.SX.sym(name)
        self.intermediates.append((S, expr))
        return S

    def expand(self, expr):
        """expr in terms of q, v, t only."""
        return expand(expr, self.c.intermediates + self.intermediates)

    def kin(self, name):
        return Ground() if name == "ground" else self.c.bodies[self.bodies.index(name)]

    def _p(self):
        return ca.vertcat(*self.consts.syms) if self.consts.syms else ca.SX(0, 1)

    def select_segments(self, q, t, values):
        """Set the spline segments in values (p) for the configuration q at time t (zero
        velocities), as the runtime does: calls in order, repeated for nested calls."""
        calls = self.consts.splines
        if not calls:
            return values
        c = self.c
        fn = ca.Function("x", [c.q, c.v, c.t, self._p()], [ca.vertcat(*[self.expand(x) for x, *_ in calls])])
        for _ in range(len(calls)):
            xs = np.atleast_1d(np.array(fn(q, np.zeros(c.v.numel()), t, values), float).ravel())
            changed = False
            for xv, (_, table, n, seg) in zip(xs, calls):
                i = spl.segment(values[table:table + n], xv)
                new = [values[table + k * n + i] for k in range(5)]
                if values[seg:seg + 5] != new:
                    values[seg:seg + 5] = new
                    changed = True
            if not changed:
                break
        return values

    def value0(self, expr, t=0.0):
        """Numeric value of an expression of q (and t) at the initial configuration."""
        c, consts = self.c, self.consts
        values = self.select_segments(self.q0, t, list(consts.values))
        fn = ca.Function("v", [c.q, c.t, self._p()], [self.expand(expr)])
        return float(fn(self.q0, t, values))

    def measure(self, spec):
        return measure(spec, self.kin, self.consts, self.model, self.value0, self.c.rate)

    def translate(self, text):
        return translate(text, self.symbols, self.functions)

    def _key(self, name, out, kw):
        """Everything element() builds from: the name, the bodies (representation and number),
        the symbols of p, the intermediates, the loads or constraints, outputs and updates."""
        c, consts = self.c, self.consts
        parts = [name, self.rep, str(c.q.numel()), str(c.p)]
        parts += [f"{S}={e}" for S, e in self.intermediates]
        parts += [f"{k}={kw[k]}" for k in sorted(kw)]
        parts += [f"out {k}={v}" for k, v in (out or {}).items()]
        if consts.updates:
            parts.append(f"post={consts.post()}")
        parts += [f"spline {x} {table} {n} {seg}" for x, table, n, seg in consts.splines]
        return hashlib.sha256("\n".join(parts).encode()).hexdigest()

    def element(self, name, out=None, **kw):
        """ElementFunctions of the element; kw: Q and / or Phi (kept by their expressions)."""
        c, consts = self.c, self.consts
        c.p = ca.vertcat(*consts.syms) if consts.syms else ca.SX(0, 1)
        key = self._key(name, out, kw)
        if key in ELEMENT_CACHE:
            ELEMENT_CACHE.move_to_end(key)
            return ELEMENT_CACHE[key]
        elem = self._element(name, out, kw)
        ELEMENT_CACHE[key] = elem
        while len(ELEMENT_CACHE) > MAX_CACHED_ELEMENTS:
            ELEMENT_CACHE.popitem(last=False)
        return elem

    def _element(self, name, out, kw):
        c, consts = self.c, self.consts
        funs = c.functions(name, intermediates=self.intermediates, **kw)
        if out:
            funs["out"] = c.output_function(name, out, self.expand)
        if consts.updates:
            funs["post"] = ca.Function(f"fd_{name}_post", [c.q, c.v, c.p, c.t], [self.expand(consts.post())],
                                       ["q", "qd", "p", "t"], ["p_new"])
        if consts.splines:
            funs["spline"] = ca.Function(f"fd_{name}_spline", [c.q, c.v, c.p, c.t],
                                         [ca.vertcat(*[self.expand(x) for x, *_ in consts.splines])],
                                         ["q", "qd", "p", "t"], ["x"])
        return ElementFunctions(name, **funs)
