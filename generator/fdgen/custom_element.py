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


class CustomElement:
    """Symbols of one model-specific element: connected bodies, parameters p, measures, splines."""

    def __init__(self, model, owner, texts, bodies, allow_measures=True):
        params, measures = model.get("parameters", {}), model.get("measures", {})
        model_splines = model.get("splines", {})
        used = set().union(*[names_in(x) for x in texts])
        calls = set().union(*[calls_in(x) for x in texts])
        unknown_calls = calls - set(model_splines) - set(BUILTIN_FUNCTIONS)
        if unknown_calls:
            raise ValueError(f"{owner}: unknown functions {sorted(unknown_calls)} in {texts}")
        allowed = set(params) | {"t", "pi"} | (set(measures) if allow_measures else set())
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
        # Measures enter the laws as intermediate symbols (chain rule in the derivatives).
        self.intermediates = []
        for name in self.measure_names:
            self.symbols[name] = self.intermediate(self.measure(measures[name]), name)
        self.functions = {}
        for name in sorted(calls & set(model_splines)):
            sp = model_splines[name]
            b_, c_, d_ = spl.coefficients(sp["x"], sp["y"], sp.get("type", "akima"))
            data = [self.consts.add(v, f"{name}_{k}") for k, v in (("x", sp["x"]), ("y", sp["y"]), ("b", b_),
                                                                     ("c", c_), ("d", d_))]
            self.functions[name] = lambda xv, data=data: spl.evaluate(xv, *data)

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

    def value0(self, expr, t=0.0):
        """Numeric value of an expression of q (and t) at the initial configuration."""
        c, consts = self.c, self.consts
        fn = ca.Function("v", [c.q, c.t, ca.vertcat(*consts.syms) if consts.syms else ca.SX(0, 1)], [self.expand(expr)])
        return float(fn(self.q0, t, consts.values))

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
        return ElementFunctions(name, **funs)
