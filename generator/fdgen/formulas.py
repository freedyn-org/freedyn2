"""Formulas in number fields: a value "=expression" of the model parameters.

  stiffness: =k_susp
  position: [2.6, 0.8, =h_mount + 0.25]
  parameters: {k_front: 30000, k_rear: =0.8 * k_front}

Every number of a model file may be a formula; the parameters themselves may use other
parameters. The syntax is that of the force expressions (fdgen.expressions: + - * / ^,
sin, sqrt, min, ..., pi), with the parameters as the only names: a formula is a number fixed
before the simulation, not a function of t or of measures (those are force expressions,
written without "="). Formulas are evaluated when the model is read, before the schema check,
so everything after it sees numbers only.
"""

import copy

import casadi as ca

from .errors import ModelError
from .expressions import ExpressionError, names_in, translate

PREFIX = "="


def is_formula(value):
    return isinstance(value, str) and value.lstrip().startswith(PREFIX)


def names(text):
    """Parameter names a formula uses."""
    return names_in(text.lstrip()[len(PREFIX):].strip()) - {"pi"}


def evaluate(text, values, where):
    """Number of a formula "=..." with the parameter values given."""
    expr = text.lstrip()[len(PREFIX):].strip()
    try:
        v = translate(expr, values)
    except ExpressionError as e:
        hint = ""
        if "t" in names(text) - set(values):
            hint = " (a formula is a number; functions of time are force expressions, written without =)"
        raise ModelError(f"{where}: {e}{hint}") from None
    except (ValueError, TypeError) as e:
        raise ModelError(f"{where}: {text!r}: {e}") from None
    if isinstance(v, (ca.SX, ca.MX, ca.DM)):
        v = ca.evalf(v)
    return float(v)


def _parameters(raw):
    """Values of the parameters, formulas among them in the order of their dependencies."""
    pending = dict(raw or {})
    values = {k: float(v) for k, v in pending.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
    pending = {k: v for k, v in pending.items() if k not in values}
    while pending:
        ready = {k: v for k, v in pending.items() if is_formula(v) and names(v) <= set(values)}
        if not ready:
            k, v = next(iter(pending.items()))
            if not is_formula(v):
                raise ModelError(f"parameters.{k}: {v!r} is neither a number nor a formula (=...)")
            missing = sorted(names(v) - set(values) - set(pending))
            raise ModelError(f"parameters.{k}: " + (f"unknown name {missing[0]!r} in {v!r}" if missing
                                                    else f"formulas that use each other: {sorted(pending)}"))
        for k, v in ready.items():
            values[k] = evaluate(v, values, f"parameters.{k}")
            del pending[k]
    return values


def resolve(model):
    """(model with every formula replaced by its number, names of the parameters the formulas
    use). The model given is not changed."""
    if not isinstance(model, dict):
        return model, set()
    model = copy.deepcopy(model)
    raw = model.get("parameters")
    values = _parameters(raw) if isinstance(raw, dict) else {}
    used = {n for v in (raw or {}).values() if is_formula(v) for n in names(v)} if values else set()
    if isinstance(raw, dict):
        model["parameters"] = values

    def walk(x, path):
        if isinstance(x, dict):
            for k, v in x.items():
                x[k] = walk(v, f"{path}.{k}" if path else str(k))
        elif isinstance(x, list):
            for i, v in enumerate(x):
                x[i] = walk(v, f"{path}.{i}")
        elif is_formula(x):
            used.update(names(x))
            return evaluate(x, values, path)
        return x

    for key, section in model.items():
        if key != "parameters":
            model[key] = walk(section, key)
    return model, used & set(values)
