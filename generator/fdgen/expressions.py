"""Force law expressions: a small, safe expression language translated to CasADi.

Syntax (Python expression syntax, parsed with `ast`; nothing is executed):

  numbers, + - * / ** (also ^), unary -, parentheses
  names: t (time), pi, model parameters, measures
  splines of the model, called like functions: name(x)
  functions: sin cos tan asin acos atan atan2 sinh cosh tanh exp log sqrt abs sign min max
             step(x, x0, y0, x1, y1[, mode])  mode: "polynomial" (default, 9th order, C4),
             "cosine" or "hard" (the legacy FreeDyn STEP function)
             if_else(c, a, b)  with comparisons < <= > >= == != in c

The expression is a CasADi SX function of the given symbols, so the generator differentiates it
exactly (the legacy kernel differentiates its expressions numerically).
"""

import ast
import math

import casadi as ca


class ExpressionError(ValueError):
    pass


def step(x, x0, y0, x1, y1, mode="polynomial"):
    """Legacy FreeDyn STEP (FL_Step): y0 for x < x0, y1 for x >= x1, smooth in between."""
    if mode == "hard":
        return ca.if_else(x < x0, y0, y1)
    tau = (x - x0) / (x1 - x0)
    if mode == "polynomial":
        s = 70 * tau**9 - 315 * tau**8 + 540 * tau**7 - 420 * tau**6 + 126 * tau**5
    elif mode == "cosine":
        s = (1 - ca.cos(math.pi * tau)) / 2
    else:
        raise ExpressionError(f"step mode must be polynomial, cosine or hard, not {mode!r}")
    return ca.if_else(x >= x1, y1, ca.if_else(x >= x0, y0 + (y1 - y0) * s, y0))


FUNCTIONS = {
    "sin": ca.sin, "cos": ca.cos, "tan": ca.tan, "asin": ca.asin, "acos": ca.acos, "atan": ca.atan,
    "atan2": ca.atan2, "sinh": ca.sinh, "cosh": ca.cosh, "tanh": ca.tanh, "exp": ca.exp, "log": ca.log,
    "sqrt": ca.sqrt, "abs": ca.fabs, "sign": ca.sign, "min": ca.fmin, "max": ca.fmax,
    "step": step, "if_else": ca.if_else,
}
_BINOPS = {ast.Add: lambda a, b: a + b, ast.Sub: lambda a, b: a - b, ast.Mult: lambda a, b: a * b,
           ast.Div: lambda a, b: a / b, ast.Pow: lambda a, b: a**b, ast.BitXor: lambda a, b: a**b}
_CMPOPS = {ast.Lt: lambda a, b: a < b, ast.LtE: lambda a, b: a <= b, ast.Gt: lambda a, b: a > b,
           ast.GtE: lambda a, b: a >= b, ast.Eq: lambda a, b: a == b, ast.NotEq: lambda a, b: a != b}


def calls_in(text):
    """Names of the functions called by an expression."""
    return {n.func.id for n in ast.walk(_parse(text)) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}


def names_in(text):
    """Names (variables) used by an expression, without function names."""
    tree = _parse(text)
    funcs = {id(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    return {n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and id(n) not in funcs}


def _parse(text):
    try:
        return ast.parse(text.replace("^", "**") if "^" in text else text, mode="eval")
    except SyntaxError as e:
        raise ExpressionError(f"syntax error in {text!r}: {e.msg}") from None


def translate(text, symbols, functions=None):
    """CasADi expression of `text`; `symbols` maps names to SX scalars (t, parameters, measures),
    `functions` names to callables of one argument (splines)."""
    symbols = dict(symbols, pi=math.pi)
    funcs = dict(FUNCTIONS, **(functions or {}))

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)) and not isinstance(n.value, bool):
            return float(n.value)
        if isinstance(n, ast.Constant) and isinstance(n.value, str):
            return n.value  # step mode
        if isinstance(n, ast.Name):
            if n.id not in symbols:
                raise ExpressionError(f"unknown name {n.id!r} in {text!r}")
            return symbols[n.id]
        if isinstance(n, ast.BinOp) and type(n.op) in _BINOPS:
            return _BINOPS[type(n.op)](ev(n.left), ev(n.right))
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.USub, ast.UAdd)):
            v = ev(n.operand)
            return -v if isinstance(n.op, ast.USub) else v
        if isinstance(n, ast.Compare) and len(n.ops) == 1 and type(n.ops[0]) in _CMPOPS:
            return _CMPOPS[type(n.ops[0])](ev(n.left), ev(n.comparators[0]))
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and not n.keywords:
            if n.func.id not in funcs:
                raise ExpressionError(f"unknown function {n.func.id!r} in {text!r}")
            return funcs[n.func.id](*[ev(a) for a in n.args])
        raise ExpressionError(f"unsupported construct {ast.dump(n)[:40]}... in {text!r}")

    v = ev(_parse(text))
    if isinstance(v, str):
        raise ExpressionError(f"{text!r} is not a number")
    return ca.SX(v) if isinstance(v, float) else v
