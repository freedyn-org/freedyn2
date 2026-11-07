"""CasADi, imported on first use.

The modules of fdgen use `from ._casadi import ca` instead of `import casadi as ca`: importing
fdgen.model (validation, the .fdm writer) then needs no CasADi. Models of library elements are
written without it, e.g. in a browser (Pyodide, which has no CasADi); generating element code,
expression forces, drivers and splines import it when they first use it.
"""

import importlib


class _LazyCasadi:
    _module = None

    def __getattr__(self, name):
        if _LazyCasadi._module is None:
            try:
                _LazyCasadi._module = importlib.import_module("casadi")
            except ImportError as e:
                raise ImportError(
                    f"CasADi is needed here (casadi.{name}): pip install casadi") from e
        return getattr(_LazyCasadi._module, name)


ca = _LazyCasadi()
