"""freedyn2 from Python: load, run, check and sweep models; results as numpy arrays.

    import freedyn2
    m = freedyn2.load("model.yaml")                       # Model; m.parameters, m.bodies, ...
    r = freedyn2.run(m, end_time=2, params={"k": 2e4})    # Result; r.t, r["rod.R_x"], r.to_pandas()
    freedyn2.check(m, r).passed
    freedyn2.convert("model.fds", "model.yaml")           # legacy model as a YAML model file
    rs = freedyn2.sweep(m, "k", [1e4, 2e4, 4e4])          # [Result]

A thin layer over the generator (fdgen) and the runtime program freedyn2_run, which runs in a
subprocess as with `freedyn2 run` (docs/python-api.md). The command line is fdgen.cli.
"""

from fdgen.errors import FreeDynError, ModelError, RuntimeNotFoundError, SimulationError

from .api import Model, Report, Result, check, convert, example_path, examples, load, run, sweep, version

__version__ = version()

__all__ = ["Model", "Result", "Report", "load", "run", "check", "sweep", "convert", "examples", "example_path",
           "FreeDynError", "ModelError", "SimulationError", "RuntimeNotFoundError", "__version__"]
