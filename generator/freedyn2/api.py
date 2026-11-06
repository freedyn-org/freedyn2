"""The Python API: a thin layer over the generator (fdgen) and the runtime program freedyn2_run.

A run is what `freedyn2 run` does: read the model, write the runtime model file (.fdm) and the
model plugin, run freedyn2_run in a subprocess, read its result CSV. Nothing here changes the
model or the methods; parameter values are set in the model file's data before validation, as
the viewer's tuned values (fdgen.view.server.apply_overrides).
"""

import copy
import re
import tempfile
from pathlib import Path

import numpy as np

from fdgen import resources
from fdgen.errors import FreeDynError, ModelError, RuntimeNotFoundError, SimulationError


def _runtime(path=None):
    """freedyn2_run, found as by the command line (fdgen.cli.find_runtime)."""
    from fdgen import cli

    if path is not None and not Path(path).is_file():  # given: that one, not the next one found
        raise RuntimeNotFoundError(f"freedyn2_run not found: {path}")
    try:
        return cli.find_runtime(path)
    except FileNotFoundError as e:
        raise RuntimeNotFoundError(str(e)) from None


def _options(end_time=None, step=None, output_step=None, inputs=None, options=()):
    """Runtime options of freedyn2_run (docs/cli.md) from the keywords of run and sweep."""
    out = []
    for flag, value in (("--end-time", end_time), ("--step", step), ("--output-step", output_step)):
        if value is not None:
            out += [flag, repr(float(value))]
    for name, value in (inputs or {}).items():
        out += ["--input", f"{name}={float(value)!r}"]
    return out + [str(o) for o in options]


class Model:
    """A model file (.yaml or legacy .fds), validated, with parameter values set (overrides).

    name, path, profile, overrides; data: the validated internal model (dict); parameters,
    inputs, solver; bodies, joints, forces, sensors, measures, drivers, outputs: element names.
    """

    def __init__(self, path, profile="v2", overrides=None):
        self.path = Path(path).resolve()
        self.profile = profile
        self.overrides = dict(overrides or {})
        if self.overrides and self.is_fds:
            raise ModelError(f"{self.path.name}: parameter values can be set in .yaml models only")
        self._data = None

    @property
    def is_fds(self):
        return self.path.suffix.lower() == ".fds"

    @property
    def data(self):
        if self._data is None:
            from fdgen.model import load_yaml, validate

            if self.is_fds:
                from fdgen.fds import load_fds

                self._data = load_fds(self.path, self.profile)
            else:
                raw = load_yaml(self.path)
                if self.overrides:
                    from fdgen.view.server import apply_overrides

                    try:
                        raw = apply_overrides(copy.deepcopy(raw), self.overrides)
                    except (ValueError, KeyError, IndexError, TypeError) as e:
                        raise ModelError(str(e)) from None
                self._data = validate(raw, self.path.parent)
        return self._data

    def _path_of(self, key):
        """The path of a value in the model file: a bare name is a model parameter."""
        if "." in key:
            return key
        if self.is_fds:
            raise ModelError(f"{self.path.name}: parameter values can be set in .yaml models only")
        names = self.data["parameters"]
        if key not in names:
            raise ModelError(f"no parameter {key!r}; parameters: {', '.join(names) or 'none'} "
                             "(other values: their path, e.g. forces.spring.stiffness)")
        return f"parameters.{key}"

    def with_parameters(self, values=None, **kwargs):
        """A copy with these values set: {name or path: number}; a bare name is a model
        parameter (parameters.<name>), a path any number of the model file
        (forces.spring.stiffness, bodies.rod.position.2)."""
        values = {**(values or {}), **kwargs}
        m = Model(self.path, self.profile, {**self.overrides, **{self._path_of(k): float(v) for k, v in values.items()}})
        m.data  # validate now: errors at the call that caused them
        return m

    @property
    def name(self):
        return self.data["model"]

    @property
    def parameters(self):
        return dict(self.data["parameters"])

    @property
    def inputs(self):
        return dict(self.data.get("inputs") or {})

    @property
    def solver(self):
        return dict(self.data["solver"])

    def _names(self, key):
        section = self.data.get(key) or {}
        return list(section) if isinstance(section, dict) else [e["name"] for e in section]

    bodies = property(lambda self: self._names("bodies"))
    joints = property(lambda self: self._names("joints"))
    forces = property(lambda self: self._names("forces"))
    sensors = property(lambda self: self._names("sensors"))
    measures = property(lambda self: self._names("measures"))
    drivers = property(lambda self: self._names("drivers"))
    outputs = property(lambda self: self._names("outputs"))

    def build(self, workdir):
        """Write <workdir>/<name>.fdm (and compile the model plugin if it needs one); its path."""
        from fdgen.model import write_fdm

        workdir = Path(workdir)
        workdir.mkdir(parents=True, exist_ok=True)
        fdm = workdir / f"{self.name}.fdm"
        write_fdm(self.data, fdm)
        return fdm

    def run(self, **kwargs):
        """run(self, ...): see freedyn2.run."""
        return run(self, **kwargs)

    def check(self, results=None, **kwargs):
        """check(self, results, ...): see freedyn2.check."""
        return check(self, results, **kwargs)

    def __repr__(self):
        extra = f", overrides={self.overrides}" if self.overrides else ""
        return f"<freedyn2.Model {self.path.name} profile={self.profile}{extra}>"


def load(path, profile="v2", params=None):
    """The model of a .yaml or legacy .fds file, validated (ModelError if not). profile: for
    .fds models, FreeDyn 2.0 methods (v2) or the legacy algorithm (legacy). params: values to
    set (Model.with_parameters)."""
    if isinstance(path, Model):
        m = path
    else:
        m = Model(path, profile)
        m.data
    return m.with_parameters(params) if params else m


class Result:
    """The results of one run: channels of the result CSV as numpy arrays.

    r.t, r["rod.R_x"], r.channels, r.to_pandas(); r.csv_path, r.fdm_path, r.workdir, r.log
    (the output of freedyn2_run), r.build_seconds, r.simulation_seconds; r.model (Model).
    A run in a temporary directory keeps it while the Result exists.
    """

    def __init__(self, model, csv_path, fdm_path=None, log="", build_seconds=None, simulation_seconds=None,
                 tmp=None):
        self.model, self.csv_path = model, Path(csv_path)
        self.fdm_path = Path(fdm_path) if fdm_path else None
        self.log, self.build_seconds, self.simulation_seconds = log, build_seconds, simulation_seconds
        self._tmp, self._data = tmp, None  # tmp: a TemporaryDirectory, removed with the last Result in it

    @property
    def workdir(self):
        return self.csv_path.parent

    @property
    def data(self):
        """{channel: numpy array}, with "t"."""
        if self._data is None:
            from fdgen.view.scene import read_results

            self._data = read_results(self.csv_path)
        return self._data

    @property
    def t(self):
        return self.data["t"]

    @property
    def channels(self):
        """The channel names (without t)."""
        return [n for n in self.data if n != "t"]

    def __getitem__(self, name):
        try:
            return self.data[name]
        except KeyError:
            raise KeyError(f"no channel {name!r}; r.channels lists them") from None

    def __contains__(self, name):
        return name in self.data

    def __len__(self):
        return len(self.t)

    def to_pandas(self):
        """A pandas DataFrame, indexed by t (needs pandas)."""
        try:
            import pandas as pd
        except ImportError:
            raise ImportError("Result.to_pandas needs pandas (pip install pandas)") from None
        return pd.DataFrame({n: self.data[n] for n in self.channels}, index=pd.Index(self.t, name="t"))

    def kpi(self, channel, name="max", t0=None, t1=None):
        """A key figure of a channel over t0 <= t <= t1: max, min, max |x|, peak-peak, mean, rms,
        final (fdgen.sweep.kpi)."""
        from fdgen.sweep import KPIS, kpi

        if name not in KPIS:
            raise ValueError(f"no key figure {name!r}; one of {', '.join(KPIS)}")
        return kpi(self.t, self[channel], name, t0, t1)

    def check(self):
        """The checks of the model and these results (freedyn2.check)."""
        return check(self.model, self)

    def eigenvalues(self, time=None):
        """Eigenvalues of the model linearized about the output at time (default: the first),
        fdgen.linearize; the model's bodies must be Lie group bodies (profile v2)."""
        from fdgen.linearize import linearize

        if self.fdm_path is None:
            raise FreeDynError("this result has no runtime model file (.fdm)")
        k = 0 if time is None else int(np.argmin(np.abs(self.t - time)))
        row = {n: v[k] for n, v in self.data.items()}
        lam, _, _ = linearize(self.model.data, self.fdm_path, row)
        return lam

    def __repr__(self):
        return (f"<freedyn2.Result {self.model.name}: t = {self.t[0]:g} .. {self.t[-1]:g} s, {len(self)} outputs, "
                f"{len(self.channels)} channels>")


def _workdir(workdir):
    """(directory, TemporaryDirectory or None): workdir, or a new temporary directory."""
    if workdir is not None:
        Path(workdir).mkdir(parents=True, exist_ok=True)
        return Path(workdir), None
    tmp = tempfile.TemporaryDirectory(prefix="freedyn2_")
    return Path(tmp.name), tmp


def run(model, end_time=None, step=None, output_step=None, params=None, inputs=None, options=(), workdir=None,
        profile="v2", runtime=None):
    """Build and simulate a model (a Model or the path of a .yaml or .fds file); the Result.

    end_time, step, output_step override the model's solver section; params: values to set
    (Model.with_parameters); inputs: runtime inputs held constant ({name: value}); options:
    further options of freedyn2_run (docs/cli.md), e.g. ["--linear-solver", "dense"].
    workdir: keep the model file, plugin and result CSV there (default: a temporary directory,
    removed with the Result). runtime: path of freedyn2_run (default: as the command line).
    Raises ModelError (model), SimulationError (freedyn2_run failed), RuntimeNotFoundError.
    """
    import time

    from fdgen import cli

    model = load(model, profile, params)
    exe = _runtime(runtime)
    d, tmp = _workdir(workdir)
    t0 = time.perf_counter()
    fdm = model.build(d)
    build_seconds = time.perf_counter() - t0
    csv = d / f"{model.name}.csv"
    try:
        seconds, log = cli.simulate(exe, fdm, csv, _options(end_time, step, output_step, inputs, options))
    except RuntimeError as e:
        raise SimulationError(str(e)) from None
    return Result(model, csv, fdm, log, build_seconds, seconds, tmp)


_run = run  # check(run=...) shadows it


class Report:
    """The checks of a model (and its results): summary, findings, references; passed: no
    errors and no failed references; str(report): the text of freedyn2 check."""

    def __init__(self, model_report, results_report=None):
        self.model_report, self.results_report = model_report, results_report
        self.summary = model_report["summary"]
        self.findings = model_report["findings"] + (results_report["findings"] if results_report else [])
        self.references = (results_report or model_report).get("references") or []

    @property
    def passed(self):
        return not any(e.get("severity") == "error" or e.get("passed") is False
                       for e in self.findings + self.references)

    def __str__(self):
        from fdgen.checks import report_text

        return report_text(self.model_report, self.results_report)

    def __repr__(self):
        return f"<freedyn2.Report {'passed' if self.passed else 'FAILED'}: {len(self.findings)} findings, " \
               f"{len(self.references)} references>"


def check(model, results=None, run=False, profile="v2", **run_kwargs):
    """The checks of a model (fdgen.checks): plausibility and the references of its `checks`
    section; with results (a Result, the path of a result CSV) or run=True (simulate it first,
    run_kwargs as freedyn2.run) also of the results. A Report."""
    from fdgen.checks import check_model, check_results

    model = load(model, profile)
    if run:
        results = _run(model, **run_kwargs)
    data = None
    if isinstance(results, Result):
        data = results.data
    elif results is not None:
        from fdgen.view.scene import read_results

        data = read_results(results)
    return Report(check_model(model.data), check_results(model.data, data) if data is not None else None)


def sweep(model, parameter, values, jobs=None, end_time=None, step=None, output_step=None, params=None, inputs=None,
          options=(), workdir=None, runtime=None, strict=True):
    """Simulate a .yaml model with one value (a parameter name or path, as Model.with_parameters)
    set to each of values, in parallel (jobs processes, default: one per core; fdgen.sweep).
    params: further values for all runs. The Results in the order of values; a failed value
    raises SimulationError (strict) or gives None."""
    from fdgen.sweep import run_sweep

    model = load(model, params=params)
    if model.is_fds:
        raise ModelError(f"{model.path.name}: sweeps need a .yaml model")
    path = model._path_of(parameter)
    values = [float(v) for v in values]
    exe = _runtime(runtime)
    d, tmp = _workdir(workdir)
    out = run_sweep(model.path, path, values, exe, d, overrides=model.overrides,
                    options=_options(end_time, step, output_step, inputs, options), jobs=jobs)
    failed = [r for r in out if isinstance(r, str)]
    if failed and strict:
        raise SimulationError(f"sweep of {path}: {len(failed)} of {len(values)} values failed:\n" + "\n".join(failed))
    results = []
    for v, r in zip(values, out):
        if isinstance(r, str):
            results.append(None)
            continue
        fdm = next(iter(sorted(Path(r).parent.glob("*.fdm"))), None)
        results.append(Result(Model(model.path, model.profile, {**model.overrides, path: v}), r, fdm, tmp=tmp))
    return results


def examples():
    """{name: path} of the YAML example models that come with FreeDyn (freedyn2 examples)."""
    base = resources.path("examples")
    return {p.relative_to(base).with_suffix("").as_posix(): p for p in sorted(base.rglob("*.yaml"))}


def example_path(name):
    """The path of an example: a name of examples() (pendulum, scaling/...) or a file below
    examples/ (compare/pendulum.fds)."""
    found = examples()
    if name in found:
        return found[name]
    p = resources.path("examples") / name
    if p.is_file():
        return p
    raise ModelError(f"no example {name!r}; freedyn2.examples() lists them")


def version():
    """The version: of the repository's pyproject.toml when run from the sources, else of the
    installed package."""
    project = resources.REPOSITORY / "pyproject.toml"
    if (resources.REPOSITORY / "generator" / "fdgen").is_dir() and project.is_file():
        m = re.search(r'^version\s*=\s*"([^"]+)"', project.read_text(encoding="utf-8"), re.M)
        if m:
            return m.group(1)
    try:
        from importlib.metadata import version as installed

        return installed("freedyn2")
    except Exception:  # noqa: BLE001  (not installed, no metadata)
        return "unknown"
