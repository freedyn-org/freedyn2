# Python API

`import freedyn2` loads, runs, checks and sweeps models from Python; results come as numpy
arrays. It does what the [command line](cli.md) does: read the model (`.yaml` or a legacy
`.fds`), write the runtime model file, compile the model plugin if the model needs one, run
`freedyn2_run` in a subprocess and read its result CSV. The package comes with
`pip install freedyn2`; from the sources, `PYTHONPATH=generator` and a built `build/freedyn2_run`.

```python
import freedyn2

m = freedyn2.load("model.yaml")                 # validated; ModelError if not
m.name, m.parameters, m.bodies, m.joints, m.forces

r = freedyn2.run(m, end_time=2, step=1e-3, params={"k": 2e4})
r.t                                             # numpy array
r["rod.R_x"]                                    # a channel (docs: result channels)
r.channels                                      # their names, without t
r.kpi("rod.R_z", "peak-peak", t0=1.0)           # max, min, max |x|, peak-peak, mean, rms, final
r.to_pandas()                                   # a DataFrame indexed by t (needs pandas)
r.csv_path, r.log, r.simulation_seconds
```

## Functions

| | |
|---|---|
| `load(path, profile="v2", params=None)` | the `Model` of a `.yaml` or `.fds` file; `profile="legacy"`: `.fds` models with the legacy algorithms |
| `run(model, end_time=, step=, output_step=, params=, inputs=, options=, workdir=, profile=, runtime=)` | build and simulate a `Model` or a path; a `Result` |
| `check(model, results=None, run=False, **run_kwargs)` | the [checks](reference/checks.md) of the model, and of its results (a `Result`, a CSV path, or `run=True`); a `Report` |
| `sweep(model, parameter, values, jobs=None, **run_kwargs)` | one value over several, simulated in parallel; a list of `Result` |
| `convert(path, output=None, profile="v2", force=False)` | write a legacy `.fds` model as a YAML model file, as [`freedyn2 convert`](cli.md#convert); its path |
| `examples()`, `example_path(name)` | the example models: `{name: path}`; the path of one (`"quarter_car"`, `"compare/pendulum.fds"`) |
| `__version__` | the version |

- `params`: `{name: value}` sets values of the model file before it is validated, as the
  viewer's tuning sliders do. A bare name is a model parameter (`parameters` section), a path
  any number of the file: `"forces.spring.stiffness"`, `"bodies.rod.position.2"`. `.yaml`
  models only. `m.with_parameters({...})` gives the changed model; `m` stays as it is.
- `end_time`, `step`, `output_step` override the model's `solver` section, `inputs`
  (`{name: value}`) sets [runtime inputs](reference/model-file.md#runtime-inputs), `options`
  passes further [runtime options](cli.md#runtime-options), e.g. `["--linear-solver", "dense"]`.
- `workdir`: keep the model file, plugin and result CSV there. By default a run goes into a
  temporary directory that is removed with its `Result`.
- `runtime`: the `freedyn2_run` to use; by default found as by the command line (the
  installed one, `FREEDYN2_RUN`, `build/freedyn2_run`, the PATH).

`Model.to_yaml(path=None, force=False)` writes a `.fds` model (loaded with its `profile`) as
a YAML file, as `convert`:

```python
m = freedyn2.load("crank.fds", profile="legacy")
y = m.to_yaml("crank.yaml")                     # runs as crank.fds with --profile legacy
r = freedyn2.run(y)
```

`Result.check()` checks the run, `Result.eigenvalues(time=None)` linearizes the model about
the output at `time` (the first by default, Lie group bodies), as the viewer's eigenmodes.

Errors are exceptions: `ModelError` (invalid model, unknown parameter; a `ValueError`),
`SimulationError` (`freedyn2_run` failed; the message holds its output),
`RuntimeNotFoundError` (a `FileNotFoundError`); `FreeDynError` is the base of all three.

## In Jupyter

```python
import freedyn2
import matplotlib.pyplot as plt

path = freedyn2.example_path("quarter_car")
m = freedyn2.load(path)
print(m.parameters)

r = freedyn2.run(m, end_time=3)
plt.plot(r.t, r["body.R_z"], label="body")
plt.plot(r.t, r["wheel.R_z"], label="wheel")
plt.xlabel("t [s]"); plt.ylabel("z [m]"); plt.legend()

print(freedyn2.check(m, r))                     # the text of freedyn2 check
r.to_pandas().describe()
```

`freedyn2 examples quarter_car` copies the example next to the notebook to change it; the
[viewer](viewer.md) (`freedyn2 view model.yaml`) shows the same model in 3D.

## Parameter studies

`sweep` builds the model once per value and runs the simulations in parallel (`jobs`, default
one per core), as the viewer's sweep:

```python
import numpy as np
import freedyn2

path = freedyn2.example_path("forced_oscillator")
values = np.linspace(100, 800, 8)
runs = freedyn2.sweep(path, "k", values, end_time=5, params={"d": 2.0})
amplitude = [r.kpi("mass.R_z", "max |x|", t0=3.0) for r in runs]
```

A value that fails raises `SimulationError` naming each failed value; with `strict=False` it
gives `None` in its place. For more than one value at a time, a loop over `run` with `params`
does it, one run after the other:

```python
for k in (1e4, 2e4):
    for d in (500, 1000):
        r = freedyn2.run(path, params={"k": k, "d": d}, end_time=5)
```

## What it is not

The API runs whole simulations: `freedyn2_run` is a separate program, a run is a subprocess
and the results are read from its CSV file. There is no step-by-step control yet (advance one
step, read the state, set an input, go on); between runs, runtime inputs over time
(`options=["--inputs", "inputs.csv"]`) are the way to drive a model. A binding of the runtime
to Python, for co-simulation and control loops, is planned; this API is meant to stay the
same with it.
