# Command line

```text
freedyn2 run model.yaml [-o result.csv] [--workdir DIR] [-- runtime options]
freedyn2 view model.yaml [result.csv] [--no-run] [--export page.html] [--port N] [-- runtime options]
freedyn2 check model.yaml [result.csv | --run] [-- runtime options]
freedyn2 examples [name [-o DIR]]
freedyn2 convert model.fds [-o model.yaml] [--profile v2|legacy] [--force]
freedyn2 mcp [--workdir DIR]
freedyn2 compare model.fds ... --legacy <FreeDyn>
```

| command | |
|---|---|
| `run` | read the model (`.yaml`, or a legacy `.fds`), build it (and its plugin), simulate; reports build and simulation time |
| `view` | the [viewer](viewer.md) in the browser; simulates again on every save |
| `check` | the [checks](reference/checks.md) of the model and its results; exit status 1 on errors or failed references |
| `examples` | list the example models that come with FreeDyn, or copy one into the current directory |
| `convert` | write a legacy `.fds` model as a YAML model file ([below](#convert)) |
| `mcp` | the MCP server for [assistants](assistants.md) (stdio) |
| `compare` | legacy FreeDyn against FreeDyn 2.0 on `.fds` models (needs the legacy program) |

`.fds` models of the legacy FreeDyn are read by `run`, `view` and `check`; `--profile legacy`
runs them with the legacy algorithms (for comparisons), the default `v2` with FreeDyn 2.0's.

## convert

```sh
freedyn2 convert crank.fds                      # crank.yaml next to it
freedyn2 convert crank.fds -o models/crank.yaml --profile legacy
```

writes the model that `run`, `view` and `check` read from the `.fds` as a YAML
[model file](reference/model-file.md): the elements in the order of the `.fds`, comments naming
the `.fds` blocks they come from (`BodyID`, `ConstraintID`, …), every number exactly as
imported. The YAML runs as the `.fds` does with the same profile: `freedyn2 run crank.yaml`
gives the results of `freedyn2 run crank.fds --profile v2` (with `--profile legacy`: of
`--profile legacy`; no `--profile` is needed for the YAML, it carries the settings).

| option | |
|---|---|
| `-o FILE` | the YAML file, or a directory for `<model>.yaml` in it (default: next to the `.fds`) |
| `--profile v2\|legacy` | the solver settings (and, for `legacy`, the Euler-parameter bodies) written to the YAML; default `v2` |
| `--force` | overwrite an existing file (without it, `convert` stops) |

- Spline data in files (`PathToSplineData`) stay in their files, referenced relative to the YAML
  file; inline data (`XData`, `YData`) are written as `x`, `y`.
- Names must be identifiers in YAML. Constraints are named by their `ConstraintID` in an
  `.fds`, so a joint `1` becomes `joint_1` (and its channels `joint_1.dx.lambda`, …); bodies,
  forces and drivers with other characters in their names are renamed the same way. The comment
  at the element gives the `.fds` name. Measures, parameters and splines are used in
  expressions by name and are not renamed: a name that is not an identifier is an error.
- The written file is read back and compared with the imported model; what cannot be written
  or would read back differently is an error naming the element, so nothing is left out.
- Joint frames become quaternions and are turned back into rotation matrices when the YAML is
  read, and body orientations are normalized again: in the comparison models the results agree
  with the `.fds` bitwise or to 1e-12 (round-off).

## Runtime options

Everything after `--` goes to the solver `freedyn2_run` and overrides the model's `solver`
section:

| option | |
|---|---|
| `--end-time T`, `--step H`, `--output-step H`, `--alpha A` | the solver values |
| `--tolerance E`, `--fixed-step` | step size control (legacy adaptive models) / none |
| `--newton-fixed N` | [real-time mode](reference/solver.md#real-time-mode): exactly N Newton corrections per step |
| `--linear-solver klu\|sparselu\|dense` | the linear solver (default KLU) |
| `--step-log FILE`, `--timing`, `--timing-log FILE` | diagnosis: step sizes and Newton iterations per step; the distribution of the time per step; times per step |
| `--input NAME=VALUE` | a [runtime input](reference/model-file.md#runtime-inputs) held constant (repeatable) |
| `--inputs FILE.csv` | runtime inputs over time: header `t,NAME,...`, linear between the rows, held before the first and after the last; a step uses the value at its end time |

```sh
freedyn2 run model.yaml -- --end-time 2 --step 1e-4
freedyn2 run examples/input_oscillator.yaml -- --input F=1.5 --inputs ramp.csv
```

## Environment

| variable | |
|---|---|
| `CC` | the C compiler for model plugins (default: the first of cc, gcc, clang, cl) |
| `FREEDYN2_PLUGIN_CACHE` | where compiled plugins are kept (default `~/.cache/freedyn2/plugins`; `off`) |
| `FREEDYN2_RUN` | a `freedyn2_run` to use instead of the installed one |
| `FREEDYN2_CHROMIUM` | a Chromium for the MCP `screenshot` tool |
