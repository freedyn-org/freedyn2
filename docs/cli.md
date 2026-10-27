# Command line

```text
freedyn2 run model.yaml [-o result.csv] [--workdir DIR] [-- runtime options]
freedyn2 view model.yaml [result.csv] [--no-run] [--export page.html] [--port N] [-- runtime options]
freedyn2 check model.yaml [result.csv | --run] [-- runtime options]
freedyn2 examples [name [-o DIR]]
freedyn2 mcp [--workdir DIR]
freedyn2 compare model.fds ... --legacy <FreeDyn>
```

| command | |
|---|---|
| `run` | read the model (`.yaml`, or a legacy `.fds`), build it (and its plugin), simulate; reports build and simulation time |
| `view` | the [viewer](viewer.md) in the browser; simulates again on every save |
| `check` | the [checks](reference/checks.md) of the model and its results; exit status 1 on errors or failed references |
| `examples` | list the example models that come with FreeDyn, or copy one into the current directory |
| `mcp` | the MCP server for [assistants](assistants.md) (stdio) |
| `compare` | legacy FreeDyn against FreeDyn 2.0 on `.fds` models (needs the legacy program) |

`.fds` models of the legacy FreeDyn are read by `run`, `view` and `check`; `--profile legacy`
runs them with the legacy algorithms (for comparisons), the default `v2` with FreeDyn 2.0's.

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

```sh
freedyn2 run model.yaml -- --end-time 2 --step 1e-4
```

## Environment

| variable | |
|---|---|
| `CC` | the C compiler for model plugins (default: the first of cc, gcc, clang, cl) |
| `FREEDYN2_PLUGIN_CACHE` | where compiled plugins are kept (default `~/.cache/freedyn2/plugins`; `off`) |
| `FREEDYN2_RUN` | a `freedyn2_run` to use instead of the installed one |
| `FREEDYN2_CHROMIUM` | a Chromium for the MCP `screenshot` tool |
