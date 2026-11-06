# Command line

```text
freedyn2 run model.yaml [-o result.csv] [--workdir DIR] [-- runtime options]
freedyn2 view model.yaml [result.csv] [--no-run] [--export page.html] [--port N] [-- runtime options]
freedyn2 check model.yaml [result.csv | --run] [-- runtime options]
freedyn2 examples [name [-o DIR]]
freedyn2 mcp [--workdir DIR]
freedyn2 setup-mcp [--client claude-desktop|claude-code|all] [--remove] [--dry-run]
freedyn2 config [KEY [VALUE] | KEY --unset]
freedyn2 doctor
freedyn2 compare model.fds ... --legacy <FreeDyn>
```

| command | |
|---|---|
| `run` | read the model (`.yaml`, or a legacy `.fds`), build it (and its plugin), simulate; reports build and simulation time |
| `view` | the [viewer](viewer.md) in the browser; simulates again on every save |
| `check` | the [checks](reference/checks.md) of the model and its results; exit status 1 on errors or failed references |
| `examples` | list the example models that come with FreeDyn, or copy one into the current directory |
| `mcp` | the MCP server for [assistants](assistants.md) (stdio) |
| `setup-mcp` | register the MCP server with Claude Desktop and Claude Code ([below](#setup-mcp)) |
| `config` | show or set the user configuration, e.g. `freedyn2 config compiler zig` ([below](#config)) |
| `doctor` | what this installation finds: runtime, compiler, directories, MCP registration ([below](#doctor)) |
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
| `--input NAME=VALUE` | a [runtime input](reference/model-file.md#runtime-inputs) held constant (repeatable) |
| `--inputs FILE.csv` | runtime inputs over time: header `t,NAME,...`, linear between the rows, held before the first and after the last; a step uses the value at its end time |

```sh
freedyn2 run model.yaml -- --end-time 2 --step 1e-4
freedyn2 run examples/input_oscillator.yaml -- --input F=1.5 --inputs ramp.csv
```

## setup-mcp

`freedyn2 setup-mcp` registers the MCP server of this installation with the assistants, so that
they start `freedyn2 mcp` themselves:

- **Claude Desktop**: the entry `mcpServers.freedyn2` in `claude_desktop_config.json`
  (Windows `%APPDATA%\Claude\`, macOS `~/Library/Application Support/Claude/`, Linux
  `~/.config/Claude/`) is set; everything else in the file stays, the previous file is kept as
  `claude_desktop_config.json.bak`, a missing file is created. Restart Claude Desktop afterwards.
- **Claude Code**: with `claude` on the PATH, `claude mcp add --scope user freedyn2 -- <freedyn2> mcp`
  is run; without it the command is printed.

The command registered is the absolute path of this installation's `freedyn2` program (or this
Python with `-m fdgen.cli`). `--client` picks one (default `all`), `--remove` unregisters,
`--dry-run` only shows what would change. Running it again changes nothing that is already
right. Exit status 0: done or nothing to do; 1: failed (e.g. a `claude_desktop_config.json`
that is not valid JSON is left alone). The Windows installer runs
`freedyn2 setup-mcp --client claude-desktop` (and `--remove` when uninstalling).

## config

```sh
freedyn2 config                    # all values, and where the file is
freedyn2 config compiler zig       # set one (true, false and numbers are JSON, the rest text)
freedyn2 config compiler           # show one
freedyn2 config compiler --unset   # back to the default
```

| key | |
|---|---|
| `compiler` | the C compiler for model plugins: a path or a command, `zig`, `msvc` or `auto` (see [below](#c-compiler)) |
| `update_check` | `false`: never ask GitHub for a newer release |

The file is `config.json` in the configuration directory (Windows `%APPDATA%\freedyn2`, else
`~/.config/freedyn2`); `FREEDYN2_HOME` puts it and the other per-user directories below one
directory.

## doctor

`freedyn2 doctor` prints what this installation finds – for a support request, paste it:
the version, Python, where FreeDyn is installed, the runtime `freedyn2_run`, the C compiler
(which one, where it was found, its version), the plugin cache, the configuration, data and log
directories, whether the MCP server is registered with Claude Desktop and Claude Code, and
whether pywebview is installed. Exit status 1 if the runtime or the compiler is missing.

## C compiler

Models with expression forces, drivers or outputs are compiled into a model plugin. The
compiler is the first of:

1. the environment variable `CC`
2. the configuration: `freedyn2 config compiler ...`
3. `cc`, `gcc`, `clang`, `cl` on the PATH
4. Windows: Visual Studio (or the Build Tools) with the C++ tools, found with `vswhere`
5. zig, if the Python package `ziglang` is installed: `pip install freedyn2[compiler]` (or `pip install ziglang`)

`CC` and `compiler` take a path or a command (`gcc-13`, `zig cc`) or one of

| value | |
|---|---|
| `zig` | zig from the `ziglang` package (`python -m ziglang cc`), else `zig` on the PATH; on Windows it builds MinGW-ABI DLLs |
| `msvc` | Visual Studio without a developer prompt: `vcvars64.bat` is run once, its environment is kept in the cache directory |
| `auto` | the order above (the default) |

zig is the compiler that comes with pip: nothing else to install, on Windows too. Its first
plugin takes some seconds longer (zig builds its C runtime parts once, into its own cache).

## Environment

| variable | |
|---|---|
| `CC` | the C compiler for model plugins (see [C compiler](#c-compiler)) |
| `FREEDYN2_PLUGIN_CACHE` | where compiled plugins are kept (default: `plugins` in the cache directory, `~/.cache/freedyn2/plugins`, Windows `%LOCALAPPDATA%\freedyn2\cache\plugins`; `off`: no cache) |
| `FREEDYN2_HOME` | all per-user directories (configuration, data, cache, logs) below this one |
| `FREEDYN2_RUN` | a `freedyn2_run` to use instead of the installed one |
| `FREEDYN2_CHROMIUM` | a Chromium for the MCP `screenshot` tool |
