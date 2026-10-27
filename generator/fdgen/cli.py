"""freedyn2: read a model (.yaml or legacy .fds), build it and simulate it in one call.

  freedyn2 run model.fds [-o result.csv] [--profile v2|legacy] [--workdir DIR] [-- runtime options]
  freedyn2 view model.yaml [result.csv] [--no-run] [--export page.html] [-- runtime options]
  freedyn2 check model.yaml [result.csv | --run] [-- runtime options]
  freedyn2 mcp                                           (MCP server for assistants, fdgen.mcp_server)
  freedyn2 examples [name [-o dir]]                     (list the examples, copy one)
  freedyn2 compare model.fds ... --legacy <FreeDyn>     (legacy/benchmark.py, see there)

`run` imports the model, writes the runtime model file (.fdm) and, for models with expression
forces or drivers, compiles the model plugin; then it runs the runtime `freedyn2_run`. Building
(once per model) and simulating are timed separately. The runtime is found by --runtime, the
environment variable FREEDYN2_RUN, build/freedyn2_run of this repository or the PATH.
`view` shows the model and its results in the web browser (fdgen.view.server). `check` prints
the checks of the model and its results (fdgen.checks); exit status 1 on errors or failed
references.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from . import resources

ROOT = resources.REPOSITORY


EXE = ".exe" if sys.platform == "win32" else ""
BUILD_DIRS = [ROOT / "build", ROOT / "build" / "Release", ROOT / "build" / "RelWithDebInfo"]  # single/multi-config


def find_tool(name, path=None, env=None):
    """A runtime program (freedyn2_run, freedyn2_road): path, the environment variable env, the
    installed package (fdgen/bin), the build directory of this repository, the PATH; None if not
    found."""
    candidates = [path, os.environ.get(env) if env else None, resources.PACKAGE / "bin" / (name + EXE),
                  *(d / (name + EXE) for d in BUILD_DIRS), shutil.which(name)]
    for c in candidates:
        if c and Path(c).is_file():
            return Path(c).resolve()
    return None


def find_runtime(path=None):
    rt = find_tool("freedyn2_run", path, "FREEDYN2_RUN")
    if rt is None:
        raise FileNotFoundError("freedyn2_run not found: build it (cmake) or pass --runtime / set FREEDYN2_RUN")
    return rt


def load(path, profile="v2"):
    """Validated internal model of a .yaml or legacy .fds file."""
    from .fds import load_fds
    from .model import load_yaml, validate

    path = Path(path)
    return load_fds(path, profile) if path.suffix.lower() == ".fds" else validate(load_yaml(path), path.parent)


def build(path, workdir, profile="v2"):
    """Import the model and write <workdir>/<model>.fdm (plus its plugin). Returns (fdm, model, seconds)."""
    from .model import write_fdm

    t0 = time.perf_counter()
    model = load(path, profile)
    fdm = Path(workdir) / f"{model['model']}.fdm"
    write_fdm(model, fdm)
    return fdm, model, time.perf_counter() - t0


def simulate(runtime, fdm, csv_path, options=(), env=None):
    """Run the runtime; returns (wall-clock seconds, stdout)."""
    t0 = time.perf_counter()
    r = subprocess.run([str(runtime), str(fdm), "-o", str(csv_path), *options], capture_output=True, text=True, env=env)
    seconds = time.perf_counter() - t0
    if r.returncode != 0:
        raise RuntimeError(f"{runtime.name} failed:\n{r.stdout}{r.stderr}")
    return seconds, r.stdout


def run(args):
    runtime = find_runtime(args.runtime)
    with tempfile.TemporaryDirectory() as tmp:
        workdir = Path(args.workdir or tmp)
        workdir.mkdir(parents=True, exist_ok=True)
        fdm, model, build_seconds = build(args.model, workdir, args.profile)
        csv_path = Path(args.output or f"{model['model']}.csv").resolve()
        sim_seconds, stdout = simulate(runtime, fdm, csv_path, args.options)
    print(stdout, end="")
    print(f"build {build_seconds:.3f} s (import, model file, plugin), simulation {sim_seconds:.3f} s")
    return 0


def view(args):
    from .view.server import View, export, serve

    try:
        runtime = find_runtime(args.runtime)  # also for freedyn2_road beside it (OpenCRG roads)
    except FileNotFoundError:
        if not (args.results or args.no_run):
            raise
        runtime = None
    v = View(args.model, args.results, run=not args.no_run, profile=args.profile, runtime=runtime,
             options=args.options)
    if args.export:
        print(f"wrote {export(v, args.export)}")
        return 0
    return serve(v, args.port, not args.no_browser)


def check(args):
    from .checks import check_model, check_results, report_text
    from .view.scene import read_results

    model = load(args.model, args.profile)
    results = None
    if args.results:
        results = read_results(args.results)
    elif args.run:
        with tempfile.TemporaryDirectory() as tmp:
            fdm, model, _ = build(args.model, tmp, args.profile)
            simulate(find_runtime(args.runtime), fdm, Path(tmp) / "r.csv", args.options)
            results = read_results(Path(tmp) / "r.csv")
    report = check_model(model)
    res = check_results(model, results) if results is not None else None
    print(report_text(report, res))
    entries = report["findings"] + (res["findings"] + res["references"] if res else report["references"])
    return 1 if any(e.get("severity") == "error" or e.get("passed") is False for e in entries) else 0


def _examples():
    """name -> path of the YAML examples that come with FreeDyn (in the package or the repository)."""
    base = resources.path("examples")
    return {p.relative_to(base).with_suffix("").as_posix(): p for p in sorted(base.rglob("*.yaml"))}


def examples(args):
    from .model import data_files

    found = _examples()
    if not args.name:
        for name, path in found.items():
            first = path.read_text(encoding="utf-8").splitlines()[0]
            print(f"{name:32s} {first.lstrip('# ') if first.startswith('#') else ''}"[:110])
        print("\nfreedyn2 examples NAME copies one into the current directory (-o DIR: there)")
        return 0
    if args.name not in found:
        raise ValueError(f"no example {args.name!r}; freedyn2 examples lists them")
    src = found[args.name]
    dest = Path(args.output or ".") / src.name
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        raise ValueError(f"{dest} exists")
    shutil.copyfile(src, dest)
    print(f"wrote {dest}")
    for rel in data_files(src):  # road, tire and spline files, at the same place relative to it
        target = dest.parent / rel
        if target.exists():
            print(f"kept {target}")
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src.parent / rel, target)
        print(f"wrote {target}")
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "compare":
        sys.path.insert(0, str(ROOT / "legacy"))
        try:
            import benchmark
        except ImportError:
            print("freedyn2 compare needs the legacy/ directory of the freedyn2 repository", file=sys.stderr)
            return 2
        return benchmark.main(argv[1:])
    ap = argparse.ArgumentParser(prog="freedyn2", description="FreeDyn 2.0: build and simulate a model")
    sub = ap.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="import, build and simulate a model (.yaml or .fds)")
    r.add_argument("model")
    r.add_argument("-o", "--output", help="result CSV (default <model>.csv)")
    r.add_argument("--profile", choices=("v2", "legacy"), default="v2",
                   help=".fds models: FreeDyn 2.0 methods (default) or the legacy algorithm")
    r.add_argument("--workdir", help="keep the model file and plugin here (default: temporary)")
    r.add_argument("--runtime", help="path of freedyn2_run")
    r.epilog = "Runtime options follow after --, e.g. freedyn2 run model.fds -- --step 1e-4 --end-time 2"
    v = sub.add_parser("view", help="show a model and its results in the web browser; reloads on every save")
    v.add_argument("model")
    v.add_argument("results", nargs="?", help="result CSV to show (default: simulate the model)")
    v.add_argument("--no-run", action="store_true", help="only the model at t = 0")
    v.add_argument("--export", metavar="HTML", help="write one self-contained HTML file and exit")
    v.add_argument("--port", type=int, default=8765, help="of the local server (0: any free port)")
    v.add_argument("--no-browser", action="store_true", help="do not open the browser")
    v.add_argument("--profile", choices=("v2", "legacy"), default="v2")
    v.add_argument("--runtime", help="path of freedyn2_run")
    sub.add_parser("mcp", help="MCP server for assistants on stdio (pip install mcp)").add_argument(
        "--workdir", help="keep runs here (default: temporary)")
    c = sub.add_parser("check", help="check a model (and its results, or --run it) for plausibility and references")
    c.add_argument("model")
    c.add_argument("results", nargs="?", help="result CSV to check")
    c.add_argument("--run", action="store_true", help="simulate the model and check the results")
    c.add_argument("--profile", choices=("v2", "legacy"), default="v2")
    c.add_argument("--runtime", help="path of freedyn2_run")
    e = sub.add_parser("examples", help="list the example models, or copy one (freedyn2 examples quarter_car)")
    e.add_argument("name", nargs="?")
    e.add_argument("-o", "--output", help="directory to copy it to (default: the current one)")
    sub.add_parser("compare", help="legacy FreeDyn against FreeDyn 2.0 (see freedyn2 compare -h)")
    k = argv.index("--") if "--" in argv else len(argv)
    args = ap.parse_args(argv[:k])
    args.options = argv[k + 1:]
    try:
        if args.command == "mcp":
            from .mcp_server import serve

            return serve(args.workdir)
        return {"view": view, "check": check, "examples": examples}.get(args.command, run)(args)
    except Exception as e:  # noqa: BLE001  (command line: message, not traceback)
        print(f"freedyn2: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
