"""FreeDyn 2.0 for assistants: an MCP server (freedyn2 mcp, stdio).

An assistant writes the model file (YAML), and through these tools validates and checks it,
simulates it, reads and compares results, and looks at it (viewer in the browser for the person,
screenshots for the assistant). The person checks; the assistant does the work.

Tools: guide, schema, validate, simulate, channels, results, compare, check, view, screenshot,
modes, locus, sweep, notes, note_image, answer_note.
Runs are kept for the session (run ids r1, r2, ...) so results can be compared after a change.

The tool logic is the class Session (plain Python, tested without MCP); serve() registers it
with the MCP SDK (mcp >= 2: MCPServer; mcp 1.x: FastMCP).
"""

import json
import os
import shutil
import tempfile
import threading
from pathlib import Path

import numpy as np

from . import cli, extensions, resources
from .checks import check_model, check_results, report_text
from .model import ModelError, load_schema
from .view import notes as viewer_notes
from .view.scene import read_results

MAX_TABLE_POINTS = 500
NOISE = 1e-10  # absolute differences below this (and below 1e-9 of the values) are round-off

CONVENTIONS = """\
# FreeDyn 2.0 models: how to write them

A model is a YAML file (JSON Schema: tool `schema`). Units are SI (m, kg, s, N, rad).
All geometry is given GLOBALLY AT t = 0: body positions (center of mass), points of forces,
joint locations, frames. Orientations are Euler parameters (unit quaternions)
[theta0, theta1, theta2, theta3], theta0 the scalar part:
  identity [1, 0, 0, 0];  z axis -> global y: [0.7071068, -0.7071068, 0, 0];
  z axis -> global x: [0.7071068, 0, 0.7071068, 0];  90 deg about z: [0.7071068, 0, 0, 0.7071068].
Bodies: mass, principal moments of inertia about the center of mass in the body frame (the
body's orientation gives the principal axes); a real body satisfies I1 + I2 >= I3.
Joints: a joint frame (location, orientation) fixed on body1 and body2 (or the ground);
revolute turns about its z axis, prismatic and cylindrical move along z; generic: list the fixed
directions (dx dy dz ax ay az). body2 defaults to the ground.
Forces: spring_damper between two points, bushing (6 directions), force / torque, torsion
spring, expressions of t, parameters and measures (expression_force), tire on a road.
Measures (displacement, distance, angle, rates, ...) are scalars used in expressions and
drivers; drivers prescribe a measure over time (motion).
Solver: HHT, end_time, step, output_step; alpha -0.1 (default) damps numerical noise; with
joints alpha must be < 0. solver.newton: fixed for real-time runs.
Shared points: `points` (hardpoints: [x, y, z], {mirror: name}, {from: name, offset},
{in: body or frame, local: [x, y, z]}) and `frames` (in, origin, z: [from, to] or a direction,
x, or orientation); their names stand for the numbers in point
fields (location, point, point1, point2, position) and frames in orientation fields.
Geometry for the viewer: `shapes` per body (box, cylinder, sphere, STL/OBJ mesh), `ground.shapes`.
References for checks: `checks` (total mass, values at times, measured curves over time or
over another channel, e.g. toe over wheel travel).

Result channels: <body>.R_x/R_y/R_z (center of mass), theta0..3, Rdot_*, omega_* (global),
omega_body_*, T (kinetic energy), H_* (angular momentum), Rddot_*, omegadot_*, omegadot_body_*
(accelerations from the solver, not difference quotients); `sensors` (a point on a body, a
frame fixed in it): <sensor>.acceleration_*, specific_force_* (a - g, as an accelerometer),
angular_velocity_*, angular_acceleration_* in the sensor frame; forces and joints: <name>.<output>
(e.g. spring.length/velocity/force, tire.force_x/y/z, tire.slip0/1, joint.dx.lambda per fixed
direction, expression_force.value); measures are not channels (they enter the forces).

Workflow: write the YAML -> validate (findings!) -> simulate -> check (plausibility and
references) -> view / screenshot -> change -> simulate again -> compare with the run before.
Report what you changed and what the checks say; the person decides.

The person answers in the viewer: notes to you (tool notes: text, time, selected element,
plotted channels, values they tuned with the sliders, a picture: note_image). Work on the open
notes, change the model file (tuned values: write them into the file, they are not there yet),
then answer_note with what you did. Model parameters (`parameters`) get sliders in the viewer:
put the values a person will want to vary there and use them: every number may be a formula
"=expression" of the parameters (stiffness: =k_susp, position: [0, 0, =h + 0.25], a parameter
=0.8 * k_front), force expressions use them by name.
"""


def _example():
    return resources.path("examples/double_pendulum.yaml").read_text(encoding="utf-8")


def _describe(defs, name):
    """Types of a schema definition and its keys with their descriptions."""
    d = defs[name]
    lines = [f"## {name}"]
    for key, spec in d.get("properties", {}).items():
        desc = spec.get("description", "")
        if "enum" in spec:
            desc = f"one of {', '.join(map(str, spec['enum']))}. {desc}"
        lines.append(f"- {key}: {desc}".rstrip())
    return "\n".join(lines)


class Run:
    def __init__(self, run_id, model_file, csv, log, seconds):
        self.id, self.model_file, self.csv, self.log, self.seconds = run_id, Path(model_file), Path(csv), log, seconds
        self._results = None

    @property
    def results(self):
        if self._results is None:
            self._results = read_results(self.csv)
        return self._results


class Session:
    """The tools; runs and viewers live as long as the session."""

    def __init__(self, workdir=None, runtime=None):
        self.workdir = Path(workdir or tempfile.mkdtemp(prefix="freedyn2_mcp_"))
        self.workdir.mkdir(parents=True, exist_ok=True)
        self._runtime = runtime
        self.runs = {}
        self.viewers = {}
        self._shots = 0

    def runtime(self):
        if self._runtime is None:
            self._runtime = cli.find_runtime()
        return self._runtime

    def run(self, run_id):
        if run_id not in self.runs:
            raise ValueError(f"unknown run {run_id!r}; runs: {', '.join(self.runs) or 'none yet (simulate first)'}")
        return self.runs[run_id]

    # ---- the tools ----
    def guide(self):
        defs = load_schema()["$defs"]
        parts = [CONVENTIONS] + [e.conventions for e in extensions.extensions() if e.conventions]
        parts += ["# Element keys (from the schema)"]
        parts += [_describe(defs, n) for n in ("body", "joint", "force", "measure", "shape", "solver") if n in defs]
        parts += ["# Example (examples/double_pendulum.yaml)", "```yaml", _example().rstrip(), "```"]
        return "\n\n".join(parts)

    def schema(self, section=""):
        s = load_schema()
        if not section:
            return json.dumps(s, indent=1)
        for where in (s.get("$defs", {}), s.get("properties", {})):
            if section in where:
                return json.dumps(where[section], indent=1)
        raise ValueError(f"no section {section!r}; top level: {', '.join(s['properties'])}; "
                         f"definitions: {', '.join(s.get('$defs', {}))}")

    def validate(self, model_file):
        try:
            model = cli.load(model_file)
        except (ModelError, OSError) as e:
            return f"INVALID: {e}"
        return "valid\n" + report_text(check_model(model))

    def simulate(self, model_file, end_time=None, step=None, options=()):
        options = list(options)
        if end_time is not None:
            options += ["--end-time", repr(float(end_time))]
        if step is not None:
            options += ["--step", repr(float(step))]
        run_id = f"r{len(self.runs) + 1}"
        d = self.workdir / run_id
        d.mkdir()
        fdm, model, build_s = cli.build(model_file, d, "v2")
        seconds, log = cli.simulate(self.runtime(), fdm, d / "result.csv", options)
        run = Run(run_id, model_file, d / "result.csv", log, seconds)
        self.runs[run_id] = run
        r = run.results
        res = check_results(model, r)
        head = (f"run {run_id}: {model['model']}, t = {r['t'][0]:g} .. {r['t'][-1]:g} s, {len(r['t'])} outputs, "
                f"{len(r) - 1} channels; build {build_s:.2f} s, simulation {seconds:.2f} s")
        return "\n".join([head, log.strip(), report_text(check_model(model), res)])

    def channels(self, run_id, filter=""):
        words = filter.lower().split()
        names = [n for n in self.run(run_id).results if n != "t" and all(w in n.lower() for w in words)]
        more = f"\n... {len(names) - 400} more (narrow the filter)" if len(names) > 400 else ""
        return f"{len(names)} channels\n" + "\n".join(names[:400]) + more

    def results(self, run_id, channels, t_start=None, t_end=None, points=50):
        r = self.run(run_id).results
        missing = [c for c in channels if c not in r]
        if missing:
            raise ValueError(f"unknown channels {missing}; see the tool channels")
        t = r["t"]
        mask = (t >= (t[0] if t_start is None else t_start)) & (t <= (t[-1] if t_end is None else t_end))
        idx = np.flatnonzero(mask)
        if not len(idx):
            raise ValueError("no outputs in this time range")
        lines = ["channel: min (at t), max (at t), mean, final"]
        for c in channels:
            v = r[c][idx]
            lines.append(f"{c}: {v.min():.6g} (t = {t[idx][v.argmin()]:.4g}), {v.max():.6g} (t = {t[idx][v.argmax()]:.4g}), "
                         f"{v.mean():.6g}, {v[-1]:.6g}")
        n = max(2, min(int(points), MAX_TABLE_POINTS))
        sel = idx[np.unique(np.linspace(0, len(idx) - 1, n).round().astype(int))]
        lines += ["", "t," + ",".join(channels)]
        lines += [",".join(f"{x:.6g}" for x in [t[k], *(r[c][k] for c in channels)]) for k in sel]
        return "\n".join(lines)

    def compare(self, run_a, run_b, channels=None, top=20):
        a, b = self.run(run_a).results, self.run(run_b).results
        names = channels or [n for n in a if n != "t" and n in b]
        rows = []
        for n in names:
            if n not in a or n not in b:
                rows.append((np.inf, f"{n}: only in {'run ' + run_a if n in a else 'run ' + run_b}"))
                continue
            vb = np.interp(a["t"], b["t"], b[n])
            d = np.abs(a[n] - vb)
            k = int(d.argmax())
            size = max(np.abs(a[n]).max(), np.abs(vb).max())
            if d[k] <= 1e-9 * size + NOISE:  # round-off: the same
                rows.append((0.0, ""))
                continue
            span = max(np.ptp(a[n]), np.ptp(vb), 1e-6 * size)
            rows.append((d[k] / span, f"{n}: max |difference| {d[k]:.4g} at t = {a['t'][k]:.4g} "
                                      f"({d[k] / span:.1%} of its range); {run_a} {a[n][k]:.6g}, {run_b} {vb[k]:.6g}"))
        rows.sort(key=lambda x: -x[0])
        same = sum(1 for x, _ in rows if x == 0)
        rows = [r for r in rows if r[0] > 0]
        out = [f"{run_a} against {run_b}: {len(names)} channels, {same} the same (up to round-off)" +
               ("; largest differences:" if rows else "")]
        return "\n".join(out + [text for _, text in rows[:int(top)]])

    def check(self, model_file, run_id=None):
        model = cli.load(model_file)
        res = check_results(model, self.run(run_id).results) if run_id else None
        return report_text(check_model(model), res)

    def view(self, model_file, run_id=None, open_browser=True):
        """The viewer for the person: simulates the model and reloads on every save, or shows a run."""
        from http.server import ThreadingHTTPServer

        from .view import server

        key = (str(Path(model_file).resolve()), run_id)
        if key in self.viewers:
            return f"viewer already open: {self.viewers[key][1]}"
        csv = self.run(run_id).csv if run_id else None
        v = server.View(model_file, csv, run=csv is None, runtime=self.runtime())
        v.rebuild()
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), server._handler(v))
        stop = threading.Event()
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        threading.Thread(target=v.watch, args=(stop,), daemon=True).start()
        url = f"http://127.0.0.1:{httpd.server_address[1]}/"
        self.viewers[key] = (v, url, httpd, stop)
        if open_browser:
            import webbrowser

            webbrowser.open(url)
        return f"viewer: {url} (reloads when the model file is saved)" + (f"\nerror: {v.error}" if v.error else "")

    def screenshot(self, model_file, run_id=None, time=None, channels=(), select=None, width=1200, height=800):
        """PNG of the viewer (needs the Python package playwright and a Chromium for it)."""
        try:
            from playwright.sync_api import sync_playwright
        except ImportError:
            raise RuntimeError("screenshots need playwright: pip install playwright && playwright install chromium") from None
        from .view import server

        csv = self.run(run_id).csv if run_id else None
        self._shots += 1
        page_file = self.workdir / f"shot_{self._shots}.html"
        server.export(server.View(model_file, csv, run=False, runtime=self._runtime_or_none()), page_file)
        launch = {"executable_path": os.environ["FREEDYN2_CHROMIUM"]} if os.environ.get("FREEDYN2_CHROMIUM") else {}
        with sync_playwright() as pw:
            browser = pw.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
                                         **launch)
            page = browser.new_page(viewport={"width": int(width), "height": int(height)})
            page.goto(page_file.as_uri())
            page.wait_for_function("window.fdViewer && window.fdViewer.model", timeout=60000)
            page.evaluate("""([time, channels, select]) => {
                const v = window.fdViewer;
                for (const c of channels) if (v.results && v.results.map.has(c)) v.addChannel(c);
                if (time !== null) v.setTime(time);
                if (select) v.select(select);
            }""", [time, list(channels), select])
            page.wait_for_timeout(600)
            png = page.screenshot()
            browser.close()
        page_file.unlink(missing_ok=True)
        return png

    def modes(self, model_file, run_id, time=None):
        """Eigenfrequencies and damping about the state of a run at time (default: its start)."""
        from .linearize import modes
        from .model import write_fdm

        model = cli.load(model_file)
        fdm = self.workdir / f"modes_{model['model']}.fdm"
        write_fdm(model, fdm)
        data = modes(model, fdm, self.run(run_id).csv, time)
        lines = [f"eigenmodes about t = {data['time']:g} s (meaningful in an equilibrium or a steady motion):"]
        for k, m in enumerate(data["modes"], 1):
            re, im = m["eigenvalue"]
            amp = {b: float(np.hypot(*np.array(c[:3]).T).max()) + float(np.hypot(*np.array(c[3:]).T).max())
                   for b, c in m["bodies"].items()}
            main = ", ".join(b for b, _ in sorted(amp.items(), key=lambda x: -x[1])[:3])
            kind = f"{m['frequency']:.4g} Hz, damping ratio {m['damping']:.3f}" if im > 1e-9 else \
                f"non-oscillating, eigenvalue {re:.4g} 1/s" + (" (UNSTABLE)" if re > 0 else "")
            lines.append(f"{k:3d}  {kind};  moves mostly {main}")
        return "\n".join(lines) if data["modes"] else lines[0] + " no eigenvalues other than 0"

    def locus(self, model_file, run_id, path, values, time=None, count=8):
        """Eigenfrequencies and damping for each value of one number of the model file (a path as
        the viewer's: parameters.k, forces.spring.stiffness), linearized about the state of a run."""
        from .linearize import modes
        from .model import load_yaml, validate, write_fdm
        from .view.server import apply_overrides

        run = self.run(run_id)
        work = self.workdir / "locus"
        work.mkdir(exist_ok=True)
        lines = []
        for v in values:
            model = validate(apply_overrides(load_yaml(model_file), {path: float(v)}), Path(model_file).parent)
            fdm = work / f"{model['model']}.fdm"
            write_fdm(model, fdm)
            data = modes(model, fdm, run.csv, time, consistent=True)
            osc = [m for m in data["modes"] if m["eigenvalue"][1] > 1e-9][:count]
            real = [m["eigenvalue"][0] for m in data["modes"] if m["eigenvalue"][1] <= 1e-9]
            text = "  ".join(f"{m['frequency']:.4g} Hz ({100 * m['damping']:.1f} %)" for m in osc)
            if any(r > 0 for r in real):
                text += "  UNSTABLE: real eigenvalue " + ", ".join(f"{r:.4g}" for r in real if r > 0)
            lines.append(f"{path} = {float(v):g}: {text}")
        head = (f"eigenfrequencies (damping ratio) about the state of {run_id} at t = {data['time']:g} s, "
                f"lowest {count} oscillating modes; the state is that of the run, not of each value:")
        return "\n".join([head] + lines)

    def sweep(self, model_file, path, values, channels, kpi="max", t_start=None, t_end=None, end_time=None):
        """Simulate the model with one number (path as the viewer's: parameters.k_susp,
        forces.spring.stiffness) at each of values, in parallel; each becomes a run (for results
        and compare). Returns a table of the key figure kpi of the channels over [t_start, t_end]."""
        from .sweep import KPIS, kpi as key_figure, run_sweep

        if kpi not in KPIS:
            raise ValueError(f"kpi must be one of {', '.join(KPIS)}")
        options = ["--end-time", repr(float(end_time))] if end_time is not None else []
        first = len(self.runs) + 1
        work = self.workdir / f"sweep_r{first}"
        results = run_sweep(model_file, path, values, self.runtime(), work, options=options)
        lines = [f"{kpi} over t = {t_start if t_start is not None else 'start'} .. {t_end if t_end is not None else 'end'} s, "
                 f"{path} over {len(values)} values (runs as listed):", ",".join(["run", path, *channels])]
        for v, res in zip(values, results):
            if not isinstance(res, Path):
                lines.append(f"-,{float(v):g},FAILED: {res}")
                continue
            run_id = f"r{len(self.runs) + 1}"
            self.runs[run_id] = run = Run(run_id, model_file, res, "", 0.0)
            r = run.results
            missing = [c for c in channels if c not in r]
            if missing:
                raise ValueError(f"unknown channels {', '.join(missing)} (tool channels lists them)")
            row = [f"{key_figure(r['t'], r[c], kpi, t_start, t_end):.6g}" for c in channels]
            lines.append(",".join([run_id, f"{float(v):g}", *row]))
        return "\n".join(lines)

    def notes(self, model_file, everything=False):
        ns = viewer_notes.read_notes(model_file)
        shown = ns if everything else [n for n in ns if n["status"] == "open"]
        if not shown:
            return "no open notes" if ns else "no notes (the person writes them in the viewer: tool view)"
        return "\n\n".join(viewer_notes.note_text(n, model_file) for n in shown)

    def note_image(self, model_file, note_id):
        for n in viewer_notes.read_notes(model_file):
            if n["id"] == int(note_id):
                if not n.get("image"):
                    raise ValueError(f"note {note_id} has no picture")
                return (viewer_notes.notes_dir(model_file) / n["image"]).read_bytes()
        raise ValueError(f"no note {note_id}")

    def answer_note(self, model_file, note_id, answer, done=True):
        n = viewer_notes.answer_note(model_file, note_id, answer, done)
        return f"note {n['id']} answered ({n['status']}); the person sees the answer in the viewer"

    def _runtime_or_none(self):
        try:
            return self.runtime()
        except FileNotFoundError:
            return None

    def close(self):
        for v, _, httpd, stop in self.viewers.values():
            stop.set()
            httpd.shutdown()
            shutil.rmtree(v.workdir, ignore_errors=True)
        self.viewers.clear()


def serve(workdir=None):
    """Run the MCP server on stdio."""
    try:
        from mcp.server.mcpserver import Image, MCPServer
    except ImportError:
        try:
            from mcp.server.fastmcp import FastMCP as MCPServer, Image
        except ImportError:
            raise SystemExit("freedyn2 mcp needs the MCP SDK: pip install mcp") from None
    session = Session(workdir)
    mcp = MCPServer("freedyn2", instructions=(
        "FreeDyn 2.0 multibody simulation. Call guide first; write models as YAML files; validate, simulate, "
        "check and compare runs; open the viewer for the person (view) or look yourself (screenshot)."))

    @mcp.tool()
    def guide() -> str:
        """How to write a FreeDyn 2.0 model: conventions, element keys, an example, the workflow. Read first."""
        return session.guide()

    @mcp.tool()
    def schema(section: str = "") -> str:
        """The JSON Schema of the model file, or one section of it (e.g. force, joint, road, solver, checks)."""
        return session.schema(section)

    @mcp.tool()
    def validate(model_file: str) -> str:
        """Validate a model file (YAML or legacy .fds) and check it: degrees of freedom, mass, findings."""
        return session.validate(model_file)

    @mcp.tool()
    def simulate(model_file: str, end_time: float | None = None, step: float | None = None,
                 options: list[str] | None = None) -> str:
        """Build and simulate a model; returns a run id (keep it), the solver log and the checks of the results.
        options: further freedyn2_run options, e.g. ["--newton-fixed", "2"]."""
        return session.simulate(model_file, end_time, step, options or [])

    @mcp.tool()
    def channels(run_id: str, filter: str = "") -> str:
        """Names of the result channels of a run; filter: words that must all occur, e.g. "tire_fl force"."""
        return session.channels(run_id, filter)

    @mcp.tool()
    def results(run_id: str, channels: list[str], t_start: float | None = None, t_end: float | None = None,
                points: int = 50) -> str:
        """Statistics (min, max with times, mean, final) and a table of the channels, evenly thinned to points rows."""
        return session.results(run_id, channels, t_start, t_end, points)

    @mcp.tool()
    def compare(run_a: str, run_b: str, channels: list[str] | None = None, top: int = 20) -> str:
        """Differences between two runs (e.g. before and after a change), the largest first."""
        return session.compare(run_a, run_b, channels, top)

    @mcp.tool()
    def check(model_file: str, run_id: str | None = None) -> str:
        """Plausibility checks of the model and, with a run, of its results and the model's references (checks section)."""
        return session.check(model_file, run_id)

    @mcp.tool()
    def view(model_file: str, run_id: str | None = None) -> str:
        """Open the viewer in the person's browser: 3D model, animation, plots, checks; reloads when the file is saved."""
        return session.view(model_file, run_id)

    @mcp.tool()
    def screenshot(model_file: str, run_id: str | None = None, time: float | None = None,
                   channels: list[str] | None = None, select: str | None = None) -> Image:
        """A picture of the viewer, to look at the model yourself: at a time, with channels plotted, an element selected."""
        return Image(data=session.screenshot(model_file, run_id, time, channels or [], select), format="png")

    @mcp.tool()
    def modes(model_file: str, run_id: str, time: float | None = None) -> str:
        """Eigenfrequencies, damping ratios and the bodies that move in each mode, linearized about a run's state."""
        return session.modes(model_file, run_id, time)

    @mcp.tool()
    def locus(model_file: str, run_id: str, path: str, values: list[float], time: float | None = None) -> str:
        """Root locus as a table: eigenfrequencies and damping ratios for each value of one number of the
        model file (path like parameters.k_susp or forces.spring.stiffness), about the state of a run."""
        return session.locus(model_file, run_id, path, values, time)

    @mcp.tool()
    def sweep(model_file: str, path: str, values: list[float], channels: list[str], kpi: str = "max",
              t_start: float | None = None, t_end: float | None = None, end_time: float | None = None) -> str:
        """Variant study: simulate the model with one number (path like parameters.k_susp or
        forces.spring.stiffness) at each value, in parallel; a table of a key figure (max, min, max |x|,
        peak-peak, mean, rms, final) of the channels over [t_start, t_end] per value. Each value is a run."""
        return session.sweep(model_file, path, values, channels, kpi, t_start, t_end, end_time)

    @mcp.tool()
    def notes(model_file: str, everything: bool = False) -> str:
        """What the person wrote in the viewer about this model (open notes; everything: also the done ones),
        with time, selection, plotted channels and the values they tuned. Read them and act on them."""
        return session.notes(model_file, everything)

    @mcp.tool()
    def note_image(model_file: str, note_id: int) -> Image:
        """The view (3D and plots) the person saw when writing a note."""
        return Image(data=session.note_image(model_file, note_id), format="png")

    @mcp.tool()
    def answer_note(model_file: str, note_id: int, answer: str, done: bool = True) -> str:
        """Answer a note (shown in the viewer) and mark it done (done=False: keep it open, e.g. with a question)."""
        return session.answer_note(model_file, note_id, answer, done)

    try:
        mcp.run("stdio")
    finally:
        session.close()
