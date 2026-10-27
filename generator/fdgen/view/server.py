"""freedyn2 view: serve the viewer, rebuild and re-simulate when the model file changes.

  freedyn2 view model.yaml                  simulate, show; again after every save of the model
  freedyn2 view model.yaml result.csv       show these results (reloaded when the file changes)
  freedyn2 view model.yaml --no-run         the model at t = 0
  freedyn2 view ... --export page.html      one self-contained HTML file instead of a server

The server listens on localhost only. The page polls /api/state; a new version means: fetch
/api/scene and /api/results.csv again (the camera and the time stay). Every simulation is kept
as a run (/api/runs/<id>.csv, the last MAX_RUNS and the reference run), so the page compares
runs; while a simulation runs, /api/running.csv is its CSV so far and the state has its
progress. The scene carries the checks of the model and its results (fdgen.checks) and the
YAML source of each element.

From the page (POST, JSON):
  /api/overrides  {path: value}: numbers of the model file tuned in the viewer (apply_overrides),
                  kept by the server (not written to the file) and applied to every rebuild;
                  simulates in the background, a simulation still running is stopped
  /api/runs/<id>  {reference: true | false} / {delete: true}
  /api/sweep      {path, values}: one tuned number over several values, simulated in parallel
                  (fdgen.sweep; the fan of curves, key figures over the value) / {stop: true}
                  / {clear: true}; its results: GET /api/sweep/<k>.csv
  /api/notes      {text, context, image}: a note to the assistant (fdgen.view.notes)
  /api/notes/<id> {done} / {delete}: close, reopen or delete a note
GET /api/modes?t=T: eigenmodes about the results at time T (fdgen.linearize.modes).
POST /api/modes {t, overrides}: the same for other tuned values, about the state of the current
run (a preview while a slider moves); POST /api/locus {t, path, values}: the eigenvalues for
each value of one tuned number (root locus), likewise about the state of the current run.
"""

import base64
import json
import mimetypes
import shutil
import subprocess
import tempfile
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from ..checks import check_model, check_references, check_results
from . import notes
from .scene import attach_sources, read_results, scene

WEB = Path(__file__).resolve().parent / "web"
POLL_SECONDS = 0.5
MAX_POST = 20_000_000  # bytes (a note with its picture)
MAX_RUNS = 20  # kept for comparison (the reference always)


def apply_overrides(raw, overrides):
    """Set numbers in the YAML data of a model: path "section.name.key[.index]" (e.g.
    forces.spring.stiffness, bodies.rod.position.2, parameters.k) -> value. A missing last key
    is added (a default made explicit); everything before it must exist."""
    for path, value in overrides.items():
        keys = path.split(".")
        node = raw
        for k in keys[:-1]:
            node = node[int(k)] if isinstance(node, list) else node.get(k) if isinstance(node, dict) else None
            if node is None:
                raise ValueError(f"tuned value {path}: no {k} in the model file")
        last = keys[-1]
        if isinstance(node, list):
            node[int(last)] = float(value)
        elif isinstance(node, dict):
            node[last] = float(value)
        else:
            raise ValueError(f"tuned value {path}: not a number in the model file")
    return raw


class Cancelled(Exception):
    """A rebuild overtaken by a newer request (new tuned values, a save of the model)."""


class View:
    """Model, results and scene of the viewer; rebuilt when one of the watched files changes or
    tuned values arrive. Every simulation is a run (self.runs): the page compares runs with each
    other and with a reference run. A new request cancels the rebuild in progress."""

    def __init__(self, model_path, results_path=None, run=True, profile="v2", runtime=None, options=()):
        self.model_path = Path(model_path).resolve()
        self.results_path = Path(results_path).resolve() if results_path else None
        self.run, self.profile, self.runtime, self.options = run and not results_path, profile, runtime, list(options)
        self.workdir = Path(tempfile.mkdtemp(prefix="freedyn2_view_"))
        self.version, self.status, self.error, self.log = 0, "loading", "", ""
        self.scene, self.csv, self.meshes = None, None, []
        self.overrides, self.model, self.fdm, self._modes = {}, None, None, {}
        self.runs, self.current, self.reference = [], None, None  # runs: [{id, csv, overrides, edit, seconds, created}]
        self._edits, self._file_stamp = 0, None  # versions of the model file the runs were made with
        self._progress = None  # (csv, end time) of the simulation in progress
        self.sweep, self._sweep_csv, self._sweeps = None, [], 0  # the parameter sweep (fan) and its results
        self._generation = 0  # raised by every request; a rebuild of an older one stops
        self._stamps = {}
        self._lock = threading.Lock()
        self._building = threading.Lock()  # one rebuild at a time
        self._symbolic = threading.Lock()  # CasADi model building: one thread at a time

    def road_tool(self):
        if self.runtime is None:
            return None
        tool = self.runtime.with_name("freedyn2_road" + self.runtime.suffix)
        return tool if tool.is_file() else None

    def _watched(self):
        files = [self.model_path] + ([self.results_path] if self.results_path else []) + self.meshes + self._data()
        return {f: f.stat().st_mtime_ns if f.exists() else None for f in files}

    def _data(self):
        """The data files the model reads (roads, splines, .tir), found again when it changes."""
        from ..model import data_files

        stamp = self.model_path.stat().st_mtime_ns if self.model_path.exists() else None
        if getattr(self, "_data_stamp", ()) != stamp:
            self._data_stamp = stamp
            self._data_paths = [self.model_path.parent / p for p in data_files(self.model_path)]
        return self._data_paths

    def load_model(self):
        """The validated model with the tuned values (overrides)."""
        from .. import cli
        from ..model import load_yaml, validate

        if not self.overrides:
            return cli.load(self.model_path, self.profile)
        if self.model_path.suffix.lower() not in (".yaml", ".yml"):
            raise ValueError("tuning values needs a YAML model file")
        return validate(apply_overrides(load_yaml(self.model_path), self.overrides), self.model_path.parent)

    def set_overrides(self, overrides, wait=True):
        """New tuned values; simulate with them (wait=False: in the background, the page polls)."""
        with self._lock:
            self.overrides = {str(k): float(v) for k, v in overrides.items()}
        self.request(wait)

    def modes(self, time):
        """Eigenmodes about the results at time (fdgen.linearize.modes), kept per version."""
        from ..linearize import modes
        from ..model import write_fdm

        with self._lock:
            model, fdm, csv_path, version = self.model, self.fdm, self.csv, self.version
        if model is None or csv_path is None:
            raise ValueError("eigenmodes need results")
        key = (version, round(float(time), 9))
        if key not in self._modes:
            if fdm is None:  # results given: the model file of these results
                fdm = self.workdir / f"{model['model']}.fdm"
                write_fdm(model, fdm)
            self._modes = {key: modes(model, fdm, csv_path, time)}
        return self._modes[key]

    def preview_modes(self, overrides, time, count=24):
        """Eigenmodes (fdgen.linearize.modes) of the model with other tuned values, about the
        positions and velocities of the current run at time, with the constraint forces of the
        changed model there: a preview, before simulating with them (the motion of the new
        values differs by what the change moves, e.g. a softer spring sags further)."""
        from ..linearize import modes
        from ..model import load_yaml, validate, write_fdm

        with self._lock:
            csv_path = self.csv
        if csv_path is None:
            raise ValueError("eigenmodes need results")
        if self.model_path.suffix.lower() not in (".yaml", ".yml"):
            raise ValueError("tuning values needs a YAML model file")
        work = self.workdir / "preview"
        work.mkdir(exist_ok=True)
        with self._symbolic:
            model = validate(apply_overrides(load_yaml(self.model_path), overrides), self.model_path.parent)
            fdm = work / f"{model['model']}.fdm"
            write_fdm(model, fdm)
            return modes(model, fdm, csv_path, time, count, consistent=True)

    def locus(self, path, values, time):
        """Eigenvalues for each value of one tuned number (the others as tuned now), about the
        state of the current run: {path, values, time, eigenvalues: [[[re, im], ...] per value]}."""
        out = []
        for v in values:
            data = self.preview_modes({**self.overrides, path: float(v)}, time, count=1000)
            out.append([m["eigenvalue"] for m in data["modes"]])
        return {"path": path, "values": [float(v) for v in values], "time": data["time"], "eigenvalues": out}

    def start_sweep(self, path, values):
        """Simulate the model with the tuned number path set to each of values, in the
        background, the other tuned values as they are; a sweep still running is stopped."""
        from ..sweep import Cancelled, run_sweep

        if self.runtime is None:
            raise ValueError("a sweep needs freedyn2_run")
        values = [float(v) for v in values]
        if not 2 <= len(values) <= 25:
            raise ValueError("a sweep needs 2 to 25 values")
        with self._lock:
            self._sweeps += 1
            sid = self._sweeps
            self.sweep = {"id": sid, "path": str(path), "values": values, "results": [None] * len(values),
                          "status": "running", "error": "", "overrides": dict(self.overrides)}
            self._sweep_csv = [None] * len(values)
            overrides = dict(self.overrides)
        workdir = self.workdir / f"sweep_{sid}"

        def done(k, result):
            with self._lock:
                if self.sweep and self.sweep["id"] == sid:
                    self.sweep["results"][k] = "ok" if isinstance(result, Path) else result
                    self._sweep_csv[k] = result if isinstance(result, Path) else None

        def work():
            try:
                run_sweep(self.model_path, path, values, self.runtime, workdir, overrides, self.options,
                          cancelled=lambda: self._sweeps != sid, done=done, lock=self._symbolic)
                status, error = "done", ""
            except Cancelled:
                return
            except Exception as e:  # noqa: BLE001  (shown on the page)
                status, error = "error", str(e)
            with self._lock:
                if self.sweep and self.sweep["id"] == sid:
                    self.sweep["status"], self.sweep["error"] = status, error

        threading.Thread(target=work, daemon=True).start()
        return self.sweep

    def stop_sweep(self, clear=False):
        with self._lock:
            self._sweeps += 1  # the running one sees it and stops
            if self.sweep and self.sweep["status"] == "running":
                self.sweep["status"] = "stopped"
            if clear:
                self.sweep, self._sweep_csv = None, []

    def sweep_csv(self, k):
        with self._lock:
            return self._sweep_csv[k] if 0 <= k < len(self._sweep_csv) else None

    def changed(self):
        return self._watched() != self._stamps

    def request(self, wait=True):
        """Rebuild (and simulate) the current model; a rebuild still running for an older
        request stops (its simulation is killed)."""
        with self._lock:
            self._generation += 1
            gen = self._generation
            self.status = "simulating" if self.run else "loading"
        if wait:
            self._build(gen)
        else:
            threading.Thread(target=self._build, args=(gen,), daemon=True).start()

    def rebuild(self):
        """Load, (simulate,) and make the scene; errors go to the page."""
        self.request(wait=True)

    def _build(self, gen):
        with self._building:
            if gen != self._generation:
                return  # overtaken while waiting
            try:
                self._rebuild(gen)
            except Cancelled:
                pass  # the newer request builds

    def _simulate(self, fdm, csv_path, gen):
        """freedyn2_run as cli.simulate, but stopped when a newer request arrives."""
        t0 = time.perf_counter()
        proc = subprocess.Popen([str(self.runtime), str(fdm), "-o", str(csv_path), *self.options],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
        while True:
            try:
                out, _ = proc.communicate(timeout=0.05)
                break
            except subprocess.TimeoutExpired:
                if gen != self._generation:
                    proc.kill()
                    proc.communicate()
                    raise Cancelled() from None
        if proc.returncode != 0:
            raise RuntimeError(f"{Path(self.runtime).name} failed:\n{out}")
        return time.perf_counter() - t0, out

    def _end_time(self, model):
        opts = self.options
        for k, o in enumerate(opts[:-1]):
            if o == "--end-time":
                return float(opts[k + 1])
        return float(model["solver"].get("end_time", 0.0))

    def _new_run(self, csv_path, seconds):
        """A finished simulation (or a given result file) as a run; the oldest runs go (not the reference)."""
        stamp = self._watched().get(self.model_path)
        if stamp != self._file_stamp:
            self._edits, self._file_stamp = self._edits + 1, stamp
        rid = 1 + max([r["id"] for r in self.runs] or [0])
        path = self.workdir / f"run_{rid}.csv"
        for attempt in range(20):  # Windows: not while the page reads the running file
            try:
                shutil.move(str(csv_path), path)
                break
            except PermissionError:
                if attempt == 19:
                    raise
                time.sleep(0.05)
        self.runs.append({"id": rid, "csv": path, "overrides": dict(self.overrides), "edit": self._edits,
                          "seconds": seconds, "created": time.strftime("%H:%M:%S")})
        while len(self.runs) > MAX_RUNS:
            old = next(r for r in self.runs if r["id"] != self.reference)
            self.runs.remove(old)
            old["csv"].unlink(missing_ok=True)
        return rid, path

    def run_csv(self, rid):
        return next((r["csv"] for r in self.runs if r["id"] == rid), None)

    def set_reference(self, rid):
        with self._lock:
            if rid is not None and self.run_csv(rid) is None:
                raise ValueError(f"no run {rid}")
            self.reference = rid

    def delete_run(self, rid):
        with self._lock:
            run = next((r for r in self.runs if r["id"] == rid), None)
            if run is None or rid == self.current:
                raise ValueError(f"run {rid}: " + ("the current run stays" if run else "no such run"))
            self.runs.remove(run)
            run["csv"].unlink(missing_ok=True)
            if self.reference == rid:
                self.reference = None

    def partial(self):
        """The CSV of the simulation in progress, up to its last complete row (b"" if none)."""
        prog = self._progress
        if prog is None or not prog[0].is_file():
            return b""
        data = prog[0].read_bytes()
        return data[:data.rfind(b"\n") + 1]

    def progress(self):
        """{t, end} of the simulation in progress, or None."""
        prog = self._progress
        if prog is None or not prog[0].is_file():
            return None
        with open(prog[0], "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 65536))
            tail = f.read().rsplit(b"\n", 2)
        try:
            t = float(tail[-2].split(b",", 1)[0]) if len(tail) >= 2 else 0.0
        except ValueError:
            t = 0.0
        return {"t": t, "end": prog[1]}

    def _rebuild(self, gen):
        from ..model import write_fdm

        self._stamps = self._watched()
        with self._lock:
            self.status = "simulating" if self.run else "loading"
            self.version += 1
        try:
            with self._symbolic:
                model = self.load_model()
            csv_path, log, fdm, seconds = None, "", None, 0.0
            if self.run:
                fdm = self.workdir / f"{model['model']}.fdm"
                with self._symbolic:
                    write_fdm(model, fdm)  # (compiles the plugin of a changed model: not stopped)
                if gen != self._generation:
                    raise Cancelled()
                csv_path = self.workdir / "running.csv"
                self._progress = (csv_path, self._end_time(model))
                try:
                    seconds, log = self._simulate(fdm, csv_path, gen)
                finally:
                    self._progress = None
                log += f"simulation {seconds:.2f} s\n"
            elif self.results_path:
                csv_path = self.workdir / "given.csv"
                shutil.copyfile(self.results_path, csv_path)
            results = read_results(csv_path) if csv_path else None
            s = scene(model, results, self.road_tool())
            s["checks"] = check_model(model)
            if results is not None:
                s["checks"]["findings"] += check_results(model, results)["findings"]
                s["checks"]["references"] = check_references(model, results)
            if self.model_path.suffix.lower() in (".yaml", ".yml"):
                attach_sources(s, self.model_path)
            meshes = []
            for b in s["bodies"] + [{"shapes": s["ground_shapes"]}]:
                for shape in b["shapes"]:
                    if shape["type"] == "mesh":
                        shape["url"] = f"api/mesh/{len(meshes)}{Path(shape['file']).suffix.lower()}"
                        meshes.append(Path(shape["file"]))
            with self._lock:
                if csv_path is not None:
                    self.current, csv_path = self._new_run(csv_path, seconds)
                self.scene, self.csv, self.meshes, self.model, self.fdm = s, csv_path, meshes, model, fdm
                self.status, self.error, self.log = "ready", "", log
                self.version += 1
        except Cancelled:
            raise
        except Exception as e:  # noqa: BLE001  (shown over the last good scene; the server keeps watching)
            with self._lock:
                self.status, self.error = "error", f"{e}"
                self.log = traceback.format_exc() if not isinstance(e, (ValueError, RuntimeError, OSError)) else ""
                self.version += 1
        self._stamps = self._watched()

    def state(self):
        with self._lock:
            runs = [{k: v for k, v in r.items() if k != "csv"} for r in self.runs]
            return {"version": self.version, "status": self.status, "error": self.error, "log": self.log,
                    "model_file": str(self.model_path), "scene": self.scene is not None, "results": self.csv is not None,
                    "runs": runs, "current": self.current, "reference": self.reference,
                    "progress": self.progress() if self.status == "simulating" else None,
                    "overrides": self.overrides, "notes": notes.stamp(self.model_path),
                    "sweep": None if self.sweep is None else {**self.sweep, "results": list(self.sweep["results"])}}

    def watch(self, stop):
        while not stop.is_set():
            if self.changed():
                time.sleep(0.1)  # let the editor finish writing
                self._stamps = self._watched()
                self.request(wait=False)
            stop.wait(POLL_SECONDS)


def _handler(view):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _file(self, path):
            if not path.is_file():
                return self._send(b"not found", "text/plain", 404)
            ctype = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            if path.suffix == ".js":
                ctype = "text/javascript"
            self._send(path.read_bytes(), ctype)

        def do_GET(self):
            url = self.path.split("?", 1)[0]
            if url == "/":
                return self._file(WEB / "index.html")
            if url == "/api/state":
                return self._send(json.dumps(view.state()).encode(), "application/json")
            if url == "/api/scene":
                return self._send(json.dumps(view.scene).encode(), "application/json")
            if url == "/api/results.csv":
                return self._file(view.csv) if view.csv else self._send(b"", "text/csv")
            if url == "/api/notes":
                return self._send(json.dumps(notes.read_notes(view.model_path)).encode(), "application/json")
            if url.startswith("/api/notes/") and url.endswith(".png"):
                return self._file(notes.notes_dir(view.model_path) / Path(url).name)
            if url == "/api/modes":
                from urllib.parse import parse_qs

                query = parse_qs(self.path.partition("?")[2])
                try:
                    data = view.modes(float(query.get("t", ["0"])[0]))
                except Exception as e:  # noqa: BLE001  (shown on the page)
                    return self._send(json.dumps({"error": str(e)}).encode(), "application/json", 400)
                return self._send(json.dumps(data).encode(), "application/json")
            if url == "/api/running.csv":
                return self._send(view.partial(), "text/csv")
            if url.startswith("/api/sweep/") and url.endswith(".csv"):
                k = url[len("/api/sweep/"):-4]
                path = view.sweep_csv(int(k)) if k.isdigit() else None
                return self._file(path) if path else self._send(b"not found", "text/plain", 404)
            if url.startswith("/api/runs/") and url.endswith(".csv"):
                k = url[len("/api/runs/"):-4]
                path = view.run_csv(int(k)) if k.isdigit() else None
                return self._file(path) if path else self._send(b"not found", "text/plain", 404)
            if url.startswith("/api/mesh/"):
                k = url.rsplit("/", 1)[1].split(".")[0]
                meshes = view.meshes
                return self._file(meshes[int(k)]) if k.isdigit() and int(k) < len(meshes) else self._send(b"", "text/plain", 404)
            path = (WEB / url.lstrip("/")).resolve()
            if WEB not in path.parents:
                return self._send(b"not found", "text/plain", 404)
            return self._file(path)

        def do_POST(self):
            url = self.path.split("?", 1)[0]
            n = int(self.headers.get("Content-Length") or 0)
            if n > MAX_POST or self.headers.get("Origin", "").split("//")[-1] not in ("", self.headers.get("Host")):
                return self._send(b"refused", "text/plain", 403)  # only the page of this server
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
                if url == "/api/overrides":
                    view.set_overrides(body, wait=False)
                    out = view.state()
                elif url == "/api/modes":
                    out = view.preview_modes({str(k): float(v) for k, v in (body.get("overrides") or {}).items()},
                                             float(body.get("t", 0.0)))
                elif url == "/api/sweep":
                    if body.get("stop") or body.get("clear"):
                        view.stop_sweep(clear=bool(body.get("clear")))
                    else:
                        view.start_sweep(str(body["path"]), body["values"])
                    out = view.state()
                elif url == "/api/locus":
                    values = [float(v) for v in body["values"]][:41]
                    out = view.locus(str(body["path"]), values, float(body.get("t", 0.0)))
                elif url.startswith("/api/runs/"):
                    rid = int(url.rsplit("/", 1)[1])
                    if body.get("delete"):
                        view.delete_run(rid)
                    else:
                        view.set_reference(rid if body.get("reference", True) else None)
                    out = view.state()
                elif url == "/api/notes":
                    out = notes.add_note(view.model_path, body.get("text", ""), body.get("context"), body.get("image"))
                elif url.startswith("/api/notes/"):
                    nid = int(url.rsplit("/", 1)[1])
                    if body.get("delete"):
                        notes.delete_note(view.model_path, nid)
                        out = {}
                    else:
                        out = notes.answer_note(view.model_path, nid, done=bool(body.get("done", True)))
                else:
                    return self._send(b"not found", "text/plain", 404)
            except (ValueError, KeyError, TypeError, RuntimeError, np.linalg.LinAlgError) as e:
                return self._send(json.dumps({"error": str(e)}).encode(), "application/json", 400)
            return self._send(json.dumps(out).encode(), "application/json")

    return Handler


def serve(view, port=8765, open_browser=True):
    view.rebuild()
    server = ThreadingHTTPServer(("127.0.0.1", port), _handler(view))
    stop = threading.Event()
    threading.Thread(target=view.watch, args=(stop,), daemon=True).start()
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"freedyn2 view: {url}  (watching {view.model_path.name}; Ctrl+C stops)", flush=True)
    if view.error:
        print(f"freedyn2 view: {view.error}", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        shutil.rmtree(view.workdir, ignore_errors=True)
    return 0


def export(view, path):
    """One HTML file with the viewer, three.js, the scene, the results and the meshes."""
    view.rebuild()
    if view.error:
        raise RuntimeError(view.error)
    html = (WEB / "index.html").read_text(encoding="utf-8")
    data = {"scene": view.scene, "csv": Path(view.csv).read_text(encoding="utf-8") if view.csv else "",
            "meshes": {f"api/mesh/{k}{m.suffix.lower()}": base64.b64encode(m.read_bytes()).decode()
                       for k, m in enumerate(view.meshes)}}
    modules = {name: (WEB / rel).read_text(encoding="utf-8") for name, rel in
               (("three", "vendor/three.module.min.js"), ("three/addons/OrbitControls.js", "vendor/OrbitControls.js"),
                ("three/addons/STLLoader.js", "vendor/STLLoader.js"), ("three/addons/OBJLoader.js", "vendor/OBJLoader.js"),
                ("fdview/channels.js", "channels.js"), ("fdview/plot.js", "plot.js"), ("fdview/panels.js", "panels.js"), ("fdview/runs.js", "runs.js"), ("./viewer.js", "viewer.js"))}
    payload = json.dumps(data).replace("</", "<\\/")
    blobs = json.dumps(modules).replace("</", "<\\/")
    boot = ("<script>window.FD_STATIC = " + payload + ";</script>\n"
            "<script>(function(){const m = " + blobs + ";const map = {};"
            "for (const k in m) map[k] = URL.createObjectURL(new Blob([m[k]], {type: 'text/javascript'}));"
            "const s = document.createElement('script'); s.type = 'importmap';"
            "s.textContent = JSON.stringify({imports: map}); document.currentScript.after(s);"
            "const v = document.createElement('script'); v.type = 'module'; v.src = map['./viewer.js'];"
            "document.body.appendChild(v);})();</script>")
    start, end = html.index("<!-- modules -->"), html.index("<!-- /modules -->") + len("<!-- /modules -->")
    Path(path).write_text(html[:start] + boot + html[end:], encoding="utf-8")
    shutil.rmtree(view.workdir, ignore_errors=True)
    return Path(path)
