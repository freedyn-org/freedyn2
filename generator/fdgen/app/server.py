"""The server of the app: the start page, and a viewer per opened model in one process.

  /                     the start page (web/start.html)
  /app/...              its files (web/), the icon; /app/brand/...: the brand assets of the
                        viewer (fdgen/view/web/brand: tokens, fonts, logos)
  /api/app/...          the start page's API (below)
  /m/<id>/...           the viewer of opened model <id>: fdgen.view.server, unchanged, under
                        this prefix (its page uses relative URLs); the page gets the app shell
                        (web/shell.js: the update notice, errors of the app)

GET  /api/app/status    {title, version, native, runtime, update, errors, docs, ...}
GET  /api/app/recent    the recently opened models (fdgen.app.recent)
GET  /api/app/examples  the examples that come with FreeDyn [{name, title}]
POST /api/app/open      {path}: open a model -> {id, url, window}
POST /api/app/upload    {name, text}: a model file sent by the browser, kept in data_dir()/uploads
POST /api/app/example   {name, folder}: copy an example (and its data files) there and open it
POST /api/app/browse    {kind: model | folder}: the native file dialog (app window only)
POST /api/app/recent    {remove: path} | {clear: true}
POST /api/app/errors    {clear: true}
POST /api/app/quit      end the app
POST /api/app/handoff   {paths}: from a second start (fdgen.app.instance; needs the token)

The server listens on 127.0.0.1 only; POSTs are taken from its own pages only (Origin and
Host), the handoff with the token of the instance file.
"""

import base64
import json
import logging
import re
import shutil
import sys
import threading
import time
import webbrowser
from http.server import ThreadingHTTPServer
from pathlib import Path

from .. import __version__, update, userdirs
from ..view import server as view_server
from . import instance, recent

WEB = Path(__file__).resolve().parent / "web"
ICON = Path(__file__).resolve().parent / "freedyn2.png"
DOCS = "https://freedyn-org.github.io/freedyn2/"
MODEL_SUFFIXES = (".yaml", ".yml", ".fds")
PREFERRED_PORT = 8767  # a fixed origin keeps the viewer's settings (local storage) between starts
IDLE_SECONDS = 180  # browser mode: no page asked for so long -> the app ends (tabs closed)
TITLE = "freedyn2"  # the version: start page, freedyn2 --version

log = logging.getLogger("freedyn2.app")


def documents_dir():
    """The user's Documents folder (Windows: the known folder, also when moved to OneDrive)."""
    if sys.platform == "win32":
        try:
            import ctypes
            from ctypes import wintypes

            buf = ctypes.create_unicode_buffer(wintypes.MAX_PATH)
            if ctypes.windll.shell32.SHGetFolderPathW(None, 5, None, 0, buf) == 0 and buf.value:  # CSIDL_PERSONAL
                return Path(buf.value)
        except Exception:  # noqa: BLE001
            pass
    docs = Path.home() / "Documents"
    return docs if docs.is_dir() or sys.platform == "win32" else Path.home()


def examples_dir():
    return documents_dir() / "freedyn2"


def list_examples():
    from ..cli import _examples

    out = []
    for name, path in _examples().items():
        try:
            first = path.read_text(encoding="utf-8").splitlines()[0]
        except (OSError, IndexError):
            first = ""
        out.append({"name": name, "file": path.name, "title": first.lstrip("# ").strip().rstrip(":") if first.startswith("#") else ""})
    return out


def copy_example(name, folder):
    """Copy example name and the data files it reads into folder (as freedyn2 examples does);
    files already there are kept. Returns (model path, [copied paths])."""
    from ..cli import _examples
    from ..model import data_files

    found = _examples()
    if name not in found:
        raise ValueError(f"no example {name!r}")
    src, folder = found[name], Path(folder).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    copied = []
    for rel in [Path(src.name)] + [Path(r) for r in data_files(src)]:
        target = folder / rel
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src.parent / rel, target)
            copied.append(str(target))
    return folder / src.name, copied


class App:
    """The opened models (a View each) and the windows; native is a fdgen.app.native.Native
    (pywebview) or None (the default web browser)."""

    def __init__(self, runtime=None, native=None, open_browser=True, idle_seconds=IDLE_SECONDS, profile="v2"):
        self.runtime, self.native, self.open_browser, self.profile = runtime, native, open_browser, profile
        self.idle_seconds = idle_seconds
        self.views, self._ids = {}, 0  # id -> {view, stop, path}
        self.errors, self.update = [], None
        self.token = instance.new_token()
        self.httpd, self.quit = None, threading.Event()
        self.last_seen = time.monotonic()
        self._lock = threading.Lock()

    # --- server -----------------------------------------------------------------------------
    def start(self, port=None, publish=True):
        """Listen (port None: PREFERRED_PORT, else any free one) and serve in a thread."""
        handler = _handler(self)
        for p in ([PREFERRED_PORT, 0] if port is None else [port]):
            try:
                self.httpd = ThreadingHTTPServer(("127.0.0.1", p), handler)
                break
            except OSError:
                if p == 0 or port is not None:
                    raise
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, name="freedyn2-http", daemon=True).start()
        if publish:
            instance.publish(self.port, self.token)
        log.info("serving %s", self.url())
        return self

    @property
    def port(self):
        return self.httpd.server_address[1]

    def url(self, vid=None):
        return f"http://127.0.0.1:{self.port}/" + (f"m/{vid}/" if vid is not None else "")

    def shutdown(self):
        self.quit.set()
        for vid in list(self.views):
            self.close_model(vid)
        if self.httpd:
            instance.withdraw(self.token)
            self.httpd.shutdown()
            self.httpd.server_close()

    def check_updates(self):
        def found(r):
            self.update = r
            log.info("freedyn2 %s is available", r["latest"])

        return update.check_in_background(found)

    def error(self, message):
        """An error of the app: logged, shown on the pages (and in a dialog of the app window)."""
        log.error("%s", message)
        with self._lock:
            self.errors = (self.errors + [{"time": time.strftime("%H:%M:%S"), "message": str(message)}])[-20:]

    # --- models -----------------------------------------------------------------------------
    def open_model(self, path):
        """A View of the model file (simulated in the background); its id."""
        path = Path(path).expanduser()
        if path.suffix.lower() not in MODEL_SUFFIXES:
            raise ValueError(f"{path.name}: not a model file (.yaml or .fds)")
        if not path.is_file():
            raise ValueError(f"{path}: no such file")
        path = path.resolve()
        view = view_server.View(path, run=self.runtime is not None, profile=self.profile, runtime=self.runtime)
        stop = threading.Event()
        with self._lock:
            self._ids += 1
            vid = self._ids
            self.views[vid] = {"view": view, "stop": stop, "path": path}

        def work():  # first build, then rebuild on every save (as freedyn2 view)
            view.rebuild()
            view.watch(stop)

        threading.Thread(target=work, name=f"freedyn2-view-{vid}", daemon=True).start()
        try:
            recent.add(path)
        except OSError as e:
            log.warning("recent models: %s", e)
        log.info("opened %s as /m/%d/", path, vid)
        return vid

    def close_model(self, vid):
        entry = self.views.pop(vid, None)
        if entry:
            entry["stop"].set()
            entry["view"].stop_sweep()
            shutil.rmtree(entry["view"].workdir, ignore_errors=True)
            log.info("closed %s", entry["path"])

    def show(self, vid=None):
        """A window (app) or a browser tab with the model vid, or the start page. True if a
        native window was opened."""
        title = TITLE if vid is None else f"{self.views[vid]['path'].name} - {TITLE}"
        if self.native is not None:
            self.native.window(self.url(vid), title, on_closed=None if vid is None else lambda: self.close_model(vid))
            return True
        if self.open_browser:
            webbrowser.open(self.url(vid))
        return False

    def handoff(self, paths):
        """Models from a second start: a window each; none: the start page."""
        if not paths:
            self.show()
        for p in paths:
            try:
                self.show(self.open_model(p))
            except Exception as e:  # noqa: BLE001  (a bad file from a double click)
                self.error(f"cannot open {p}: {e}")
                self.show()

    def run_browser(self):
        """Browser mode: wait for quit; also end when no page asked for idle_seconds."""
        while not self.quit.wait(5):
            if self.idle_seconds and time.monotonic() - self.last_seen > self.idle_seconds:
                log.info("no page open for %d s: ending", self.idle_seconds)
                break

    # --- the API of the start page ----------------------------------------------------------
    def status(self):
        with self._lock:
            errors = list(self.errors)
        return {"title": TITLE, "version": __version__, "native": self.native is not None,
                "runtime": self.runtime is not None, "update": self.update, "errors": errors, "docs": DOCS,
                "releases": update.RELEASE_PAGE, "update_check": update.enabled(),
                "examples_dir": str(examples_dir()),
                "open": [{"id": k, "path": str(v["path"]), "url": f"/m/{k}/"} for k, v in self.views.items()]}

    def api_get(self, url):
        if url == "/api/app/status":
            return self.status()
        if url == "/api/app/recent":
            return recent.items()
        if url == "/api/app/examples":
            return list_examples()
        raise KeyError(url)

    def _opened(self, path):
        vid = self.open_model(path)
        return {"id": vid, "url": f"/m/{vid}/", "path": str(self.views[vid]["path"]), "window": self.show(vid) if self.native else False}

    def api_post(self, url, body):
        if url == "/api/app/open":
            return self._opened(str(body["path"]).strip().strip('"'))
        if url == "/api/app/upload":
            name = Path(str(body["name"])).name
            if Path(name).suffix.lower() not in MODEL_SUFFIXES:
                raise ValueError(f"{name}: not a model file (.yaml or .fds)")
            folder = userdirs.data_dir() / "uploads" / time.strftime("%Y%m%d-%H%M%S")
            folder.mkdir(parents=True, exist_ok=True)
            data = base64.b64decode(body["base64"]) if "base64" in body else str(body["text"]).encode("utf-8")
            (folder / name).write_bytes(data)
            return self._opened(folder / name)
        if url == "/api/app/example":
            path, copied = copy_example(str(body["name"]), body.get("folder") or examples_dir())
            return {**self._opened(path), "copied": copied}
        if url == "/api/app/browse":
            if self.native is None:
                raise ValueError("file dialogs need the app window (pip install pywebview)")
            if body.get("kind") == "folder":
                return {"paths": self.native.choose_folder(str(body.get("start") or examples_dir()))}
            return {"paths": self.native.choose_model()}
        if url == "/api/app/recent":
            if body.get("clear"):
                recent.clear()
            elif body.get("remove"):
                recent.remove(str(body["remove"]))
            return recent.items()
        if url == "/api/app/errors":
            with self._lock:
                self.errors = []
            return {}
        if url == "/api/app/quit":
            threading.Timer(0.2, self.quit.set).start()
            if self.native is not None:
                threading.Timer(0.2, self.native.close_all).start()
            return {}
        raise KeyError(url)


_VIEW = re.compile(r"^/m/(\d+)(/.*)?$")


def _handler(app):
    class Handler(view_server.ViewHandler):
        def _page(self):
            """The viewer's page with the app shell: title, shell.js."""
            html = (view_server.WEB / "index.html").read_text(encoding="utf-8")
            name = self._entry["path"].name if self._entry else "freedyn2"
            html = html.replace("<title>freedyn2 viewer</title>", f"<title>{_esc(name)} - {TITLE}</title>", 1)
            html = html.replace("</body>", '<script src="/app/shell.js"></script>\n</body>', 1)
            self._send(html.encode("utf-8"), "text/html; charset=utf-8")

        def _json(self, data, code=200):
            self._send(json.dumps(data).encode(), "application/json", code)

        def _local(self):
            host = self.headers.get("Host", "")
            return host in (f"127.0.0.1:{app.port}", f"localhost:{app.port}")

        def _route(self):
            app.last_seen = time.monotonic()
            m = _VIEW.match(self.path.split("?", 1)[0])
            if not m:
                return None, None
            entry = app.views.get(int(m.group(1)))
            return entry, (m.group(2) or "")

        def do_GET(self):
            if not self._local():
                return self._send(b"refused", "text/plain", 403)  # (DNS rebinding)
            entry, rest = self._route()
            url = self.path.split("?", 1)[0]
            if entry is not None or rest is not None:
                if entry is None:
                    return self._send(b"this model is closed", "text/plain", 404)
                if rest == "":
                    self.send_response(301)
                    self.send_header("Location", url + "/")
                    self.end_headers()
                    return None
                self._entry = entry
                return self.view_get(entry["view"], rest + self.path[len(url):])
            if url in ("/", "/index.html"):
                return self._file(WEB / "start.html")
            if url == "/app/freedyn2.png":
                return self._file(ICON)
            if url.startswith("/app/brand/"):
                brand = view_server.WEB / "brand"
                path = (brand / url[len("/app/brand/"):]).resolve()
                return self._file(path) if brand in path.parents else self._send(b"not found", "text/plain", 404)
            if url.startswith("/app/"):
                path = (WEB / url[len("/app/"):]).resolve()
                return self._file(path) if WEB in path.parents else self._send(b"not found", "text/plain", 404)
            if url.startswith("/api/app/"):
                try:
                    return self._json(app.api_get(url))
                except KeyError:
                    pass
            return self._send(b"not found", "text/plain", 404)

        def do_POST(self):
            if not self._local():
                return self._send(b"refused", "text/plain", 403)
            entry, rest = self._route()
            url = self.path.split("?", 1)[0]
            if entry is not None or rest is not None:
                if entry is None:
                    return self._send(b"this model is closed", "text/plain", 404)
                return self.view_post(entry["view"], rest or "/")
            n = int(self.headers.get("Content-Length") or 0)
            if url == "/api/app/handoff":
                if self.headers.get(instance.HEADER) != app.token:
                    return self._send(b"refused", "text/plain", 403)
                body = json.loads(self.rfile.read(n) or b"{}")
                threading.Thread(target=app.handoff, args=([str(p) for p in body.get("paths") or []],), daemon=True).start()
                return self._json({})
            if n > view_server.MAX_POST or self.headers.get("Origin", "").split("//")[-1] not in ("", self.headers.get("Host")):
                return self._send(b"refused", "text/plain", 403)
            try:
                return self._json(app.api_post(url, json.loads(self.rfile.read(n) or b"{}")))
            except KeyError as e:
                if e.args and e.args[0] == url:
                    return self._send(b"not found", "text/plain", 404)
                return self._json({"error": f"missing {e}"}, 400)
            except (ValueError, TypeError, OSError) as e:
                return self._json({"error": str(e)}, 400)
            except Exception as e:  # noqa: BLE001  (an error of the app: logged, shown)
                log.exception("POST %s", url)
                return self._json({"error": f"{type(e).__name__}: {e}"}, 500)

    Handler._entry = None
    return Handler


def _esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
