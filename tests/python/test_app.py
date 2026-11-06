"""The app (fdgen.app): the start page in browser mode, models opened under /m/<id>/, the recent
models, one app per user (handoff), the update notice (fdgen.update, network mocked)."""

import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from fdgen import update, userdirs
from fdgen.app import instance, native, recent, server

ROOT = Path(__file__).resolve().parents[2]
NO_PROXY = urllib.request.build_opener(urllib.request.ProxyHandler({}))
REAL_FETCH = update.fetch


@pytest.fixture(autouse=True)
def home(tmp_path, monkeypatch):
    """Per-user files of the test in tmp_path/home; no update check."""
    monkeypatch.setenv("FREEDYN2_HOME", str(tmp_path / "home"))
    monkeypatch.delenv("FREEDYN2_NO_UPDATE_CHECK", raising=False)
    monkeypatch.setattr(update, "fetch", lambda timeout=None: pytest.fail("no network in tests"))
    return tmp_path / "home"


@pytest.fixture
def app():
    a = server.App(runtime=None, native=None, open_browser=False, idle_seconds=None).start(port=0, publish=False)
    yield a
    a.shutdown()


def request(url, body=None, headers=None):
    """(status, parsed JSON or text) of a GET, or a POST of body."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json", **(headers or {})})
    try:
        with NO_PROXY.open(req, timeout=20) as r:
            text, code, ctype = r.read(), r.status, r.headers.get("Content-Type", "")
    except urllib.error.HTTPError as e:
        text, code, ctype = e.read(), e.code, e.headers.get("Content-Type", "")
    return code, (json.loads(text) if "json" in ctype else text)


def wait_ready(a, vid, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        code, state = request(a.url(vid) + "api/state")
        if state["status"] in ("ready", "error"):
            return state
        time.sleep(0.1)
    raise AssertionError("the model did not load")


# --- the start page ---------------------------------------------------------------------------

def test_start_page_in_browser_mode(app):
    code, page = request(app.url())
    assert code == 200 and b"<title>freedyn2</title>" in page and b"/app/start.js" in page
    assert request(app.url() + "app/start.js")[0] == 200
    with NO_PROXY.open(app.url() + "app/freedyn2.png") as r:
        assert r.headers["Content-Type"] == "image/png" and r.read()[:4] == b"\x89PNG"
    for brand in ("view/brand/tokens.css", "view/fonts.css", "view/brand/lockup-small-light.svg",
                  "view/vendor/fonts/jetbrains-mono-latin-400-normal.woff2"):  # the brand of the viewer
        assert request(app.url() + brand)[0] == 200, brand
    assert request(app.url() + "view/index.html")[0] == 404  # only style sheets, images, fonts
    code, s = request(app.url() + "api/app/status")
    assert code == 200 and s["native"] is False and s["runtime"] is False and s["title"] == server.TITLE
    assert s["docs"] == "https://freedyn-org.github.io/freedyn2/" and s["update"] is None
    names = [e["name"] for e in request(app.url() + "api/app/examples")[1]]
    assert "double_pendulum" in names and "quarter_car" in names
    assert request(app.url() + "app/../server.py")[0] == 404
    code, err = request(app.url() + "api/app/browse", {"kind": "model"})
    assert code == 400 and "app window" in err["error"]  # file dialogs only with pywebview


def test_open_an_example_and_its_viewer(app, tmp_path):
    folder = tmp_path / "Documents" / "FreeDyn2"
    code, r = request(app.url() + "api/app/example", {"name": "double_pendulum", "folder": str(folder)})
    assert code == 200, r
    assert r["url"] == "/m/1/" and r["window"] is False and Path(r["path"]) == folder / "double_pendulum.yaml"
    state = wait_ready(app, 1)
    assert state["status"] == "ready" and state["model_file"] == str(folder / "double_pendulum.yaml")
    code, page = request(app.url(1))
    assert b'<script src="/app/shell.js"></script>' in page and b"double_pendulum.yaml - freedyn2" in page
    assert b"viewer.js" in page  # the viewer's own page
    code, scene = request(app.url(1) + "api/scene")
    assert code == 200 and len(scene["bodies"]) == 2
    assert request(app.url(1) + "vendor/three.module.min.js")[0] == 200
    # the viewer's POSTs under the prefix
    code, notes = request(app.url(1) + "api/notes", {"text": "a note"})
    assert code == 200
    with NO_PROXY.open(urllib.request.Request(app.url().rstrip("/") + "/m/1")) as r:
        assert r.url.endswith("/m/1/")  # redirected: the page's relative URLs need the slash
    assert request(app.url() + "m/7/api/state")[0] == 404
    # again: the copy is kept (not overwritten), a second view of it opens
    code, r2 = request(app.url() + "api/app/example", {"name": "double_pendulum", "folder": str(folder)})
    assert r2["copied"] == [] and r2["id"] == 2
    assert [Path(e["path"]) for e in request(app.url() + "api/app/recent")[1]] == [folder / "double_pendulum.yaml"]
    code, s = request(app.url() + "api/app/status")
    assert [o["id"] for o in s["open"]] == [1, 2]
    app.close_model(1)
    assert request(app.url(1) + "api/state")[0] == 404


def test_example_with_data_files(tmp_path):
    path, copied = server.copy_example("car_on_crg", tmp_path / "ex")
    assert path == tmp_path / "ex" / "car_on_crg.yaml" and path.is_file()
    assert (tmp_path / "ex" / "data" / "test_track.crg").is_file() and len(copied) >= 2
    with pytest.raises(ValueError, match="no example"):
        server.copy_example("nope", tmp_path)


def test_open_by_path_and_upload(app, tmp_path):
    model = (ROOT / "examples" / "double_pendulum.yaml").read_text(encoding="utf-8")
    (tmp_path / "m.yaml").write_text(model, encoding="utf-8")
    code, r = request(app.url() + "api/app/open", {"path": f'"{tmp_path / "m.yaml"}"'})  # quotes of "copy as path"
    assert code == 200 and r["id"] == 1
    code, err = request(app.url() + "api/app/open", {"path": str(tmp_path / "missing.yaml")})
    assert code == 400 and "no such file" in err["error"]
    code, err = request(app.url() + "api/app/open", {"path": str(tmp_path)})
    assert code == 400
    code, r = request(app.url() + "api/app/upload", {"name": "up.yaml", "text": model})
    assert code == 200 and Path(r["path"]).read_text(encoding="utf-8") == model
    assert userdirs.data_dir() in Path(r["path"]).parents
    assert request(app.url() + "api/app/upload", {"name": "x.exe", "text": "MZ"})[0] == 400
    assert wait_ready(app, 2)["status"] == "ready"


def test_only_its_own_pages(app):
    """POSTs from another origin, and requests for another host name (DNS rebinding), are refused."""
    code, _ = request(app.url() + "api/app/open", {"path": "x.yaml"}, {"Origin": "http://evil.example"})
    assert code == 403
    assert request(app.url() + "api/app/status", headers={"Host": "evil.example"})[0] == 403
    assert request(app.url() + "api/app/handoff", {"paths": []})[0] == 403  # no token


def test_quit(app):
    assert request(app.url() + "api/app/quit", {})[0] == 200
    assert app.quit.wait(5)


# --- recent models ----------------------------------------------------------------------------

def test_recent_list(tmp_path, home):
    assert recent.items() == []
    files = []
    for k in range(recent.MAX + 3):
        f = tmp_path / f"m{k}.yaml"
        f.write_text("model: m\n")
        files.append(f)
        recent.add(f)
    recent.add(files[0])  # again: to the top, once
    items = recent.items()
    assert len(items) == recent.MAX and Path(items[0]["path"]) == files[0].resolve()
    assert [i["path"] for i in items].count(str(files[0].resolve())) == 1
    assert json.loads((home / "recent.json").read_text())[0]["path"] == str(files[0].resolve())
    files[0].unlink()
    assert recent.items()[0]["exists"] is False and recent.items()[1]["exists"] is True
    recent.remove(files[0].resolve())
    assert Path(recent.items()[0]["path"]) == files[-1].resolve()
    recent.clear()
    assert recent.items() == []
    (home / "recent.json").write_text("{not json")
    assert recent.items() == []
    recent.add(files[1])
    assert len(recent.items()) == 1


def test_recent_api(app, tmp_path):
    f = tmp_path / "a.fds"
    f.write_text("")
    recent.add(f)
    code, items = request(app.url() + "api/app/recent")
    assert code == 200 and items[0]["name"] == "a.fds" and items[0]["exists"]
    assert request(app.url() + "api/app/recent", {"remove": str(f.resolve())})[1] == []
    recent.add(f)
    assert request(app.url() + "api/app/recent", {"clear": True})[1] == []


# --- one app per user -------------------------------------------------------------------------

def test_single_instance_handoff(tmp_path, home):
    assert instance.hand_off([]) is False  # nobody running: start
    a = server.App(runtime=None, open_browser=False, idle_seconds=None).start(port=0)
    try:
        data = instance.read()
        assert data["port"] == a.port and data["token"] == a.token
        model = tmp_path / "dp.yaml"
        model.write_text((ROOT / "examples" / "double_pendulum.yaml").read_text(encoding="utf-8"), encoding="utf-8")
        shown = []
        a.show = lambda vid=None: shown.append(vid)
        assert instance.hand_off([model]) is True
        t0 = time.time()
        while not shown and time.time() - t0 < 10:
            time.sleep(0.05)
        assert shown == [1] and a.views[1]["path"] == model.resolve()
        assert instance.hand_off([]) is True
        t0 = time.time()
        while len(shown) < 2 and time.time() - t0 < 10:
            time.sleep(0.05)
        assert shown[1] is None  # the start page
        assert instance.hand_off([tmp_path / "missing.yaml"]) is True  # taken; the error is shown by the app
        t0 = time.time()
        while not a.errors and time.time() - t0 < 10:
            time.sleep(0.05)
        assert "missing.yaml" in a.errors[0]["message"]
        bad = {"Content-Type": "application/json", instance.HEADER: "wrong"}
        assert request(a.url() + "api/app/handoff", {"paths": []}, bad)[0] == 403
    finally:
        a.shutdown()
    assert instance.read() is None  # withdrawn
    instance.publish(1, "stale")  # a crashed app: nobody answers on its port
    assert instance.hand_off([]) is False


# --- the update notice ------------------------------------------------------------------------

@pytest.mark.parametrize("parse", ["packaging", "simple"])
def test_version_comparison(parse, monkeypatch):
    if parse == "simple":
        monkeypatch.setattr(update, "parse_version", update._simple_key)
    newer = update.is_newer
    assert newer("0.2.0", "0.1.0") and newer("v0.10.0", "0.9.1") and newer("1.0", "1.0rc2")
    assert newer("1.0.1", "1.0") and newer("1.0rc2", "1.0rc1") and newer("1.0a1", "1.0.dev3")
    assert newer("2.0.post1", "2.0")
    assert not newer("0.1.0", "0.1.0") and not newer("1.0", "1.0.0") and not newer("0.9", "0.10")
    assert not newer("1.0rc1", "1.0") and not newer("nightly", "0.1.0") and newer("0.2", "0+unknown")


def test_update_check_once_a_day(monkeypatch, home):
    calls = []

    def fetch(timeout=None):
        calls.append(timeout)
        return {"latest": "99.0.0", "url": "https://github.com/freedyn-org/freedyn2/releases/tag/v99.0.0"}

    monkeypatch.setattr(update, "fetch", fetch)
    r = update.check()
    assert r["newer"] and r["latest"] == "99.0.0" and r["current"] == update.__version__ and len(calls) == 1
    assert abs(userdirs.get("last_update_check") - time.time()) < 60
    r2 = update.check()  # today again: not asked, the release found before
    assert len(calls) == 1 and r2 == r
    assert update.check(force=True) and len(calls) == 2
    userdirs.set("last_update_check", time.time() - update.INTERVAL - 1)  # a day later
    update.check()
    assert len(calls) == 3


def test_update_check_is_silent_on_errors(monkeypatch):
    def fetch(timeout=None):
        raise urllib.error.URLError("offline")

    monkeypatch.setattr(update, "fetch", fetch)
    assert update.check() is None and "offline" in update.last_error
    assert userdirs.get("last_update_check")  # not again today
    assert update.check() is None


def test_update_check_can_be_turned_off(monkeypatch):
    for env in ("1", "true", "yes"):
        monkeypatch.setenv("FREEDYN2_NO_UPDATE_CHECK", env)
        assert not update.enabled() and update.check() is None and update.check_in_background(print) is None
    monkeypatch.setenv("FREEDYN2_NO_UPDATE_CHECK", "0")
    assert update.enabled()
    userdirs.set("update_check", False)
    assert not update.enabled() and update.check() is None  # (fetch would fail the test)
    userdirs.set("update_check", True)
    assert update.enabled()


def test_update_in_the_background_and_on_the_page(monkeypatch, app):
    monkeypatch.setattr(update, "fetch", lambda timeout=None: {"latest": "99.0", "url": update.RELEASE_PAGE})
    t = app.check_updates()
    t.join(10)
    s = request(app.url() + "api/app/status")[1]
    assert s["update"]["newer"] and s["update"]["latest"] == "99.0" and s["update_check"]
    monkeypatch.setattr(update, "fetch", lambda timeout=None: {"latest": "0.0.1", "url": update.RELEASE_PAGE})
    found = []
    update.check_in_background(found.append, force=True).join(10)
    assert found == []  # not newer: nothing to show


def test_fetch_reads_the_github_answer(monkeypatch):
    class Answer:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read(self):
            return json.dumps({"tag_name": "v2.3.4", "html_url": "https://github.com/x/releases/tag/v2.3.4"}).encode()

    seen = {}

    def urlopen(req, timeout):
        seen["url"], seen["timeout"], seen["ua"] = req.full_url, timeout, req.get_header("User-agent")
        return Answer()

    monkeypatch.setattr(urllib.request, "urlopen", urlopen)
    assert REAL_FETCH() == {"latest": "2.3.4", "url": "https://github.com/x/releases/tag/v2.3.4"}
    assert seen["url"] == update.RELEASES_API and seen["timeout"] <= 10 and seen["ua"].startswith("freedyn2/")


def test_cli_update_check(monkeypatch, capsys):
    from fdgen import cli

    monkeypatch.setattr(update, "fetch", lambda timeout=None: {"latest": "99.0", "url": update.RELEASE_PAGE})
    assert cli.main(["update-check"]) == 0
    assert "freedyn2 version 99.0 is available" in capsys.readouterr().out
    monkeypatch.setattr(update, "fetch", lambda timeout=None: {"latest": "0.0.1", "url": update.RELEASE_PAGE})
    assert cli.main(["update-check"]) == 0
    assert "is up to date" in capsys.readouterr().out


# --- packaging: entry points, icon ------------------------------------------------------------

def test_entry_points_and_icon():
    tomllib = pytest.importorskip("tomllib")
    project = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))["project"]
    assert project["gui-scripts"]["freedyn2-gui"] == "fdgen.app:main"
    assert any(d.startswith("pywebview") for d in project["optional-dependencies"]["gui"])
    import struct

    app_dir = ROOT / "generator" / "fdgen" / "app"
    ico = (app_dir / "freedyn2.ico").read_bytes()  # ICONDIR, then one 16-byte entry per image
    reserved, kind, count = struct.unpack_from("<HHH", ico)
    assert (reserved, kind) == (0, 1)
    sizes = {(ico[6 + 16 * i] or 256, ico[7 + 16 * i] or 256) for i in range(count)}  # 0 means 256
    assert {(16, 16), (32, 32), (48, 48), (256, 256)} <= sizes
    png = (app_dir / "freedyn2.png").read_bytes()  # signature, then IHDR: width, height
    assert png[:8] == b"\x89PNG\r\n\x1a\n" and struct.unpack(">II", png[16:24]) == (256, 256)
    assert (app_dir / "freedyn2.svg").read_text(encoding="utf-8").startswith("<svg")


def test_no_window_without_a_display(monkeypatch):
    monkeypatch.setattr(native.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    assert native.available() == "no display"  # -> the browser


def test_the_app_in_browser_mode(monkeypatch, home, tmp_path):
    """freedyn2-gui --browser: serves, opens the browser, takes a second start's model, ends on Quit."""
    import logging
    import sys
    import threading

    import fdgen.app as fdapp

    opened = []
    monkeypatch.setattr(server.webbrowser, "open", opened.append)
    monkeypatch.setenv("FREEDYN2_NO_UPDATE_CHECK", "1")
    monkeypatch.setattr(sys, "excepthook", sys.excepthook)
    monkeypatch.setattr(threading, "excepthook", threading.excepthook)
    out = {}
    t = threading.Thread(target=lambda: out.setdefault("status", fdapp.main(["--browser", "--port", "0"])))
    t.start()
    try:
        t0 = time.time()
        while not (instance.read() and opened) and time.time() - t0 < 20:
            time.sleep(0.05)
        port = instance.read()["port"]
        assert opened == [f"http://127.0.0.1:{port}/"]  # the start page
        model = tmp_path / "dp.yaml"
        model.write_text((ROOT / "examples" / "double_pendulum.yaml").read_text(encoding="utf-8"), encoding="utf-8")
        assert fdapp.main([str(model)]) == 0  # a second start: handed over, done
        t0 = time.time()
        while len(opened) < 2 and time.time() - t0 < 20:
            time.sleep(0.05)
        assert opened[1] == f"http://127.0.0.1:{port}/m/1/"
        assert request(f"http://127.0.0.1:{port}/api/app/quit", {})[0] == 200
        t.join(20)
        assert out["status"] == 0 and instance.read() is None
        log = (userdirs.log_dir() / "freedyn2.log").read_text(encoding="utf-8")
        assert "starting" in log and "handed 1 model(s)" in log and "ended" in log
    finally:
        if t.is_alive():
            request(f"http://127.0.0.1:{instance.read()['port']}/api/app/quit", {})
            t.join(20)
        root = logging.getLogger("freedyn2")
        for h in [h for h in root.handlers if getattr(h, "_freedyn2", False)]:
            root.removeHandler(h)
            h.close()


class FakeWebview:
    """Enough of pywebview for fdgen.app.native: windows, their closed event, file dialogs."""

    class FileDialog:
        OPEN, FOLDER, SAVE = 10, 20, 30

    class Event(list):
        def __iadd__(self, f):
            self.append(f)
            return self

    class Window:
        def __init__(self, title, url, **kw):
            self.title, self.url, self.kw, self.dialogs = title, url, kw, []
            self.events = type("Events", (), {})()
            self.events.closed = FakeWebview.Event()

        def create_file_dialog(self, kind, **kw):
            self.dialogs.append((kind, kw))
            return ("/models/a.yaml",) if kind == FakeWebview.FileDialog.OPEN else ("/models",)

        def destroy(self):
            for f in self.events.closed:
                f()

    def __init__(self):
        self.settings, self.windows, self.started = {}, [], None

    def create_window(self, title, url, **kw):
        w = self.Window(title, url, **kw)
        self.windows.append(w)
        return w

    def start(self, func=None, **kw):
        self.started = kw


def test_app_windows_with_pywebview(monkeypatch, tmp_path):
    """The app window path, with a stand-in for pywebview (no display in the tests)."""
    fake = FakeWebview()
    monkeypatch.setitem(sys.modules, "webview", fake)
    nat = native.Native(icon=Path("icon.png"))
    a = server.App(runtime=None, native=nat, idle_seconds=None).start(port=0, publish=False)
    try:
        a.show()
        assert fake.windows[0].title == server.TITLE and fake.windows[0].url == a.url()
        assert fake.settings["OPEN_EXTERNAL_LINKS_IN_BROWSER"] is True
        code, r = request(a.url() + "api/app/example", {"name": "double_pendulum", "folder": str(tmp_path)})
        assert r["window"] is True and fake.windows[1].url == a.url(1)  # a window of its own
        assert fake.windows[1].title == f"double_pendulum.yaml - {server.TITLE}"
        assert request(a.url() + "api/app/browse", {"kind": "model"})[1] == {"paths": ["/models/a.yaml"]}
        assert request(a.url() + "api/app/browse", {"kind": "folder"})[1] == {"paths": ["/models"]}
        assert "yaml" in fake.windows[1].dialogs[0][1]["file_types"][0]
        assert request(a.url() + "api/app/status")[1]["native"] is True
        fake.windows[1].destroy()  # closing the model's window closes the model
        assert 1 not in a.views and nat.windows == [fake.windows[0]]
        nat.run()
        assert fake.started["icon"] == "icon.png" and fake.started["private_mode"] is False
    finally:
        a.shutdown()
