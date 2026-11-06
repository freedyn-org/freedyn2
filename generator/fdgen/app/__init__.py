"""FreeDyn 2 as a desktop program: freedyn2-gui [model ...] (also freedyn2 gui).

With pywebview (pip install freedyn2[gui]) the start page and every opened model get a window
of their own ("FreeDyn 2"); without it they open in the default web browser. The
process ends with the last window (browser: when no page has been open for a few minutes, or
with Quit on the start page). One app per user: a second start (a double click on a model
file) hands its models to the running one (fdgen.app.instance). The log goes to
userdirs.log_dir()/freedyn2.log. At start the app asks GitHub, at most once a day, whether
there is a newer release (fdgen.update) and shows a notice.

The pieces: fdgen.app.server (the start page, the viewers), fdgen.app.native (pywebview),
fdgen.app.recent (recently opened models), fdgen.app.instance (one app per user).
"""

import argparse
import os
import logging
import logging.handlers
import sys
import threading
import traceback
from pathlib import Path

from .. import __version__, userdirs

ICON_ICO = Path(__file__).resolve().parent / "freedyn2.ico"
ICON_PNG = Path(__file__).resolve().parent / "freedyn2.png"
log = logging.getLogger("freedyn2")


class _LogWriter:
    """sys.stdout / sys.stderr of a program without console (pythonw, freedyn2-gui.exe)."""

    def __init__(self, level):
        self.level = level

    def write(self, text):
        if text.strip():
            log.log(self.level, "%s", text.rstrip())
        return len(text)

    def flush(self):
        pass

    def isatty(self):
        return False


def setup_logging():
    """The rotating log file in log_dir(); also stderr when there is a console. Returns its path."""
    path = userdirs.log_dir() / "freedyn2.log"
    root = logging.getLogger("freedyn2")
    if not any(getattr(h, "_freedyn2", False) for h in root.handlers):
        h = logging.handlers.RotatingFileHandler(path, maxBytes=1_000_000, backupCount=3, encoding="utf-8")
        h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
        h._freedyn2 = True
        root.addHandler(h)
        if sys.stderr is not None:
            c = logging.StreamHandler(sys.stderr)
            c.setFormatter(logging.Formatter("freedyn2 gui: %(message)s"))
            c.setLevel(logging.WARNING)
            c._freedyn2 = True
            root.addHandler(c)
        root.setLevel(logging.INFO)
    if sys.stdout is None:
        sys.stdout = _LogWriter(logging.INFO)
    if sys.stderr is None:
        sys.stderr = _LogWriter(logging.ERROR)
    return path


def message_box(title, text):
    """An error dialog without a window of the app (start failed): Windows MessageBox, else
    Tk if it is there; always the log."""
    log.error("%s: %s", title, text)
    if sys.platform == "win32":
        try:
            import ctypes

            ctypes.windll.user32.MessageBoxW(None, str(text), str(title), 0x10)
            return
        except Exception:  # noqa: BLE001
            pass
    try:
        import tkinter
        from tkinter import messagebox

        root = tkinter.Tk()
        root.withdraw()
        messagebox.showerror(title, text)
        root.destroy()
    except Exception:  # noqa: BLE001  (no display, no Tk: the log has it)
        pass


def _parser():
    ap = argparse.ArgumentParser(prog="freedyn2-gui", description=f"FreeDyn 2 (version {__version__}): the app")
    ap.add_argument("models", nargs="*", help="model files to open (.yaml, .fds); none: the start page")
    ap.add_argument("--browser", action="store_true", help="the default web browser instead of an app window")
    ap.add_argument("--port", type=int, help="of the local server (default 8767, else any free one; 0: any)")
    ap.add_argument("--new-instance", action="store_true", help="do not hand the models to a running app")
    ap.add_argument("--profile", choices=("v2", "legacy"), default="v2", help=".fds models: the methods")
    ap.add_argument("--runtime", help="path of freedyn2_run")
    return ap


def run(models=(), browser=False, port=None, new_instance=False, profile="v2", runtime=None):
    """The app; returns the exit status when the last window is closed."""
    from . import instance, native as native_mod
    from .server import App

    logfile = setup_logging()
    log.info("FreeDyn 2 version %s starting (%s)", __version__, sys.executable)
    if not new_instance and instance.hand_off(models):
        return 0
    from ..cli import find_tool

    rt = find_tool("freedyn2_run", runtime, "FREEDYN2_RUN")
    if rt is None:
        log.warning("freedyn2_run not found: models are shown at t = 0 only")
    why = "--browser" if browser else native_mod.available()
    native = None
    if why is None:
        try:
            native = native_mod.Native(icon=ICON_PNG if sys.platform != "win32" else ICON_ICO)
        except Exception as e:  # noqa: BLE001
            why = f"pywebview: {e}"
    if why:
        log.info("no app window (%s): the web browser", why)
    app = App(runtime=rt, native=native, profile=profile)

    def excepthook(kind, value, tb):
        text = "".join(traceback.format_exception(kind, value, tb))
        app.error(f"{kind.__name__}: {value}")
        log.error("unhandled error\n%s", text)
        if native is not None and native.started:
            native.message("FreeDyn 2: error", f"{kind.__name__}: {value}\n\nLog: {logfile}")

    sys.excepthook = excepthook
    threading.excepthook = lambda a: excepthook(a.exc_type, a.exc_value, a.exc_traceback)

    try:
        app.start(port)
    except OSError as e:
        message_box("FreeDyn 2", f"The app could not start its local server: {e}\n\nLog: {logfile}")
        return 1
    app.check_updates()
    try:
        opened = []
        for m in models:
            try:
                opened.append(app.open_model(m))
            except Exception as e:  # noqa: BLE001  (shown on the start page)
                app.error(f"cannot open {m}: {e}")
        if native is not None:
            try:
                for vid in opened or [None]:
                    app.show(vid)
                if app.errors and opened:
                    app.show()
                native.run()
                app.shutdown()
                log.info("FreeDyn 2 ended (last window closed)")
                logging.shutdown()
                os._exit(0)  # the GUI toolkit's threads (Qt WebEngine) can keep the process alive
                return 0
            except Exception as e:  # noqa: BLE001  (no GUI library after all: the browser)
                log.warning("app window failed (%s): the web browser", e)
                app.native = None
        for vid in opened or [None]:
            app.show(vid)
        if app.errors and opened:
            app.show()
        if sys.stderr is not None and not isinstance(sys.stderr, _LogWriter):
            print(f"freedyn2 gui: {app.url()}  (Quit on the start page, or Ctrl+C, ends it; log {logfile})",
                  file=sys.stderr, flush=True)
        try:
            app.run_browser()
        except KeyboardInterrupt:
            pass
        return 0
    finally:
        app.shutdown()
        log.info("FreeDyn 2 ended")


def main(argv=None):
    """freedyn2-gui (the GUI entry point: no console on Windows)."""
    setup_logging()  # (also sys.stdout / sys.stderr when there is no console: argparse writes there)
    args = _parser().parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return run(args.models, args.browser, args.port, args.new_instance, args.profile, args.runtime)
    except Exception as e:  # noqa: BLE001  (no console: a dialog and the log)
        log.exception("the app failed")
        message_box("FreeDyn 2", f"{type(e).__name__}: {e}\n\nLog: {userdirs.log_dir() / 'freedyn2.log'}")
        return 1
