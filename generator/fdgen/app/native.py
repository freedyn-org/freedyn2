"""The app windows with pywebview (WebView2 on Windows, GTK or Qt on Linux).

available() says whether pywebview can open a window here; Native opens windows, the native
file dialogs and an error dialog. webview.start() runs the GUI loop in the main thread and
returns when the last window is closed.
"""

import importlib
import logging
import os
import sys
import threading

from .. import userdirs

log = logging.getLogger("freedyn2.app")
MODEL_TYPES = ("FreeDyn models (*.yaml;*.yml;*.fds)", "All files (*.*)")


def available():
    """pywebview imports, and (Linux) there is a display. None, or why not."""
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return "no display"
    try:
        importlib.import_module("webview")
    except Exception as e:  # noqa: BLE001  (ImportError, or a GUI library that fails to load)
        return f"pywebview is not installed ({e}); pip install freedyn2[gui]"
    return None


def _dialog(webview, name):
    """webview.FileDialog.OPEN (pywebview 5.1+) or webview.OPEN_DIALOG (older)."""
    fd = getattr(webview, "FileDialog", None)
    return getattr(fd, name) if fd is not None else getattr(webview, name + "_DIALOG")


class Native:
    def __init__(self, icon=None):
        import webview

        self.webview, self.icon = webview, icon
        self.windows, self._lock = [], threading.Lock()
        self.started = False
        try:  # links with target=_blank (documentation, release page) in the default browser
            webview.settings["OPEN_EXTERNAL_LINKS_IN_BROWSER"] = True
        except Exception:  # noqa: BLE001  (older pywebview)
            pass

    def window(self, url, title, on_closed=None):
        w = self.webview.create_window(title, url, width=1400, height=900, min_size=(640, 420), text_select=True)
        with self._lock:
            self.windows.append(w)

        def closed():
            with self._lock:
                if w in self.windows:
                    self.windows.remove(w)
            if on_closed:
                on_closed()

        w.events.closed += closed
        return w

    def _front(self):
        with self._lock:
            return self.windows[-1] if self.windows else None

    def choose_model(self):
        w = self._front()
        if w is None:
            return []
        paths = w.create_file_dialog(_dialog(self.webview, "OPEN"), allow_multiple=True, file_types=MODEL_TYPES)
        return [str(p) for p in paths or []]

    def choose_folder(self, start=""):
        w = self._front()
        if w is None:
            return []
        paths = w.create_file_dialog(_dialog(self.webview, "FOLDER"), directory=start)
        return [str(p) for p in paths or []]

    def message(self, title, text):
        """An error dialog in the front window (an alert; nothing if there is none)."""
        w = self._front()
        if w is not None:
            import json

            try:
                w.evaluate_js(f"alert({json.dumps(title + chr(10) + chr(10) + text)})")
            except Exception:  # noqa: BLE001  (the window is closing)
                log.exception("error dialog")

    def close_all(self):
        with self._lock:
            windows = list(self.windows)
        for w in windows:
            try:
                w.destroy()
            except Exception:  # noqa: BLE001
                pass

    def run(self, func=None):
        """The GUI loop (blocks until the last window is closed)."""
        self.started = True
        kwargs = {"private_mode": False, "storage_path": str(userdirs.data_dir() / "webview")}
        if self.icon:
            kwargs["icon"] = str(self.icon)
        try:
            self.webview.start(func, **kwargs)
        except TypeError:  # an older pywebview without some of these arguments
            self.webview.start(func)
