"""One app per user: a second start hands its models to the running one.

The running app writes data_dir()/instance.json: {port, token, pid}. A new start reads it and
POSTs the models to http://127.0.0.1:<port>/api/app/handoff with the token (header
X-freedyn2-Token); if that works, the new start is done (the running app opens a window per
model, or the start page). If there is no file, or nobody answers, the new start runs the app.
"""

import json
import logging
import os
import secrets
import urllib.request
from pathlib import Path

from .. import userdirs

HEADER = "X-freedyn2-Token"
log = logging.getLogger("freedyn2.app")


def _file():
    return userdirs.data_dir() / "instance.json"


def read():
    try:
        data = json.loads(_file().read_text(encoding="utf-8"))
        return data if isinstance(data, dict) and data.get("port") and data.get("token") else None
    except (OSError, ValueError):
        return None


def new_token():
    return secrets.token_urlsafe(24)


def publish(port, token):
    f = _file()
    tmp = f.with_suffix(".tmp")
    tmp.write_text(json.dumps({"port": port, "token": token, "pid": os.getpid()}), encoding="utf-8")
    os.replace(tmp, f)


def withdraw(token):
    """Remove the instance file if it is still ours."""
    data = read()
    if data and data.get("token") == token:
        try:
            _file().unlink()
        except OSError:
            pass


def hand_off(paths, timeout=3.0):
    """Give paths (models; [] for the start page) to the running app. True if it took them."""
    data = read()
    if not data:
        return False
    body = json.dumps({"paths": [str(Path(p).resolve()) for p in paths]}).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{int(data['port'])}/api/app/handoff", data=body, method="POST",
                                 headers={"Content-Type": "application/json", HEADER: str(data["token"])})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # localhost: never a proxy
    try:
        with opener.open(req, timeout=timeout) as r:
            ok = r.status == 200
    except Exception as e:  # noqa: BLE001  (not running any more: start ourselves)
        log.info("no running instance on port %s: %s", data.get("port"), e)
        return False
    if ok:
        log.info("handed %d model(s) to the running instance (pid %s)", len(paths), data.get("pid"))
    return ok
