"""Is there a newer FreeDyn 2? A notice only: nothing is downloaded or installed.

  check()                 the latest release on GitHub, at most once a day (force=True: now)
  check_in_background(f)  the same in a thread; f(result) when a newer release is there
  freedyn2 update-check   prints the result (always asks GitHub)

The answer of GitHub (RELEASES_API) is kept in data_dir()/update.json, the time of the last
question in the configuration (last_update_check), so the app asks at most once a day and still
shows a newer release found earlier. Any error (offline, a proxy, a rate limit) is silent: the
result is None. Turned off by the configuration update_check = false or the environment
variable FREEDYN2_NO_UPDATE_CHECK=1.
"""

import json
import logging
import os
import re
import threading
import time
import urllib.request

from . import __version__, userdirs

RELEASES_API = "https://api.github.com/repos/freedyn-org/freedyn2/releases/latest"
RELEASE_PAGE = "https://github.com/freedyn-org/freedyn2/releases/latest"
INTERVAL = 24 * 3600  # seconds between two questions to GitHub
TIMEOUT = 4.0  # seconds

log = logging.getLogger("freedyn2.update")
last_error = ""  # why the last check() found nothing (for freedyn2 update-check)


def enabled():
    if os.environ.get("FREEDYN2_NO_UPDATE_CHECK", "").strip().lower() not in ("", "0", "false", "no"):
        return False
    return userdirs.get("update_check", True) is not False


_PRE = {"dev": 0, "a": 1, "alpha": 1, "b": 2, "beta": 2, "c": 3, "rc": 3, "pre": 3, "preview": 3}


def _simple_key(text):
    """A sort key of a version without packaging: release numbers, then pre-releases before
    the release (1.0rc1 < 1.0), post releases after; a local part (+...) is ignored."""
    s = text.strip().lower().lstrip("v").split("+", 1)[0]
    m = re.match(r"(\d+(?:\.\d+)*)(.*)$", s)
    if not m:
        raise ValueError(f"not a version: {text!r}")
    release = [int(p) for p in m.group(1).split(".")]
    while len(release) > 1 and release[-1] == 0:
        release.pop()
    rest = m.group(2).lstrip(".-_")
    if not rest:
        return (release, 4, 0)
    m2 = re.match(r"(dev|alpha|beta|preview|pre|rc|a|b|c|post|r|rev)[.\-_]?(\d*)", rest)
    if not m2:
        raise ValueError(f"not a version: {text!r}")
    tag, num = m2.group(1), int(m2.group(2) or 0)
    if tag in ("post", "r", "rev"):
        return (release, 5, num)
    return (release, _PRE[tag], num)


def parse_version(text):
    """A comparable version (packaging.version.Version when packaging is there); a leading v
    (a git tag v2.1.0) is dropped. ValueError if it is not one."""
    text = str(text).strip()
    try:
        from packaging.version import InvalidVersion, Version
    except ImportError:
        return _simple_key(text)
    try:
        return Version(text[1:] if text[:1] in "vV" else text)
    except InvalidVersion as e:
        raise ValueError(f"not a version: {text!r}") from e


def is_newer(latest, current=__version__):
    """True if the version latest is after current; False if either is not a version."""
    try:
        return parse_version(latest) > parse_version(current)
    except (ValueError, TypeError):
        return False


def _cache_file():
    return userdirs.data_dir() / "update.json"


def _cached():
    try:
        data = json.loads(_cache_file().read_text(encoding="utf-8"))
        return data if isinstance(data, dict) and data.get("latest") else None
    except (OSError, ValueError):
        return None


def fetch(timeout=TIMEOUT):
    """{latest, url} of the latest release on GitHub; raises on any error."""
    req = urllib.request.Request(RELEASES_API, headers={"Accept": "application/vnd.github+json",
                                                        "User-Agent": f"freedyn2/{__version__}"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        data = json.loads(r.read().decode("utf-8"))
    tag = str(data.get("tag_name") or data.get("name") or "").strip()
    if not tag:
        raise ValueError("no tag_name in the answer")
    return {"latest": tag[1:] if tag[:1] in "vV" else tag, "url": data.get("html_url") or RELEASE_PAGE}


def result(info, current=__version__):
    """The result of a check: {current, latest, newer, url}."""
    return {"current": current, "latest": info["latest"], "newer": is_newer(info["latest"], current),
            "url": info.get("url") or RELEASE_PAGE}


def check(force=False, timeout=TIMEOUT):
    """{current, latest, newer, url}, or None (turned off, not asked today and nothing found
    before, or an error). Asks GitHub at most once a day unless force."""
    global last_error
    last_error = ""
    if not force and not enabled():
        return None
    try:
        last = float(userdirs.get("last_update_check", 0) or 0)
    except (TypeError, ValueError):
        last = 0.0
    now = time.time()
    if not force and 0 <= now - last < INTERVAL:
        cached = _cached()
        return result(cached) if cached else None
    try:
        info = fetch(timeout)
    except Exception as e:  # noqa: BLE001  (offline, proxy, rate limit, bad answer: silent)
        log.info("update check failed: %s", e)
        last_error = "no release published yet" if getattr(e, "code", None) == 404 else str(e)
        try:
            userdirs.set("last_update_check", now)  # not again today
        except OSError:
            pass
        return None
    try:
        userdirs.set("last_update_check", now)
        _cache_file().write_text(json.dumps({**info, "checked": now}), encoding="utf-8")
    except OSError as e:
        log.info("update check: %s", e)
    return result(info)


def check_in_background(on_newer, force=False):
    """check() in a daemon thread; on_newer(result) if a newer release is there. Returns the
    thread (None if turned off)."""
    if not force and not enabled():
        return None

    def work():
        try:
            r = check(force=force)
            if r and r["newer"]:
                on_newer(r)
        except Exception:  # noqa: BLE001  (never disturbs the app)
            log.exception("update check")

    t = threading.Thread(target=work, name="freedyn2-update-check", daemon=True)
    t.start()
    return t


def main():
    """freedyn2 update-check: ask GitHub now and print the answer; 0, or 1 if it failed."""
    r = check(force=True)
    if r is None:
        print(f"FreeDyn 2 version {__version__}: no answer from GitHub ({last_error}; {RELEASES_API})")
        return 1
    if r["newer"]:
        print(f"FreeDyn 2 version {r['latest']} is available (this is {r['current']}): {r['url']}")
    else:
        print(f"FreeDyn 2 version {r['current']} is up to date (latest release {r['latest']})")
    if not enabled():
        print("(the automatic check of the app is off: update_check = false or FREEDYN2_NO_UPDATE_CHECK)")
    return 0
