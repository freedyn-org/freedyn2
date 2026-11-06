"""The screenshots of the documentation (docs/img/*.png), made again from the examples.

  python tools/docs_screenshots.py              all of them
  python tools/docs_screenshots.py fan viewer   these

Needs freedyn2_run (built; FREEDYN2_RUN or on the PATH), the Python package playwright and a
Chromium for it (playwright install chromium; FD_CHROMIUM: another Chromium). The pages are
the light theme at the size the documentation shows them; the viewer and the app run as for a
user, on local ports, with copies of the examples in a temporary folder.
"""

import os
import shutil
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "generator"))

from fdgen import cli  # noqa: E402
from fdgen.view import server  # noqa: E402

IMG = ROOT / "docs" / "img"
# the model file in the header as ~/work/... instead of the temporary folder
HIDE_TMP = """addEventListener("DOMContentLoaded", () => { const el = document.getElementById("status"); if (!el) return;
  new MutationObserver(() => { if (el.textContent.includes("TMP")) el.textContent = el.textContent.replace("TMP", "~/work"); })
    .observe(el, { childList: true, characterData: true, subtree: true }); });"""
DAMPER = "parameters.c_damper"


def viewer(model, runtime, **kw):
    """A viewer of the model on a free port: (url, stop)."""
    v = server.View(model, runtime=runtime, **kw)
    v.rebuild()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server._handler(v))
    stop = threading.Event()
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    threading.Thread(target=v.watch, args=(stop,), daemon=True).start()

    def close():
        stop.set()
        httpd.shutdown()
        shutil.rmtree(v.workdir, ignore_errors=True)

    return f"http://127.0.0.1:{httpd.server_address[1]}/", close


def wait(page, condition, timeout=120):
    """Until the JavaScript condition holds in the page."""
    t0 = time.time()
    while not page.evaluate(condition):
        if time.time() - t0 > timeout:
            raise TimeoutError(condition)
        page.wait_for_timeout(200)


def ready(page):
    wait(page, "window.fdViewer && window.fdViewer.results && window.fdViewer.panels.server"
               " && window.fdViewer.panels.server.status === 'ready'")
    page.wait_for_timeout(400)


def plots(page, *panes, time_=None):
    """One time plot per tuple of channels; the time cursor at time_."""
    page.evaluate("""(panes) => { const a = window.fdViewer.plots;
        for (const names of panes) { a.addPane("time"); for (const n of names) a.addChannel(n, a.panes.length - 1); } }""",
                  [list(p) for p in panes])
    if time_ is not None:
        page.evaluate(f"window.fdViewer.setTime({time_})")


def quarter_car(page, url):
    page.goto(url)
    ready(page)
    page.evaluate("document.getElementById('tune-details').open = true")
    page.select_option("#tune-policy", "manual")


def damper(page, value):
    page.evaluate(f"window.fdViewer.panels.set('{DAMPER}', {value}, 1800)")


def compute(page):
    page.evaluate("window.fdViewer.panels.compute()")
    page.wait_for_timeout(500)
    ready(page)
    wait(page, "document.querySelectorAll('#runs .run').length >= 2")


def model_file_run_as_reference(page):
    page.locator("#runs .run", has_text="model file").locator(".star").click()
    wait(page, "window.fdViewer.panels.server.reference !== null")
    page.wait_for_timeout(600)


def sweep(page):
    page.locator(f".tune-row[data-path='{DAMPER}'] .sweep").click()
    wait(page, "(s => s.sweep && s.sweep.status !== 'running')(window.fdViewer.panels.server)", timeout=300)
    page.wait_for_timeout(1500)


# --- the pictures: name -> (viewport, function(page, ctx)) -----------------------------------

QUARTER_PLOTS = (("body.R_z", "tire.value", "body.Rddot_z"),)


def shot_viewer(page, ctx):
    quarter_car(page, ctx["quarter"]())
    plots(page, *QUARTER_PLOTS, time_=0.62)


def shot_tune_pending(page, ctx):
    quarter_car(page, ctx["quarter"]())
    plots(page, *QUARTER_PLOTS, time_=0.62)
    damper(page, 800)


def shot_runs_reference(page, ctx):
    quarter_car(page, ctx["quarter"]())
    plots(page, *QUARTER_PLOTS, time_=0.62)
    damper(page, 800)
    compute(page)
    model_file_run_as_reference(page)


def shot_runs_delta(page, ctx):
    shot_runs_reference(page, ctx)
    page.click("#plot-delta")


def shot_fan(page, ctx):
    shot_runs_reference(page, ctx)
    sweep(page)


def shot_root_locus(page, ctx):
    shot_fan(page, ctx)
    page.evaluate("window.fdViewer.setTime(0.2)")
    page.locator(f".tune-row[data-path='{DAMPER}'] .locus").click()
    page.wait_for_timeout(3000)


def shot_pendulum(page, ctx):
    url, close = viewer(ctx["examples"] / "pendulum.yaml", ctx["runtime"])
    ctx["close"].append(close)
    page.goto(url)
    ready(page)
    plots(page, ("rod.R_x", "rod.R_z"), time_=0.62)


def shot_car_crg(page, ctx):
    url, close = viewer(ctx["examples"] / "car_on_crg.yaml", ctx["runtime"])
    ctx["close"].append(close)
    page.goto(url)
    ready(page)
    page.select_option("#follow", "chassis")
    plots(page, ("tire_fl.force_z",), ("tire_rl.force_z",), ("head.specific_force_z",), time_=8.62)


def shot_app_start(page, ctx):
    from fdgen.app.server import App

    app = App(runtime=ctx["runtime"], native=None, open_browser=False, idle_seconds=None).start(port=0, publish=False)
    ctx["close"].append(app.shutdown)
    page.goto(app.url())
    page.wait_for_timeout(1500)


SHOTS = {
    "viewer": ((1500, 950), shot_viewer),
    "tune-pending": ((1400, 900), shot_tune_pending),
    "runs-reference": ((1400, 900), shot_runs_reference),
    "runs-delta": ((1400, 900), shot_runs_delta),
    "fan": ((1400, 900), shot_fan),
    "root-locus": ((1400, 900), shot_root_locus),
    "pendulum": ((1300, 760), shot_pendulum),
    "car-crg": ((1400, 860), shot_car_crg),
    "app-start": ((1200, 760), shot_app_start),
}


def main(names):
    from playwright.sync_api import sync_playwright

    runtime = cli.find_runtime()
    tmp = Path(tempfile.mkdtemp(prefix="freedyn2_shots_"))
    examples = tmp / "examples"
    shutil.copytree(ROOT / "examples", examples)
    ctx = {"runtime": runtime, "examples": examples, "close": []}

    def quarter():
        url, close = viewer(examples / "quarter_car.yaml", runtime)
        ctx["close"].append(close)
        return url

    ctx["quarter"] = quarter
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(**({"executable_path": os.environ["FD_CHROMIUM"]} if "FD_CHROMIUM" in os.environ else {}))
            for name in names or SHOTS:
                (w, h), shot = SHOTS[name]
                page = browser.new_page(viewport={"width": w, "height": h}, color_scheme="light", reduced_motion="reduce")
                page.add_init_script(HIDE_TMP.replace("TMP", str(examples).replace("\\", "/")))
                shot(page, ctx)
                page.wait_for_timeout(600)
                page.screenshot(path=str(IMG / f"{name}.png"))
                page.close()
                for close in ctx["close"]:
                    close()
                ctx["close"].clear()
                print(f"docs/img/{name}.png", flush=True)
            browser.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main(sys.argv[1:])
