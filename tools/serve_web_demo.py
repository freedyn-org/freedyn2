"""Serve the browser demo (tools/build_web_demo.py) on localhost with the right content types.

    python tools/serve_web_demo.py build-web          # http://localhost:8000/
    python tools/serve_web_demo.py build-web --port 8080

python -m http.server takes the content types from the system; on Windows from the registry,
where .mjs (or even .js) can be missing or text/plain, and the browser then refuses the page's
modules: the page stays at "loading". This server sets them itself.
"""

import argparse
import functools
import sys
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TYPES = {".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript", ".css": "text/css",
         ".json": "application/json", ".wasm": "application/wasm", ".svg": "image/svg+xml",
         ".woff2": "font/woff2", ".zip": "application/zip", ".whl": "application/zip",
         ".stl": "application/octet-stream", ".tar": "application/x-tar"}


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, **TYPES}

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")  # a rebuilt site is seen at once
        super().end_headers()


def main(argv=None):
    ap = argparse.ArgumentParser(description="serve the freedyn2 browser demo")
    ap.add_argument("site", help="the directory written by tools/build_web_demo.py")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)
    site = Path(args.site).resolve()
    if not (site / "index.html").exists():
        sys.exit(f"{site}: no index.html (python tools/build_web_demo.py build-wasm/freedyn2.wasm {args.site})")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), functools.partial(Handler, directory=str(site)))
    url = f"http://localhost:{args.port}/"
    print(f"freedyn2 demo: {url}  (Ctrl+C stops)", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
