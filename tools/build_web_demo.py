"""Assemble the browser demo (runtime/wasm/demo) as a static site: no server code, any static host.

    python tools/build_wasm.py build/generated build-wasm
    python tools/build_web_demo.py build-wasm/freedyn2.wasm site
    python -m http.server -d site 8000            # then http://localhost:8000/

The site: index.html, app.mjs, fd_wasm.mjs, freedyn2.wasm, fdgen.zip (the Python package with the
viewer and the schema, for Pyodide), examples.json (the examples that run without CasADi, and
two_links from runtime/wasm/demo/examples with meshes as a CAD export) and the brand files of
the viewer. Pyodide comes from its CDN; --pyodide DIR copies a local Pyodide
distribution into the site (offline; open the page with ?pyodide=./pyodide/).
"""

import argparse
import base64
import json
import math
import struct
import shutil
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FDGEN = ROOT / "generator/fdgen"
WEB = FDGEN / "view/web"


def fdgen_zip(path):
    """fdgen without the native parts (bin, app): its modules, the viewer's web files, the schema."""
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(FDGEN.rglob("*")):
            rel = f.relative_to(FDGEN)
            if f.is_dir() or "__pycache__" in rel.parts or rel.parts[0] in ("bin", "app"):
                continue
            z.write(f, Path("fdgen") / rel)
        for f in sorted((ROOT / "schema").glob("*.json")):
            z.write(f, Path("fdgen/schema") / f.name)


def link_stl(x0, x1, y0, y1, width, segments=24):
    """Binary STL of a link: a slot outline (round ends) between x0 and x1 in the x-z plane,
    extruded from y0 to y1; millimeters, as a CAD export."""
    r = width / 2
    outline = [(x1 + r * math.cos(a), r * math.sin(a))
               for a in (-math.pi / 2 + math.pi * k / segments for k in range(segments + 1))]
    outline += [(x0 + r * math.cos(a), r * math.sin(a))
                for a in (math.pi / 2 + math.pi * k / segments for k in range(segments + 1))]
    tris = []
    n = len(outline)
    for k in range(1, n - 1):  # the outline is convex: fans for the two faces
        a, b, c = outline[0], outline[k], outline[k + 1]
        tris.append(((0, 1, 0), [(a[0], y1, a[1]), (c[0], y1, c[1]), (b[0], y1, b[1])]))
        tris.append(((0, -1, 0), [(a[0], y0, a[1]), (b[0], y0, b[1]), (c[0], y0, c[1])]))
    for k in range(n):  # the sides
        (ax, az), (bx, bz) = outline[k], outline[(k + 1) % n]
        nx, nz = bz - az, -(bx - ax)
        ln = math.hypot(nx, nz) or 1.0
        nrm = (nx / ln, 0, nz / ln)
        tris.append((nrm, [(ax, y0, az), (bx, y0, bz), (bx, y1, bz)]))
        tris.append((nrm, [(ax, y0, az), (bx, y1, bz), (ax, y1, az)]))
    out = bytearray(b"freedyn2 demo link".ljust(80, b" ") + struct.pack("<I", len(tris)))
    for nrm, vs in tris:
        out += struct.pack("<12fH", *nrm, *(c for v in vs for c in v), 0)
    return bytes(out)


def demo_examples():
    """The examples of the demo with the files they read: {name: {yaml, files: {path: base64}}}."""
    files = {"parts/link1.stl": link_stl(0, 500, -5, 5, 40), "parts/link2.stl": link_stl(500, 1000, 5, 15, 40)}
    d = ROOT / "runtime/wasm/demo/examples"
    return {"two_links": {"yaml": (d / "two_links.yaml").read_text(encoding="utf-8"),
                          "files": {k: base64.b64encode(v).decode() for k, v in files.items()}}}


def library_examples():
    """The example models that fdgen.web can prepare without CasADi (library elements only)."""
    sys.path.insert(0, str(ROOT / "generator"))
    saved = sys.modules.get("casadi")
    sys.modules["casadi"] = None  # as in Pyodide: import casadi fails
    try:
        from fdgen import web

        examples = {}
        for y in sorted((ROOT / "examples").glob("*.yaml")):
            text = y.read_text(encoding="utf-8")
            try:
                web.prepare(text, y.name)
            except Exception:  # noqa: BLE001  (needs CasADi, data files, ...: not in the demo)
                continue
            examples[y.stem] = {"yaml": text, "files": {}}
        return examples
    finally:
        if saved is None:
            sys.modules.pop("casadi", None)
        else:
            sys.modules["casadi"] = saved


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("wasm", help="freedyn2.wasm (tools/build_wasm.py)")
    ap.add_argument("out", help="output directory (the site)")
    ap.add_argument("--pyodide", help="a local Pyodide distribution directory to copy into the site")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for f in ("index.html", "app.mjs"):
        shutil.copy(ROOT / "runtime/wasm/demo" / f, out / f)
    shutil.copy(ROOT / "runtime/wasm/fd_wasm.mjs", out / "fd_wasm.mjs")
    shutil.copy(args.wasm, out / "freedyn2.wasm")
    shutil.copy(WEB / "fonts.css", out / "fonts.css")
    shutil.copytree(WEB / "brand", out / "brand", dirs_exist_ok=True)
    shutil.copytree(WEB / "vendor/fonts", out / "vendor/fonts", dirs_exist_ok=True)
    fdgen_zip(out / "fdgen.zip")
    examples = {**demo_examples(), **library_examples()}
    (out / "examples.json").write_text(json.dumps(examples), encoding="utf-8")
    if args.pyodide:
        shutil.copytree(args.pyodide, out / "pyodide", dirs_exist_ok=True)
    size = sum(f.stat().st_size for f in out.rglob("*") if f.is_file() and "pyodide" not in f.parts)
    print(f"wrote {out} ({size / 1e6:.1f} MB without Pyodide; examples: {', '.join(examples)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
