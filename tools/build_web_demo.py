"""Assemble the browser demo (runtime/wasm/demo) as a static site: no server code, any static host.

    python tools/build_wasm.py build/generated build-wasm
    python tools/build_web_demo.py build-wasm/freedyn2.wasm site
    python -m http.server -d site 8000            # then http://localhost:8000/

The site: index.html, app.mjs, fd_wasm.mjs, freedyn2.wasm, fdgen.zip (the Python package with the
viewer and the schema, for Pyodide), examples.json (the examples that run without CasADi) and the
brand files of the viewer. Pyodide comes from its CDN; --pyodide DIR copies a local Pyodide
distribution into the site (offline; open the page with ?pyodide=./pyodide/).
"""

import argparse
import json
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
            examples[y.stem] = text
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
    examples = library_examples()
    (out / "examples.json").write_text(json.dumps(examples), encoding="utf-8")
    if args.pyodide:
        shutil.copytree(args.pyodide, out / "pyodide", dirs_exist_ok=True)
    size = sum(f.stat().st_size for f in out.rglob("*") if f.is_file() and "pyodide" not in f.parts)
    print(f"wrote {out} ({size / 1e6:.1f} MB without Pyodide; examples: {', '.join(examples)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
