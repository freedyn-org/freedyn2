"""freedyn2 in a browser (Pyodide): from YAML to the .fdm text, from results to the viewer page.

The browser page (runtime/wasm/demo) runs this module in Pyodide and the runtime as WebAssembly
(runtime/wasm): no server, no CasADi (fdgen._casadi). Library elements only: models with
expression forces or drivers need CasADi and a C compiler, i.e. the desktop freedyn2.

    add_file("parts/arm.stl", data)      # files the model reads (meshes), relative to it
    fdm_text = prepare(yaml_text)        # validate, write the model file; raises ModelError
    html = page(csv_text)                # the viewer with the results, one HTML page
    data = bundle(yaml_text, "m.yaml")   # the model and its files as one zip (download)
"""

import io
import shutil
import zipfile
from pathlib import Path, PurePosixPath

from .errors import ModelError
from .model import load_yaml, validate, write_fdm

WORK = Path("/tmp/freedyn2-web")  # Pyodide's in-memory file system
_state = {}


def _bytes(data):
    """bytes of a Python bytes-like object or of a JavaScript Uint8Array (Pyodide's JsProxy)."""
    return data.to_bytes() if hasattr(data, "to_bytes") else bytes(data)


def _target(name):
    """The place of a file given by its path relative to the model; not outside WORK."""
    rel = PurePosixPath(str(name).replace("\\", "/"))
    if rel.is_absolute() or ".." in rel.parts or not rel.parts:
        raise ValueError(f"{name}: a path relative to the model, without '..'")
    return WORK.joinpath(*rel.parts)


def files():
    """The files next to the model (relative paths), without the model itself and the results."""
    if not WORK.exists():
        return []
    own = {_state.get("path"), WORK / "results.csv"}
    return sorted(f.relative_to(WORK).as_posix() for f in WORK.rglob("*")
                  if f.is_file() and f not in own and f.suffix.lower() not in (".fdm", ".yaml", ".yml"))


def clear_files():
    shutil.rmtree(WORK, ignore_errors=True)
    _state.clear()


def add_file(name, data):
    """A file the model reads (a mesh), at its path relative to the model."""
    path = _target(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_bytes(data))
    return path.relative_to(WORK).as_posix()


def add_zip(data):
    """The files of a zip archive (a model with its meshes). Returns the YAML model files in it
    {path: text} and adds every other file."""
    models = {}
    with zipfile.ZipFile(io.BytesIO(_bytes(data))) as z:
        for info in z.infolist():
            if info.is_dir():
                continue
            if info.filename.lower().endswith((".yaml", ".yml")):
                models[info.filename] = z.read(info).decode("utf-8")
            else:
                add_file(info.filename, z.read(info))
    # a zip with one folder around everything: paths relative to the folder of the model
    if len(models) == 1:
        top = PurePosixPath(next(iter(models))).parent
        if top.parts:
            for f in list(WORK.rglob("*")):
                rel = PurePosixPath(f.relative_to(WORK).as_posix())
                if f.is_file() and rel.parts[:len(top.parts)] == top.parts:
                    add_file(PurePosixPath(*rel.parts[len(top.parts):]), f.read_bytes())
                    f.unlink()
    return models


def bundle(yaml_text, name="model.yaml"):
    """A zip with the model and the files next to it (files())."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(PurePosixPath(name).name, yaml_text)
        for rel in files():
            z.write(WORK / rel, rel)
    return buf.getvalue()


def prepare(yaml_text, name="model.yaml"):
    """The text of the .fdm file of a YAML model (kept for page())."""
    WORK.mkdir(parents=True, exist_ok=True)
    path = WORK / Path(name).name
    path.write_text(yaml_text, encoding="utf-8")
    try:
        model = validate(load_yaml(path), path.parent)
        fdm = WORK / f"{model['model']}.fdm"
        write_fdm(model, fdm)
    except ImportError as e:  # CasADi (fdgen._casadi): expression forces, drivers, splines
        raise ModelError("this model needs the desktop freedyn2: expression forces, drivers and "
                         f"splines are compiled with CasADi and a C compiler ({e})") from e
    except ModelError as e:  # paths as the user gave them, relative to the model
        raise ModelError(str(e).replace(f"{WORK}/", "")) from e
    _state.update(model=model, path=path)
    return fdm.read_text(encoding="utf-8")


def page(csv_text):
    """The viewer page of the model of the last prepare() with these results (CSV text)."""
    from .view.scene import read_results
    from .view.server import build_scene, static_page

    if "model" not in _state:
        raise ModelError("no model: prepare() first")
    model, path = _state["model"], _state["path"]
    csv_path = WORK / "results.csv"
    csv_path.write_text(csv_text, encoding="utf-8")
    s, meshes = build_scene(model, read_results(csv_path), path)
    return static_page(s, csv_text, meshes)
