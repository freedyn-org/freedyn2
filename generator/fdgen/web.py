"""freedyn2 in a browser (Pyodide): from YAML to the .fdm text, from results to the viewer page.

The browser page (runtime/wasm/demo) runs this module in Pyodide and the runtime as WebAssembly
(runtime/wasm): no server, no CasADi (fdgen._casadi). Library elements only: models with
expression forces or drivers need CasADi and a C compiler, i.e. the desktop freedyn2.

    fdm_text = prepare(yaml_text)        # validate, write the model file; raises ModelError
    html = page(csv_text)                # the viewer with the results, one HTML page
"""

from pathlib import Path

from .errors import ModelError
from .model import load_yaml, validate, write_fdm

WORK = Path("/tmp/freedyn2-web")  # Pyodide's in-memory file system
_state = {}


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
    _state.update(model=model, path=path)
    return fdm.read_text(encoding="utf-8")


def page(csv_text):
    """The viewer page of the model of the last prepare() with these results (CSV text)."""
    from .view.scene import read_results
    from .view.server import build_scene, static_page

    model, path = _state["model"], _state["path"]
    csv_path = WORK / "results.csv"
    csv_path.write_text(csv_text, encoding="utf-8")
    s, meshes = build_scene(model, read_results(csv_path), path)
    return static_page(s, csv_text, meshes)
