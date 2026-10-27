"""The live examples of the documentation: each a viewer exported to one HTML file
(freedyn2 view --export), embedded in docs/examples.md.

  python tools/docs_examples.py [OUTDIR]     (default docs/live; needs the runtime)

The files are generated, not kept in the repository (docs/live is git-ignored); the
docs workflow runs this before mkdocs build.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "generator"))

# name: (model file, runtime options)
EXAMPLES = {
    "double_pendulum": ("examples/double_pendulum.yaml", ["--end-time", "8"]),
    "quarter_car": ("examples/quarter_car.yaml", []),
    "double_wishbone": ("examples/double_wishbone.yaml", []),
    "car_on_crg": ("examples/car_on_crg.yaml", ["--output-step", "0.02"]),  # 0.04 aliases the 12.5 Hz washboard
    "dzhanibekov": ("examples/dzhanibekov.yaml", []),
    "spring_pendulum": ("examples/spring_pendulum.yaml", []),
    "driven_rotor": ("examples/driven_rotor.yaml", []),
}


def main(outdir=None):
    from fdgen import cli
    from fdgen.view import server

    out = Path(outdir or ROOT / "docs" / "live")
    out.mkdir(parents=True, exist_ok=True)
    runtime = cli.find_runtime()
    for name, (model, options) in EXAMPLES.items():
        page = server.export(server.View(ROOT / model, runtime=runtime, options=options), out / f"{name}.html")
        print(f"{page} ({page.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:2]))
