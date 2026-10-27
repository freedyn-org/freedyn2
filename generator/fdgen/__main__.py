"""Command line: python -m fdgen [model.yaml ...] -o outdir [--models-only]"""

import argparse
import sys
from pathlib import Path

from .codegen import generate_elements
from .fds import load_fds
from .model import ModelError, load_yaml, validate, write_fdm


def main(argv=None):
    ap = argparse.ArgumentParser(prog="fdgen", description="FreeDyn 2.0 model generator")
    ap.add_argument("models", nargs="*", help="model files (.yaml or legacy FreeDyn .fds)")
    ap.add_argument("-o", "--outdir", required=True)
    ap.add_argument("--models-only", action="store_true", help="skip element C code generation")
    ap.add_argument("--profile", choices=("legacy", "v2"), default="legacy",
                    help="how .fds models are solved: legacy algorithm (default) or FreeDyn 2.0 defaults")
    args = ap.parse_args(argv)

    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    if not args.models_only:
        for f in generate_elements(out):
            print(f"wrote {f}")
    for m in args.models:
        try:
            model = load_fds(m, args.profile) if m.lower().endswith(".fds") else validate(load_yaml(m), Path(m).parent)
        except ModelError as e:
            print(f"{m}: {e}", file=sys.stderr)
            return 1
        path = out / f"{model['model']}.fdm"
        write_fdm(model, path)
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
