"""Copy the brand files that the package and the docs ship (one source: brand/).

  python tools/sync_brand.py

The wheel and the docs site have no brand/, so they carry copies: the viewer and the app in
generator/fdgen/view/web/brand/ (the app serves them under /app/view/brand/, freedyn2 view
--export inlines them), the docs in docs/assets/brand/. tests/python/test_view.py and
tests/python/test_docs.py check that the copies equal brand/. The fonts are vendored once
(generator/fdgen/view/web/vendor/fonts, docs/assets/fonts; see the README files there).
"""

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRAND = ROOT / "brand"
TARGETS = {  # directory -> brand files (relative to brand/)
    ROOT / "generator" / "fdgen" / "view" / "web" / "brand":
        ["tokens.css", "logo/favicon.svg", "logo/lockup-small-light.svg", "logo/lockup-small-dark.svg"],
    ROOT / "docs" / "assets" / "brand": ["tokens.css", "logo/favicon.svg", "logo/lockup-small-dark.svg"],
}


def main():
    for out, names in TARGETS.items():
        out.mkdir(parents=True, exist_ok=True)
        for name in names:
            shutil.copyfile(BRAND / name, out / Path(name).name)
            print(f"copied brand/{name} -> {(out / Path(name).name).relative_to(ROOT)}")


if __name__ == "__main__":
    main()
