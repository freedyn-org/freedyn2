"""Copy the brand assets that the app and the viewer ship into the package (one source: brand/).

  python tools/sync_brand.py           copies brand/tokens.css and the logos the pages use
  python tools/sync_brand.py --fonts   also downloads the fonts (pinned npm packages, checked)

Target: generator/fdgen/view/web/brand/ (served by the viewer, the app under /app/brand/, inlined by
freedyn2 view --export), so that the wheel is self-contained and the pages work offline.
tests/python/test_app.py checks that the copies equal brand/.

The fonts are JetBrains Mono (UI, labels, headings) and Inter (running text), SIL OFL 1.1, from
the npm packages @fontsource/jetbrains-mono and @fontsource/inter: the latin subset, weights 400
and 600, woff2 only, with the license texts (fonts/OFL-*.txt).
"""

import argparse
import base64
import hashlib
import io
import shutil
import tarfile
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRAND = ROOT / "brand"
OUT = ROOT / "generator" / "fdgen" / "view" / "web" / "brand"

# brand file -> name in OUT
COPIES = {"tokens.css": "tokens.css", "logo/favicon.svg": "favicon.svg",
          "logo/lockup-small-light.svg": "lockup-small-light.svg", "logo/lockup-small-dark.svg": "lockup-small-dark.svg"}

FONT_VERSION = "5.3.0"
FONTS = {  # npm package -> (sha512 integrity of the tarball, family, file stem, license file name)
    "jetbrains-mono": ("fqDfB5I9f1p1TV486aUgB9t8zP84P0O1FtQR5Ol9vjwPy+S+EIGlVYm1cvj2W5shcZMTg2nZFdVMoH5wFu8a1A==",
                       "JetBrains Mono", "jetbrains-mono", "OFL-JetBrainsMono.txt"),
    "inter": ("RofMylZmjlJEfELXeNHFWBRcSs75rGU/6bV2S2jfnvv/3rPXPGe0LgUJTklcHZ9lM4OZmAVFhcJPnACfb91A3g==",
              "Inter", "inter", "OFL-Inter.txt"),
}
WEIGHTS = (400, 600)


def copy_brand():
    OUT.mkdir(parents=True, exist_ok=True)
    for src, dst in COPIES.items():
        shutil.copyfile(BRAND / src, OUT / dst)
        print(f"copied brand/{src} -> {(OUT / dst).relative_to(ROOT)}")


def fetch_fonts():
    fonts = OUT / "fonts"
    fonts.mkdir(parents=True, exist_ok=True)
    for pkg, (integrity, _family, stem, license_name) in FONTS.items():
        url = f"https://registry.npmjs.org/@fontsource/{pkg}/-/{pkg}-{FONT_VERSION}.tgz"
        data = urllib.request.urlopen(url, timeout=60).read()
        if base64.b64encode(hashlib.sha512(data).digest()).decode() != integrity:
            raise SystemExit(f"{url}: sha512 differs from the pinned one")
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
            def member(name):
                return tar.extractfile(f"package/{name}").read()

            (fonts / license_name).write_bytes(member("LICENSE"))
            for w in WEIGHTS:
                name = f"{stem}-latin-{w}-normal.woff2"
                (fonts / name).write_bytes(member(f"files/{name}"))
                print(f"wrote {(fonts / name).relative_to(ROOT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fonts", action="store_true", help="also download the fonts")
    args = ap.parse_args()
    copy_brand()
    if args.fonts:
        fetch_fonts()


if __name__ == "__main__":
    main()
