"""The app icon from the brand (brand/logo): app-icon.svg, and favicon.svg for the small sizes.

  python tools/make_app_icon.py      writes generator/fdgen/app/freedyn2.{svg,png,ico}

freedyn2.svg is brand/logo/app-icon.svg; the PNG (256 px) and the Windows icon (48 and 256 px
from app-icon.svg, 16 and 32 px from favicon.svg: the brand has no body axes below 32 px) are
drawn by Chromium (the Python package playwright; FD_CHROMIUM: another Chromium) and scaled
with Pillow.
"""

import io
import os
import shutil
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
LOGO = ROOT / "brand" / "logo"
OUT = ROOT / "generator" / "fdgen" / "app"


def render(svg, size):
    """The SVG file at size x size pixels (RGBA), transparent around it."""
    from playwright.sync_api import sync_playwright

    with sync_playwright() as p:
        browser = p.chromium.launch(**({"executable_path": os.environ["FD_CHROMIUM"]} if "FD_CHROMIUM" in os.environ else {}))
        page = browser.new_page(viewport={"width": size, "height": size})
        page.set_content(f'<html><body style="margin:0;background:transparent">'
                         f'<img src="data:image/svg+xml;base64,{_b64(svg)}" width="{size}" height="{size}" style="display:block">'
                         "</body></html>")
        png = page.screenshot(omit_background=True)
        browser.close()
    return Image.open(io.BytesIO(png)).convert("RGBA")


def _b64(path):
    import base64

    return base64.b64encode(path.read_bytes()).decode()


def main():
    shutil.copyfile(LOGO / "app-icon.svg", OUT / "freedyn2.svg")
    big, small = render(LOGO / "app-icon.svg", 1024), render(LOGO / "favicon.svg", 256)
    big.resize((256, 256), Image.LANCZOS).save(OUT / "freedyn2.png", optimize=True)
    frames = [small.resize((s, s), Image.LANCZOS) for s in (16, 32)] + [big.resize((s, s), Image.LANCZOS) for s in (48, 256)]
    frames[-1].save(OUT / "freedyn2.ico", format="ICO", sizes=[f.size for f in frames], append_images=frames[:-1])
    for name in ("freedyn2.svg", "freedyn2.png", "freedyn2.ico"):
        print(OUT / name)


if __name__ == "__main__":
    main()
