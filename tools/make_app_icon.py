"""The freedyn2 app icon and the installer's wizard images, rendered from the brand SVGs.

  python tools/make_app_icon.py

writes
  generator/fdgen/app/freedyn2.svg   copy of brand/logo/app-icon.svg
  generator/fdgen/app/freedyn2.png   256 px
  generator/fdgen/app/freedyn2.ico   16, 32, 48, 256 px
  installer/windows/wizard/*.bmp     WizardImageFile, WizardSmallImageFile of Inno Setup 6
                                     at 100, 150 and 200 % display scaling

The design is brand/logo/app-icon.svg (brand/make_logos.py); this script only rasterizes it with
Chromium (Playwright, transparent background) and packs the images with Pillow. Brand rule: the
body axes only where the cursor is at least 32 px high. The cursor is 182/512 of the icon, so
the sizes below 90 px (16, 32, 48 and the small wizard image) use the icon with a plain cursor
(the same SVG without its axes elements). Needs playwright with Chromium (PLAYWRIGHT_CHROMIUM:
the browser executable, else the one of `playwright install chromium`) and Pillow.
"""

import base64
import io
import os
import re
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
LOGO = ROOT / "brand" / "logo"
APP = ROOT / "generator" / "fdgen" / "app"
WIZARD = ROOT / "installer" / "windows" / "wizard"
INK = (0x0B, 0x0B, 0x0C)  # brand/tokens.json color.ink
PAPER = (0xED, 0xED, 0xE8)  # color.paper
CURSOR_SHARE = 182 / 512  # cursor height / icon size in app-icon.svg
AXES_MIN = 32  # px, brand: body axes only from this cursor height
ICO_SIZES = (16, 32, 48, 256)
# Inno Setup 6: WizardImageFile and WizardSmallImageFile at 100, 150, 200 %
WIZARD_LARGE = {100: (164, 314), 150: (246, 459), 200: (328, 604)}
WIZARD_SMALL = {100: (55, 55), 150: (83, 80), 200: (110, 106)}


def plain(svg):
    """The SVG without the body axes (lines and the origin dot inside the cursor)."""
    return re.sub(r"<line [^>]*/>|<circle [^>]*/>", "", svg)


def icon_svg(px):
    full = (LOGO / "app-icon.svg").read_text(encoding="utf-8")
    return full if px * CURSOR_SHARE >= AXES_MIN else plain(full)


def _uri(svg):
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode("utf-8")).decode()


def chromium():
    exe = os.environ.get("PLAYWRIGHT_CHROMIUM")
    if not exe:
        import glob

        found = sorted(glob.glob("/opt/pw-browsers/chromium*/chrome-linux*/chrome"))
        exe = found[-1] if found else None
    return exe


class Renderer:
    """Draws HTML with images at an exact pixel size, transparent where nothing is drawn."""

    def __enter__(self):
        from playwright.sync_api import sync_playwright

        self._pw = sync_playwright().start()
        exe = chromium()
        self._browser = self._pw.chromium.launch(**({"executable_path": exe} if exe else {}))
        self._page = self._browser.new_page(device_scale_factor=1)
        return self

    def __exit__(self, *exc):
        self._browser.close()
        self._pw.stop()

    def html(self, body, w, h, background="transparent"):
        self._page.set_viewport_size({"width": w, "height": h})
        self._page.set_content(f"<!doctype html><html><body style='margin:0;width:{w}px;height:{h}px;"
                               f"background:{background};position:relative;overflow:hidden'>{body}</body></html>")
        self._page.wait_for_load_state("load")
        png = self._page.screenshot(omit_background=True, clip={"x": 0, "y": 0, "width": w, "height": h})
        return Image.open(io.BytesIO(png)).convert("RGBA")

    def svg(self, svg, w, h):
        return self.html(f"<img src='{_uri(svg)}' style='display:block;width:{w}px;height:{h}px'>", w, h)


def wizard_large(r, w, h):
    """Ink ground, the app icon (with axes) and below it the small lockup, centred."""
    icon = round(w * 0.56)
    lock_w = round(w * 0.82)
    lock_h = round(lock_w * 168 / 690)
    lockup = (LOGO / "lockup-small-dark.svg").read_text(encoding="utf-8")
    top = round(h * 0.30)
    body = (f"<img src='{_uri(icon_svg(icon))}' style='position:absolute;left:{(w - icon) // 2}px;top:{top}px;"
            f"width:{icon}px;height:{icon}px'>"
            f"<img src='{_uri(lockup)}' style='position:absolute;left:{(w - lock_w) // 2}px;"
            f"top:{top + icon + round(h * 0.05)}px;width:{lock_w}px;height:{lock_h}px'>")
    return r.html(body, w, h, background="#%02X%02X%02X" % INK)


def wizard_small(r, w, h):
    """The app icon (plain cursor) centred on white (the wizard's page header is white)."""
    s = min(w, h)
    img = r.svg(icon_svg(s), s, s)
    out = Image.new("RGBA", (w, h), (255, 255, 255, 255))
    out.alpha_composite(img, ((w - s) // 2, (h - s) // 2))
    return out


def bmp(img, path):
    flat = Image.new("RGB", img.size, (255, 255, 255))
    flat.paste(img, mask=img.getchannel("A"))
    flat.quantize(256, dither=Image.Dither.NONE).save(path, format="BMP")  # 8 bit: a third of the size


def main():
    (APP / "freedyn2.svg").write_text((LOGO / "app-icon.svg").read_text(encoding="utf-8"), encoding="utf-8")
    WIZARD.mkdir(parents=True, exist_ok=True)
    with Renderer() as r:
        frames = {s: r.svg(icon_svg(s), s, s) for s in ICO_SIZES}
        frames[256].save(APP / "freedyn2.png", optimize=True)
        order = sorted(ICO_SIZES, reverse=True)
        frames[order[0]].save(APP / "freedyn2.ico", format="ICO", sizes=[(s, s) for s in order],
                              append_images=[frames[s] for s in order[1:]])
        for scale, (w, h) in WIZARD_LARGE.items():
            bmp(wizard_large(r, w, h), WIZARD / f"wizard-large-{scale}.bmp")
        for scale, (w, h) in WIZARD_SMALL.items():
            bmp(wizard_small(r, w, h), WIZARD / f"wizard-small-{scale}.bmp")
    for p in [APP / "freedyn2.svg", APP / "freedyn2.png", APP / "freedyn2.ico"] + sorted(WIZARD.glob("*.bmp")):
        print(f"wrote {p.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
