"""The freedyn2 app icon: a double pendulum on a blue tile.

  python tools/make_app_icon.py      writes generator/fdgen/app/freedyn2.{svg,png,ico}

One design (the numbers below, on a 1024 grid) gives the SVG source and, drawn with Pillow at
1024 px and scaled down, the PNG (256 px) and the Windows icon (16, 32, 48, 256 px).
"""

from pathlib import Path

from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parents[1] / "generator" / "fdgen" / "app"
N = 1024
TILE = {"radius": 224, "top": (63, 127, 236), "bottom": (36, 86, 190)}  # the viewer's accent blue
GROUND = ((330, 214), (694, 214), 40)  # bar from, to, thickness
PIVOT = ((512, 234), 50)
JOINT = ((330, 560), 58)
MASS = ((676, 790), 118)
LINK = 70  # width of the links
WHITE, ORANGE, RING = (255, 255, 255), (247, 159, 31), (255, 236, 205)


def _hex(c):
    return "#%02x%02x%02x" % c


def svg():
    (gx0, gy), (gx1, _), gt = GROUND
    (px, py), pr = PIVOT
    (jx, jy), jr = JOINT
    (mx, my), mr = MASS
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {N} {N}" width="{N}" height="{N}">
  <title>freedyn2</title>
  <defs>
    <linearGradient id="tile" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="{_hex(TILE['top'])}"/>
      <stop offset="1" stop-color="{_hex(TILE['bottom'])}"/>
    </linearGradient>
  </defs>
  <rect width="{N}" height="{N}" rx="{TILE['radius']}" fill="url(#tile)"/>
  <line x1="{gx0}" y1="{gy}" x2="{gx1}" y2="{gy}" stroke="#ffffff" stroke-opacity="0.75" stroke-width="{gt}" stroke-linecap="round"/>
  <polyline points="{px},{py} {jx},{jy} {mx},{my}" fill="none" stroke="#ffffff" stroke-width="{LINK}" stroke-linecap="round" stroke-linejoin="round"/>
  <circle cx="{px}" cy="{py}" r="{pr}" fill="#ffffff"/>
  <circle cx="{px}" cy="{py}" r="{pr * 0.4:g}" fill="{_hex(TILE['bottom'])}"/>
  <circle cx="{jx}" cy="{jy}" r="{jr}" fill="#ffffff"/>
  <circle cx="{jx}" cy="{jy}" r="{jr * 0.4:g}" fill="{_hex(TILE['bottom'])}"/>
  <circle cx="{mx}" cy="{my}" r="{mr}" fill="{_hex(ORANGE)}" stroke="#ffffff" stroke-width="{LINK * 0.5:g}"/>
  <circle cx="{mx - mr * 0.3:g}" cy="{my - mr * 0.3:g}" r="{mr * 0.22:g}" fill="{_hex(RING)}" fill-opacity="0.7"/>
</svg>
"""


def _disc(d, centre, r, fill):
    x, y = centre
    d.ellipse((x - r, y - r, x + r, y + r), fill=fill)


def draw():
    """The icon at N x N pixels (RGBA)."""
    grad = Image.new("RGBA", (1, N))
    for y in range(N):
        t = y / (N - 1)
        grad.putpixel((0, y), tuple(round(a + (b - a) * t) for a, b in zip(TILE["top"], TILE["bottom"])) + (255,))
    img = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    mask = Image.new("L", (N, N), 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, N - 1, N - 1), TILE["radius"], fill=255)
    img.paste(grad.resize((N, N)), (0, 0), mask)

    over = Image.new("RGBA", (N, N), (0, 0, 0, 0))  # the ground bar, translucent
    (gx0, gy), (gx1, _), gt = GROUND
    ImageDraw.Draw(over).rounded_rectangle((gx0 - gt / 2, gy - gt / 2, gx1 + gt / 2, gy + gt / 2), gt / 2,
                                           fill=WHITE + (191,))
    img = Image.alpha_composite(img, over)

    d = ImageDraw.Draw(img)
    pts = [PIVOT[0], JOINT[0], MASS[0]]
    d.line(pts, fill=WHITE, width=LINK, joint="curve")
    for p in pts:
        _disc(d, p, LINK / 2, WHITE)  # round caps
    for centre, r in (PIVOT, JOINT):
        _disc(d, centre, r, WHITE)
        _disc(d, centre, r * 0.4, TILE["bottom"])
    (mx, my), mr = MASS
    _disc(d, (mx, my), mr + LINK * 0.25, WHITE)
    _disc(d, (mx, my), mr - LINK * 0.25, ORANGE)
    hl = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    _disc(ImageDraw.Draw(hl), (mx - mr * 0.3, my - mr * 0.3), mr * 0.22, RING + (179,))
    return Image.alpha_composite(img, hl)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "freedyn2.svg").write_text(svg(), encoding="utf-8")
    big = draw()
    big.resize((256, 256), Image.LANCZOS).save(OUT / "freedyn2.png", optimize=True)
    sizes = [16, 32, 48, 256]
    frames = [big.resize((s, s), Image.LANCZOS) for s in sizes]
    frames[-1].save(OUT / "freedyn2.ico", format="ICO", sizes=[(s, s) for s in sizes], append_images=frames[:-1])
    for name in ("freedyn2.svg", "freedyn2.png", "freedyn2.ico"):
        print(f"wrote {OUT / name}")


if __name__ == "__main__":
    main()
