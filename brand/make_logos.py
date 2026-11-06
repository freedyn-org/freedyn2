"""Generate freedyn2 logo SVGs with JetBrains Mono glyphs converted to outlines.

Usage: python brand/make_logos.py <font_dir> brand/logo

<font_dir> holds jetbrains-mono-latin-{400,500,700}-normal.woff, e.g. from the npm package
@fontsource/jetbrains-mono (files/). Needs fonttools.
"""
import os
import sys

from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen
from fontTools.ttLib import TTFont

FONT_DIR, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)

INK, PAPER, RULE = "#0B0B0C", "#EDEDE8", "#3A3A38"
CURSOR = "#C8F031"
SLASH_DARK, SLASH_LIGHT = "#6E6E6A", "#5E5E5A"
CLAIM_DARK, CLAIM_LIGHT = "#B5B5B0", "#5E5E5A"

fonts = {w: TTFont(os.path.join(FONT_DIR, f"jetbrains-mono-latin-{w}-normal.woff")) for w in (400, 500, 700)}


def text_paths(text, weight, size, x, baseline, tracking=-0.04):
    """Return (list of path d strings, end x) for text set at size px."""
    font = fonts[weight]
    upm = font["head"].unitsPerEm
    gs = font.getGlyphSet()
    cmap = font.getBestCmap()
    hmtx = font["hmtx"]
    k = size / upm
    out = []
    for ch in text:
        name = cmap[ord(ch)]
        pen = SVGPathPen(gs)
        gs[name].draw(TransformPen(pen, (k, 0, 0, -k, x, baseline)))
        d = pen.getCommands()
        if d:
            out.append(d)
        x += hmtx[name][0] * k + tracking * size
    return out, x


def baseline_in_box(weight, size):
    """Baseline offset from the top of a line-height:1 box (CSS model)."""
    font = fonts[weight]
    upm = font["head"].unitsPerEm
    asc, desc = font["hhea"].ascent, -font["hhea"].descent
    return (asc - (asc + desc - upm) / 2) / upm * size


def axes(cx, cy, w, color):
    """Body-frame axes inside a cursor block whose bottom-left corner is (cx, cy); w = cursor width."""
    s = w / 58.0
    ox, oy = cx + 8 * s + 4 * s, cy - 8 * s - 4 * s
    L, sw, r = 30 * s, 3.5 * s, 4.5 * s
    return (f'<line x1="{ox:.2f}" y1="{oy:.2f}" x2="{ox + L:.2f}" y2="{oy:.2f}" stroke="{color}" stroke-width="{sw:.2f}"/>'
            f'<line x1="{ox:.2f}" y1="{oy:.2f}" x2="{ox:.2f}" y2="{oy - L:.2f}" stroke="{color}" stroke-width="{sw:.2f}"/>'
            f'<circle cx="{ox:.2f}" cy="{oy:.2f}" r="{r:.2f}" fill="{color}"/>')


def svg(w, h, body, label):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {h:.0f}" width="{w:.0f}" height="{h:.0f}" '
            f'role="img" aria-label="{label}">\n{body}\n</svg>\n')


def lockup(theme, with_axes, with_claim, bg):
    size = 112.0
    pad = 0.25 * size
    fg = PAPER if theme == "dark" else INK
    slash = SLASH_DARK if theme == "dark" else SLASH_LIGHT
    cur = CURSOR if theme == "dark" else INK
    axc = INK if theme == "dark" else PAPER
    top = pad
    base = top + baseline_in_box(500, size)
    parts = []
    p, x = text_paths("/", 500, size, pad, base)
    parts += [f'<path d="{d}" fill="{slash}"/>' for d in p]
    p, x = text_paths("freedyn2", 500, size, x, base)
    parts += [f'<path d="{d}" fill="{fg}"/>' for d in p]
    x += 0.04 * size + 0.06 * size  # undo trailing tracking, then cursor margin
    cw, ch = 0.52 * size, 0.86 * size
    cy0 = top + 0.07 * size
    parts.append(f'<rect x="{x:.2f}" y="{cy0:.2f}" width="{cw:.2f}" height="{ch:.2f}" fill="{cur}"/>')
    if with_axes:
        parts.append(axes(x, cy0 + ch, cw, axc))
    right = x + cw
    bottom = top + size
    if with_claim:
        csize = 26.0
        cbase = bottom + 28 + baseline_in_box(400, csize) - 4
        p, cx = text_paths("ai-first multibody dynamics", 400, csize, pad + 4, cbase, tracking=0.02)
        claim_col = CLAIM_DARK if theme == "dark" else CLAIM_LIGHT
        parts += [f'<path d="{d}" fill="{claim_col}"/>' for d in p]
        p, cx = text_paths(".", 400, csize, cx, cbase, tracking=0.02)
        parts += [f'<path d="{d}" fill="{cur}"/>' for d in p]
        bottom = cbase + 0.3 * csize
        right = max(right, cx)
    W, H = right + pad, bottom + pad
    bgrect = f'<rect width="{W:.0f}" height="{H:.0f}" fill="{INK if theme == "dark" else PAPER}"/>' if bg else ""
    return svg(W, H, bgrect + "\n".join(parts), "freedyn2")


def mark(theme):
    S = 80.0
    fg = PAPER if theme == "dark" else INK
    bg = INK if theme == "dark" else PAPER
    c, sw = 12, 2
    corners = (f'<path d="M1 {c} V1 H{c} M{S - c} 1 H{S - 1} V{c} M{S - 1} {S - c} V{S - 1} H{S - c} M{c} {S - 1} H1 V{S - c}" '
               f'fill="none" stroke="{fg}" stroke-width="{sw}"/>')
    size = 28.0
    widths = sum(fonts[700]["hmtx"][fonts[700].getBestCmap()[ord(ch)]][0] for ch in "fd2") / 1000 * size - 0.05 * size * 2
    base = S / 2 - size / 2 + baseline_in_box(700, size)
    p, _ = text_paths("fd2", 700, size, (S - widths) / 2, base, tracking=-0.05)
    body = f'<rect width="{S}" height="{S}" fill="{bg}"/>' + corners + "".join(f'<path d="{d}" fill="{fg}"/>' for d in p)
    return svg(S, S, body, "freedyn2")


def favicon():
    S = 32.0
    size = 22.0
    font = fonts[700]
    adv = font["hmtx"][font.getBestCmap()[ord("/")]][0] / 1000 * size
    base = S / 2 - size / 2 + baseline_in_box(700, size)
    p, _ = text_paths("/", 700, size, (S - adv) / 2, base, tracking=0)
    body = f'<rect width="{S}" height="{S}" fill="{CURSOR}"/>' + "".join(f'<path d="{d}" fill="{INK}"/>' for d in p)
    return svg(S, S, body, "freedyn2")


def app_icon():
    S = 512.0
    size = 200.0
    font = fonts[700]
    adv = font["hmtx"][font.getBestCmap()[ord("/")]][0] / 1000 * size
    cw, ch, gap = 110.0, 182.0, 18.0
    total = adv + gap + cw
    x0 = (S - total) / 2
    base = S / 2 - size / 2 + baseline_in_box(700, size)
    p, _ = text_paths("/", 700, size, x0, base, tracking=0)
    cx, cy0 = x0 + adv + gap, (S - ch) / 2
    body = (f'<rect width="{S}" height="{S}" rx="100" fill="{PAPER}"/>'
            + "".join(f'<path d="{d}" fill="{INK}"/>' for d in p)
            + f'<rect x="{cx:.2f}" y="{cy0:.2f}" width="{cw}" height="{ch}" fill="{INK}"/>'
            + axes(cx, cy0 + ch, cw, PAPER))
    return svg(S, S, body, "freedyn2")


def avatar():
    S = 512.0
    size = 150.0
    font = fonts[700]
    cm = font.getBestCmap()
    w = sum(font["hmtx"][cm[ord(c)]][0] for c in "fd2") / 1000 * size - 0.05 * size * 2
    base = S / 2 - size / 2 + baseline_in_box(700, size)
    p, _ = text_paths("fd2", 700, size, (S - w) / 2, base, tracking=-0.05)
    body = f'<rect width="{S}" height="{S}" fill="{CURSOR}"/>' + "".join(f'<path d="{d}" fill="{INK}"/>' for d in p)
    return svg(S, S, body, "freedyn2")


files = {
    "lockup-dark.svg": lockup("dark", True, False, True),
    "lockup-light.svg": lockup("light", True, False, True),
    "lockup-dark-transparent.svg": lockup("dark", True, False, False),
    "lockup-light-transparent.svg": lockup("light", True, False, False),
    "lockup-small-dark.svg": lockup("dark", False, False, True),
    "lockup-small-light.svg": lockup("light", False, False, True),
    "lockup-claim-dark.svg": lockup("dark", True, True, True),
    "lockup-claim-light.svg": lockup("light", True, True, True),
    "mark-dark.svg": mark("dark"),
    "mark-light.svg": mark("light"),
    "favicon.svg": favicon(),
    "app-icon.svg": app_icon(),
    "avatar.svg": avatar(),
}
for name, content in files.items():
    with open(os.path.join(OUT, name), "w") as f:
        f.write(content)
print("\n".join(files))
