"""Minimal dependency-free SVG line charts.

matplotlib is not a dependency of this project and the strawrunway page
services are all stdlib-only, so the plots are hand-emitted SVG: no assets, no
build step, and the page stays a single self-contained file.
"""
from __future__ import annotations

import html

PALETTE = ["#9ec1de", "#e3a9a0", "#a9d6a0", "#d8c48a", "#c3a6d8", "#8fc4c0"]


def _nice(lo: float, hi: float, n: int = 5) -> list[float]:
    """Round tick values spanning [lo, hi]."""
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / n
    mag = 10 ** (len(str(int(abs(raw)))) - 1 if abs(raw) >= 1 else -1)
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if raw <= step:
            break
    start = step * (int(lo / step) - 1)
    ticks = []
    v = start
    while v <= hi + step * 0.5:
        if lo - step * 0.5 <= v <= hi + step * 0.5:
            ticks.append(round(v, 6))
        v += step
    return ticks


def line_chart(series, *, width=760, height=260, xlabel="", ylabel="",
               title="", xlim=None, ylim=None, bands=(), hlines=(),
               legend=True):
    """series: list of (label, [(x, y), ...]). Returns an <svg> string."""
    ml, mr, mt, mb = 54, 14, 26 if title else 10, 34
    pw, ph = width - ml - mr, height - mt - mb

    xs = [p[0] for _, pts in series for p in pts]
    ys = [p[1] for _, pts in series for p in pts]
    if not xs:
        return ""
    x0, x1 = xlim if xlim else (min(xs), max(xs))
    y0, y1 = ylim if ylim else (min(ys), max(ys))
    if y1 == y0:
        y0, y1 = y0 - 1, y1 + 1
    pad = (y1 - y0) * 0.08
    y0, y1 = y0 - pad, y1 + pad

    def sx(x):
        return ml + (x - x0) / (x1 - x0) * pw

    def sy(y):
        return mt + ph - (y - y0) / (y1 - y0) * ph

    out = [
        f'<svg viewBox="0 0 {width} {height}" width="100%" '
        f'preserveAspectRatio="xMidYMid meet" role="img" '
        f'aria-label="{html.escape(title or ylabel)}" '
        'style="font-family:\'SF Mono\',Menlo,monospace;font-size:10px">'
    ]
    if title:
        out.append(f'<text x="{ml}" y="14" fill="#ffffff" '
                   f'font-size="11.5">{html.escape(title)}</text>')

    for bx0, bx1, colour, label in bands:
        bx0c, bx1c = max(bx0, x0), min(bx1, x1)
        if bx1c <= bx0c:
            continue
        out.append(f'<rect x="{sx(bx0c):.1f}" y="{mt}" '
                   f'width="{sx(bx1c) - sx(bx0c):.1f}" height="{ph}" '
                   f'fill="{colour}" opacity="0.16"/>')
        if label:
            out.append(f'<text x="{sx(bx0c) + 5:.1f}" y="{mt + 12}" '
                       f'fill="#a8b0b8">{html.escape(label)}</text>')

    for ty in _nice(y0 + pad, y1 - pad):
        yy = sy(ty)
        if not (mt - 1 <= yy <= mt + ph + 1):
            continue
        out.append(f'<line x1="{ml}" y1="{yy:.1f}" x2="{ml + pw}" '
                   f'y2="{yy:.1f}" stroke="#5d666f" stroke-width="0.5" '
                   f'opacity="0.55"/>')
        out.append(f'<text x="{ml - 6}" y="{yy + 3.4:.1f}" fill="#a8b0b8" '
                   f'text-anchor="end">{ty:g}</text>')
    for tx in _nice(x0, x1):
        xx = sx(tx)
        if not (ml - 1 <= xx <= ml + pw + 1):
            continue
        out.append(f'<line x1="{xx:.1f}" y1="{mt + ph}" x2="{xx:.1f}" '
                   f'y2="{mt + ph + 4}" stroke="#5d666f" stroke-width="0.5"/>')
        out.append(f'<text x="{xx:.1f}" y="{mt + ph + 15}" fill="#a8b0b8" '
                   f'text-anchor="middle">{tx:g}</text>')

    for hy, colour, dash in hlines:
        if y0 <= hy <= y1:
            out.append(f'<line x1="{ml}" y1="{sy(hy):.1f}" x2="{ml + pw}" '
                       f'y2="{sy(hy):.1f}" stroke="{colour}" '
                       f'stroke-width="0.9" stroke-dasharray="{dash}" '
                       f'opacity="0.75"/>')

    out.append(f'<line x1="{ml}" y1="{mt}" x2="{ml}" y2="{mt + ph}" '
               f'stroke="#8b949e" stroke-width="0.8"/>')
    out.append(f'<line x1="{ml}" y1="{mt + ph}" x2="{ml + pw}" '
               f'y2="{mt + ph}" stroke="#8b949e" stroke-width="0.8"/>')

    for i, (label, pts) in enumerate(series):
        colour = PALETTE[i % len(PALETTE)]
        d = " ".join(
            ("M" if j == 0 else "L") + f"{sx(x):.1f},{sy(y):.1f}"
            for j, (x, y) in enumerate(pts) if x0 <= x <= x1
        )
        out.append(f'<path d="{d}" fill="none" stroke="{colour}" '
                   f'stroke-width="1.1" stroke-linejoin="round"/>')
        if legend and label:
            lx = ml + 8 + i * 118
            out.append(f'<line x1="{lx}" y1="{mt + 8}" x2="{lx + 14}" '
                       f'y2="{mt + 8}" stroke="{colour}" stroke-width="1.8"/>')
            out.append(f'<text x="{lx + 19}" y="{mt + 11.5}" fill="#dddddd">'
                       f'{html.escape(label)}</text>')

    if ylabel:
        out.append(f'<text x="13" y="{mt + ph / 2}" fill="#a8b0b8" '
                   f'text-anchor="middle" transform="rotate(-90 13 '
                   f'{mt + ph / 2})">{html.escape(ylabel)}</text>')
    if xlabel:
        out.append(f'<text x="{ml + pw / 2}" y="{height - 2}" fill="#a8b0b8" '
                   f'text-anchor="middle">{html.escape(xlabel)}</text>')
    out.append("</svg>")
    return "".join(out)


def bar_chart(rows, *, width=760, bar_h=22, xlabel="", title="", colour=None):
    """rows: list of (label, value). Horizontal bars."""
    ml, mr, mt, mb = 108, 46, 26 if title else 8, 26
    ph = bar_h * len(rows)
    height = mt + ph + mb
    pw = width - ml - mr
    vmax = max([abs(v) for _, v in rows] + [1e-9])

    out = [
        f'<svg viewBox="0 0 {width} {height}" width="100%" '
        f'preserveAspectRatio="xMidYMid meet" role="img" '
        f'aria-label="{html.escape(title or xlabel)}" '
        'style="font-family:\'SF Mono\',Menlo,monospace;font-size:10px">'
    ]
    if title:
        out.append(f'<text x="8" y="14" fill="#ffffff" font-size="11.5">'
                   f'{html.escape(title)}</text>')
    for i, (label, v) in enumerate(rows):
        y = mt + i * bar_h
        w = abs(v) / vmax * pw
        c = colour(label, v) if callable(colour) else (colour or PALETTE[0])
        out.append(f'<rect x="{ml}" y="{y + 3}" width="{w:.1f}" '
                   f'height="{bar_h - 8}" fill="{c}" opacity="0.85"/>')
        out.append(f'<text x="{ml - 7}" y="{y + bar_h / 2 + 2.5:.1f}" '
                   f'fill="#dddddd" text-anchor="end">'
                   f'{html.escape(label)}</text>')
        out.append(f'<text x="{ml + w + 6:.1f}" y="{y + bar_h / 2 + 2.5:.1f}" '
                   f'fill="#a8b0b8">{v:g}</text>')
    if xlabel:
        out.append(f'<text x="{ml + pw / 2}" y="{height - 6}" fill="#a8b0b8" '
                   f'text-anchor="middle">{html.escape(xlabel)}</text>')
    out.append("</svg>")
    return "".join(out)
