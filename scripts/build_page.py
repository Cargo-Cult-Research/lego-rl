#!/usr/bin/env python3
"""Build the lego-rl page from the run directories in data/.

The middle of the page is a lab book that grows itself: every `data/run_NN_*/`
directory becomes an entry, in order, with its own charts rendered from its
telemetry. Adding a run is adding a directory — no edit to this file.

See CLAUDE.md ("Recording a run") for the directory contract.

    .venv/bin/python scripts/build_page.py [--out DIR]

Writes one self-contained HTML file (inline SVG, no assets) into the
strawrunway pages directory, which com.strawrunway.share serves privately at
lego-rl.strawrunway.com and publicly at strawrunway.com/lego-rl while a share
timer is live.
"""
from __future__ import annotations

import argparse
import html
import importlib.util
import json
import math
import os
import re
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plotlib import bar_chart, line_chart  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
DEFAULT_OUT = Path.home() / "code/housekeeping/strawrunway/pages/lego-rl"


def _load_gains():
    """The gains come from robot/gains.py — the single source — via importlib
    so the page builder stays stdlib-only."""
    spec = importlib.util.spec_from_file_location("_g", ROOT / "robot" / "gains.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return tuple(mod.GAINS_SIM_TUNED)


GAINS = _load_gains()
MEDIA_VIDEO = {".mp4", ".webm", ".mov"}
MEDIA_IMAGE = {".png", ".jpg", ".jpeg", ".gif", ".svg"}
STANDARD = {"meta.json", "notes.md", "telemetry.csv"}
BAND_COLOURS = ["#9ec1de", "#a9d6a0", "#c3a6d8", "#d8c48a", "#e3a9a0"]
VERDICTS = {
    "guilty": ("guilty", "#e3a9a0"),
    "ruled-out": ("ruled out", "#a9d6a0"),
    "progress": ("progress", "#9ec1de"),
    "inconclusive": ("inconclusive", "#d8c48a"),
    "void": ("no data", "#a8b0b8"),
    "open": ("open", "#d8c48a"),
}


# --------------------------------------------------------------------------
# data


class RunError(Exception):
    """A malformed run directory. The build FAILS on these rather than
    skipping: run 22 — the project's best result — sat invisible on the
    published page for a day because the old code skipped it with a print
    into a log nobody reads."""


def read_run(d: Path) -> dict:
    """One run directory -> {meta, notes, cols, rows, media, files}.

    Contract: meta.json and notes.md are REQUIRED; telemetry.csv is optional
    (not every run yields a time series — see run 22). Any other file in the
    directory is published alongside the page: video/images render inline,
    everything else becomes a download link.
    """
    meta_p, csv_p, notes_p = d / "meta.json", d / "telemetry.csv", d / "notes.md"
    if not meta_p.exists():
        raise RunError(f"{d.name}: no meta.json (record_run.py writes it; "
                       "for a hand-made run, copy one and edit)")
    if not notes_p.exists():
        raise RunError(f"{d.name}: no notes.md — the prose is not optional")
    try:
        meta = json.loads(meta_p.read_text())
    except json.JSONDecodeError as e:
        raise RunError(f"{d.name}: meta.json does not parse: {e}")
    for field in ("n", "title", "verdict"):
        if field not in meta:
            raise RunError(f"{d.name}: meta.json is missing '{field}'")
    if meta["verdict"] not in VERDICTS:
        raise RunError(f"{d.name}: unknown verdict {meta['verdict']!r} "
                       f"(one of {sorted(VERDICTS)})")

    cols, rows = [], []
    if csv_p.exists():
        lines = csv_p.read_text().splitlines()
        if not lines:
            raise RunError(f"{d.name}: telemetry.csv is empty — delete it "
                           "or record something")
        cols = lines[0].split(",")
        pat = re.compile(r"^" + ",".join([r"-?\d+"] * len(cols)) + r"$")
        bad = [i + 2 for i, l in enumerate(lines[1:]) if l.strip() and not pat.match(l.strip())]
        if bad:
            raise RunError(f"{d.name}: telemetry.csv has {len(bad)} rows that "
                           f"are not all-integer CSV (first at line {bad[0]})")
        rows = [[int(x) for x in l.split(",")] for l in lines[1:] if l.strip()]

    media, files = [], []
    for f in sorted(d.iterdir()):
        if f.name in STANDARD or f.name.startswith("."):
            continue
        (media if f.suffix.lower() in MEDIA_VIDEO | MEDIA_IMAGE else files).append(f)
    if csv_p.exists():
        files.insert(0, csv_p)   # raw data is part of the record — link it

    return {
        "dir": d.name,
        "meta": meta,
        "notes": notes_p.read_text(),
        "cols": cols,
        "rows": rows,
        "media": media,
        "files": files,
    }


def load_runs() -> list[dict]:
    runs, errors = [], []
    for d in sorted(DATA.glob("run_*")):
        if not d.is_dir():
            continue
        try:
            runs.append(read_run(d))
        except RunError as e:
            errors.append(str(e))
    seen = {}
    for r in runs:
        n = r["meta"]["n"]
        if n in seen:
            errors.append(f"duplicate run number {n}: {seen[n]} and {r['dir']}")
        seen[n] = r["dir"]
    if seen:
        missing = sorted(set(range(1, max(seen) + 1)) - set(seen))
        if missing:
            errors.append(f"gap in run numbering: missing {missing} — every "
                          "hardware run gets a directory, no exceptions "
                          "(CLAUDE.md); retro-file it, even as verdict=void")
    if errors:
        for e in errors:
            print(f"BUILD FAILED: {e}", file=sys.stderr)
        raise SystemExit(1)
    runs.sort(key=lambda r: r["meta"]["n"])
    return runs


# --------------------------------------------------------------------------
# analysis


def spectrum(vals, dt, lo=1.0, hi=40.0, step=0.4):
    n = len(vals)
    mean = sum(vals) / n
    v = [x - mean for x in vals]
    pts = []
    f = lo
    while f <= hi:
        re_ = sum(x * math.cos(2 * math.pi * f * i * dt) for i, x in enumerate(v))
        im_ = sum(x * math.sin(2 * math.pi * f * i * dt) for i, x in enumerate(v))
        pts.append((f, math.hypot(re_, im_) / n * 2))
        f += step
    return pts


def md_to_html(text: str) -> str:
    """Paragraphs, `code`, **bold**, *italic*. Deliberately tiny."""
    out = []
    for para in re.split(r"\n\s*\n", text.strip()):
        p = html.escape(para.strip())
        p = re.sub(r"`([^`]+)`", r"<code>\1</code>", p)
        p = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", p)
        p = re.sub(r"(?<!\*)\*([^*]+)\*(?!\*)", r"<em>\1</em>", p)
        p = p.replace("\n", " ")
        if p:
            out.append(f"<p>{p}</p>")
    return "\n".join(out)


# --------------------------------------------------------------------------
# rendering


def run_bands(run: dict) -> list:
    """Coloured time bands: explicit `segments`, or derived from a `seg` column."""
    meta, cols, rows = run["meta"], run["cols"], run["rows"]
    if meta.get("segments"):
        return [(a, b, BAND_COLOURS[i % len(BAND_COLOURS)], lbl)
                for i, (a, b, lbl) in enumerate(meta["segments"])]
    labels = meta.get("segment_labels")
    if not labels or "seg" not in cols:
        return []
    ti, si = cols.index("t_ms"), cols.index("seg")
    bands, cur, start = [], None, 0.0
    for r in rows:
        if r[si] != cur:
            if cur is not None and cur < len(labels):
                bands.append((start, r[ti] / 1000,
                              BAND_COLOURS[cur % len(BAND_COLOURS)], labels[cur]))
            cur, start = r[si], r[ti] / 1000
    if cur is not None and cur < len(labels):
        bands.append((start, rows[-1][ti] / 1000,
                      BAND_COLOURS[cur % len(BAND_COLOURS)], labels[cur]))
    return bands


def run_charts(run: dict) -> str:
    meta, cols, rows = run["meta"], run["cols"], run["rows"]
    if not rows:
        return ""
    if "t_ms" not in cols:
        return ""          # not a time series; nothing to plot
    ti = cols.index("t_ms")
    t = [r[ti] / 1000 for r in rows]
    series = []
    for name, label, scale in meta.get("series", []):
        if name not in cols:
            continue
        ci = cols.index(name)
        series.append((label, list(zip(t, [r[ci] * scale for r in rows]))))
    if not series:
        return ""
    # Sample rate varies by run (50 Hz for the balancing sweeps, 200 Hz for the
    # open-loop ring test), so derive it rather than asserting it.
    steps = sorted({rows[i + 1][ti] - rows[i][ti] for i in range(min(40, len(rows) - 1))})
    dt_ms = steps[len(steps) // 2] if steps else 20
    rate = round(1000 / dt_ms) if dt_ms else 0
    out = [f'<figure>{line_chart(series, title=meta["title"], xlabel="time (s)", ylabel="", bands=run_bands(run), height=250)}'
           f'<figcaption>{html.escape(meta.get("script", ""))} '
           f'&middot; {len(rows)} samples at {rate} Hz'
           f'</figcaption></figure>']

    for spec in meta.get("extra_charts", []):
        if spec.startswith("spectrum:"):
            _, col, scale = spec.split(":")
            if col not in cols:
                continue
            ci = cols.index(col)
            sub = [r for r in rows if r[ti] > 1300] or rows
            # dt comes from the data (derived above) — an earlier version
            # hardcoded 0.02 s and misread every 200 Hz spectrum by 4x.
            out.append(
                f'<figure>{line_chart([("pitch", spectrum([r[ci] * float(scale) for r in sub], dt_ms / 1000.0))], title="Pitch spectrum — where the energy sits", xlabel="frequency (Hz)", ylabel="amplitude (deg)", height=200)}'
                f'<figcaption>Amplitude spectrum of the samples after the '
                f'hand-held first 1.3 s, at the run\'s own {rate} Hz sample '
                f'rate.</figcaption></figure>')
        elif spec == "terms" and {"pitch_x10", "rate_dps", "wheel_deg"} <= set(cols):
            ka, kr, km, _ = GAINS
            pi_, ri, wi = (cols.index(c) for c in ("pitch_x10", "rate_dps", "wheel_deg"))
            sub = [r for r in rows if r[ti] > 1300] or rows
            n = len(sub)
            out.append(
                f'<figure>{bar_chart([("K_angle x pitch", round(sum(abs(ka * r[pi_] / 10) for r in sub) / n, 1)), ("K_rate x gyro", round(sum(abs(kr * r[ri]) for r in sub) / n, 1)), ("K_wheel x angle", round(sum(abs(km * r[wi]) for r in sub) / n, 1))], title="Mean |contribution| to commanded duty", xlabel="duty %  (the motor rail is 100)", colour=lambda l, v: "#e3a9a0" if v > 100 else "#9ec1de")}'
                f'<figcaption>Mean absolute contribution of each feedback term '
                f'to the commanded duty, current gains from robot/gains.py.'
                f'</figcaption></figure>')
    return "\n".join(out)


def run_media(run: dict) -> str:
    """Inline video/image elements plus plain links for other files.

    Media are separate files copied next to index.html — never embedded in
    the page — so the page itself stays small; a video is only fetched when
    somebody presses play (preload="none").
    """
    meta = run["meta"]
    captions = meta.get("media", {})
    out = []
    for f in run["media"]:
        rel = f"media/{run['dir']}/{f.name}"
        cap = html.escape(captions.get(f.name, f.name))
        if f.suffix.lower() in MEDIA_VIDEO:
            out.append(
                f'<figure><video controls preload="none" src="{rel}"></video>'
                f'<figcaption>{cap} · <a href="{rel}">download</a>'
                f'</figcaption></figure>')
        else:
            out.append(
                f'<figure><img loading="lazy" src="{rel}" alt="{cap}">'
                f'<figcaption>{cap}</figcaption></figure>')
    if run["files"]:
        links = " · ".join(
            f'<a href="media/{run["dir"]}/{f.name}">{html.escape(f.name)}</a>'
            for f in run["files"])
        out.append(f'<p class="runfiles">files: {links}</p>')
    return "\n".join(out)


def run_entry(run: dict, open_by_default: bool = False) -> str:
    """One <article>, collapsed to head + title + headline by default —
    32 fully-expanded runs made the page a scrolling exercise. The newest
    run ships open; anchor links from the summary table open their target
    via the small script in PAGE."""
    meta = run["meta"]
    label, colour = VERDICTS.get(meta.get("verdict", "open"), VERDICTS["open"])
    hub = "\n".join(html.escape(l) for l in meta.get("hub_output", []))
    hub_block = (f'<details><summary>hub output</summary><pre><code>{hub}</code></pre></details>'
                 if hub else "")
    return f"""
<article class="run" id="{html.escape(run['dir'])}">
  <details class="rundetails"{" open" if open_by_default else ""}>
  <summary>
    <div class="runhead">
      <span class="runno">run {meta.get('n', '?')}</span>
      <span class="verdict" style="border-color:{colour};color:{colour}">{label}</span>
      <span class="rundate">{html.escape(meta.get('date', ''))}</span>
    </div>
    <h3>{html.escape(meta['title'])}</h3>
    <p class="headline">{html.escape(meta.get('headline', ''))}</p>
  </summary>
  <p class="question"><em>{html.escape(meta.get('question', ''))}</em></p>
  {md_to_html(run['notes'])}
  {run_charts(run)}
  {run_media(run)}
  {hub_block}
  </details>
</article>"""


def summary_table(runs: list[dict]) -> str:
    rows = []
    for r in runs:
        m = r["meta"]
        label, colour = VERDICTS.get(m.get("verdict", "open"), VERDICTS["open"])
        rows.append(
            f'<tr><td class="num">{m.get("n","?")}</td>'
            f'<td><a href="#{html.escape(r["dir"])}">{html.escape(m["title"])}</a></td>'
            f'<td style="color:{colour}">{label}</td></tr>')
    return ("<table class='summary'><tr><th>#</th><th>run</th><th>verdict</th></tr>"
            + "".join(rows) + "</table>")


def build(out_dir: Path) -> Path:
    runs = load_runs()
    print(f"  {len(runs)} runs")

    newest = max(r["meta"]["n"] for r in runs) if runs else None
    html_out = PAGE.format(
        labbook="\n".join(run_entry(r, open_by_default=(r["meta"]["n"] == newest))
                          for r in runs),
        summary=summary_table(runs),
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    # Media and data files ride ALONGSIDE the page, never inside it: the page
    # stays one small file, and a video costs nothing until played.
    for run in runs:
        srcs = run["media"] + run["files"]
        if not srcs:
            continue
        mdir = out_dir / "media" / run["dir"]
        mdir.mkdir(parents=True, exist_ok=True)
        for f in srcs:
            dst = mdir / f.name
            if not dst.exists() or f.stat().st_mtime > dst.stat().st_mtime:
                shutil.copy2(f, dst)

    # Atomic replace: the deploy wrapper promises "stale, never broken", so a
    # crash mid-write must not leave a truncated page.
    dest = out_dir / "index.html"
    tmp = out_dir / ".index.html.tmp"
    tmp.write_text(html_out)
    os.replace(tmp, dest)
    return dest


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Teaching a LEGO set to stand up</title>
<meta name="description" content="Sim-to-real RL on a LEGO 42124 inverted pendulum: a lab book of what the hardware runs actually measured.">
<meta name="robots" content="noindex, nofollow">
<style>
  :root {{
    --bg: #4a525a; --ink: #dddddd; --bright: #ffffff;
    --dim: #a8b0b8; --link: #9ec1de; --rule: #5d666f;
    --code-bg: #3e454c; --good: #a9d6a0; --bad: #e3a9a0;
  }}
  * {{ margin: 0; box-sizing: border-box; }}
  body {{
    background: var(--bg); color: var(--ink);
    font-family: Scala, "FF Scala", "Palatino Linotype", Palatino, Georgia, serif;
    font-size: 17px; line-height: 1.65;
    max-width: 46em; margin: 0 auto; padding: 8vh 1.3rem 5rem;
  }}
  header {{ text-align: center; margin-bottom: 2.4rem; }}
  h1 {{
    color: var(--bright); font-size: 1.55rem; font-weight: normal;
    font-variant: small-caps; letter-spacing: .12em; line-height: 1.3;
  }}
  .sub {{ color: var(--dim); font-style: italic; margin-top: .35rem; }}
  a {{ color: var(--link); text-decoration: underline; text-underline-offset: 2px; }}
  a:hover {{ color: var(--bright); }}
  h2 {{
    color: var(--bright); font-size: 1.12rem; font-weight: normal;
    font-variant: small-caps; letter-spacing: .08em;
    margin: 2.4rem 0 .6rem; padding-bottom: .2rem;
    border-bottom: 1px solid var(--rule);
  }}
  h3 {{ color: var(--bright); font-size: 1.04rem; font-weight: normal;
       font-style: italic; margin: .3rem 0 .4rem; }}
  p {{ margin: .85rem 0; }}
  strong {{ color: var(--bright); font-weight: normal;
           border-bottom: 1px dotted var(--rule); }}
  code {{
    font-family: "SF Mono", "Menlo", "Consolas", monospace;
    font-size: .82em; background: var(--code-bg);
    padding: .1em .35em; border-radius: 3px; color: var(--bright);
  }}
  pre {{ background: #363c42; border-left: 2px solid var(--rule);
        padding: .7rem .9rem; margin: .8rem 0; border-radius: 3px;
        overflow-x: auto; }}
  pre code {{ background: none; padding: 0; font-size: .74rem; line-height: 1.5; }}
  figure {{
    margin: 1.3rem 0; background: var(--code-bg);
    border: 1px solid var(--rule); border-radius: 4px;
    padding: .7rem .6rem .3rem;
  }}
  figcaption {{
    font-size: .8rem; color: var(--dim); font-style: italic;
    padding: .1rem .4rem .5rem; line-height: 1.5;
  }}
  figure video, figure img {{ width: 100%; border-radius: 3px; display: block; }}
  .runfiles {{ font-size: .78rem; color: var(--dim); }}
  table {{ border-collapse: collapse; width: 100%; margin: 1.2rem 0;
          font-size: .88rem; }}
  th, td {{ text-align: left; padding: .35rem .6rem;
           border-bottom: 1px solid var(--rule); }}
  th {{ color: var(--bright); font-weight: normal; font-variant: small-caps;
       letter-spacing: .05em; }}
  td.num {{ font-family: "SF Mono", Menlo, monospace; font-size: .84em; }}
  .good {{ color: var(--good); }}
  .bad {{ color: var(--bad); }}
  .summary td:first-child {{ width: 2em; color: var(--dim); }}
  article.run {{
    margin: 2rem 0; padding: 1rem 1.1rem .4rem;
    background: rgba(0,0,0,.11); border: 1px solid var(--rule);
    border-radius: 5px;
  }}
  details.rundetails > summary {{
    cursor: pointer; list-style: none; margin: -.2rem 0;
  }}
  details.rundetails > summary::-webkit-details-marker {{ display: none; }}
  details.rundetails > summary .runno::before {{
    content: "▸ "; color: var(--dim);
  }}
  details.rundetails[open] > summary .runno::before {{ content: "▾ "; }}
  details.rundetails > summary:hover h3 {{ color: var(--link); }}
  .runhead {{ display: flex; gap: .6rem; align-items: center;
             flex-wrap: wrap; margin-bottom: .1rem; }}
  .runno {{ font-family: "SF Mono", Menlo, monospace; font-size: .74rem;
           color: var(--dim); letter-spacing: .08em; text-transform: uppercase; }}
  .verdict {{ font-family: "SF Mono", Menlo, monospace; font-size: .68rem;
             border: 1px solid; border-radius: 3px; padding: .05rem .4rem;
             letter-spacing: .06em; text-transform: uppercase; }}
  .rundate {{ font-size: .74rem; color: var(--dim); margin-left: auto; }}
  .question {{ color: var(--dim); margin: .1rem 0 .5rem; }}
  .headline {{ color: var(--bright); }}
  details {{ margin: .8rem 0 .4rem; }}
  summary {{ cursor: pointer; font-size: .8rem; color: var(--dim);
            font-variant: small-caps; letter-spacing: .06em; }}
  footer {{
    margin-top: 3rem; padding-top: 1rem; border-top: 1px solid var(--rule);
    font-size: .85rem; color: var(--dim); text-align: center;
  }}
</style>
</head>
<body>

<header>
  <h1>Teaching a LEGO set to stand up</h1>
  <div class="sub">a lab book of what the hardware runs actually measured</div>
</header>

<p>A LEGO Technic 42124 hoverboard rebuilt as a two-wheeled inverted pendulum:
Technic Hub, both L motors direct-driving a wheel each, no gears. The plan is
ordinary sim-to-real — measure the robot, build a MuJoCo model, train PPO,
deploy to the hub — with one twist that makes it self-checking. Near the
upright equilibrium the optimal policy <em>is</em> linear, and there is a
published four-gain controller for this class of robot. So the learned policy
gets linearised at equilibrium and its Jacobian compared against those gains.
Agreement validates the whole pipeline against a known answer.</p>

<p>What follows is every hardware run, in order — including the ones that
produced nothing and the hypotheses that turned out to be wrong. Each entry
is generated from its own run directory: the telemetry, the question the run
was meant to settle, and what it actually showed. The page has no hand-written
status section, deliberately: anything like "where it stands" goes stale the
moment the next run lands, so the runs speak in order and the newest entry is
the current state.</p>

<h2>The first surprise came before the hardware</h2>

<p>The reference gains for this controller are
<code>(88, 0.35, 0.72, 0.19)</code> — duty percent per degree of lean, per
degree/second of lean rate, per degree of wheel angle, per degree/second of
wheel speed. Dropping the <em>measured</em> robot into the simulator — 410 g,
centre of mass only 50 mm above the axle, 19 ms of actuation delay — those
gains balanced for a mean of 4.6 s and survived none of the full episodes.</p>

<p>Retuning in the measured simulator gave <code>(10.71, 0.87, 0.43, 0.30)</code>:
<strong>eight times less angle stiffness and 2.5 times more rate damping</strong>.
That is the right direction for a robot this short and light, whose natural
falling time constant is about 70 ms. On hardware, the published gains fell in
0.755 s — the simulator disagreed with the textbook answer before the hardware
did, and the simulator was right.</p>

<h2>Why it shakes: the physics that makes this hard</h2>

<p>This robot is <strong>tiny and violently over-motored</strong>. Its
rotational inertia about the axle is roughly 0.0009 kg&middot;m&sup2;, and two
L motors deliver about 0.5 N&middot;m — enough to angularly accelerate the body
at some 30,000&deg;/s&sup2;. Any step in commanded duty slams the chassis, the
gyro reads a huge rate, and 19 ms later the controller answers with an equal
step the other way. Everything below is a consequence of that.</p>

{summary}

<h2>Lab book</h2>

{labbook}

<h2>What this is really about</h2>

<p>None of this is what the project is for. It is a rehearsal — the same
measure/model/train/deploy loop, run end to end on a robot cheap enough to
drop, before the same pipeline points at a quadruped. The useful part is that
the failures keep showing up in the right order: each layer's instruments
have caught the layer above being wrong, starting with the simulator catching
the textbook gains.</p>

<footer>
Two L motors, one Technic Hub &middot; MuJoCo + PPO &middot; telemetry over BLE
</footer>

<script>
/* Runs are collapsed by default; a summary-table link must still land on an
   OPEN run. No other JS on this page. */
function openTarget() {{
  if (!location.hash) return;
  var el = document.querySelector(location.hash + " > details.rundetails");
  if (el) el.open = true;
}}
window.addEventListener("hashchange", openTarget);
openTarget();
</script>

</body>
</html>
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    dest = build(args.out)
    print("wrote", dest, f"({dest.stat().st_size / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
