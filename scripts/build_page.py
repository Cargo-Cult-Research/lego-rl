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
import json
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plotlib import bar_chart, line_chart  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
DEFAULT_OUT = Path.home() / "code/housekeeping/strawrunway/pages/lego-rl"

GAINS = (10.71, 0.87, 0.43, 0.30)
BAND_COLOURS = ["#9ec1de", "#a9d6a0", "#c3a6d8", "#d8c48a", "#e3a9a0"]
VERDICTS = {
    "guilty": ("guilty", "#e3a9a0"),
    "ruled-out": ("ruled out", "#a9d6a0"),
    "progress": ("progress", "#9ec1de"),
    "void": ("no verdict", "#a8b0b8"),
    "open": ("open", "#d8c48a"),
}


# --------------------------------------------------------------------------
# data


def read_run(d: Path) -> dict | None:
    """One run directory -> {meta, notes, cols, rows}. None if unreadable."""
    meta_p, csv_p = d / "meta.json", d / "telemetry.csv"
    if not meta_p.exists() or not csv_p.exists():
        return None
    meta = json.loads(meta_p.read_text())
    lines = csv_p.read_text().splitlines()
    if not lines:
        return None
    cols = lines[0].split(",")
    pat = re.compile(r"^" + ",".join([r"-?\d+"] * len(cols)) + r"$")
    rows = [[int(x) for x in l.split(",")] for l in lines[1:] if pat.match(l.strip())]
    notes_p = d / "notes.md"
    return {
        "dir": d.name,
        "meta": meta,
        "notes": notes_p.read_text() if notes_p.exists() else "",
        "cols": cols,
        "rows": rows,
    }


def load_runs() -> list[dict]:
    runs = []
    for d in sorted(DATA.glob("run_*")):
        if not d.is_dir():
            continue
        r = read_run(d)
        if r is None:
            print(f"  skipping {d.name}: missing meta.json or telemetry.csv")
            continue
        runs.append(r)
    runs.sort(key=lambda r: r["meta"].get("n", 0))
    return runs


# --------------------------------------------------------------------------
# analysis


def dominant_hz(vals, dt, lo=2.0, hi=45.0) -> float:
    n = len(vals)
    if n < 20:
        return 0.0
    mean = sum(vals) / n
    v = [x - mean for x in vals]
    best, bf = 0.0, 0.0
    f = lo
    while f <= hi:
        re_ = sum(x * math.cos(2 * math.pi * f * i * dt) for i, x in enumerate(v))
        im_ = sum(x * math.sin(2 * math.pi * f * i * dt) for i, x in enumerate(v))
        a = math.hypot(re_, im_) / n
        if a > best:
            best, bf = a, f
        f += 0.2
    return bf


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
    out = [f'<figure>{line_chart(series, title=meta["title"], xlabel="time (s)", ylabel="", bands=run_bands(run), height=250)}'
           f'<figcaption>{html.escape(meta.get("script", ""))} '
           f'&middot; {len(rows)} samples at 50 Hz'
           f'</figcaption></figure>']

    for spec in meta.get("extra_charts", []):
        if spec.startswith("spectrum:"):
            _, col, scale = spec.split(":")
            if col not in cols:
                continue
            ci = cols.index(col)
            sub = [r for r in rows if r[ti] > 1300] or rows
            out.append(
                f'<figure>{line_chart([("pitch", spectrum([r[ci] * float(scale) for r in sub], 0.02))], title="Pitch spectrum — where the energy sits", xlabel="frequency (Hz)", ylabel="amplitude (deg)", height=200)}'
                f'<figcaption>The robot\'s own pendulum mode is around 2 Hz, so '
                f'almost none of this motion is the robot falling.</figcaption></figure>')
        elif spec == "terms" and {"pitch_x10", "rate_dps", "wheel_deg"} <= set(cols):
            ka, kr, km, _ = GAINS
            pi_, ri, wi = (cols.index(c) for c in ("pitch_x10", "rate_dps", "wheel_deg"))
            sub = [r for r in rows if r[ti] > 1300] or rows
            n = len(sub)
            out.append(
                f'<figure>{bar_chart([("K_angle x pitch", round(sum(abs(ka * r[pi_] / 10) for r in sub) / n, 1)), ("K_rate x gyro", round(sum(abs(kr * r[ri]) for r in sub) / n, 1)), ("K_wheel x angle", round(sum(abs(km * r[wi]) for r in sub) / n, 1))], title="Mean |contribution| to commanded duty", xlabel="duty %  (the motor rail is 100)", colour=lambda l, v: "#e3a9a0" if v > 100 else "#9ec1de")}'
                f'<figcaption>The rate term alone averages more than the motor '
                f'can deliver.</figcaption></figure>')
    return "\n".join(out)


def run_entry(run: dict) -> str:
    meta = run["meta"]
    label, colour = VERDICTS.get(meta.get("verdict", "open"), VERDICTS["open"])
    hub = "\n".join(html.escape(l) for l in meta.get("hub_output", []))
    hub_block = (f'<details><summary>hub output</summary><pre><code>{hub}</code></pre></details>'
                 if hub else "")
    return f"""
<article class="run" id="{html.escape(run['dir'])}">
  <div class="runhead">
    <span class="runno">run {meta.get('n', '?')}</span>
    <span class="verdict" style="border-color:{colour};color:{colour}">{label}</span>
    <span class="rundate">{html.escape(meta.get('date', ''))}</span>
  </div>
  <h3>{html.escape(meta['title'])}</h3>
  <p class="question"><em>{html.escape(meta.get('question', ''))}</em></p>
  <p class="headline">{html.escape(meta.get('headline', ''))}</p>
  {md_to_html(run['notes'])}
  {run_charts(run)}
  {hub_block}
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
    by_n = {r["meta"].get("n"): r for r in runs}

    stats = {"hz1": "?", "hz3": "?", "peak1": "?", "peak3": "?",
             "cm1": "?", "cm3": "?", "nruns": len(runs)}
    r1, r3 = by_n.get(1), by_n.get(3)
    if r1 and r1["rows"]:
        c, rows = r1["cols"], r1["rows"]
        pi_, ri, wi, ti = (c.index(x) for x in ("pitch_x10", "rate_dps", "wheel_deg", "t_ms"))
        osc = [r for r in rows if r[ti] > 1300]
        stats["hz1"] = round(dominant_hz([r[pi_] / 10 for r in osc], 0.02), 1)
        stats["peak1"] = max(abs(r[ri]) for r in osc)
        stats["cm1"] = round(abs(rows[-1][wi] - rows[0][wi]) * math.pi / 180 * 3.75, 1)
    if r3 and r3["rows"]:
        c, rows = r3["cols"], r3["rows"]
        pi_, ri, wi, ti = (c.index(x) for x in ("pitch_x10", "rate_dps", "wheel_deg", "t_ms"))
        stats["hz3"] = round(dominant_hz([r[pi_] / 10 for r in rows if r[ti] > 1000], 0.02), 1)
        stats["peak3"] = max(abs(r[ri]) for r in rows)
        stats["cm3"] = round(abs(rows[-1][wi] - rows[0][wi]) * math.pi / 180 * 3.75, 1)

    html_out = PAGE.format(
        labbook="\n".join(run_entry(r) for r in runs),
        summary=summary_table(runs),
        **stats,
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    dest = out_dir / "index.html"
    dest.write_text(html_out)
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
  <div class="sub">a lab book, kept while the robot refuses to</div>
</header>

<p>A LEGO Technic 42124 hoverboard rebuilt as a two-wheeled inverted pendulum:
Technic Hub, both L motors direct-driving a wheel each, no gears. The plan is
ordinary sim-to-real — measure the robot, build a MuJoCo model, train PPO,
deploy to the hub — with one twist that makes it self-checking. Near the
upright equilibrium the optimal policy <em>is</em> linear, and there is a
published four-gain controller for this class of robot. So the learned policy
gets linearised at equilibrium and its Jacobian compared against those gains.
Agreement validates the whole pipeline against a known answer.</p>

<p>The robot does not balance yet. What follows is every hardware run so far,
in order, including the ones that produced nothing and the hypotheses that
turned out to be wrong. Each entry is generated from its own telemetry —
{nruns} runs, all captured off the hub over Bluetooth at 200 Hz.</p>

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

<h2>Where it stands</h2>

<table>
<tr><th>&nbsp;</th><th>run 1 — as shipped</th><th>run 3 — filtered</th></tr>
<tr><td>oscillation</td><td class="num">{hz1} Hz</td><td class="num">{hz3} Hz</td></tr>
<tr><td>peak gyro</td><td class="num bad">{peak1} &deg;/s</td><td class="num">{peak3} &deg;/s</td></tr>
<tr><td>net travel</td><td class="num bad">{cm1} cm</td><td class="num good">{cm3} cm</td></tr>
<tr><td>outcome</td><td class="bad">fell over</td><td class="good">survived the full run</td></tr>
</table>

<p>The oscillation has now proved indifferent to gain magnitude, to a duty
clamp, to the yaw loop, to the wheel-speed term and its filtering, and to the
friction compensation's slope. What remains on the list is the gyro filter
itself — the one parameter whose changes have visibly moved the frequency —
and mechanical compliance between the hub, where the IMU lives, and the
wheels. Closing a loop around a sensor that is not rigidly attached to the
thing being controlled is a classic way to build an oscillator, and no amount
of gain tuning fixes it.</p>

<h2>What this is really about</h2>

<p>None of this is what the project is for. It is a rehearsal — the same
measure/model/train/deploy loop, run end to end on a robot cheap enough to
drop, before the same pipeline points at a quadruped. The useful part is that
the failures keep showing up in the right order: the simulator caught the
textbook gains, the telemetry caught the discontinuities, and the sweeps have
now caught three of my own explanations being wrong.</p>

<footer>
Two L motors, one Technic Hub, Pybricks 3.6.1 &middot; MuJoCo + PPO on an
M3 Ultra &middot; telemetry at 200 Hz over BLE
</footer>

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
