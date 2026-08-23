#!/usr/bin/env python3
"""Build the lego-rl page from the raw hub telemetry in data/.

Writes a single self-contained HTML file (inline SVG, no assets) into the
strawrunway pages directory, which com.strawrunway.share serves privately at
lego-rl.strawrunway.com and publicly at strawrunway.com/lego-rl while a share
timer is live.

    .venv/bin/python scripts/build_page.py [--out DIR]
"""
from __future__ import annotations

import argparse
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


def read_csv(path: Path, ncol: int) -> list[list[int]]:
    """Rows of the hub's CSV dump; ignores the surrounding chatter."""
    pat = re.compile(r"^" + ",".join([r"(-?\d+)"] * ncol) + r"$")
    rows = []
    for line in path.read_text().splitlines():
        m = pat.match(line.strip())
        if m:
            rows.append([int(x) for x in m.groups()])
    return rows


def dominant_hz(vals: list[float], dt: float, lo=2.0, hi=45.0) -> float:
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


def spectrum(vals: list[float], dt: float, lo=1.0, hi=40.0, step=0.4):
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


def build(out_dir: Path) -> Path:
    charts: dict[str, str] = {}

    # --- run 1: unfiltered, the original wild shake -----------------------
    r1 = read_csv(DATA / "run1_unfiltered.log", 5)
    t1 = [r[0] / 1000 for r in r1]
    charts["run1"] = line_chart(
        [("pitch (deg)", list(zip(t1, [r[1] / 10 for r in r1]))),
         ("duty (%)", list(zip(t1, [r[3] for r in r1])))],
        title="Run 1 — as shipped: raw gyro term, hard friction step",
        xlabel="time (s)", ylabel="deg  /  % duty",
        bands=[(0, 1.16, "#a9d6a0", "held by hand"),
               (1.16, 5.0, "#e3a9a0", "released")],
    )
    charts["run1_zoom"] = line_chart(
        [("pitch (deg)", list(zip(t1, [r[1] / 10 for r in r1]))),
         ("duty (%)", list(zip(t1, [r[3] for r in r1])))],
        title="Run 1, 700 ms zoom — duty is a square wave slamming rail to rail",
        xlabel="time (s)", ylabel="deg  /  % duty", xlim=(1.25, 1.95),
        hlines=[(100, "#e3a9a0", "3 3"), (-100, "#e3a9a0", "3 3")],
        height=220,
    )
    osc1 = [r for r in r1 if r[0] > 1300]
    charts["spec1"] = line_chart(
        [("run 1 (raw gyro)", spectrum([r[1] / 10 for r in osc1], 0.02))],
        title="Pitch spectrum after release — where the energy sits",
        xlabel="frequency (Hz)", ylabel="amplitude (deg)", height=210,
        legend=True,
    )

    # --- run 3: filtered ---------------------------------------------------
    r3 = read_csv(DATA / "run3_filtered.log", 5)
    t3 = [r[0] / 1000 for r in r3]
    charts["run3"] = line_chart(
        [("pitch (deg)", list(zip(t3, [r[1] / 10 for r in r3]))),
         ("duty (%)", list(zip(t3, [r[3] for r in r3])))],
        title="Run 3 — 30 ms gyro low-pass + ramped friction term",
        xlabel="time (s)", ylabel="deg  /  % duty",
    )
    charts["drift"] = line_chart(
        [("run 1 — as shipped", list(zip(t1, [r[4] for r in r1]))),
         ("run 3 — filtered", list(zip(t3, [r[4] for r in r3])))],
        title="Wheel travel: the position loop starts doing its job",
        xlabel="time (s)", ylabel="wheel angle (deg)", height=210,
    )

    # --- term decomposition ------------------------------------------------
    ka, kr, km, ks = GAINS
    n = len(osc1)
    charts["terms"] = bar_chart(
        [("K_angle x pitch", round(sum(abs(ka * r[1] / 10) for r in osc1) / n, 1)),
         ("K_rate x gyro", round(sum(abs(kr * r[2]) for r in osc1) / n, 1)),
         ("K_wheel x angle", round(sum(abs(km * r[4]) for r in osc1) / n, 1))],
        title="Mean |contribution| to commanded duty, run 1 after release",
        xlabel="duty %  (the motor rail is 100)",
        colour=lambda label, v: "#e3a9a0" if v > 100 else "#9ec1de",
    )

    # --- gain sweep --------------------------------------------------------
    sweep = [("baseline 10.71/0.87", 7.4, 5.38, 40),
             ("half gains 5.36/0.44", 9.8, 6.30, 3),
             ("duty clamped to 40", 10.0, 4.66, 70),
             ("soft angle 5.36/0.87", 5.8, 17.02, 70)]
    charts["sweep_hz"] = bar_chart(
        [(nme, hz) for nme, hz, _, _ in sweep],
        title="Oscillation frequency barely moves across a 2x gain change",
        xlabel="Hz", colour="#c3a6d8",
    )
    charts["sweep_sat"] = bar_chart(
        [(nme, sat) for nme, _, _, sat in sweep],
        title="...and it persists at 3% saturation, so it is not bang-bang",
        xlabel="% of samples with duty on the rail",
        colour=lambda label, v: "#a9d6a0" if v < 10 else "#9ec1de",
    )

    stats = {
        "hz1": round(dominant_hz([r[1] / 10 for r in osc1], 0.02), 1),
        "hz3": round(dominant_hz([r[1] / 10 for r in r3 if r[0] > 1000], 0.02), 1),
        "peak1": max(abs(r[2]) for r in osc1),
        "peak3": max(abs(r[2]) for r in r3),
        # Net travel, not peak excursion, and normalised per second because the
        # runs are 5 s and 8 s. Wheel radius 37.5 mm.
        "cm1": round(abs(r1[-1][4] - r1[0][4]) * math.pi / 180 * 3.75, 1),
        "cm3": round(abs(r3[-1][4] - r3[0][4]) * math.pi / 180 * 3.75, 1),
        "cms1": round(abs(r1[-1][4] - r1[0][4]) * math.pi / 180 * 3.75
                      / (r1[-1][0] / 1000), 1),
        "cms3": round(abs(r3[-1][4] - r3[0][4]) * math.pi / 180 * 3.75
                      / (r3[-1][0] / 1000), 1),
    }

    html = PAGE.format(**charts, **stats)
    out_dir.mkdir(parents=True, exist_ok=True)
    dest = out_dir / "index.html"
    dest.write_text(html)
    return dest


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Teaching a LEGO set to stand up</title>
<meta name="description" content="Sim-to-real RL on a LEGO 42124 inverted pendulum: what the first hardware runs actually measured.">
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
  p {{ margin: .85rem 0; }}
  strong {{ color: var(--bright); font-weight: normal;
           border-bottom: 1px dotted var(--rule); }}
  ul {{ margin: .85rem 0 .85rem 1.3rem; }}
  li {{ margin: .3rem 0; }}
  code {{
    font-family: "SF Mono", "Menlo", "Consolas", monospace;
    font-size: .82em; background: var(--code-bg);
    padding: .1em .35em; border-radius: 3px; color: var(--bright);
  }}
  figure {{
    margin: 1.5rem 0; background: var(--code-bg);
    border: 1px solid var(--rule); border-radius: 4px;
    padding: .7rem .6rem .3rem;
  }}
  figcaption {{
    font-size: .82rem; color: var(--dim); font-style: italic;
    padding: .1rem .4rem .5rem; line-height: 1.5;
  }}
  table {{
    border-collapse: collapse; width: 100%; margin: 1.2rem 0;
    font-size: .88rem;
  }}
  th, td {{
    text-align: left; padding: .35rem .6rem;
    border-bottom: 1px solid var(--rule);
  }}
  th {{ color: var(--bright); font-weight: normal; font-variant: small-caps;
       letter-spacing: .05em; }}
  td.num {{ font-family: "SF Mono", Menlo, monospace; font-size: .84em; }}
  .good {{ color: var(--good); }}
  .bad {{ color: var(--bad); }}
  footer {{
    margin-top: 3rem; padding-top: 1rem; border-top: 1px solid var(--rule);
    font-size: .85rem; color: var(--dim); text-align: center;
  }}
</style>
</head>
<body>

<header>
  <h1>Teaching a LEGO set to stand up</h1>
  <div class="sub">what the first hardware runs actually measured</div>
</header>

<p>A LEGO Technic 42124 hoverboard rebuilt as a two-wheeled inverted pendulum:
Technic Hub, both L motors direct-driving a wheel each, no gears. The plan is
ordinary sim-to-real — measure the robot, build a MuJoCo model, train PPO,
deploy to the hub — with one twist that makes it self-checking. Near the
upright equilibrium the optimal policy <em>is</em> linear, and there is a
published four-gain controller for this class of robot. So the learned policy
gets linearised at equilibrium and its Jacobian compared against those gains.
Agreement validates the whole pipeline against a known answer.</p>

<p>This page is the boring part that comes first: the robot does not balance
yet, and the sensor logs explain why. Every number below came off the hub over
Bluetooth at 200 Hz.</p>

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

<h2>Run 1: the tuned gains, and a robot having a seizure</h2>

<p>The first second is calm because a hand is holding it. The moment the wheels
are free to react against the body, it detonates.</p>

<figure>{run1}
<figcaption>Pitch and commanded duty, 200 Hz control loop logged at 50 Hz.
Green band: held. Red band: released.</figcaption></figure>

<figure>{run1_zoom}
<figcaption>Zoomed to 700 ms. This is not a controller correcting a fall — it
is a square wave, pinned to +/-100% duty, reversing every couple of
samples.</figcaption></figure>

<figure>{spec1}
<figcaption>The energy is at {hz1} Hz. The robot's actual pendulum mode is
around 2 Hz, so almost none of this motion is the robot falling.</figcaption>
</figure>

<p>Decomposing the commanded duty into its three terms shows what is doing the
shouting:</p>

<figure>{terms}
<figcaption>The rate term alone averages more than the motor can deliver. The
gyro is driving the robot, not damping it.</figcaption></figure>

<p>The mechanism is that this robot is <strong>tiny and violently
over-motored</strong>. Its rotational inertia about the axle is roughly
0.0009 kg&middot;m&sup2;, and two L motors can deliver about 0.5 N&middot;m —
enough to angularly accelerate the body at some 30,000&deg;/s&sup2;. Any step
in duty slams the chassis, the gyro reads a huge rate, and 19 ms later the
controller answers with an equal step the other way.</p>

<p>Two things were injecting those steps. The rate gain is a pure
differentiator feeding a delayed loop. And the coulomb-friction compensation —
a fixed +/-10% duty added in the direction of travel, straight from the
reference design — is a hard 20-point discontinuity every time the command
crosses zero. On a body this light that step alone is worth about 130&deg;/s
of gyro reading within a single sample. It is a bang-bang oscillator by
construction.</p>

<h2>Run 3: filter the gyro, ramp the friction term</h2>

<p>A 30 ms low-pass on the rate term (leaving the ~2 Hz pendulum dynamics it
exists to damp), and the friction compensation ramped in linearly over
&plusmn;4% duty instead of stepped:</p>

<figure>{run3}
<figcaption>Same gains, same robot, both discontinuities removed.</figcaption>
</figure>

<table>
<tr><th>&nbsp;</th><th>run 1 — as shipped</th><th>run 3 — filtered</th></tr>
<tr><td>oscillation</td><td class="num">{hz1} Hz</td><td class="num">{hz3} Hz</td></tr>
<tr><td>peak gyro</td><td class="num bad">{peak1} &deg;/s</td><td class="num">{peak3} &deg;/s</td></tr>
<tr><td>net travel</td><td class="num bad">{cm1} cm ({cms1} cm/s)</td><td class="num good">{cm3} cm ({cms3} cm/s)</td></tr>
<tr><td>outcome</td><td class="bad">fell over at 5 s</td><td class="good">survived the full 8 s</td></tr>
</table>

<p class="sub" style="font-size:.85rem">Run 1 was 5 s and run 3 was 8 s, so
travel is given per second as well as in total.</p>

<figure>{drift}
<figcaption>The clearest sign of progress: run 1 drove itself off the desk,
run 3 stayed within a couple of centimetres.</figcaption></figure>

<p>Real improvement — and still a robot vibrating at &plusmn;12&deg;. Which
raised the obvious question: is it still saturating its way into a limit
cycle?</p>

<h2>The sweep that killed my explanation</h2>

<p>Every configuration change costs a human hold-and-release, so instead of
guessing one at a time the hub now runs a sweep inside a single launch: four
controller configurations, 2.5 s each, computing its own summary statistics on
the fly.</p>

<figure>{sweep_hz}
<figcaption>Halving every gain moves the frequency by 2 Hz. A control-loop
instability should shift much harder than that.</figcaption></figure>

<figure>{sweep_sat}
<figcaption>The decisive row is the green one: at half gains the duty is on the
rail only 3% of the time, and the robot oscillates <em>anyway</em> — slightly
worse, in fact.</figcaption></figure>

<p>A saturation-driven limit cycle goes away when you stop saturating. This one
does not, and its frequency is nearly independent of the gains. That points at
something whose frequency is set by the hardware rather than the software:
compliance somewhere between the hub — where the IMU lives — and the wheels,
so that the gyro is partly measuring the hub twisting on its own mounting
rather than the robot's true lean. Closing a loop around a sensor that is not
rigidly attached to the thing being controlled is a classic way to build an
oscillator, and no amount of gain tuning fixes it.</p>

<p>There is also a second feedback loop in this controller that none of the
logs above could see. The two wheels are kept in step by
<code>sync = 0.15 &times; (left angle &minus; right angle)</code> — a
proportional-only yaw controller, no damping, same 19 ms delay, on a body with
very little yaw inertia. Every run so far logged the <em>mean</em> wheel angle,
which cancels that channel exactly. It is being measured next.</p>

<h2>What this is really about</h2>

<p>None of this is what the project is for. It is a rehearsal — the same
measure/model/train/deploy loop, run end to end on a robot cheap enough to
drop, before the same pipeline points at a quadruped. The useful part is that
the failures are all showing up in the right order: the simulator caught the
textbook gains, the telemetry caught the discontinuities, and the sweep caught
my own explanation being wrong.</p>

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
