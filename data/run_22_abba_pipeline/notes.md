# Run 22 — ABBA crossover, classical direct vs the SAME law through the export pipeline

2026-08-24. Hardware, one continuous 120 s balance, 20 segments, **zero falls**,
200 Hz held throughout. Program: `experimental/ab/_ab_cycle.py` built with `--pipeline`
(KINDS = (0, 1); the learned policy is not in this build).

- kind 0 = classical gains evaluated directly on the hub
- kind 1 = the identical law exported into the net and quantised to Q12

`MAX_DUTY = 100` for the first time, so nothing is clipped: `clamp_pct = 0` in
all 20 segments. The duty the law *asks for* is now measured, not truncated.

## Result

Paired by cycle, on **sigma** (RMS about the segment mean — the drift-immune
statistic added for this run):

| cycle | order | k0 σ° | k1 σ° | k0 dmax | k1 dmax |
|---|---|---|---|---|---|
| 0 | 0→1 | 2.81 | 2.06 | 117% | 87% |
| 1 | 1→0 | 1.21 | 1.96 | 55% | 84% |
| 2 | 0→1 | 1.12 | 1.91 | 60% | 85% |
| 3 | 1→0 | 1.06 | 1.96 | 64% | 85% |
| 4 | 0→1 | 0.95 | 1.94 | 44% | 84% |
| 5 | 1→0 | 0.94 | 1.90 | 61% | 85% |
| 6 | 0→1 | 1.33 | 2.19 | 73% | 88% |
| 7 | 1→0 | 0.85 | 2.06 | 43% | 88% |
| 8 | 0→1 | 0.91 | 1.85 | 38% | 87% |
| 9 | 1→0 | 1.13 | 1.81 | 65% | 79% |

**The direct law is better in 9 of 10 cycles** (sign test, one-sided p = 0.011).
Mean σ dropping the cold-start cycle: **1.06° direct vs 1.95° through the
pipeline, a factor of 1.85.**

Cycle 0 is the only loss and is the first segment of the run — a cold start off
the arming transient (σ 2.81, dmax 117%, the single largest values in the run).

Unlike run 20 this is **order-balanced**: the conditions alternate every cycle,
so accumulated drift is common-mode. The result does not depend on which
condition ran second.

## The instrumentation change did its job

`rms` (about the arm-time reference) gives k0 15.05 vs k1 14.83 — a dead heat,
and meaningless. The segment mean walks from **−8.8° to −28.0° over 120 s**
(−0.16 °/s), the gyro-bias drift diagnosed after run 20, and `rms` is almost
entirely that drift. Wheel angle walks 228° → 730° in step, the controller
holding the null line `K_ANGLE·θ + K_MOTOR·φ = 0` against a false tilt.

σ is flat across the same window. Run 20's "10/10, p≈0.002" was this artifact;
this run separates the two cleanly.

## Mechanism: it is NOT a gain error — the export is faithful

The duty histograms are the striking part (% of samples, |commanded duty|):

```
           0-10 10-20 20-30 30-40 40-50 50-60 60-70 70-80 80-90  90+
kind 0:    39.4  36.2  14.8   5.8   2.3   1.0   0.3   0.1   0.1   0.1
kind 1:     8.9   8.9   8.9   9.2  11.1  13.6  19.2  17.5   2.7   0.0
```

The direct law lives below 30% duty 90% of the time. The net spends **50% of
its time above 50% duty** and peaks at 60–80%. That looked like a 3–4× gain
error in the export.

**It is not.** Feeding identical state vectors through both, offline
(no robot required):

| state (pitch°, rate°/s, angle°, speed°/s) | classical | net | ratio |
|---|---|---|---|
| (1, 0, 0, 0) | 10.71 | 11.87 | 1.108 |
| (0, 10, 0, 0) | 8.70 | 9.67 | 1.111 |
| (0, 0, 10, 0) | 4.30 | 4.76 | 1.107 |
| (0, 0, 0, 50) | 15.00 | 16.60 | 1.107 |
| (2, 5, 20, 30) | 43.37 | 46.09 | 1.063 |

The ratio is **1.108 ± 0.004 on every basis vector** — a single scalar output
scale error of +10.8%, almost certainly one rounded constant in the Q12 export,
not a per-weight quantisation spread. (It falls to 1.06 at large inputs as the
tanh begins to compress, which is a second and separate concern: the net's
loop gain *drops* exactly when the robot is furthest from upright.)

So the pipeline reproduces the law to within 11%, and 11% more loop gain
produces 85% duty instead of 60% and 1.85× the pitch oscillation.

## Hypothesis (untested): an 11% gain excess ignites the backlash limit cycle

That response is far too large to be proportional, and a bifurcation explains
it: this plant sits near the deadband limit-cycle boundary, and 11% is enough
to cross it. Once it ignites, the amplitude is set by the describing function,
not by the gain — which is why a small gain change buys a large duty change.

That is a **prediction the fixed sim can now test**, since the backlash model
became trustworthy only after the two MuJoCo bugs were fixed (`<exclude
body1="chassis" body2="tyre"/>` and `solreflimit="0.001 1"`):

> Sweep loop gain 1.00 vs 1.11 at the nominal gap. If the limit cycle ignites
> between them, the mechanism is confirmed and the fix is one constant in the
> exporter, not a redesign.

If it holds, it also explains why every previous hardware comparison read as
"the learned policy is worse": whatever passes through this pipeline gets an
11% gain bump and lands on the far side of the deadband boundary. The policy
would then have been carrying the pipeline's defect.

## Files

- `segments.csv` — per-segment stats, hub CSV verbatim
- `duty_hist.csv` — per-segment |duty| histogram, 10 bins of 10%

## Caveats

- One run. n = 10 paired cycles, one robot, one battery charge (8002 → ~6700 mV
  under load, both conditions equally).
- σ is computed on the hub in fixed point; no raw per-sample trace was captured,
  so the *frequency* of the k1 oscillation is not measured here. That is the
  direct test of the limit-cycle hypothesis and it needs a per-sample stream.
- The +10.8% scale factor is measured against `policy_linear_fast.py` as it
  exists today; it has not yet been traced to a specific line in the exporter.
