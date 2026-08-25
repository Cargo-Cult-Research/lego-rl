# Run 20 — pipeline crossover, retracted: the statistic was measuring drift

2026-08-23 evening, two sessions. **Retro-filed 2026-08-24 from the raw
session logs** — this run happened without a directory, in violation of the
lab-book contract, and its citations in code pointed at nothing until now.
The raw hub output is in this directory: `raw_3way.log` (first session,
KINDS 0/1/2 — classical, law-through-pipeline, learned policy, 7 cycles) and
`raw_abba.log` (second session, KINDS 0/1, 10 cycles — the one the retracted
result came from).

## What it claimed

The law-through-pipeline (kind 1) lost to the direct law (kind 0) in **10 of
10 cycles, p ≈ 0.002** on `rms`. That looked like the cleanest result the
project had produced.

## Why it was retracted (commit 546da4d)

The `rms` was taken about the **arm-time pitch reference**, and that
reference drifts at ~0.2 °/s — Pybricks only re-zeros the gyro bias while
the hub is stationary, and a balancer never is. The controller absorbs the
false tilt by walking the wheel out along the null line
`K_ANGLE·pitch + K_MOTOR·wheel = 0`, so the reported rms tracked wheel angle
at **r = 0.999** and carried essentially no information about oscillation.
The robot was never tilting: 618° of wheel travel is 40 cm in 120 s, 3.4 mm/s.

The order made it worse: every cycle ran the conditions in the same order,
so the second condition always carried ~6 s more accumulated drift. That
alone manufactured the 10/10.

## What it changed

- `sigma` (RMS about the **segment mean**) logged alongside `rms` — drift
  moves the mean, not the spread. Verified off-hub against a synthetic
  0.2 °/s ramp: rms climbs 1.41 → 23.56 across a run while sigma stays
  within 0.03° of truth.
- **ABBA ordering**, so accumulated drift is common-mode.
- `MAX_DUTY` 40 → 100: both controllers were found operating right at the
  old clamp, which made every comparison hypersensitive; the duty
  *distribution* is logged instead.

The corrected protocol ran as run 22 and reached the same directional
conclusion (9/10, p = 0.011) for a reason that survives scrutiny.
