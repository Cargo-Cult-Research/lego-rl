# Run 30 — the real lag cliff is beyond 90 ms (hardware)

2026-08-24. `robot/sysid_ratetau.py`: the classical law with OUR gyro filter
stepped upward, ABBA'd around the 30 ms anchor, ~12 s per segment, one
continuous balance, zero falls. Raw log: `raw.log` (`run30.log` as recorded).

| seg | tau | σ (deg) | bracketing 30 ms anchors |
|---|---|---|---|
| 1 | 45 ms | 2.57 | 2.35 / 2.52 → +5% |
| 3 | 60 ms | **1.79** | 2.52 / 2.92 → **−34%** |
| 5 | 90 ms | **2.21** | 2.92 / 2.97 → **−25%** |

The anchors degrade monotonically through the session (2.35 → 2.97; battery
sagging under load, and the reference walked ~0.35 °/s — near the top of the
DR range) — exactly the common-mode wear the ABBA design absorbs. Against
their local brackets, 60 and 90 ms are *quieter* than 30.

## What this settles

**On hardware, tripling the rate-channel lag is neutral to mildly
beneficial** — plausibly because a heavier filter suppresses the ~11 Hz
deadband chatter harder while the ~2 Hz pendulum mode still gets enough
damping. The sim, meanwhile, falls 6/6 at 30 ms even with zero actuation
delay (run 28). The inversion is now a measured number: **the sim is at
least 3–6× too lag-fragile**, and that fragility — not any real cliff — is
the standing explanation for why gain-heavy net controllers (pipeline
control, the 8M candidate) died on hardware while the sim promoted them.

## Calibration targets, now ranked by this number

Whatever makes the sim fragile must be worth ≥3× of lag margin:
1. **No static friction at standstill** (`env.py` motor model is
   frictionless below 1e-3 rad/s; the real drivetrain is stiction-locked
   below ~22% duty, run 17) — free damping the sim lacks.
2. `stall_torque` (GUESS) — sets loop gain, and gain is what the hardware
   actually cares about (run 24: ignition at +3–6%).
3. Effective pendulum time constant (com_height / inertia distribution).

The fix is falsifiable: a calibrated sim must simultaneously (a) balance
with a 90 ms filter, (b) fall on raw gyro at 200 Hz (run 16), and (c) keep
the gain-ignition boundary near +3–6% (runs 22/24). Three anchors, three
knobs at most.
