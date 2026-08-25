# Run 28 — the anchoring gate: two sensor truths, one refutation, one inversion (sim-only)

2026-08-24. Before training anything for roomba mode, the sim's sensor model
gets the two mechanisms the hub actually has, and each is tested against the
hardware record.

## 1. The pitch reference now walks (modeled, kept)

`imu_rate_bias` now INTEGRATES into the measured pitch (env `_obs`), exactly
as Pybricks integrates a biased gyro — the runs 20/26 mechanism, ~0.2 °/s on
hardware. A contract test locks it (2 s at 1 °/s ⇒ ~2° of reference error).
The DR range tightens from the pre-integration ±1.0 to ±0.4 °/s (2× the
worst observed walk).

**Run 27's prime suspect is refuted by its own test.** Prediction was that
the walking reference poisons the wheel channel and punishes the 8M
candidate's doubled position gain. The opposite holds: under a 0.2–0.4 °/s
walk the *old* policy walks out of bounds (6–8 of 8 episodes terminate)
while the 8M candidate — whose strong position gain actively fights the
false-tilt walk — is unbothered (0 falls, duty unchanged). The honest
encoder resists the lying gyro. Why the candidate ran hot on hardware
remains open; the walk is not it.

## 2. The rate filter (modeled, DEFAULT OFF — the inversion)

The hub low-passes the gyro at 30 ms (`RATE_TAU_MS`) before any controller
sees it; the sim handed back the raw rate, so every policy ever trained here
met ~15 ms of unmodeled phase lag at deployment. Modeling it produced the
strongest sim-real contradiction the project has:

| condition | hardware | sim (filter modeled) |
|---|---|---|
| classical + 30 ms filter | stands 120 s (runs 22, 26) | **falls 6/6, even at zero actuation delay** |
| policy + raw gyro | falls at 4.0 s (run 16) | works |
| policy + 30 ms filter | best controller measured | falls 6/6 |

The real robot NEEDS the filter; the sim robot cannot TOLERATE it. Something
structural is miscalibrated — the sim plant is far more lag-fragile than the
hardware. Suspects, in order: no static friction at zero speed (sim motors
are frictionless at standstill; the real drivetrain is stiction-locked below
~22% duty, run 17), `stall_torque` (a GUESS that sets loop gain), effective
pendulum time constant. Per the repo's rule — a model element not trusted as
a default is not trusted to randomize — the filter ships default-off with
the contradiction documented at the parameter.

This also reframes runs 26–27: the sim under-penalizing lag is one candidate
for why the pipeline-control and the 8M policy (both lag-adding, gain-heavy)
died on hardware while sim promoted them.

## Consequences

- Retraining with the drift walk in the randomization is running
  (`runs/ppo_driftwalk_8m_seed3`).
- The lag-fragility hunt is the next sysID target: measure the REAL robot's
  tolerance by sweeping RATE_TAU_MS upward on hardware (a one-battery
  crossover: 30/45/60 ms) and match the sim's failure point to it.
- Roomba-mode work proceeds in parallel — its scripted baseline does not
  depend on the policy pipeline.
