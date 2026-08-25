# Run 26 — 3-way crossover on the fixed pipeline (hardware)

2026-08-24, hardware. `experimental/ab/_ab_cycle.py`, KINDS (0, 1, 2) =
classical direct / the run-24-fixed pipeline-control / the deployed learned
policy. 20 of 21 segments captured (stopped by hand one short), 6 complete
cycles for the 0-vs-2 pairing, 7 for 0-vs-1. Battery 7555 → ~7300 mV, zero
falls, 200 Hz held, `clamp_pct` 0 everywhere. Raw log: `raw.log`.

## Result 1: the fixed pipeline-control is better, but still not clean

Paired by cycle on sigma: **classical quieter in 7/7 cycles, mean 1.38° vs
1.82° (×1.32)**, and the pipeline-control's duty distribution still has the
hot hump (41.7% of samples above 50% duty, vs classical's 16.5%). Before the
fix this was ×1.85 with the hump at 60–80%; the +11% fit excess is gone and
it shows — but something structural remains.

**The sim says the remainder is not a gain error.** In closed loop on the
nominal plant, the same three controllers rank differently: the float and
Q12 nets run *quieter* than the raw law (σ 0.62/0.63 vs 0.89, no hot tail) —
likely the tanh's soft saturation acting as a mild gain reduction at large
excursions. So whatever penalises the net on hardware is something the sim
comparison does not model. The prime suspect is **compute latency**: the
Q12 net costs ~1.6 ms per pass that the classical arithmetic does not, and a
delay sweep in sim found a cliff between 20 and 25 ms (σ 0.61 → 1.87,
duty p90 14% → 66%) — with measured hub latency already 15–19 ms, the net
runs at the cliff's edge. This also explains why the *learned policy*
(same 1.6 ms) is fine: its gentle gains sit far from the gain-ignition
boundary (run 24: ×1.03–1.09), so it can afford the latency; the
pipeline-control carries classical's near-boundary gains *plus* the latency.
Testable: an `--extra-delay` classical segment (software-delayed by 1.6 ms)
should reproduce the pipeline-control's excess without any net in the loop.

## Result 2: under honest measurement, the learned policy now BEATS classical

**Policy quieter in 4 of 6 complete cycles, mean sigma 1.20° vs 1.37°
(×0.87), at roughly half the duty effort** (dmax 37–48% vs classical's
55–96%; not one sample above 50% duty all session). Run 18 concluded the
opposite (classical 5/6) — but run 18 measured drift-contaminated rms with
both controllers pinned against the untrusted MAX_DUTY=40 clamp. With the
sigma statistic and the clamp removed, the verdict inverts. n=6 cycles on
one battery, so treat it as a lead rather than a theorem — but the direction
now agrees with the duty character: gentle *and* quieter.

Session-wide drift behaved as run 20 diagnosed: the arm-time reference
walked to −13.8° while sigma stayed in its band throughout.

## What this sets up

- `robot/policy_fast_8m.py` — the first policy trained on the deadband
  plant (runs 25's failure → 8M steps converged; verifier: pitch ratio
  1.00, position gain doubled to 0.64, drift 1–3 cm in sim) — is exported,
  Q12-verified (max err 0.0025 vs SB3), and inlined into the next 3-way:
  classical / old policy / new policy.
- The latency hypothesis for the pipeline residual wants its own paired
  test before the pipeline is declared understood.
