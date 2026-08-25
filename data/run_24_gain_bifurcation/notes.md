# Run 24 — run 22's prediction, tested in the fixed sim (sim-only)

2026-08-24. Run 22 left two things open: *where* the +10.8% pipeline gain
excess lives, and *why* an 11% gain bump produces 1.85× the oscillation. Both
are answered.

## Where the excess lives: the fit, not the quantisation

Splitting the pipeline (`policy_linear_weights.py` float net vs
`policy_linear_fast.py` Q12, both evaluated offline against the law):

| stage | gain vs law |
|---|---|
| numerical fit (`linear_to_net.py`) | **×1.116** on every basis vector |
| Q12 quantisation (`make_fast_policy.py`) | ×0.995 |

Run 22's "one rounded constant in the Q12 export" hypothesis was wrong — the
quantisation is clean. The excess comes from the MSE fit: nothing constrained
the net's slope at the origin, and a power-law sample distribution let it
settle ~11% steep uniformly. `linear_to_net.py` now measures the fitted
equilibrium Jacobian against the target law and normalises the output layer;
the regenerated pipeline reproduces the law to within 1% at operating
amplitudes (the tanh's compression above ~40% duty remains, as run 22 noted,
a separate concern).

## Why 11% costs 1.85×: the deadband bifurcation, confirmed

The exact experiment run 22 prescribed — same classical law, loop gain 1.00
vs 1.11, nominal backlash, 10 × 10 s nominal episodes each, arm transient
dropped:

| | σ (pitch) | duty p50 | duty p90 | samples >50% duty | falls |
|---|---|---|---|---|---|
| gain ×1.00 | 0.89° | 9.5% | 17.5% | 4.6% | 0 |
| gain ×1.11 | **1.65°** | 10.1% | **77.3%** | 15.4% | 0 |

σ ratio **1.85× in sim** against **1.85× measured on hardware** (run 22:
1.06° vs 1.95°). The duty histogram reproduces too: the median barely moves
while the upper tail explodes — the signature of a limit cycle igniting, not
of a proportional response. The p50/p90 split is the describing-function
picture exactly: most of the time the loop idles at normal duty, and the
ignited cycle periodically slams the tail.

So the causal chain is closed end to end: fit excess (+11%) → crosses the
deadband limit-cycle boundary → amplitude set by the describing function →
1.85× oscillation and a hot duty tail.

## Correction (2026-08-24, later the same day): the learned policy was NOT affected

Run 22 inferred "whatever passes through this pipeline gets an 11% gain bump
… the policy would then have been carrying the pipeline's defect." That
inference does not survive the trace. The defect lives in `linear_to_net.py`'s
fit, and only the pipeline-CONTROL ever passes through that step. The learned
policy's chain is `export_policy.py` (direct weight copy, no fit) → Q12, and
both stages measure clean offline: float export vs SB3 max err < 0.002
(tests/test_contracts.py), Q12 vs float export max err 0.0019, mean ratio
0.9999 over 2000 states. **Run 18's verdict therefore stands un-impeached:
classical really was quieter in 5 of 6 paired cycles.** The policy's real,
separately-measured deficiencies remain the weak position gain the verifier
flagged (ratio 0.35) and training on the pre-contact-fix sim.

Videos: `gain_100.mp4` vs `gain_111.mp4`, same seed, same plant — the ×1.11
robot's wheel-rattle is visible to the eye.

## Addendum: the boundary, mapped (same day)

Sweeping the gain scale (8 nominal episodes per point, arm transient
dropped):

| scale | σ (deg) | duty p90 | >50% duty |
|---|---|---|---|
| 0.90 | 0.72 | 14.2% | 0.0% |
| 0.95 | 0.66 | 14.2% | 0.0% |
| 1.00 | 0.61 | 14.0% | 0.0% |
| 1.03 | 0.97 | 20.3% | 5.6% |
| 1.06 | 1.01 | 23.6% | 6.1% |
| 1.09 | 1.45 | 66.3% | 12.6% |
| 1.12 | 1.46 | 68.5% | 13.2% |
| 1.15 | 1.48 | 71.8% | 14.3% |

The transition starts at **×1.03** and saturates by **×1.09** — the current
gains sit only **3–6% below the ignition boundary** (in the nominal-parameter
sim; the margin on hardware will differ but the sharpness is the point). That
is thin against known variation: battery compensation error, gearbox
temperature, and the ~2× unit spread measured between the two motors
(run 17). It reframes the chatter question: the residual is not cosmetic
noise, it is proximity to a bifurcation, and margin — not amplitude — is the
number to engineer.

**Still to do on hardware:** rerun the crossover with the regenerated
`policy_linear_fast.py` — direct and through-pipeline should now be
statistically indistinguishable, which validates the fix and retires the
pipeline as a suspect in any future comparison.
