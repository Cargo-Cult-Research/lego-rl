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
1.85× oscillation and a hot duty tail. Every pre-fix hardware comparison of
"learned vs classical" was carrying this; the policy travelled through a
pipeline that made whatever it computed 11% hotter.

Videos: `gain_100.mp4` vs `gain_111.mp4`, same seed, same plant — the ×1.11
robot's wheel-rattle is visible to the eye.

**Still to do on hardware:** rerun the run-22 ABBA with the regenerated
`policy_linear_fast.py`. If the pipeline is truly clean now, direct and
through-pipeline should be statistically indistinguishable.
