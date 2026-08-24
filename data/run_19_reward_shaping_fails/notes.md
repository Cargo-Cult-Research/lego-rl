Run 18 measured a real deficiency on hardware: the learned policy travels 1.9x
further than the classical controller and drove into a wall. The verifier had
flagged the same axis in advance — wheel gain 0.148 against 0.430, a ratio of
0.35. So the target was clear and the intervention was chosen by a behavioural
criterion (match classical's sim drift) rather than by the Jacobian, to avoid
tuning the thing being used as a check.

Two mechanisms were tried. Both backfired, in the same direction.

| | survival | full | drift | pitch RMS | wheel-gain ratio |
| --- | --- | --- | --- | --- | --- |
| classical (target) | 10.00 s | 100% | 3.0 cm | 0.88 | — |
| **policy, original weights** | 10.00 s | 100% | 7.2 cm | 1.09 | 0.35 |
| cut PITCH_WEIGHT 0.25 → 0.085 | 5.15 s | 13% | 23.7 cm | 2.18 | 0.08 |
| halve X_LIMIT 0.5 → 0.25 m | 2.33 s | 0% | 25.2 cm | 5.21 | 0.03 |

**Pressing harder on position made the position gain smaller**, 0.35 → 0.08 →
0.03, by two independent mechanisms, while survival fell from 100% to 13% to 0%.
Two failures pointing the same way is a finding, not two accidents, so the third
variant was not attempted.

Two explanations, and they are not competing — both are probably operating.

**Position control is downstream of attitude control on this plant.** You steer
position by leaning, so precise pitch regulation is the *mechanism* for holding
station. Cutting the pitch weight to pay for position weight rewards the goal
while defunding the means. The Jacobian of that run makes it vivid: the
wheel_rate gain came out NEGATIVE, the wrong sign entirely.

**Tightening the bound starves the agent of the data it needs.** A 0.25 m limit
terminates episodes early, so less experience is collected inside the balanced
regime — which is exactly where position control has to be learned. Harder
task, less data about it, worse at everything including the pressed objective.

So the next thing to try is a curriculum (start loose, tighten through
training) or simply more steps, not a different weight. Both reverted; the
environment is back to the configuration that produces the best policy
measured, and that policy re-verified at 10.00 s, 100%, 7.2 cm on the restored
environment.

A separate bug surfaced during this and is worth its own note: **a
domain-randomization range silently overrides the nominal default.** Both
compliance models had been carefully defaulted OFF after failing validation,
and were then trained against anyway on every randomized episode because their
ranges were still open. It was caught because the CLASSICAL controller
regressed to 2.52 s — a control that had no reason to change is what exposed
it. The ranges are now pinned at zero, with the rule written down: if a model
is not trusted enough to be a default, it is not trusted enough to randomize.
