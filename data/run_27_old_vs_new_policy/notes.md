# Run 27 — old policy vs the deadband-trained policy (hardware)

2026-08-24, hardware. 3-way crossover, 7 cycles, 21 segments, one fall
(classical, segment 11, re-armed and resumed). Kinds in THIS build:
**GREEN = classical direct · YELLOW = `policy_fast.py` (the deployed, old
policy) · CYAN = `policy_fast_8m.py` (the deadband-trained candidate).**
(A color-confusion bug was found during this run — yellow meant the pipeline
control in run 26's build and the old policy here. The generated program now
prints its kind/color/module mapping at startup so every future log is
self-describing.) Raw log: `raw.log`.

## Result: the candidate does NOT transfer; the old policy shines

Paired by cycle on sigma:

- **old policy vs classical: old quieter 5/7**, mean σ 1.98 vs 1.85 overall
  (the means are skewed by the old policy's one bad late-session segment,
  5.88° at seg 19; its typical segment runs 1.2–1.4° against classical's
  1.4–2.8°). Duty character unchanged from run 26: the old policy essentially
  never exceeds 50% duty (3.0% of samples) while classical rides high
  (44.9% above 50%, dmax repeatedly >130 pre-clamp).
- **new 8M candidate vs classical: classical quieter 7/7**, σ 2.69 vs 1.85.
  The candidate runs HOT — 69% of samples above 50% duty, dmax pinned at
  100 in five of seven segments. In sim this same policy was the best ever
  measured (10/10 randomized, drift 1–3 cm, dmax 42%). The sim-real gap is
  the finding.

## Why the candidate fails on hardware — two probed hypotheses

**Latency (partial at best).** In sim the 8M policy degrades with delay
(σ 0.93 → 2.11 from 20 → 30 ms) but never reaches the hardware duty
signature, and the old policy degrades comparably. Extra net compute
(~1.6 ms) cannot alone explain a 69%-hot-duty session.

**Gyro-bias walk against a doubled position gain (untested, prime
suspect).** The hub's pitch reference walks ~0.2 °/s (run 20), and the
controller absorbs it by walking the wheel out along the null line — so the
*wheel angle channel is contaminated by drift on hardware*. The 8M policy's
selling point is 2× the position gain on exactly that channel; the old
policy's weak position gain (0.35 ratio, its verifier-flagged "deficiency")
is accidentally immune. In sim, `imu_rate_bias` is a per-episode CONSTANT,
so training never saw a walking reference. Testable two ways: (a) model the
bias as a random walk in `env.py` and re-evaluate both policies; (b) on
hardware, a candidate segment should get quieter right after a re-arm
(fresh reference) and worse with time — the per-cycle trend in this run is
consistent with that but confounded with battery sag.

## Standing

`policy_fast.py` (old) stays DEPLOYED and is now the best controller on the
robot by the honest statistic, two sessions running. `policy_fast_8m.py` is
demoted to a sim-real gap probe. The next training round should model the
bias walk before anything else — it is one line of env change and it targets
the exact channel the next generation of policies needs to trust.
