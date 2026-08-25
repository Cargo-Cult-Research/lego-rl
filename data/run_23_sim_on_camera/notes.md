# Run 23 — the sim, on camera (sim-only)

2026-08-24. Until today the MuJoCo model was a black box: every claim about
it arrived as a survival statistic or a Jacobian, never as something you
could watch. `scripts/render_rollout.py` (new) renders any rollout offscreen
to mp4 — no viewer, frames piped to ffmpeg, visual skin injected at render
time so the physics XML is untouched. Videos land in the run directory and
the page embeds them; from here on, a sim experiment can ship a video next
to its chart.

Three 8-second rollouts, all seed-controlled:

- `classical_nominal.mp4` — the classical law (robot/gains.py) on nominal
  measured parameters.
- `policy_seed0_nominal.mp4` — the deployed PPO policy
  (`runs/ppo_balance_seed0.zip`), same plant.
- `policy_seed0_randomized.mp4` — the same policy on a domain-randomized
  draw (seed 3), the training distribution's view of the world.

## The first render caught a live bug

The first classical rollout **fell on camera**. `ClassicalController`'s
defaults were still the *published reference gains* with the *retired*
±10% friction compensation — the configuration the hardware abandoned
(runs 1, 8) — so every default-constructed sim rollout, including
`scripts/view.py`'s, was quietly simulating a control law the robot does not
run. With the defaults now read from `robot/gains.py`: 0 falls in 10 × 10 s
nominal episodes. This is exactly the class of drift the constants
consolidation exists to kill, found within minutes of the sim becoming
visible.

## What the eye adds that the statistics hid

Both controllers hold the upright, but they do not *look* alike: the policy
sits nearly still with small corrections, while the classical law works
visibly harder against the backlash deadband (the model's 1° of play is in
these rollouts — `motor_backlash_deg` defaults on). The wander is small on
nominal parameters and larger under the randomized draw. None of this is
quantitative; the point of this run is that the next person can *see* the
plant the numbers describe.
