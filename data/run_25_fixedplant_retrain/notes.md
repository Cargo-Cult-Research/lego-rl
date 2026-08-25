# Run 25 — first retrain on the fixed plant: it wanders (sim-only)

2026-08-24. PPO, identical recipe to the deployed policy (2M steps, same
reward), on the plant as it now stands: wheel/chassis contact fix + backlash
deadband on. First policy ever trained *with* the resonance mechanism in the
sim.

**It did not converge.** `ep_len_mean` oscillated between ~310 and ~510 of a
possible 2000 for the whole second half of training. The verifier's verdict
on the result is damning and specific:

- duty at equilibrium **−0.18** — it holds a lean instead of balancing
- wheel gain ratio vs sim-tuned: **0.03** (the deployed policy's known-weak
  0.35, an order of magnitude worse)
- pitch gain 2.4× the sim-tuned value; signs all agree

Evaluation: **0/10 full episodes** (nominal and randomized) — every episode
ends by drifting to the ±0.5 m limit, mean |drift| 50 cm. Meanwhile the
*deployed* policy, trained on the broken plant, evaluates on the fixed plant
at 10/10 nominal (σ 0.28°) and 7/10 randomized. The deadband made the plant
harder in exactly the way `env.py`'s reward notes predicted: position
control is downstream of attitude control, and a harder attitude problem
starves the position signal. Lean-into-the-stiction-band may even be a local
optimum the deadband created.

Consequences: the deployed policy keeps its crown for hardware sessions; an
8M-step run is probing whether this is a compute problem; if that also
wanders, the CURRICULUM env.py already prescribes (start loose, tighten
X_LIMIT during training) is the next real change. Checkpoint kept at
`runs/ppo_fixedplant_seed0.zip` for comparison.
