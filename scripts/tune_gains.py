#!/usr/bin/env python
"""Cross-entropy search over the four linear gains in the measured-param sim.
The reference gains were tuned for a taller, heavier robot; this re-tunes the
same controller structure for ours. Prints gains in Pybricks units (duty% per
deg / deg/s) ready to paste into robot/balance_classical.py."""
import argparse

import numpy as np

from lego_rl.classical import ClassicalController, pybricks_to_si, si_to_pybricks
from lego_rl.env import BalancerEnv


def score(gains_si, episodes, seed, seconds=5.0):
    env = BalancerEnv(task="balance", randomize=True, max_seconds=seconds)
    ctrl = ClassicalController(gains_si=gains_si)
    total = 0.0
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        steps, drift = 0, 0.0
        while True:
            obs, _, term, trunc, info = env.step(ctrl.act(obs, battery_v=env.p.battery_v))
            steps += 1
            if term or trunc:
                break
        drift = abs(info["true_state"][2]) * env.p.wheel_radius
        total += steps / env.p.control_hz - 0.3 * min(drift, 2.0)
    return total / episodes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    mu = np.log(pybricks_to_si())          # search in log-space, start at reference
    sigma = np.array([0.8, 0.8, 0.8, 0.8])
    elite = max(4, args.pop // 4)

    for it in range(args.iters):
        cand = np.exp(mu + sigma * rng.standard_normal((args.pop, 4)))
        scores = np.array([score(c, args.episodes, args.seed + 1000 * it) for c in cand])
        idx = np.argsort(scores)[::-1][:elite]
        mu = np.log(cand[idx]).mean(axis=0)
        sigma = np.log(cand[idx]).std(axis=0) + 0.05
        best = cand[idx[0]]
        print(f"iter {it}: best {scores[idx[0]]:5.2f}s  mean-elite {scores[idx].mean():5.2f}s  "
              f"gains(pb) {np.round(si_to_pybricks(best), 2)}")

    final = np.exp(mu)
    held = score(final, 20, 999, seconds=10.0)
    print(f"\nfinal gains (SI):       {np.round(final, 3)}")
    print(f"final gains (Pybricks): {np.round(si_to_pybricks(final), 2)}  "
          f"# duty% per deg/degs on pitch, rate, motor, speed")
    print(f"held-out score (10 s cap, 20 eps): {held:.2f}s")


if __name__ == "__main__":
    main()
