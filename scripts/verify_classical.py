#!/usr/bin/env python
"""Milestone 1 verifier: the published Pybricks gains should balance our sim.

If they do, the model + motor + sensor stack is at least in the right regime.
If they don't, there is a specific bug to hunt (signs, units, torque scale)
BEFORE spending CPU-hours on PPO.
"""
import argparse

import numpy as np

from lego_rl.classical import ClassicalController
from lego_rl.env import BalancerEnv
from lego_rl.params import unmeasured


def run(randomize, episodes, seconds, seed):
    env = BalancerEnv(task="balance", randomize=randomize, max_seconds=seconds)
    ctrl = ClassicalController()
    times = []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        steps = 0
        while True:
            obs, _, term, trunc, _ = env.step(ctrl.act(obs, battery_v=env.p.battery_v))
            steps += 1
            if term or trunc:
                break
        times.append(steps / env.p.control_hz)
    return np.array(times)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    print("still-GUESSED params:", ", ".join(unmeasured()), "\n")
    for label, rand in [("nominal   ", False), ("randomized", True)]:
        t = run(rand, args.episodes, args.seconds, args.seed)
        ok = float(np.mean(t >= args.seconds))
        print(f"{label}  survival mean {t.mean():5.2f}s  min {t.min():5.2f}s  "
              f"max {t.max():5.2f}s  full-episode {ok:.0%}")


if __name__ == "__main__":
    main()
