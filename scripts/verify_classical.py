#!/usr/bin/env python
"""Milestone 1 verifier: does the sim agree with what the hardware showed?

Two gain sets, two expectations, both from measurement:
  reference   the published Pybricks gains — tuned for a tall heavy robot,
              fell in 0.755 s on OUR hardware. The sim should agree they fail.
  sim-tuned   robot/gains.py — what the robot actually balances on. The sim
              should hold full episodes with them.
A sim that passes both is in the right regime; a sim that fails either has a
specific bug to hunt (signs, units, torque scale) BEFORE spending CPU-hours
on PPO.
"""
import argparse

import numpy as np

from lego_rl.classical import (ClassicalController, PYBRICKS_GAINS,
                               SIM_TUNED_GAINS, pybricks_to_si)
from lego_rl.env import BalancerEnv
from lego_rl.params import unmeasured


def run(gains_pb, randomize, episodes, seconds, seed):
    env = BalancerEnv(task="balance", randomize=randomize, max_seconds=seconds)
    ctrl = ClassicalController(gains_si=pybricks_to_si(gains_pb))
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
    for gname, gains, expect in [
            ("reference", PYBRICKS_GAINS, "hardware fell in 0.755 s — sim should agree"),
            ("sim-tuned", SIM_TUNED_GAINS, "the robot balances on these — sim should hold")]:
        print(f"{gname} gains {tuple(gains)}  ({expect})")
        for label, rand in [("nominal   ", False), ("randomized", True)]:
            t = run(gains, rand, args.episodes, args.seconds, args.seed)
            ok = float(np.mean(t >= args.seconds))
            print(f"  {label}  survival mean {t.mean():5.2f}s  min {t.min():5.2f}s  "
                  f"max {t.max():5.2f}s  full-episode {ok:.0%}")
        print()


if __name__ == "__main__":
    main()
