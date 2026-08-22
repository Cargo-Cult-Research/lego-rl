#!/usr/bin/env python
"""THE verifier: linearize the learned policy at the upright equilibrium and
compare its Jacobian to the four classical gains. Rough agreement validates
the whole pipeline (model, sysID, randomization, training, export) against a
known answer; disagreement is a specific bug to hunt."""
import argparse

import numpy as np
from stable_baselines3 import PPO

from lego_rl.classical import PYBRICKS_GAINS, si_to_pybricks
from lego_rl.env import OBS_SCALE

NAMES = ["pitch", "pitch_rate", "wheel", "wheel_rate"]


def policy_jacobian(model, eps=1e-3):
    def act(obs):
        a, _ = model.predict(obs.astype(np.float32), deterministic=True)
        return float(np.asarray(a).reshape(-1)[0])

    J = np.zeros(4)
    for i in range(4):
        d = np.zeros(4)
        d[i] = eps
        J[i] = (act(d) - act(-d)) / (2 * eps)
    return J  # d(duty)/d(scaled obs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model_path")
    args = ap.parse_args()

    model = PPO.load(args.model_path, device="cpu")
    J_scaled = policy_jacobian(model)
    J_si = J_scaled * OBS_SCALE            # chain rule: obs = OBS_SCALE * state
    g_pb = si_to_pybricks(J_si)            # -> duty% per deg / deg/s

    a0, _ = model.predict(np.zeros(4, dtype=np.float32), deterministic=True)
    print(f"duty at equilibrium: {float(np.asarray(a0).reshape(-1)[0]):+.4f} "
          "(should be ~0)\n")
    print(f"{'state':<11} {'learned':>10} {'pybricks':>10} {'ratio':>8}")
    for i, n in enumerate(NAMES):
        r = g_pb[i] / PYBRICKS_GAINS[i]
        print(f"{n:<11} {g_pb[i]:>10.3f} {PYBRICKS_GAINS[i]:>10.3f} {r:>8.2f}")
    print("\n(units: duty% per deg or deg/s, positive = same sign convention "
          "as the reference)")


if __name__ == "__main__":
    main()
