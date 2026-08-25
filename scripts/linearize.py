#!/usr/bin/env python
"""THE verifier: linearize the learned policy at the upright equilibrium and
compare its Jacobian to the classical gains.

Two baselines are printed, deliberately, because they answer different
questions and only one of them is a fair test:

  reference   the published Pybricks gains (88, 0.35, 0.72, 0.19), tuned for a
              tall heavy robot. We have since shown they are WRONG for this
              build -- they survive 0% of full episodes in the measured sim and
              fell in 0.755 s on hardware. A mismatch here is expected and
              means nothing. It is printed for context, not as a test.
  sim-tuned   (10.71, 0.87, 0.43, 0.30), from CEM in the measured sim
              (scripts/tune_gains.py), which holds 100% of full episodes and
              is what the hardware actually balances on. This is the real
              comparison: it is the best linear controller for the same plant
              PPO was trained on, derived independently of PPO.

Near the upright equilibrium the optimal policy IS approximately linear, so a
correctly trained policy should land near the sim-tuned column. Per CLAUDE.md,
do not tune either side to force agreement -- a mismatch is a bug to hunt.
"""
import argparse

import numpy as np
from stable_baselines3 import PPO

from lego_rl.classical import PYBRICKS_GAINS, si_to_pybricks
from lego_rl.env import OBS_SCALE
from lego_rl.gains import GAINS_SIM_TUNED

NAMES = ["pitch", "pitch_rate", "wheel", "wheel_rate"]
SIM_TUNED = np.array(GAINS_SIM_TUNED)  # single source: robot/gains.py


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
    ap.add_argument("--eps", type=float, default=1e-3)
    args = ap.parse_args()

    model = PPO.load(args.model_path, device="cpu")
    J_scaled = policy_jacobian(model, eps=args.eps)
    J_si = J_scaled * OBS_SCALE            # chain rule: obs = OBS_SCALE * state
    g_pb = si_to_pybricks(J_si)            # -> duty% per deg / deg/s

    a0, _ = model.predict(np.zeros(4, dtype=np.float32), deterministic=True)
    bias = float(np.asarray(a0).reshape(-1)[0])
    print(f"duty at equilibrium: {bias:+.4f} (should be ~0)")
    if abs(bias) > 0.05:
        print("  ^ a real offset: the policy pushes even when perfectly upright,")
        print("    so it is holding a lean rather than balancing about zero.")
    print()

    ref = np.asarray(PYBRICKS_GAINS, dtype=float)
    print(f"{'state':<11} {'learned':>10} {'sim-tuned':>10} {'ratio':>7}"
          f"   |{'reference':>10} {'ratio':>7}")
    print("-" * 62)
    for i, n in enumerate(NAMES):
        rt = g_pb[i] / SIM_TUNED[i]
        rr = g_pb[i] / ref[i]
        print(f"{n:<11} {g_pb[i]:>10.3f} {SIM_TUNED[i]:>10.3f} {rt:>7.2f}"
              f"   |{ref[i]:>10.3f} {rr:>7.2f}")

    print("\nunits: duty% per deg or per deg/s; positive = the reference's sign\n"
          "convention. The sim-tuned column is the test; reference is context.")

    ratios = g_pb / SIM_TUNED
    signs_ok = bool(np.all(np.sign(g_pb) == np.sign(SIM_TUNED)))
    within = np.abs(np.log(np.abs(ratios) + 1e-12))
    print(f"\nsigns all agree: {signs_ok}")
    print("worst |log ratio| vs sim-tuned: "
          f"{within.max():.2f} on {NAMES[int(np.argmax(within))]} "
          f"(0 = exact, 0.69 = 2x, 1.6 = 5x)")

    # Scale vs shape. The two controllers optimise DIFFERENT objectives on the
    # same plant -- CEM maximised survival time alone, PPO pays a quadratic
    # cost on lean, drift and effort -- so a uniform scale difference between
    # them is expected and is not evidence of a pipeline fault. What the
    # verifier is really asking is whether the policy learned the same
    # feedback STRUCTURE, so normalise both by their own pitch gain and
    # compare what is left.
    scale = g_pb[0] / SIM_TUNED[0]
    shape_l = g_pb / g_pb[0]
    shape_c = SIM_TUNED / SIM_TUNED[0]
    print(f"\noverall scale vs sim-tuned: {scale:.2f}x"
          "  (a uniform factor is an objective difference, not a fault)")
    print(f"{'state':<11} {'learned/pitch':>14} {'tuned/pitch':>12} {'ratio':>7}")
    for i, n in enumerate(NAMES):
        r = shape_l[i] / shape_c[i]
        print(f"{n:<11} {shape_l[i]:>14.4f} {shape_c[i]:>12.4f} {r:>7.2f}")
    sh = np.abs(np.log(np.abs(shape_l[1:] / shape_c[1:]) + 1e-12))
    print(f"worst shape mismatch: {sh.max():.2f} on "
          f"{NAMES[1 + int(np.argmax(sh))]}")


if __name__ == "__main__":
    main()
