#!/usr/bin/env python
"""Watch a rollout in the MuJoCo viewer: classical gains, or --policy X.zip."""
import argparse
import time

import mujoco.viewer
import numpy as np

from lego_rl.classical import ClassicalController
from lego_rl.env import BalancerEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="balance", choices=["balance", "swingup"])
    ap.add_argument("--policy", default=None)
    args = ap.parse_args()

    env = BalancerEnv(task=args.task, randomize=False)  # fixed model: viewer-safe resets
    if args.policy:
        from stable_baselines3 import PPO
        model = PPO.load(args.policy, device="cpu")
        act = lambda o: model.predict(o.astype(np.float32), deterministic=True)[0]
    else:
        ctrl = ClassicalController()
        act = lambda o: ctrl.act(o, battery_v=env.p.battery_v)

    obs, _ = env.reset(seed=0)
    with mujoco.viewer.launch_passive(env.model, env.data) as v:
        while v.is_running():
            obs, _, term, trunc, _ = env.step(act(obs))
            v.sync()
            time.sleep(1.0 / env.p.control_hz)
            if term or trunc:
                obs, _ = env.reset()


if __name__ == "__main__":
    main()
