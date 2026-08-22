#!/usr/bin/env python
"""PPO on the balancer. The pi head is what must run on the STM32: 4->8->8->1
tanh (~100 MACs). The value net never leaves the Mac, so it gets room."""
import argparse
from pathlib import Path

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from lego_rl.env import BalancerEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="balance", choices=["balance", "swingup"])
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    out = Path(args.out or f"runs/ppo_{args.task}_seed{args.seed}")
    out.parent.mkdir(parents=True, exist_ok=True)

    env = make_vec_env(lambda: BalancerEnv(task=args.task), n_envs=args.envs,
                       seed=args.seed)
    model = PPO(
        "MlpPolicy", env,
        n_steps=512, batch_size=1024, learning_rate=3e-4,
        gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
        policy_kwargs=dict(net_arch=dict(pi=[8, 8], vf=[64, 64]),
                           activation_fn=torch.nn.Tanh),
        seed=args.seed, verbose=1,
    )
    model.learn(total_timesteps=args.steps)
    model.save(str(out))
    print(f"saved {out}.zip")


if __name__ == "__main__":
    main()
