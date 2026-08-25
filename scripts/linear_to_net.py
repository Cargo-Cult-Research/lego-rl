#!/usr/bin/env python3
"""Cast the classical linear controller into the policy network's architecture.

The point is a CONTROL for the deployment pipeline. Every hardware comparison
so far confounds two things: is the learned policy worse than the classical
controller, or does export -> fixed point -> hub degrade whatever passes
through it? A network that IS the classical controller separates them, because
it travels the identical pipeline while implementing a control law already
measured on hardware at 1.5-3.3 deg RMS.

    NN-classical ~= classical on hardware  ->  pipeline is clean, gaps are the policy
    NN-classical  < classical on hardware  ->  pipeline is eating performance

A 4-8-8-1 tanh net can represent a linear map to whatever accuracy the
quantisation allows, but not by a trick: driving tanh into its linear region
means tiny activations, and tiny activations are exactly what Q12 fixed point
resolves worst (one LSB is 1/4096). So this fits numerically instead, over the
state distribution the robot actually visits, and then reports the error AFTER
quantisation -- which is the number that matters.

    .venv/bin/python scripts/linear_to_net.py [--gains 10.71 0.87 0.43 0.30]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
from torch import nn

from lego_rl.classical import pybricks_to_si
from lego_rl.env import OBS_SCALE
from lego_rl.gains import GAINS_SIM_TUNED

ROOT = Path(__file__).resolve().parent.parent
# The operating envelope, in SI. Deliberately wider than the robot's usual
# excursion so the fit does not fall apart during a recovery.
STATE_SCALE = np.array([np.radians(12.0), np.radians(150.0), 1.5, 12.0])


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.l1 = nn.Linear(4, 8)
        self.l2 = nn.Linear(8, 8)
        self.l3 = nn.Linear(8, 1)

    def forward(self, x):
        h = torch.tanh(self.l1(x))
        h = torch.tanh(self.l2(h))
        return self.l3(h)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gains", nargs=4, type=float,
                    default=list(GAINS_SIM_TUNED),
                    help="Pybricks-unit gains: duty%% per deg or deg/s "
                         "(default: robot/gains.py)")
    ap.add_argument("--steps", type=int, default=6000)
    ap.add_argument("--out", default="policies/policy_linear_weights.py")
    args = ap.parse_args()

    k_si = pybricks_to_si(np.array(args.gains))     # duty per rad, rad/s
    print(f"target linear law: {np.array(args.gains)} (Pybricks units)")
    print(f"                 = {k_si.round(4)} (duty per rad / rad/s)\n")

    rng = np.random.default_rng(0)
    torch.manual_seed(0)

    def sample(n):
        """Sample where the robot actually lives, not uniformly over a box.

        A uniform box out to +-12 deg puts most samples in hard saturation
        (k.s reaches 5.0), so the net spends its capacity learning a clipping
        function while the region that matters -- 1-3 deg, 12% mean duty --
        gets a handful of points. Magnitudes are drawn with a power law so
        most samples are small and a tail still covers recoveries."""
        direction = rng.normal(size=(n, 4))
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        mag = rng.uniform(0, 1, size=(n, 1)) ** 2.5      # concentrate near zero
        return direction * mag * STATE_SCALE

    def batch(n):
        s = sample(n)
        u = np.clip(s @ k_si, -1.0, 1.0)            # the controller, saturated
        obs = s * OBS_SCALE                          # what the net actually sees
        return (torch.tensor(obs, dtype=torch.float32),
                torch.tensor(u, dtype=torch.float32).unsqueeze(1))

    net = Net()
    opt = torch.optim.Adam(net.parameters(), lr=3e-3)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.steps)
    for i in range(args.steps):
        x, y = batch(512)
        loss = nn.functional.mse_loss(net(x), y)
        opt.zero_grad()
        loss.backward()
        opt.step()
        sched.step()
        if (i + 1) % 1500 == 0:
            print(f"  step {i+1:5d}  loss {loss.item():.3e}")

    # PIN THE EQUILIBRIUM GAIN. Run 22 measured the law-through-pipeline
    # running +10.8% hot on every basis vector and blamed "one rounded
    # constant in the Q12 export"; splitting the pipeline showed the Q12 step
    # is clean (-0.5%) and the excess lives HERE, in the numerical fit: MSE
    # over a power-law sample distribution puts no constraint on the slope at
    # the origin, and the fitted net came out uniformly ~11% steep. A control
    # for the pipeline must implement the law it claims to, so measure the
    # fitted equilibrium Jacobian against the target and scale the linear
    # output layer by the uniform excess. Shape is untouched; the per-axis
    # residual after the rescale is printed and should be within ~1%.
    with torch.no_grad():
        eps = 1e-3
        J = np.zeros(4)
        for i in range(4):
            d = np.zeros((1, 4)); d[0, i] = eps
            hi_ = net(torch.tensor(d * STATE_SCALE * OBS_SCALE, dtype=torch.float32))
            lo_ = net(torch.tensor(-d * STATE_SCALE * OBS_SCALE, dtype=torch.float32))
            J[i] = float(hi_ - lo_) / (2 * eps * STATE_SCALE[i])
        ratios = J / k_si
        scale = float(np.mean(ratios))
        print(f"\nequilibrium gain vs law, pre-normalisation: {np.round(ratios, 4)}"
              f"  (uniform excess {scale:.4f})")
        net.l3.weight /= scale
        net.l3.bias /= scale
        for i in range(4):
            d = np.zeros((1, 4)); d[0, i] = eps
            hi_ = net(torch.tensor(d * STATE_SCALE * OBS_SCALE, dtype=torch.float32))
            lo_ = net(torch.tensor(-d * STATE_SCALE * OBS_SCALE, dtype=torch.float32))
            J[i] = float(hi_ - lo_) / (2 * eps * STATE_SCALE[i])
        print(f"after normalisation:                        {np.round(J / k_si, 4)}")

    # Accuracy BY BAND: the operating region is what matters, and a single
    # max over a box dominated by saturation hides it.
    print("\nfloat net vs linear law, by commanded-duty band:")
    st = sample(60000)
    u = np.clip(st @ k_si, -1.0, 1.0)
    with torch.no_grad():
        pred = net(torch.tensor(st * OBS_SCALE, dtype=torch.float32)).numpy()[:, 0]
    err = np.abs(pred - u)
    for lo, hi, lbl in ((0.0, 0.15, "|duty| < 15%   (cruise)"),
                        (0.15, 0.40, "15-40%         (working)"),
                        (0.40, 1.01, ">40%           (saturating)")):
        m = (np.abs(u) >= lo) & (np.abs(u) < hi)
        if m.sum():
            print(f"  {lbl}  n={m.sum():6d}  max {err[m].max():.4f}  "
                  f"mean {err[m].mean():.4f}")

    W1 = net.l1.weight.detach().numpy()
    B1 = net.l1.bias.detach().numpy()
    W2 = net.l2.weight.detach().numpy()
    B2 = net.l2.bias.detach().numpy()
    W3 = net.l3.weight.detach().numpy()[0]
    B3 = float(net.l3.bias.detach().numpy()[0])

    def fmt_m(m):
        return "[\n" + "".join(
            "    [" + ", ".join(f"{v:.6g}" for v in row) + "],\n" for row in m) + "]"

    def fmt_v(v):
        return "[" + ", ".join(f"{x:.6g}" for x in v) + "]"

    src = f'''"""Generated by scripts/linear_to_net.py -- do not edit.

THIS IS NOT A LEARNED POLICY. It is the classical linear controller
{np.array(args.gains)} (Pybricks units) cast into the policy network's
architecture, so that it travels the identical export / fixed-point / hub
pipeline while implementing a control law already validated on hardware.

It is a CONTROL: comparing it against the real classical controller on the
robot separates "the learned policy is worse" from "the pipeline degrades
whatever passes through it".

act(state) -> duty in [-1, 1]; state = [pitch, pitch_rate, wheel_angle,
wheel_rate] in rad and rad/s, sign conventions as in src/lego_rl/env.py.
"""
from umath import exp


def tanh(x):
    """Pybricks umath has exp but no tanh."""
    if x > 8.0:
        return 1.0
    if x < -8.0:
        return -1.0
    e = exp(2.0 * x)
    return (e - 1.0) / (e + 1.0)


OBS_SCALE = {fmt_v(OBS_SCALE)}
W1 = {fmt_m(W1)}
B1 = {fmt_v(B1)}
W2 = {fmt_m(W2)}
B2 = {fmt_v(B2)}
W3 = {fmt_v(W3)}
B3 = {B3:.6g}


def act(state):
    x = [state[i] * OBS_SCALE[i] for i in range(4)]
    h1 = [tanh(sum(W1[j][i] * x[i] for i in range(4)) + B1[j]) for j in range(8)]
    h2 = [tanh(sum(W2[j][i] * h1[i] for i in range(8)) + B2[j]) for j in range(8)]
    u = sum(W3[i] * h2[i] for i in range(8)) + B3
    return -1.0 if u < -1.0 else (1.0 if u > 1.0 else u)
'''
    out = ROOT / args.out
    out.write_text(src)
    print(f"wrote {out}")
    print("\nnext: scripts/make_fast_policy.py to quantise, which reports the")
    print("error that actually matters -- after Q12, not before.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
