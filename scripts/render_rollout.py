#!/usr/bin/env python
"""Render a MuJoCo rollout to .mp4 — the sim, visible.

    .venv/bin/python scripts/render_rollout.py out.mp4 [--policy runs/X.zip]
        [--task balance|swingup] [--seconds 6] [--seed 0] [--randomize]

Offscreen (mujoco.Renderer), frames piped straight to ffmpeg — no viewer, no
extra Python deps; ffmpeg comes from homebrew. Write the file into a
data/run_NN_*/ directory (or record_run.py --attach it) and the lab-book page
embeds it next to that run's charts.

The physics model carries no visual assets (it was never meant to be looked
at), so a visual-only skin — lights, checker floor, colours — is injected
into the MJCF at render time. Physics is untouched: the skin only ADDS
elements and attributes that MuJoCo ignores for dynamics.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys

import mujoco
import numpy as np

from lego_rl.classical import ClassicalController
from lego_rl.env import BalancerEnv

VISUAL = """
  <visual>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
    <quality shadowsize="4096"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.53 0.71 0.92" rgb2="0.85 0.90 0.98" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.82 0.84" rgb2="0.70 0.70 0.73" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="60 60" reflectance="0.1"/>
  </asset>
"""


def beautify(xml: str) -> str:
    """Visual-only additions for offscreen rendering; physics untouched."""
    xml = xml.replace("<worldbody>", VISUAL + """  <worldbody>
    <light pos="0.4 -0.8 1.2" dir="-0.3 0.6 -0.9" directional="true" castshadow="true"/>""")
    xml = xml.replace('name="floor" type="plane"',
                      'name="floor" type="plane" material="grid"')
    xml = xml.replace('name="body" type="box"',
                      'name="body" type="box" rgba="0.79 0.16 0.12 1"')
    xml = xml.replace('name="wheel_geom" type="cylinder"',
                      'name="wheel_geom" type="cylinder" rgba="0.15 0.15 0.16 1"')
    return xml


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="output .mp4 (put it in a data/run_NN_*/ dir)")
    ap.add_argument("--policy", default=None, help="PPO .zip; default: classical gains")
    ap.add_argument("--task", default="balance", choices=["balance", "swingup"])
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--randomize", action="store_true",
                    help="sample domain-randomized params (default: nominal)")
    args = ap.parse_args()
    w, h = (int(x) for x in args.size.split("x"))

    if not shutil.which("ffmpeg"):
        print("ffmpeg not on PATH (brew install ffmpeg)", file=sys.stderr)
        return 1

    # Wrap the model builder so the env constructs the beautified MJCF.
    import lego_rl.env as _env_mod
    import lego_rl.model as _model_mod
    _orig = _model_mod.build_mjcf
    _env_mod.build_mjcf = _model_mod.build_mjcf = lambda p: beautify(_orig(p))

    env = BalancerEnv(task=args.task, randomize=args.randomize)
    if args.policy:
        from stable_baselines3 import PPO
        model = PPO.load(args.policy, device="cpu")
        act = lambda o: model.predict(o.astype(np.float32), deterministic=True)[0]
    else:
        ctrl = ClassicalController()
        act = lambda o: ctrl.act(o, battery_v=env.p.battery_v)

    renderer = mujoco.Renderer(env.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.55, -12, 100
    cam.lookat[:] = (0.0, 0.0, 0.06)

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
         "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(args.fps), "-i", "-",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", args.out],
        stdin=subprocess.PIPE,
    )

    obs, _ = env.reset(seed=args.seed)
    steps_per_frame = max(1, round(env.p.control_hz / args.fps))
    falls = 0
    for _ in range(int(args.seconds * args.fps)):
        for _ in range(steps_per_frame):
            obs, _, term, trunc, _ = env.step(act(obs))
            if term or trunc:
                falls += term
                obs, _ = env.reset()
        cam.lookat[0] = env.data.qpos[0]     # camera tracks the robot
        renderer.update_scene(env.data, camera=cam)
        ff.stdin.write(renderer.render().tobytes())
    ff.stdin.close()
    rc = ff.wait()
    if falls:
        print(f"note: {falls} fall(s) during the rollout (episode resets on-camera)")
    print(f"wrote {args.out}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
