#!/usr/bin/env python
"""The roomba task's known-answer controller: balance, cruise, bounce, turn.

    .venv/bin/python scripts/roomba_baseline.py [--episodes 3] [--video out.mp4]

A hand-coded state machine on top of the classical balance law:
  CRUISE   balance + track a forward wheel-speed target (steady lean emerges)
  BACKOFF  wall hit detected -> reverse target briefly
  TURN     differential duty for a random duration, then cruise on

Wall hits are detected the way the HUB will have to detect them — from the
IMU and encoders only (a pitch-rate spike while the wheels stall against the
commanded speed), not from any world knowledge.

This is the verifier analog for roomba mode: a learned exploration policy
earns its keep only by beating this controller's coverage under the same
observation limits. Coverage = fraction of the 10x10 arena grid visited.
"""
from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys

import numpy as np

from lego_rl.gains import GAINS_SIM_TUNED
from lego_rl.roomba import OBS_SCALE_3D, RoombaEnv

DEG = math.pi / 180.0


class BounceController:
    """Duties from hub-visible observations only."""

    def __init__(self, rng, v_cruise=6.0):
        k = np.array(GAINS_SIM_TUNED)
        # Balance terms from the single-source gains (duty per rad, rad/s);
        # the wheel-POSITION term is deliberately dropped — a roomba must
        # not station-keep — and velocity tracking is added GENTLY: the
        # target is slewed and the gain kept small, because a hard velocity
        # loop fights the balance loop (first attempt thrashed at ±20 rad/s).
        self.k_pitch = k[0] / 100 / DEG
        self.k_rate = k[1] / 100 / DEG
        # FULL-authority speed term: it is the loop's damping and run 6
        # already proved it load-bearing (remove it and the robot falls in
        # seconds — replicated in this sim 5/5). Cruise thrust comes from
        # slewing the REFERENCE, never from clamping the term: an earlier
        # version clamped it to ±0.15 and fell within 2 s every episode.
        self.k_speed = k[3] / 100 / DEG   # duty per rad/s, the balancer's own
        self.v_cruise = v_cruise          # rad/s of wheel speed ~ 0.15 m/s
        self.slew = 3.0                   # rad/s^2: how fast the target moves
        self.leaning = 0                  # wall signature: sustained fwd pitch
        self.rng = rng
        self.mode = "cruise"
        self.timer = 0
        self.grace = 240                  # stall detector armed after ~1.2 s
        self.turn = 0.0
        self.stall = 0
        self.v_ref = 0.0                  # the slewed target

    def act(self, obs, dt):
        pitch, rate, yaw_rate, wl, wr = obs / OBS_SCALE_3D
        v = (wl + wr) / 2
        v_goal = {"cruise": self.v_cruise, "backoff": -0.7 * self.v_cruise,
                  "turn": 0.0}[self.mode]
        step = self.slew * dt
        self.v_ref += np.clip(v_goal - self.v_ref, -step, step)

        # Wall detection, hub-style: wheels far below the reference while the
        # law is pushing = something is in the way. The detector arms only
        # after a grace period so a fresh mode change cannot retrigger it.
        self.grace -= 1
        if self.mode == "cruise":
            if self.grace <= 0:
                # Two wall signatures, both hub-visible: wheels far below the
                # reference (stall), or a SUSTAINED forward lean — pushing an
                # obstacle it cannot move, the robot pitches into it and the
                # balance law feeds the lean until it falls. Catch it early.
                self.stall = self.stall + 1 if v < 0.35 * self.v_cruise else 0
                self.leaning = self.leaning + 1 if pitch > 6 * DEG else 0
                if self.stall > int(0.15 / dt) or self.leaning > int(0.1 / dt):
                    self.mode, self.timer = "backoff", int(0.6 / dt)
                    self.stall = self.leaning = 0
        elif self.timer <= 0:
            if self.mode == "backoff":
                self.mode, self.timer = "turn", int(self.rng.uniform(0.4, 1.1) / dt)
                self.turn = float(self.rng.choice([-1.0, 1.0])) * 0.15
            else:
                self.mode, self.timer = "cruise", 0
                self.grace = int(1.2 / dt)
        self.timer -= 1

        base = (self.k_pitch * pitch + self.k_rate * rate
                + self.k_speed * (v - self.v_ref))
        # Yaw damping in cruise: with none, any wheel asymmetry curls the
        # path into circles (12 m of path once covered 14 cells). During a
        # commanded TURN the damping is off, obviously.
        if self.mode == "turn":
            diff = self.turn
        else:
            diff = -0.08 * yaw_rate
        return np.clip([base - diff, base + diff], -1.0, 1.0)


def run_episode(env, seed, frames=None, cam=None, renderer=None, fps=25):
    rng = np.random.default_rng(seed)
    ctrl = BounceController(rng)
    obs, _ = env.reset(seed=seed)
    dt = 1.0 / env.p.control_hz
    spf = max(1, round(env.p.control_hz / fps))
    n = 0
    while True:
        obs, _, term, trunc, info = env.step(ctrl.act(obs, dt))
        n += 1
        if frames is not None and n % spf == 0:
            renderer.update_scene(env.data, camera=cam)
            frames.append(renderer.render().tobytes())
        if term or trunc:
            return info["coverage"], n * dt, term


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--video", default=None, help="render episode 0 overhead to .mp4")
    ap.add_argument("--randomize", action="store_true")
    args = ap.parse_args()

    env = RoombaEnv(randomize=args.randomize, max_seconds=args.seconds)
    frames = cam = renderer = None
    if args.video:
        import mujoco

        from render_rollout import beautify  # visual skin, physics untouched
        import lego_rl.model3d as m3
        orig = m3.build_mjcf_3d
        m3.build_mjcf_3d = lambda p, a=0.75: beautify(orig(p, a)).replace(
            'name="tyre_l" type="cylinder"', 'name="tyre_l" type="cylinder" rgba="0.15 0.15 0.16 1"').replace(
            'name="tyre_r" type="cylinder"', 'name="tyre_r" type="cylinder" rgba="0.15 0.15 0.16 1"')
        env = RoombaEnv(randomize=args.randomize, max_seconds=args.seconds)
        renderer = mujoco.Renderer(env.model, height=480, width=640)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 2.1, -90, 90  # overhead
        cam.lookat[:] = (0.0, 0.0, 0.0)
        frames = []

    covs, times = [], []
    for ep in range(args.episodes):
        f = frames if (frames is not None and ep == 0) else None
        cov, t, fell = run_episode(env, seed=ep, frames=f, cam=cam, renderer=renderer)
        covs.append(cov); times.append(t)
        print(f"episode {ep}: coverage {cov * 100:4.0f}%  lasted {t:5.1f}s"
              f"{'  FELL' if fell else ''}")
    print(f"\nbaseline coverage: mean {np.mean(covs) * 100:.0f}% over "
          f"{args.episodes} x {args.seconds:.0f}s episodes")

    if args.video and frames:
        if not shutil.which("ffmpeg"):
            print("ffmpeg missing", file=sys.stderr); return 1
        ff = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
             "-pix_fmt", "rgb24", "-s", "640x480", "-r", "25", "-i", "-",
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", args.video],
            stdin=subprocess.PIPE)
        for fr in frames:
            ff.stdin.write(fr)
        ff.stdin.close()
        ff.wait()
        print(f"wrote {args.video}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
