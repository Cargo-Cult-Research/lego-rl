"""Does the sim reproduce run 35's cruise failure?

Run 35 (real robot, sysid_collision.py): asked to cruise at 300-750 deg/s
with duty = K_ANGLE*pitch + K_RATE*rate_f + K_SPEED*(speed - v_ref), the
robot limit-cycles duty rail-to-rail at ~9 Hz and travels at 640-890 deg/s
REGARDLESS of the target. If the sim, given the same law, tracks the
target instead, the friction/motor model is wrong before contact modeling
even starts -- that ordering is the point of this script.

Mirrors the hub program: same gains (single source robot/gains.py), same
30 ms rate EMA (in-controller, like hubconfig), same 150 deg/s^2 slew,
same +-100 duty clamp. Nominal params, no randomization, battery pinned
7.4 V (run 35 ran at ~7.4), measured 4-tick actuation delay.

Usage: sim_cruise_check.py [--trace N] to also dump raw rows for target N.
"""
import argparse
import math

import numpy as np

from lego_rl.env import OBS_SCALE, BalancerEnv
from lego_rl.gains import GAINS_SIM_TUNED

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED
DT_MS = 5.0
ALPHA = DT_MS / (30.0 + DT_MS)          # hubconfig RATE_TAU_MS = 30
SLEW = 150.0                            # deg/s^2, as run 35
V_TARGETS = (300, 450, 600, 750)
SETTLE_S, CRUISE_S, STATS_S = 1.5, 6.0, 3.0
FALL_DEG = 45.0


def rollout(v_target, seed=0, trace=False):
    env = BalancerEnv(task="balance", randomize=False, max_seconds=60.0,
                      param_override={"battery_v": 7.4})
    obs, _ = env.reset(seed=seed)
    rate_f = 0.0
    v_ref = 0.0
    n_settle = int(SETTLE_S * 200)
    n_total = n_settle + int(CRUISE_S * 200)
    rows = []
    for t in range(n_total):
        s = np.asarray(obs) / OBS_SCALE
        pitch = math.degrees(s[0])
        rate_f += ALPHA * (math.degrees(s[1]) - rate_f)
        wheel = math.degrees(s[2])
        speed = math.degrees(s[3])
        if t < n_settle:
            duty = (K_ANGLE * pitch + K_RATE * rate_f
                    + K_MOTOR * wheel + K_SPEED * speed)
        else:
            if v_ref < v_target:
                v_ref = min(v_target, v_ref + SLEW * DT_MS / 1000.0)
            duty = (K_ANGLE * pitch + K_RATE * rate_f
                    + K_SPEED * (speed - v_ref))
        duty = float(np.clip(duty, -100.0, 100.0))
        obs, _, _, _, info = env.step([duty / 100.0])
        true_pitch = math.degrees(info["true_state"][0])
        true_speed = math.degrees(info["true_state"][3])
        rows.append((t, true_pitch, rate_f, true_speed, duty, v_ref))
        if abs(true_pitch) > FALL_DEG:
            return rows, t / 200.0
    return rows, None


def zero_cross_hz(x, hz=200.0):
    x = np.asarray(x) - np.mean(x)
    crossings = np.sum(np.abs(np.diff(np.sign(x))) > 0)
    return crossings / 2.0 / (len(x) / hz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", type=int, default=None,
                    help="dump raw rows for this v_target")
    args = ap.parse_args()

    print("run 35 real:   duty rail-to-rail ~9 Hz, mean speed 640-890 "
          "regardless of target 300-750, pitch +-6 deg")
    print(f"{'vt':>4} {'fell_at':>8} {'mean_v':>7} {'std_v':>6} "
          f"{'mean_duty':>9} {'sat_pct':>7} {'pitch_amp':>9} {'osc_hz':>6}")
    for vt in V_TARGETS:
        rows, fell = rollout(vt, trace=args.trace == vt)
        if fell is not None:
            print(f"{vt:>4} {fell:>7.2f}s {'-':>7} {'-':>6} {'-':>9} "
                  f"{'-':>7} {'-':>9} {'-':>6}")
            continue
        tail = [r for r in rows if r[0] >= len(rows) - int(STATS_S * 200)]
        v = [r[3] for r in tail]
        d = [r[4] for r in tail]
        p = [r[1] for r in tail]
        sat = 100.0 * np.mean(np.abs(d) >= 99.0)
        print(f"{vt:>4} {'no':>8} {np.mean(v):>7.0f} {np.std(v):>6.0f} "
              f"{np.mean(d):>9.1f} {sat:>7.1f} "
              f"{np.percentile(np.abs(np.asarray(p) - np.mean(p)), 95):>9.2f} "
              f"{zero_cross_hz(d):>6.1f}")
        if args.trace == vt:
            print(f"\n  raw rows (every 5th, last second of {vt}):")
            print(f"  {'t':>5} {'pitch':>7} {'rate_f':>7} {'speed':>7} "
                  f"{'duty':>7} {'vref':>5}")
            for r in rows[-200::5]:
                print(f"  {r[0]:>5} {r[1]:>7.2f} {r[2]:>7.1f} {r[3]:>7.0f} "
                      f"{r[4]:>7.1f} {r[5]:>5.0f}")


if __name__ == "__main__":
    main()
