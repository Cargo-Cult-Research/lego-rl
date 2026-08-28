"""Joint sysid: fit the uncertain plant parameters against ALL real probes.

The one-suspect-at-a-time hunt is closed (runs 28, 35, 36): inertia is
validated, and no single knob -- stiction, delay, damping -- explains why
hardware balances with the 30 ms filter + ~19 ms delay while the sim
falls in ~3 s. So fit the uncertain dimensions TOGETHER, each candidate
scored against every real anchor at once:

  * run 36 free topple: lambda 7.46, wheels moved only ~1.5 deg
    (the topple runs on the candidate's own wheel_frictionloss -- the
    locked-wheel observation constrains stiction from below, no hand-lock)
  * run 35 cruise (the same law the hub ran): must SURVIVE, and match the
    measured limit cycle -- target-independent ~700-880 deg/s, duty
    rail-to-rail, ~9 Hz
  * station-keep with the 30 ms filter: must survive 10 s (the run 28
    anchor: hardware stands), sigma ~2.2 deg, ring ~10.6 Hz (run 34)

Free parameters include the two new signal/torque-path suspects
(wheel_frictionloss, imu_fusion_hz) alongside the GUESS-tier motor and
contact numbers. Optimizer is CEM (numpy-only, like tune_gains.py).

Usage:
  sysid_fit.py --smoke          score nominal params, print the loss table
  sysid_fit.py                  full fit; progress + best-so-far into
                                data/sysid_fit/ (JSON per generation)
"""
import argparse
import json
import math
import os
import time
from dataclasses import replace

import mujoco
import numpy as np

from lego_rl.env import OBS_SCALE, BalancerEnv
from lego_rl.gains import GAINS_SIM_TUNED
from lego_rl.model import build_mjcf
from lego_rl.params import nominal_params

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED
ALPHA_30 = 5.0 / (30.0 + 5.0)          # the hub's rate filter at 200 Hz
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "data", "sysid_fit")

# name, low, high, integer?
SPACE = [
    # stall_torque MEASURED 0.316 (run 38) -- bounds are measurement
    # uncertainty, not a search space; the 0.10 the first fit chose is
    # refuted (it was faking the dead zone, now modeled directly).
    ("stall_torque",       0.28, 0.36, False),
    ("no_load_speed",      1200, 1700, False),
    ("motor_friction_duty", 0.08, 0.16, False),
    ("wheel_frictionloss",  0.0, 0.30, False),
    # Lower bound 2, not 0: the measured 15-19 ms cmd->motion includes
    # stiction breakaway (now modeled separately), so some latency is
    # honestly attributable elsewhere -- but a fit at delay 0 would just be
    # deleting the measurement. The first unconstrained run did exactly
    # that; delay=2 still stood in ablation.
    ("delay_ctrl_steps",      2,    6, True),
    ("ground_friction",     0.4,  1.5, False),
    ("motor_inertia_mult", 0.05,  1.5, False),
    ("motor_backlash_deg",  0.0,  2.5, False),
    # Run 37 MEASURED the real fusion crossover at ~0.03 Hz (slide probe):
    # the hub's rotation() is a pure gyro integral on control timescales.
    # Capped so the optimizer can never again buy stability with
    # unphysical fusion, as the first fit did at 4.6 Hz.
    ("imu_fusion_hz",       0.0, 0.05, False),
]

# Real anchors (see data/run_35_wall_impact_capture and run_36_free_topple)
REAL = {
    "topple_lambda": 7.46,     # 1/s, run 36 energy fit
    "topple_roll": 1.5,        # deg of wheel during the fall
    "cruise_v": {300: 700.0, 750: 860.0},   # deg/s, run 35 pre-impact means
    "cruise_sat": 0.44,        # fraction of ticks with |duty| pinned;
    "cruise_osc": 9.3,         # Hz -- both computed from the run 35
                               # pre-trigger windows (12 segments)
    "keep_sigma": 2.2,         # deg, run 34 classical anchor
    "keep_ring": 10.6,         # Hz, the famous ring
}


def unpack(x):
    d = {}
    for (name, lo, hi, is_int), v in zip(SPACE, x):
        v = float(np.clip(v, lo, hi))
        d[name] = int(round(v)) if is_int else v
    return d


def make_params(d):
    return replace(nominal_params(), battery_v=7.45, **d)


# --- probes -----------------------------------------------------------------

def topple_probe(p):
    """Run 36 replicated: coasting motors = zero actuation; whether the
    drivetrain locks is up to the candidate's wheel_frictionloss."""
    model = mujoco.MjModel.from_xml_string(build_mjcf(p))
    data = mujoco.MjData(model)
    pa = model.joint("pitch").qposadr[0]
    pd = model.joint("pitch").dofadr[0]
    wa = model.joint("wheel").qposadr[0]
    data.qpos[pa] = math.radians(2.0)
    mujoco.mj_forward(model, data)
    th, om = [], []
    for i in range(int(2.5 / p.physics_dt)):
        data.ctrl[0] = 0.0
        mujoco.mj_step(model, data)
        if i % 5 == 0:
            th.append(abs(data.qpos[pa]))
            om.append(abs(data.qvel[pd]))
        if abs(data.qpos[pa]) > math.radians(45):
            break
    th, om = np.array(th), np.array(om)
    roll = abs(math.degrees(data.qpos[wa]))
    m = (th > math.radians(3)) & (th < math.radians(40))
    if m.sum() < 8:
        return {"lambda": 0.0, "roll": roll}
    A = np.vstack([np.cos(th[m]), np.ones(m.sum())]).T
    slope = np.linalg.lstsq(A, om[m] ** 2, rcond=None)[0][0]
    lam = math.sqrt(-slope / 2) if slope < 0 else 0.0
    return {"lambda": lam, "roll": roll}


def _control_rollout(p, seconds, law, seed):
    override = {name: getattr(p, name) for name, *_ in SPACE}
    override["battery_v"] = p.battery_v
    env = BalancerEnv(task="balance", randomize=False, max_seconds=60.0,
                      param_override=override)
    obs, _ = env.reset(seed=seed)
    state = {"rate_f": 0.0, "v_ref": 0.0}
    pitches, duties, speeds = [], [], []
    n = int(seconds * 200)
    for t in range(n):
        s = np.asarray(obs) / OBS_SCALE
        duty = law(t, s, state)
        obs, _, _, _, info = env.step([duty / 100.0])
        tp = math.degrees(info["true_state"][0])
        pitches.append(tp)
        duties.append(duty)
        speeds.append(math.degrees(info["true_state"][3]))
        if abs(tp) > 45:
            return t / 200.0, pitches, duties, speeds
    return None, pitches, duties, speeds


def _zero_cross_hz(x):
    x = np.asarray(x) - np.mean(x)
    return float(np.sum(np.abs(np.diff(np.sign(x))) > 0) / 2.0
                 / (len(x) / 200.0)) if len(x) > 10 else 0.0


def cruise_probe(p, v_target, seed):
    def law(t, s, st):
        pitch = math.degrees(s[0])
        st["rate_f"] += ALPHA_30 * (math.degrees(s[1]) - st["rate_f"])
        wheel, speed = math.degrees(s[2]), math.degrees(s[3])
        if t < 300:
            d = (K_ANGLE * pitch + K_RATE * st["rate_f"]
                 + K_MOTOR * wheel + K_SPEED * speed)
        else:
            st["v_ref"] = min(v_target, st["v_ref"] + 150 * 0.005)
            d = (K_ANGLE * pitch + K_RATE * st["rate_f"]
                 + K_SPEED * (speed - st["v_ref"]))
        return float(np.clip(d, -100, 100))

    fell, pitches, duties, speeds = _control_rollout(p, 7.5, law, seed)
    tail = slice(-600, None)
    d = np.asarray(duties[tail])
    return {"fell": fell,
            "mean_v": float(np.mean(speeds[tail])) if duties else 0.0,
            "sat": float(np.mean(np.abs(d) >= 99)) if len(d) else 0.0,
            "osc": _zero_cross_hz(duties[tail])}


def keep_probe(p, seed):
    def law(t, s, st):
        st["rate_f"] += ALPHA_30 * (math.degrees(s[1]) - st["rate_f"])
        return float(np.clip(
            K_ANGLE * math.degrees(s[0]) + K_RATE * st["rate_f"]
            + K_MOTOR * math.degrees(s[2]) + K_SPEED * math.degrees(s[3]),
            -100, 100))

    fell, pitches, duties, _ = _control_rollout(p, 10.0, law, seed)
    tail = slice(-1200, None)
    return {"fell": fell,
            "sigma": float(np.std(pitches[tail])) if pitches else 99.0,
            "ring": _zero_cross_hz(pitches[tail])}


# --- loss -------------------------------------------------------------------

def evaluate(d, seeds=(0, 1)):
    p = make_params(d)
    terms = {}
    top = topple_probe(p)
    terms["topple_lambda"] = ((top["lambda"] - REAL["topple_lambda"]) / 0.5) ** 2
    terms["topple_roll"] = (max(0.0, top["roll"] - 5.0) / 10.0) ** 2

    for vt in (300, 750):
        rs = [cruise_probe(p, vt, s) for s in seeds]
        surv = np.mean([r["fell"] is None for r in rs])
        terms[f"cruise{vt}_fall"] = 6.0 * (1.0 - surv)
        ok = [r for r in rs if r["fell"] is None] or rs
        terms[f"cruise{vt}_v"] = ((np.mean([r["mean_v"] for r in ok])
                                   - REAL["cruise_v"][vt]) / 150.0) ** 2
        terms[f"cruise{vt}_sat"] = ((np.mean([r["sat"] for r in ok])
                                     - REAL["cruise_sat"]) / 0.3) ** 2
        terms[f"cruise{vt}_osc"] = ((np.mean([r["osc"] for r in ok])
                                     - REAL["cruise_osc"]) / 3.0) ** 2

    rs = [keep_probe(p, s) for s in seeds]
    surv = np.mean([r["fell"] is None for r in rs])
    terms["keep_fall"] = 8.0 * (1.0 - surv)
    ok = [r for r in rs if r["fell"] is None] or rs
    terms["keep_sigma"] = ((np.mean([r["sigma"] for r in ok])
                            - REAL["keep_sigma"]) / 1.0) ** 2
    terms["keep_ring"] = ((np.mean([r["ring"] for r in ok])
                           - REAL["keep_ring"]) / 3.0) ** 2
    return float(sum(terms.values())), terms


# --- CEM --------------------------------------------------------------------

def fit(generations=30, pop=24, elite=6, seed=0, init=None):
    """init: optional params dict to warm-start from (narrow basins -- the
    standing configuration -- are easy to prove by ablation and hard for a
    cold CEM to find; run 2 got stuck at ~101 from a cold start while a
    standing vector at 60 was already known)."""
    os.makedirs(OUT_DIR, exist_ok=True)
    rng = np.random.default_rng(seed)
    lo = np.array([s[1] for s in SPACE], dtype=float)
    hi = np.array([s[2] for s in SPACE], dtype=float)
    if init is not None:
        mean = np.array([np.clip(init[name], l, h)
                         for (name, l, h, _) in SPACE], dtype=float)
        sigma = (hi - lo) / 8
    else:
        mean, sigma = (lo + hi) / 2, (hi - lo) / 3
    best, best_d, best_terms = float("inf"), None, None
    t0 = time.time()
    for g in range(generations):
        xs = rng.normal(mean, sigma, size=(pop, len(SPACE)))
        scored = []
        for x in xs:
            d = unpack(x)
            loss, terms = evaluate(d)
            scored.append((loss, x, d, terms))
        scored.sort(key=lambda s: s[0])
        if scored[0][0] < best:
            best, _, best_d, best_terms = scored[0]
        el = np.array([s[1] for s in scored[:elite]])
        mean, sigma = el.mean(axis=0), el.std(axis=0) + 1e-3 * (hi - lo)
        print(f"gen {g:>2}  best {scored[0][0]:.3f}  all-time {best:.3f}  "
              f"({time.time() - t0:.0f}s)", flush=True)
        with open(os.path.join(OUT_DIR, "best.json"), "w") as f:
            json.dump({"generation": g, "loss": best, "params": best_d,
                       "terms": best_terms,
                       "elapsed_s": round(time.time() - t0)}, f, indent=2)
    return best, best_d, best_terms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--init", type=str, default=None,
                    help="warm-start from the params in this best.json")
    args = ap.parse_args()
    if args.smoke:
        d = {name: getattr(nominal_params(), name) for name, *_ in SPACE}
        loss, terms = evaluate(d)
        print(f"nominal loss {loss:.3f}")
        for k, v in sorted(terms.items(), key=lambda kv: -kv[1]):
            print(f"  {k:>16} {v:8.3f}")
        return
    init = None
    if args.init:
        with open(args.init) as f:
            init = json.load(f)["params"]
    best, d, terms = fit(generations=args.gens, init=init)
    print(f"\nbest loss {best:.3f}")
    for k, v in d.items():
        print(f"  {k:>18} = {v}")
    for k, v in sorted(terms.items(), key=lambda kv: -kv[1]):
        print(f"  {k:>16} {v:8.3f}")


if __name__ == "__main__":
    main()
