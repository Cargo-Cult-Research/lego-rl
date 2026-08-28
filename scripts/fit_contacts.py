"""Fit the sim's wall-contact model to run 35's real impact traces.

Replicates robot/sysid_collision.py in the 3D model: the classical law
(fitted plant, data/sysid_fit_best.json) settles, slews to a cruise
reference, and drives into an arena wall. The same features measured on
hardware are extracted from the sim trace and CEM fits the contact
parameters (contact_timeconst, contact_dampratio) plus the wall-side
friction until they match.

Real targets (run 35, 11 head-on hits at ~850 deg/s impact speed, one
45-deg glancing at 750 target):
  * wheels collapse ~850 -> ~0 deg/s in ~90 ms
  * peak raw pitch rate 533-689 deg/s (whip INTO the wall)
  * peak pitch excursion 30-45 deg; classical recovery fell 5/11
  * glancing: leading wheel decays ~190 ms before the other, peak rate
    roughly half of head-on

The sim cruises at whatever its limit cycle gives (the fitted plant
tracks ~600-700 at target 300), so speed-scaled features (peak rate,
excursion) are compared per unit of impact speed; the collapse time is
compared absolutely (it is a contact property, not a momentum one).

Usage:
  fit_contacts.py --smoke        one head-on impact at defaults, print trace features
  fit_contacts.py                CEM fit, result to data/sysid_fit/contacts.json
"""
import argparse
import json
import math
import os
from dataclasses import replace

import mujoco
import numpy as np

from lego_rl.gains import GAINS_SIM_TUNED
from lego_rl.model3d import build_mjcf_3d
from lego_rl.params import nominal_params

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED
ALPHA_30 = 5.0 / (30.0 + 5.0)
ARENA = 0.7
V_TARGET = 300           # the cruise target the fitted plant survives
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "data", "sysid_fit", "contacts.json")

# Real anchors, run 35 (per unit impact speed where momentum-scaled)
REAL = {
    "t_stop_ms": 90.0,
    "rate_per_v": 610.0 / 850.0,     # peak raw pitch rate / impact speed
    "pitch_per_v": 38.0 / 850.0,     # peak excursion (deg) / impact speed
    "glance_lead_ms": 190.0,
    "glance_rate_frac": 292.0 / 610.0,   # glancing whip vs head-on whip
}


def fitted_params(overrides=None):
    with open(os.path.join(ROOT, "data", "sysid_fit_best.json")) as f:
        fit = json.load(f)["params"]
    p = replace(nominal_params(), battery_v=7.45, **fit)
    return replace(p, **(overrides or {}))


def sim_impact(p, approach_deg=0.0, seconds=9.0):
    """Drive the classical law into the east wall; return the trace.

    approach_deg rotates the start heading, so the robot strikes the wall
    at that angle off-normal (0 = head-on, 45 = the seg-11 geometry).
    Controller mirrors sysid_collision phase 0/1 with the 30 ms rate EMA
    and closes the loop on FUSED pitch (accelerometer complementary filter
    at p.imu_fusion_hz) -- the fitted plant's cruise only survives with
    fusion, in 2D and 3D alike. Both wheels get the same duty (yaw free).
    """
    model = mujoco.MjModel.from_xml_string(build_mjcf_3d(p, ARENA))
    data = mujoco.MjData(model)
    dofs = {n: model.joint(n).dofadr[0] for n in ("spin_l", "spin_r")}
    chassis = model.body("chassis").id
    gyro = model.sensor("gyro").adr[0]
    accel = model.sensor("accel").adr[0]
    k_fuse = ((1.0 / 200.0) / (1.0 / (2 * math.pi * p.imu_fusion_hz)
                               + 1.0 / 200.0) if p.imu_fusion_hz > 0 else 0.0)
    fused = 0.0
    yaw0 = math.radians(approach_deg)
    data.qpos[0] = -(ARENA - 0.15)          # west side, facing east wall
    data.qpos[3:7] = [math.cos(yaw0 / 2), 0, 0, math.sin(yaw0 / 2)]
    mujoco.mj_forward(model, data)
    steps_per_ctrl = max(1, round(1.0 / (p.control_hz * p.physics_dt)))
    delay = [0.0] * max(1, p.delay_ctrl_steps)
    rate_f, v_ref = 0.0, 0.0
    omega_free = math.radians(p.no_load_speed)

    def torque(duty, dof):
        omega = float(data.qvel[dof])
        tau = p.stall_torque * (duty * p.battery_v / p.v_nominal
                                - omega / omega_free)
        tau = float(np.clip(tau, -1.5 * p.stall_torque, 1.5 * p.stall_torque))
        if abs(omega) > 1e-3:
            tau -= (p.motor_friction_duty * p.stall_torque
                    * math.copysign(1.0, omega))
        return tau

    rows = []
    wheel_angle = 0.0
    for t in range(int(seconds * 200)):
        R = data.xmat[chassis].reshape(3, 3)
        pitch = -math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))
        rate_raw = math.degrees(float(data.sensordata[gyro + 1]))
        rate_f += ALPHA_30 * (rate_raw - rate_f)
        # accel-implied tilt: specific force in the site frame; at rest and
        # tilted +theta it reads f_x = -g sin(theta), f_z = g cos(theta)
        fx = float(data.sensordata[accel + 0])
        fz = float(data.sensordata[accel + 2])
        pitch_acc = -math.degrees(math.atan2(fx, max(abs(fz), 1e-6)))
        fused += rate_raw / 200.0 + k_fuse * (pitch_acc - fused)
        pitch_ctl = fused if k_fuse > 0 else pitch
        wl = math.degrees(float(data.qvel[dofs["spin_l"]]))
        wr = math.degrees(float(data.qvel[dofs["spin_r"]]))
        speed = (wl + wr) / 2
        wheel_angle += speed / 200.0
        if t < 300:
            duty = (K_ANGLE * pitch_ctl + K_RATE * rate_f
                    + K_MOTOR * wheel_angle + K_SPEED * speed)
        else:
            v_ref = min(V_TARGET, v_ref + 150 * 0.005)
            duty = (K_ANGLE * pitch_ctl + K_RATE * rate_f
                    + K_SPEED * (speed - v_ref))
        duty = float(np.clip(duty, -100, 100))
        delay.append(duty)
        duty_now = delay.pop(0)
        for _ in range(steps_per_ctrl):
            data.ctrl[0] = torque(duty_now / 100.0, dofs["spin_l"])
            data.ctrl[1] = torque(duty_now / 100.0, dofs["spin_r"])
            mujoco.mj_step(model, data)
        yaw_rate = math.degrees(float(data.sensordata[gyro + 2]))
        rows.append((t, pitch, rate_raw, yaw_rate, wl, wr,
                     float(data.qpos[0]), float(data.qpos[1])))
        if abs(pitch) > 70:
            break
    return rows


def features(rows):
    """Same measurements the run-35 analysis made on the real traces."""
    # distance to the NEAREST wall -- a 45 deg heading meets the north
    # wall, not the east one (the first version only gated on x and
    # declared every glancing run impact-free)
    near = np.array([max(abs(r[6]), abs(r[7])) for r in rows])
    speeds = np.array([(r[4] + r[5]) / 2 for r in rows])
    # impact = first big single-tick wheel deceleration near a wall
    contact = None
    for i in range(220, len(rows) - 2):
        if near[i] > ARENA - 0.25 and speeds[i] - speeds[i + 2] > 150:
            contact = i
            break
    if contact is None:
        return None
    v_imp = float(np.mean(speeds[contact - 40:contact - 4]))
    below = np.where(speeds[contact:] < 0.05 * max(v_imp, 1.0))[0]
    t_stop = float(below[0] * 5) if len(below) else 500.0
    post = rows[contact:contact + 200]
    peak_rate = max(abs(r[2]) for r in post)
    peak_pitch = max(abs(r[1]) for r in post)
    peak_yaw = max(abs(r[3]) for r in post)
    fell = any(abs(r[1]) > 45 for r in post)
    # per-wheel decay lead (glancing signature)
    def drop_i(col):
        arr = np.array([r[col] for r in rows])
        lo = np.where(arr[contact - 10:] < 0.5 * v_imp)[0]
        return float(lo[0] * 5) if len(lo) else None
    dl, dr = drop_i(4), drop_i(5)
    lead = abs(dl - dr) if (dl is not None and dr is not None) else 0.0
    return {"v_imp": v_imp, "t_stop": t_stop, "peak_rate": peak_rate,
            "peak_pitch": peak_pitch, "peak_yaw": peak_yaw,
            "fell": fell, "lead_ms": lead}


def evaluate(cd):
    p = fitted_params({"contact_timeconst": cd["timeconst"],
                       "contact_dampratio": cd["dampratio"],
                       "wall_friction": cd.get("wall_friction", 1.0)})
    head = features(sim_impact(p, 0.0))
    if head is None or head["v_imp"] < 150:
        return 100.0, {"no_impact": 100.0}
    terms = {
        "t_stop": ((head["t_stop"] - REAL["t_stop_ms"]) / 40.0) ** 2,
        "rate": ((head["peak_rate"] / head["v_imp"]
                  - REAL["rate_per_v"]) / 0.25) ** 2,
        "pitch": ((head["peak_pitch"] / head["v_imp"]
                   - REAL["pitch_per_v"]) / 0.015) ** 2,
    }
    gl = features(sim_impact(p, 45.0))
    if gl is not None and gl["v_imp"] > 150:
        terms["glance_rate"] = ((gl["peak_rate"] / max(head["peak_rate"], 1)
                                 - REAL["glance_rate_frac"]) / 0.2) ** 2
        terms["glance_lead"] = ((gl["lead_ms"] - REAL["glance_lead_ms"])
                                / 100.0) ** 2
    else:
        terms["glance_missing"] = 4.0
    return float(sum(terms.values())), terms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--gens", type=int, default=15)
    args = ap.parse_args()
    if args.smoke:
        p = fitted_params()
        for name, ang in (("head-on", 0.0), ("45 deg", 45.0)):
            f = features(sim_impact(p, ang))
            print(f"{name}: {f}")
        loss, terms = evaluate({"timeconst": 0.02, "dampratio": 1.0})
        print(f"default contact loss {loss:.2f}: "
              + ", ".join(f"{k} {v:.2f}" for k, v in terms.items()))
        return

    rng = np.random.default_rng(0)
    lo = np.array([0.002, 0.3, 0.15])
    hi = np.array([0.06, 1.3, 1.2])
    mean, sigma = (lo + hi) / 2, (hi - lo) / 3
    best, best_d, best_terms = float("inf"), None, None
    for g in range(args.gens):
        xs = np.clip(rng.normal(mean, sigma, size=(12, 3)), lo, hi)
        scored = []
        for x in xs:
            d = {"timeconst": float(x[0]), "dampratio": float(x[1]),
                 "wall_friction": float(x[2])}
            loss, terms = evaluate(d)
            scored.append((loss, x, d, terms))
        scored.sort(key=lambda s: s[0])
        if scored[0][0] < best:
            best, _, best_d, best_terms = scored[0]
        el = np.array([s[1] for s in scored[:4]])
        mean, sigma = el.mean(axis=0), el.std(axis=0) + 0.02 * (hi - lo)
        print(f"gen {g:>2} best {scored[0][0]:.3f} all-time {best:.3f} "
              f"{best_d}", flush=True)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump({"loss": best, "params": best_d, "terms": best_terms},
                  f, indent=2)
    print("best:", best_d, best_terms)


if __name__ == "__main__":
    main()
