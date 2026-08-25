"""MuJoCo Gymnasium env for the two-wheeled inverted pendulum.

Sign conventions — match these on the robot or nothing downstream lines up:
  +pitch = leaning forward (toward +x), rad
  +wheel = wheel rotation that rolls the robot toward +x, rad, measured
           RELATIVE to the chassis (what the motor encoder reads)
  +duty  = drives the wheels toward +x (and, by reaction, tips the body back)

State (SI):     [pitch, pitch_rate, wheel_angle, wheel_rate]
Observation:    OBS_SCALE * (state + IMU bias/noise on the first two)
Action:         duty in [-1, 1]

Actuation model per physics substep: DC motor with back-EMF, parameterized by
stall torque and no-load speed at v_nominal, scaled by battery voltage, minus
coulomb friction (the "+10%" the Pybricks reference compensates). Commands
pass through a FIFO of delay_ctrl_steps control ticks to model loop latency.
"""
import math
from collections import deque

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from .model import build_mjcf
from .params import DomainRandomization, nominal_params

# Fixed hand scaling so a ~stabilizable state maps to roughly [-1, 1].
# Baked into export; linearize.py undoes it via the chain rule.
OBS_SCALE = np.array([3.0, 0.3, 0.03, 0.03])
PITCH_LIMIT = math.radians(30.0)
# Reward weights. Both deviations are normalised by their own termination
# limit, and the weights sum to less than the 1.0 alive bonus, so per-step
# reward is ALWAYS positive and surviving always beats terminating.
#
# That last property is not decoration. The first attempt at fixing this used
# an unnormalised POS_WEIGHT = 10.0 against a 2 m bound, which made the penalty
# reach 40 against an alive bonus of 1 -- so falling over early scored better
# than staying up, and the agent duly learned to fall (episode length 2000 ->
# 674, return +1990 -> -890).
#
# The weights equate a 5 deg lean with a 5 cm drift, which is the trade-off the
# classical controller makes. The previous 0.1-on-raw-metres made a 5 cm drift
# 111x cheaper than a 5 deg lean, and the resulting policy had a wheel-position
# gain 33x below the classical controller it is verified against.
# Reweighted 2026-08-23 after run 18 measured the deficiency ON HARDWARE: the
# learned policy travels 1.9x further than the classical controller and drove
# into a wall. The verifier had flagged exactly this axis in advance (wheel
# gain 0.148 against 0.430, ratio 0.35), so this is responding to a predicted
# and then observed defect, not tuning until a number matches.
#
# TWO ATTEMPTS TO PRESS HARDER ON POSITION, BOTH REVERTED. Run 18 measured the
# policy travelling 1.9x further than the classical controller on hardware, and
# the verifier had flagged the same axis in advance (wheel gain ratio 0.35). So
# the deficiency is real. Neither fix worked, and both failed the SAME way:
#
#                              survival  full   drift   wheel-gain ratio
#   these weights, X_LIM 0.5     10.00s  100%   7.2cm   0.35
#   PITCH_WEIGHT 0.25 -> 0.085    5.15s   13%  23.7cm   0.08
#   X_LIMIT 0.5 -> 0.25           2.33s    0%  25.2cm   0.03
#
# Pressing harder on position made the position gain SMALLER, by two
# independent mechanisms. Two lessons.
#
# On an inverted pendulum, position control is DOWNSTREAM of attitude control
# -- you steer position by leaning -- so defunding pitch shaping to pay for
# position rewards the goal while removing the means.
#
# And tightening the bound terminates episodes early, which starves the agent
# of experience inside the balanced regime, exactly where position control has
# to be learned. Harder task, less data about it, worse at everything.
#
# The next thing to try is a CURRICULUM (start loose, tighten during training)
# or simply more steps, not a different weight. Left alone until then.
PITCH_WEIGHT = 0.25
POS_WEIGHT = 0.70
X_LIMIT = 0.5          # m; also the termination bound


class BalancerEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task="balance", randomize=True, max_seconds=None,
                 param_override=None):
        assert task in ("balance", "swingup")
        self.task = task
        self.randomize = randomize
        # Applied AFTER randomization, so a calibration sweep can pin one
        # parameter while everything else still varies.
        self.param_override = dict(param_override or {})
        self.dr = DomainRandomization()
        self._max_seconds = max_seconds if max_seconds is not None else (
            10.0 if task == "balance" else 20.0)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(4,), dtype=np.float64)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self.p = None
        self._rebuild(nominal_params())

    def _rebuild(self, p):
        if p == self.p:
            mujoco.mj_resetData(self.model, self.data)  # keeps viewer pointers valid
            return
        self.p = p
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(p))
        self.data = mujoco.MjData(self.model)
        self._adr = {}
        for name in ("slide_x", "slide_z", "pitch", "wheel"):
            j = self.model.joint(name)
            self._adr[name] = (int(j.qposadr[0]), int(j.dofadr[0]))
        # Present only when hub_resonance_hz > 0. When it is, the IMU rides the
        # hub, so what the policy SEES is chassis + flex while what the reward
        # and the fall check use is the chassis alone.
        try:
            j = self.model.joint("hub_flex")
            self._adr["hub_flex"] = (int(j.qposadr[0]), int(j.dofadr[0]))
        except KeyError:
            self._adr.pop("hub_flex", None)
        self.steps_per_ctrl = max(1, round(1.0 / (p.control_hz * p.physics_dt)))
        self.max_steps = int(self._max_seconds * p.control_hz)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        p = nominal_params()
        if self.randomize:
            p = self.dr.sample(p, self.np_random)
        if self.param_override:
            from dataclasses import replace as _replace
            p = _replace(p, **self.param_override)
        self._rebuild(p)
        pitch_adr = self._adr["pitch"][0]
        if self.task == "balance":
            self.data.qpos[pitch_adr] = self.np_random.normal(0.0, math.radians(2.0))
            self.data.qvel[self._adr["pitch"][1]] = self.np_random.normal(0.0, math.radians(5.0))
            mujoco.mj_forward(self.model, self.data)
        else:
            self.data.qpos[pitch_adr] = float(self.np_random.choice([-1.0, 1.0])) * math.radians(80.0)
            mujoco.mj_forward(self.model, self.data)
            for _ in range(200):  # settle onto the floor
                self.data.ctrl[0] = 0.0
                mujoco.mj_step(self.model, self.data)
        self._delay = deque([0.0] * p.delay_ctrl_steps)
        self._rate_filt = 0.0
        self._t = 0
        return self._obs(), {}

    def step(self, action):
        duty_cmd = float(np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[0], -1.0, 1.0))
        self._delay.append(duty_cmd)
        duty = self._delay.popleft()
        for _ in range(self.steps_per_ctrl):
            self.data.ctrl[0] = self._motor_torque(duty)
            mujoco.mj_step(self.model, self.data)
        self._t += 1

        pitch = self.data.qpos[self._adr["pitch"][0]]
        x = self.data.qpos[self._adr["slide_x"][0]]
        if self.task == "balance":
            reward = (1.0
                      - PITCH_WEIGHT * (pitch / PITCH_LIMIT) ** 2
                      - POS_WEIGHT * (x / X_LIMIT) ** 2
                      - 1e-3 * duty * duty)
            terminated = bool(abs(pitch) > PITCH_LIMIT or abs(x) > X_LIMIT)
        else:
            reward = math.cos(pitch) - 0.05 * x * x - 1e-3 * duty * duty
            terminated = bool(abs(x) > 3.0)
        truncated = self._t >= self.max_steps
        info = {"true_state": self._true_state(), "duty": duty}
        return self._obs(), reward, terminated, truncated, info

    # -- internals --
    def _motor_torque(self, duty):
        p = self.p
        stall = p.stall_torque * p.n_motors            # both wheels driven identically
        omega = self.data.qvel[self._adr["wheel"][1]]  # rad/s, rotor vs stator
        u = duty * p.battery_v / p.v_nominal           # normalized applied voltage
        omega_free = math.radians(p.no_load_speed)
        tau = stall * (u - omega / omega_free)
        # crude current limit (plugging can exceed stall in the linear model)
        tau = float(np.clip(tau, -1.5 * stall, 1.5 * stall))
        if abs(omega) > 1e-3:
            tau -= p.motor_friction_duty * stall * math.copysign(1.0, omega)
        return tau

    def _true_state(self):
        qp, qv = self.data.qpos, self.data.qvel
        pa, pd = self._adr["pitch"]
        wa, wd = self._adr["wheel"]
        return np.array([qp[pa], qv[pd], qp[wa], qv[wd]])

    def _imu_state(self):
        """What the gyro actually measures: the HUB's absolute angle and rate,
        which is the chassis plus whatever the mount is doing. Identical to the
        true state when the model is rigid."""
        s = self._true_state().copy()
        adr = self._adr.get("hub_flex")
        if adr is not None:
            k = self.p.hub_imu_coupling
            s[0] += k * float(self.data.qpos[adr[0]])
            s[1] += k * float(self.data.qvel[adr[1]])
        return s

    def _obs(self):
        p = self.p
        s = self._imu_state()
        # THE PITCH REFERENCE WALKS. Pybricks integrates the (biased) gyro and
        # only re-zeros while stationary, which a balancer never is — so the
        # measured pitch accumulates imu_rate_bias * t of error (~0.2 deg/s on
        # hardware, runs 20/26). The controller absorbs the false tilt by
        # walking the wheel out along the null line, so the WHEEL channel is
        # drift-contaminated too, emergently. This sim used to model the bias
        # as a constant rate offset without its integral — and the first
        # policy trained with a strong wheel-position gain transferred as a
        # duty-hot mess (run 27): it trusted a channel reality poisons.
        drift = math.radians(p.imu_rate_bias) * (self._t / p.control_hz)
        meas = s + np.array([
            math.radians(p.imu_angle_bias) + drift
            + self.np_random.normal(0.0, math.radians(p.imu_angle_noise)),
            math.radians(p.imu_rate_bias)
            + self.np_random.normal(0.0, math.radians(p.imu_rate_noise)),
            0.0,
            0.0,
        ])
        # The hub low-passes the (noisy, biased) gyro before any controller
        # sees it — hubconfig RATE_TAU_MS, load-bearing since run 3. The sim
        # used to hand back the raw rate, i.e. every policy trained here met
        # ~15 ms of unmodeled rate-channel lag on the robot.
        if p.rate_filter_tau_ms > 0:
            dt_ms = 1000.0 / p.control_hz
            alpha = dt_ms / (p.rate_filter_tau_ms + dt_ms)
            self._rate_filt += alpha * (meas[1] - self._rate_filt)
            meas[1] = self._rate_filt
        # ENCODER QUANTISATION. Pybricks hands back integer degrees and integer
        # deg/s; the sim had infinite resolution, and that is not a rounding
        # detail here. The quantum is the same size as the backlash gap, so the
        # two interact: while the gap is being crossed the encoder reports
        # rotation the wheel is not doing, and the SPEED term differentiates
        # it. A 2 deg gap crossed in ~20 ms reads as 100 deg/s of motion that
        # never happened, which K_SPEED=0.30 turns into 30% of commanded duty,
        # twice per limit cycle. Without this the sim cannot show the rattle.
        q = math.radians(p.encoder_quantum_deg)
        qv = math.radians(p.encoder_speed_quantum_dps)
        if q > 0:
            meas[2] = round(meas[2] / q) * q
        if qv > 0:
            meas[3] = round(meas[3] / qv) * qv
        return OBS_SCALE * meas
