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


class BalancerEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task="balance", randomize=True, max_seconds=None):
        assert task in ("balance", "swingup")
        self.task = task
        self.randomize = randomize
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
        self.steps_per_ctrl = max(1, round(1.0 / (p.control_hz * p.physics_dt)))
        self.max_steps = int(self._max_seconds * p.control_hz)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        p = nominal_params()
        if self.randomize:
            p = self.dr.sample(p, self.np_random)
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
            reward = 1.0 - (pitch / PITCH_LIMIT) ** 2 - 0.1 * x * x - 1e-3 * duty * duty
            terminated = bool(abs(pitch) > PITCH_LIMIT or abs(x) > 2.0)
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

    def _obs(self):
        p = self.p
        s = self._true_state()
        meas = s + np.array([
            math.radians(p.imu_angle_bias)
            + self.np_random.normal(0.0, math.radians(p.imu_angle_noise)),
            math.radians(p.imu_rate_bias)
            + self.np_random.normal(0.0, math.radians(p.imu_rate_noise)),
            0.0,
            0.0,
        ])
        return OBS_SCALE * meas
