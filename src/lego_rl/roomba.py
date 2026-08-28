"""Roomba mode: balance, locomote, bounce off walls, fill the space.

Sign conventions (matching env.py where the two overlap):
  +pitch = leaning forward (toward body +x), rad
  +duty  = drives that wheel toward body +x
  +yaw_rate = counter-clockwise seen from above (gyro z), rad/s

Observation — strictly hub-realizable, no world-frame position (the hub
cannot know where it is):
    [pitch, pitch_rate, yaw_rate, wheel_l_speed, wheel_r_speed]
pitch is integrated-gyro-style with the same walking-reference model as the
balancer (imu_rate_bias integrates); wheel speeds are encoder-relative to
the chassis. Action: [duty_l, duty_r] in [-1, 1].

Reward — exploration, not station-keeping: the arena is a COVERAGE_N^2 grid
and each first visit to a cell pays 1.0; a small alive bonus keeps balancing
worth it, and there is no wall penalty — bouncing is the locomotion
mechanic, not a failure. Termination on |pitch| or |roll| > 30 deg.

The scripted bounce controller in scripts/roomba_baseline.py is this task's
verifier analog: a known-answer controller whose coverage any learned
policy has to beat.
"""
import math
from collections import deque

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from .model3d import build_mjcf_3d
from .params import DomainRandomization, nominal_params

OBS_SCALE_3D = np.array([3.0, 0.3, 0.3, 0.03, 0.03])
TILT_LIMIT = math.radians(30.0)
COVERAGE_N = 10          # coverage grid is COVERAGE_N x COVERAGE_N cells


class RoombaEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize=True, max_seconds=40.0, arena=0.75,
                 param_override=None):
        self.randomize = randomize
        self.arena = arena
        self.param_override = dict(param_override or {})
        self.dr = DomainRandomization()
        self._max_seconds = max_seconds
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(5,), dtype=np.float64)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.p = None
        self._rebuild(nominal_params())

    def _rebuild(self, p):
        if p == self.p:
            mujoco.mj_resetData(self.model, self.data)
            return
        self.p = p
        self.model = mujoco.MjModel.from_xml_string(build_mjcf_3d(p, self.arena))
        self.data = mujoco.MjData(self.model)
        self._chassis = self.model.body("chassis").id
        self._spin_l = self.model.joint("spin_l")
        self._spin_r = self.model.joint("spin_r")
        self._gyro_adr = self.model.sensor("gyro").adr[0]
        self._accel_adr = self.model.sensor("accel").adr[0]
        self.steps_per_ctrl = max(1, round(1.0 / (p.control_hz * p.physics_dt)))
        self.max_steps = int(self._max_seconds * p.control_hz)

    # -- kinematics ---------------------------------------------------------
    def _tilt(self):
        """(pitch, roll) of the chassis, rad. R maps body->world; body x is
        forward, z is up. Leaning forward tips body x below the horizon, so
        pitch = -asin(world-z component of body x)."""
        R = self.data.xmat[self._chassis].reshape(3, 3)
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll = math.asin(max(-1.0, min(1.0, R[2, 1])))
        return pitch, roll

    def _true_state(self):
        pitch, roll = self._tilt()
        gyro = self.data.sensordata[self._gyro_adr:self._gyro_adr + 3]
        wl = float(self.data.qvel[self._spin_l.dofadr[0]])
        wr = float(self.data.qvel[self._spin_r.dofadr[0]])
        # gyro is body-frame: y is the pitch axis, z the yaw axis
        return np.array([pitch, float(gyro[1]), float(gyro[2]), wl, wr]), roll

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        p = nominal_params()
        if self.randomize:
            p = self.dr.sample(p, self.np_random)
        if self.param_override:
            from dataclasses import replace as _replace
            p = _replace(p, **self.param_override)
        self._rebuild(p)
        # random start pose: position well inside the walls, random heading
        m = self.arena - 0.15
        self.data.qpos[0] = self.np_random.uniform(-m, m)
        self.data.qpos[1] = self.np_random.uniform(-m, m)
        yaw = self.np_random.uniform(-math.pi, math.pi)
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(self.model, self.data)
        self._delay = deque([np.zeros(2)] * p.delay_ctrl_steps)
        self._t = 0
        self._rate_filt = 0.0
        self._pitch_fused = 0.0
        self._visited = set()
        self._mark_cell()
        return self._obs(), {}

    def _mark_cell(self) -> int:
        cx = int((self.data.qpos[0] + self.arena) / (2 * self.arena) * COVERAGE_N)
        cy = int((self.data.qpos[1] + self.arena) / (2 * self.arena) * COVERAGE_N)
        cell = (min(max(cx, 0), COVERAGE_N - 1), min(max(cy, 0), COVERAGE_N - 1))
        if cell in self._visited:
            return 0
        self._visited.add(cell)
        return 1

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(2), -1.0, 1.0)
        self._delay.append(a.copy())
        duty = self._delay.popleft()
        for _ in range(self.steps_per_ctrl):
            self.data.ctrl[0] = self._motor_torque(duty[0], self._spin_l)
            self.data.ctrl[1] = self._motor_torque(duty[1], self._spin_r)
            mujoco.mj_step(self.model, self.data)
        self._t += 1

        s, roll = self._true_state()
        new_cell = self._mark_cell()
        reward = (1.0 * new_cell
                  + 0.02                                   # alive
                  - 0.001 * float(duty @ duty))
        terminated = bool(abs(s[0]) > TILT_LIMIT or abs(roll) > TILT_LIMIT)
        truncated = self._t >= self.max_steps
        info = {"true_state": s, "coverage": len(self._visited) / COVERAGE_N ** 2,
                "xy": (float(self.data.qpos[0]), float(self.data.qpos[1]))}
        return self._obs(), reward, terminated, truncated, info

    def _motor_torque(self, duty, joint):
        """One L motor driving one wheel — same DC model as the balancer,
        with PER-MOTOR stall torque."""
        p = self.p
        omega = float(self.data.qvel[joint.dofadr[0]])
        u = duty * p.battery_v / p.v_nominal
        tau = p.stall_torque * (u - omega / math.radians(p.no_load_speed))
        tau = float(np.clip(tau, -1.5 * p.stall_torque, 1.5 * p.stall_torque))
        # No friction here: it lives once, as frictionloss on the spin
        # joints -- see env._motor_torque for the double-counting story.
        return tau

    def _obs(self):
        p = self.p
        s, _ = self._true_state()
        drift = math.radians(p.imu_rate_bias) * (self._t / p.control_hz)
        s = s + np.array([
            math.radians(p.imu_angle_bias) + drift
            + self.np_random.normal(0.0, math.radians(p.imu_angle_noise)),
            math.radians(p.imu_rate_bias)
            + self.np_random.normal(0.0, math.radians(p.imu_rate_noise)),
            self.np_random.normal(0.0, math.radians(p.imu_rate_noise)),
            0.0,
            0.0,
        ])
        # Fitted signal path, same as the balancer env (a policy trained on
        # raw signals meets an unmodeled filter at deployment -- run 16).
        dt = 1.0 / p.control_hz
        if p.imu_fusion_hz > 0:
            fx = float(self.data.sensordata[self._accel_adr + 0])
            fz = float(self.data.sensordata[self._accel_adr + 2])
            pitch_acc = (-math.atan2(fx, max(abs(fz), 1e-6))
                         + self.np_random.normal(
                             0.0, math.radians(p.imu_angle_noise)))
            k = dt / (1.0 / (2 * math.pi * p.imu_fusion_hz) + dt)
            self._pitch_fused += s[1] * dt + k * (pitch_acc - self._pitch_fused)
            s[0] = self._pitch_fused
        if p.rate_filter_tau_ms > 0:
            alpha = (dt * 1000.0) / (p.rate_filter_tau_ms + dt * 1000.0)
            self._rate_filt += alpha * (s[1] - self._rate_filt)
            s[1] = self._rate_filt
        qv = math.radians(p.encoder_speed_quantum_dps)
        if qv > 0:
            s[3] = round(s[3] / qv) * qv
            s[4] = round(s[4] / qv) * qv
        return OBS_SCALE_3D * s
