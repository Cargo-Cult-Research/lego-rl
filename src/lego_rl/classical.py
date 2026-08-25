"""The four-gain reference controller (Pybricks balancer), in sim units.

Pybricks form: duty% = 88*pitch_deg + 0.35*pitch_rate + 0.72*motor_deg
                     + 0.19*motor_speed, then battery + friction compensation.
All four gains positive under our sign conventions: a forward position error
first drives the wheels FORWARD to tip the body back — non-minimum phase.
"""
import math

import numpy as np

from .env import OBS_SCALE
from .gains import FRICTION_COMP, GAINS_REFERENCE, GAINS_SIM_TUNED

# duty% per (deg, deg/s, deg, deg/s) on (pitch, pitch_rate, wheel, wheel_rate)
# Single source: robot/gains.py (via lego_rl.gains).
PYBRICKS_GAINS = np.array(GAINS_REFERENCE, dtype=float)
SIM_TUNED_GAINS = np.array(GAINS_SIM_TUNED, dtype=float)


def pybricks_to_si(g=PYBRICKS_GAINS):
    """-> duty in [-1,1] per (rad, rad/s, rad, rad/s)."""
    return np.asarray(g) / 100.0 * (180.0 / math.pi)


def si_to_pybricks(g):
    return np.asarray(g) * 100.0 * (math.pi / 180.0)


class ClassicalController:
    """Defaults mirror what the ROBOT actually runs (robot/gains.py): the
    sim-tuned gains, friction compensation retired. An earlier default was
    the published reference gains with friction_comp=0.10 — i.e. the sim's
    'classical controller' was a configuration the hardware had abandoned,
    and every default-constructed rollout quietly fell."""

    def __init__(self, gains_si=None, friction_comp=FRICTION_COMP / 100.0,
                 v_nominal=7.4):
        self.g = (pybricks_to_si(SIM_TUNED_GAINS) if gains_si is None
                  else np.asarray(gains_si, dtype=float))
        self.friction_comp = friction_comp
        self.v_nominal = v_nominal

    def act(self, obs_scaled, battery_v=None):
        """battery_v mirrors the robot code: the hub knows its own voltage,
        so the sim controller is allowed to know the sampled one."""
        state = np.asarray(obs_scaled, dtype=float) / OBS_SCALE
        duty = float(state @ self.g)
        if duty != 0.0:
            duty += math.copysign(self.friction_comp, duty)
        if battery_v is not None:
            duty *= self.v_nominal / battery_v
        return np.clip([duty], -1.0, 1.0).astype(np.float32)
