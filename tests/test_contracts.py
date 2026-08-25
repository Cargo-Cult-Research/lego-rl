"""Contract tests: the invariants everything downstream silently assumes.

Each of these guards something that either already broke once or was flagged
by the 2026-08-24 audit as load-bearing and untested: the sign conventions
(env.py's docstring is the contract), the export round-trip (run 22 needed an
on-hardware ABBA to catch what an offline comparison catches for free), the
always-positive reward (the agent once learned to fall), provenance
completeness (three fields sat untagged), and the hub/sim constant mirror.
"""
import ast
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from lego_rl.classical import ClassicalController, pybricks_to_si, si_to_pybricks
from lego_rl.env import (OBS_SCALE, PITCH_LIMIT, PITCH_WEIGHT, POS_WEIGHT,
                         X_LIMIT, BalancerEnv)
from lego_rl.gains import GAINS_SIM_TUNED
from lego_rl.params import (GUESS, PROVENANCE, DomainRandomization,
                            PhysicalParams, nominal_params)
from dataclasses import fields

ROOT = Path(__file__).resolve().parent.parent

CLEAN = {"imu_angle_noise": 0.0, "imu_rate_noise": 0.0,
         "imu_angle_bias": 0.0, "imu_rate_bias": 0.0}


def test_sign_conventions():
    """env.py:3-7 is the contract: +duty drives toward +x and, by reaction,
    tips the body BACK (negative pitch). Every exported policy and every hub
    sign check assumes this."""
    env = BalancerEnv(task="balance", randomize=False,
                      param_override={**CLEAN, "motor_backlash_deg": 0.0})
    env.reset(seed=0)
    # Start exactly upright so the only excitation is the commanded duty.
    env.data.qpos[:] = 0.0
    env.data.qvel[:] = 0.0
    for _ in range(40):
        _, _, term, _, info = env.step([0.4])
        if term:
            break
    pitch, _, wheel, _ = info["true_state"]
    x = env.data.qpos[env._adr["slide_x"][0]]
    assert x > 0.005, f"+duty must drive toward +x (got x={x:.4f})"
    assert pitch < -0.01, f"+duty must tip the body back (got pitch={pitch:.4f})"
    assert wheel > 0, "+wheel must be rotation that rolls toward +x"


def test_delay_fifo_delays_actuation():
    """An action at t is applied at t + delay_ctrl_steps: with a long delay
    the wheel must not react to the first command; with no delay it must."""
    def wheel_rate_after(delay, steps=3):
        env = BalancerEnv(task="balance", randomize=False,
                          param_override={**CLEAN, "motor_backlash_deg": 0.0,
                                          "delay_ctrl_steps": delay})
        env.reset(seed=0)
        env.data.qpos[:] = 0.0
        env.data.qvel[:] = 0.0
        for _ in range(steps):
            _, _, _, _, info = env.step([1.0])
        return abs(info["true_state"][3])

    assert wheel_rate_after(delay=6) < 0.2, "delayed command acted too early"
    assert wheel_rate_after(delay=0) > 1.0, "undelayed command did nothing"


def test_reward_positive_wherever_the_episode_continues():
    """Survival must always beat termination — env.py documents the disaster
    when it did not (return +1990 -> -890, the agent learned to fall). The
    worst case inside the box is both deviations at their limits plus full
    duty effort."""
    worst = 1.0 - PITCH_WEIGHT - POS_WEIGHT - 1e-3
    assert worst > 0, (
        f"reward can go negative inside the termination box (worst {worst}); "
        "PITCH_WEIGHT + POS_WEIGHT must stay under the 1.0 alive bonus")


def test_every_param_is_tagged():
    """params.py enforces this at import; the test states it as a contract
    (the old side-table PROVENANCE dict drifted three fields behind)."""
    names = {f.name for f in fields(PhysicalParams)}
    assert names == set(PROVENANCE), "provenance view out of sync with fields"


# GUESS parameters deliberately NOT randomized, each with its reason. Adding
# a new GUESS without either a DR range or an entry here fails the test.
GUESS_EXEMPT = {
    "axle_half_width": "lumped-cylinder half-span; no effect on the planar "
                       "dynamics, only on the (unused) 3D footprint",
    "lash_damping": "numerical regularizer against free-flight chatter "
                    "inside the deadband, not a physical parameter",
}


def test_every_guess_is_randomized_or_exempt():
    """'An inferred number invites a wide randomization range' (params.py).
    The audit found two GUESSes baked in with zero robustness margin and no
    stated reason."""
    dr = DomainRandomization()
    p = nominal_params()
    probe = dr.sample(p, np.random.default_rng(0))
    varies = {f.name for f in fields(PhysicalParams)
              if getattr(probe, f.name) != getattr(p, f.name)}
    # Fields whose DR entry is currently a pinned (x, x) range still count as
    # covered: the pin is a documented decision in DomainRandomization.
    pinned = {name for name in ("hub_resonance_hz", "drivetrain_stiffness")}
    for name in (k for k, v in PROVENANCE.items() if v == GUESS):
        assert name in varies or name in pinned or name in GUESS_EXEMPT, (
            f"GUESS parameter {name!r} is neither randomized nor exempted — "
            "give it a DomainRandomization range or an entry in GUESS_EXEMPT "
            "with a reason")


def _hubconfig_constants():
    """robot/hubconfig.py imports pybricks, so read its numeric assignments
    via AST instead of importing it."""
    tree = ast.parse((ROOT / "robot" / "hubconfig.py").read_text())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    return out


def test_hub_and_sim_agree_on_shared_constants():
    """The hub's loop constants and the sim's params describe one robot."""
    hub = _hubconfig_constants()
    p = nominal_params()
    assert hub["V_NOM"] == p.v_nominal * 1000, "V_NOM (mV) vs v_nominal (V)"
    assert 1000 / hub["DT"] == p.control_hz, "DT (ms) vs control_hz"


def test_pybricks_si_roundtrip():
    g = np.array(GAINS_SIM_TUNED)
    assert np.allclose(si_to_pybricks(pybricks_to_si(g)), g)


def test_classical_default_is_what_the_robot_runs():
    """ClassicalController() must default to robot/gains.py, not the
    published reference gains — the sim once quietly simulated a control law
    the hardware had abandoned (see run 23)."""
    ctrl = ClassicalController()
    assert np.allclose(si_to_pybricks(ctrl.g), GAINS_SIM_TUNED)
    assert ctrl.friction_comp == 0.0, "friction compensation was retired (run 8)"


def test_pipeline_control_reproduces_the_law():
    """The full export pipeline (fit -> Q12) must reproduce the classical law
    it claims to implement. Run 22 measured +10.8% here on hardware; run 24
    traced it to the fit's unpinned equilibrium gain. This locks the fix at
    operating amplitudes (the tanh's compression above ~40% duty is a known,
    separate limitation)."""
    src = (ROOT / "robot" / "policy_linear_fast.py").read_text()
    ns = {}
    exec(src, ns)  # pure integer arithmetic — runs on CPython
    k = pybricks_to_si(np.array(GAINS_SIM_TUNED))
    for state_deg in [(1, 0, 0, 0), (0, 10, 0, 0), (0, 0, 10, 0),
                      (0, 0, 0, 50), (1, 3, 5, 10)]:
        s = [math.radians(v) for v in state_deg]
        law = float(np.clip(np.dot(s, k), -1.0, 1.0))
        got = ns["act"](s)
        assert abs(got - law) <= 0.03 * abs(law) + 0.005, (
            f"pipeline drifts from the law at {state_deg}: "
            f"law {law:.4f}, pipeline {got:.4f}")


def test_export_roundtrip_matches_sb3(tmp_path):
    """scripts/export_policy.py's MicroPython act() must equal
    model.predict() — the offline check that makes an on-hardware ABBA
    unnecessary for catching export defects."""
    model_zip = ROOT / "runs" / "ppo_balance_seed0.zip"
    if not model_zip.exists():
        pytest.skip("no trained model checkpoint on disk")
    sb3 = pytest.importorskip("stable_baselines3")

    out = tmp_path / "weights.py"
    subprocess.run([sys.executable, str(ROOT / "scripts" / "export_policy.py"),
                    str(model_zip), "--out", str(out)],
                   check=True, capture_output=True)
    src = out.read_text().replace("from umath import exp", "from math import exp")
    ns = {}
    exec(src, ns)

    model = sb3.PPO.load(model_zip, device="cpu")
    rng = np.random.default_rng(0)
    envelope = np.array([math.radians(12), math.radians(150), 1.5, 12.0])
    worst = 0.0
    for s in rng.uniform(-1, 1, (200, 4)) * envelope:
        obs = (s * OBS_SCALE).astype(np.float32)
        want = float(np.asarray(model.predict(obs, deterministic=True)[0]).reshape(-1)[0])
        want = max(-1.0, min(1.0, want))
        got = ns["act"](list(s))
        worst = max(worst, abs(got - want))
    assert worst < 2e-3, f"export drifts from SB3 by {worst:.5f}"
