import numpy as np

from lego_rl.classical import ClassicalController
from lego_rl.env import BalancerEnv


def test_step_api():
    env = BalancerEnv(task="balance", randomize=True)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (4,)
    for _ in range(300):
        obs, r, term, trunc, info = env.step(env.action_space.sample())
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        if term or trunc:
            obs, _ = env.reset()


def test_seed_determinism():
    def rollout():
        env = BalancerEnv(task="balance", randomize=True)
        obs, _ = env.reset(seed=123)
        total = 0.0
        for _ in range(100):
            obs, r, term, trunc, _ = env.step([0.3])
            total += r
            if term or trunc:
                break
        return total

    assert rollout() == rollout()


def test_falls_over_uncontrolled():
    env = BalancerEnv(task="balance", randomize=False)
    env.reset(seed=1)
    for i in range(2000):
        _, _, term, _, _ = env.step([0.0])
        if term:
            break
    assert term, "open-loop system should fall (it is an inverted pendulum)"


def test_classical_beats_open_loop():
    env = BalancerEnv(task="balance", randomize=False, max_seconds=5.0)
    ctrl = ClassicalController()
    obs, _ = env.reset(seed=2)
    steps = 0
    while True:
        obs, _, term, trunc, _ = env.step(ctrl.act(obs, battery_v=env.p.battery_v))
        steps += 1
        if term or trunc:
            break
    # loose bound on purpose: nominal params are still GUESSes. The real
    # acceptance check is scripts/verify_classical.py.
    assert steps / env.p.control_hz > 1.0


def test_swingup_env():
    env = BalancerEnv(task="swingup", randomize=True)
    obs, _ = env.reset(seed=3)
    assert abs(obs[0]) > 1.0  # starts far from upright (scaled pitch)
    for _ in range(200):
        obs, r, term, trunc, _ = env.step([0.0])
        assert np.all(np.isfinite(obs))
