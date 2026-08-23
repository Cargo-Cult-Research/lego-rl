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
    # The sim-tuned gains, NOT ClassicalController's published defaults. The
    # defaults (88, 0.35, 0.72, 0.19) are for a tall heavy robot and have been
    # measured to fail on this one: 0% of full episodes in the measured sim,
    # and a 0.755 s fall on hardware. This test used to pass with them only
    # because the bound is loose; modelling the hub compliance tipped it under.
    # Fixing the premise rather than the threshold.
    import numpy as np
    from lego_rl.classical import pybricks_to_si
    # Fully rigid on purpose: this test is about the CONTROLLER, and
    # ClassicalController has no gyro filter. Both compliance models punish an
    # unfiltered rate term, and the drivetrain one does so for a reason we now
    # understand -- it IS the ~11 Hz mode the hardware filter exists to reject.
    # A filter-free controller failing against it is correct behaviour, not a
    # regression. Compliance has its own tests. One thing at a time.
    env = BalancerEnv(task="balance", randomize=False, max_seconds=5.0,
                      param_override={"hub_resonance_hz": 0.0,
                                      "drivetrain_stiffness": 0.0})
    ctrl = ClassicalController(
        gains_si=pybricks_to_si(np.array([10.71, 0.87, 0.43, 0.30])))
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


def test_hub_compliance_matches_requested_mode():
    """The hub mount is load-bearing physics now, so check the model actually
    realises the frequency and damping asked for, and conserves mass."""
    import math
    import mujoco
    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params
    from dataclasses import replace

    p = replace(nominal_params(), hub_resonance_hz=13.0, hub_damping_ratio=0.12)
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    j = m.joint("hub_flex")
    inertia = float(m.body("hub").inertia[1])
    k = float(j.stiffness[0])
    c = float(m.dof_damping[j.dofadr[0]])
    assert math.isclose(math.sqrt(k / inertia) / (2 * math.pi), 13.0, rel_tol=1e-3)
    assert math.isclose(c / (2 * math.sqrt(k * inertia)), 0.12, rel_tol=1e-3)
    # splitting the body must not invent or lose mass
    # the drivetrain spring adds a 1 g motor-side rotor body
    assert math.isclose(float(m.body("chassis").subtreemass[0]),
                        p.body_mass + p.wheel_mass, abs_tol=2e-3)


def test_rigid_model_when_compliance_disabled():
    import mujoco
    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params
    from dataclasses import replace

    m = mujoco.MjModel.from_xml_string(
        build_mjcf(replace(nominal_params(), hub_resonance_hz=0.0)))
    names = [m.joint(i).name for i in range(m.njnt)]
    assert "hub_flex" not in names


def test_imu_sees_flex_but_reward_does_not():
    """The asymmetry the whole model exists for: the policy's observation
    carries the mount's motion, the true state used for reward does not."""
    import math
    import mujoco
    # compliance is default-off, so ask for it explicitly
    env = BalancerEnv(randomize=False,
                      param_override={"hub_resonance_hz": 11.5,
                                      "hub_imu_coupling": 1.0})
    env.reset(seed=0)
    adr = env._adr["hub_flex"]
    env.data.qpos[adr[0]] = math.radians(4.0)
    mujoco.mj_forward(env.model, env.data)
    assert math.isclose(env._imu_state()[0] - env._true_state()[0],
                        math.radians(4.0), abs_tol=1e-9)


def test_wheel_mass_survives_every_model_branch():
    """Regression: the rotor <inertial> override was once emitted
    unconditionally, which discards the wheel geom's mass and silently produced
    a massless-wheel plant whenever backlash was off. Several comparisons ran
    against it before an open-loop mass check caught it."""
    import math
    import mujoco
    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params
    from dataclasses import replace

    base = nominal_params()
    want = base.body_mass + base.wheel_mass
    for lash in (0.0, 2.0):
        for hz in (0.0, 11.5):
            p = replace(base, motor_backlash_deg=lash, hub_resonance_hz=hz)
            m = mujoco.MjModel.from_xml_string(build_mjcf(p))
            got = float(m.body("chassis").subtreemass[0])
            # backlash adds a 1 g rotor body; nothing may LOSE mass
            assert got >= want - 1e-9, f"lash={lash} hz={hz}: {got} < {want}"
            assert got <= want + 2e-3
            wdof = m.dof_M0[m.joint("wheel").dofadr[0]]
            assert wdof > 1e-6, f"lash={lash} hz={hz}: wheel dof inertia {wdof}"


def test_backlash_is_a_deadband_not_a_floppy_joint():
    """The encoder must move while the tyre does not, until the play is taken
    up. This is the property that makes it backlash rather than compliance."""
    import math
    import mujoco
    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params
    from dataclasses import replace

    p = replace(nominal_params(), motor_backlash_deg=2.0, hub_resonance_hz=0.0)
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    d = mujoco.MjData(m)
    wa = m.joint("wheel").qposadr[0]
    la = m.joint("lash").qposadr[0]
    d.ctrl[0] = 0.05
    for _ in range(10):
        mujoco.mj_step(m, d)
    enc = math.degrees(d.qpos[wa])
    tyre = enc + math.degrees(d.qpos[la])
    assert enc > 1.0, f"encoder barely moved: {enc}"
    assert abs(tyre) < 0.5 * enc, f"tyre moved with the encoder: {tyre} vs {enc}"
