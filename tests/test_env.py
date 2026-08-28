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
    from lego_rl.gains import GAINS_SIM_TUNED
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
        gains_si=pybricks_to_si(np.array(GAINS_SIM_TUNED)))
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


def test_lumped_body_closes_the_measured_com():
    """The lump positions are INFERRED from a photo; the one independent
    anchor is the measured 5 cm balance point of the body WITHOUT wheels,
    taken before the head existed. The head-free lump com must land on it.
    Also: lumps must sum exactly to body_mass (the whole-robot weighing
    wins over the part tally), in both the planar and 3D models."""
    import mujoco
    from lego_rl.model import body_lumps, build_mjcf
    from lego_rl.model3d import build_mjcf_3d
    from lego_rl.params import nominal_params

    p = nominal_params()
    lumps = body_lumps(p)
    assert abs(sum(m for _, m, _, _ in lumps) - p.body_mass) < 1e-9
    no_head = [(m, pos[2]) for n, m, pos, _ in lumps if n != "head"]
    com = sum(m * z for m, z in no_head) / sum(m for m, z in no_head)
    assert abs(com - p.com_height) < 0.005, f"lump com {com:.4f} vs measured"
    for xml in (build_mjcf(p), build_mjcf_3d(p)):
        m = mujoco.MjModel.from_xml_string(xml)
        total = float(m.body("chassis").subtreemass[0])
        assert abs(total - (p.body_mass + p.wheel_mass)) < 2e-3


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


def test_drivetrain_windup_matches_measured_stiffness():
    """The lash joint was a backlash deadband until run 17 measured the play at
    many torques and found it grows with torque -- a spring, not a gap. So the
    property to check is Hooke's law: under a steady torque the drivetrain
    should wind up by tau/k, with the encoder leading the wheel by that much."""
    import math
    import mujoco
    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params

    from dataclasses import replace
    # Spring in isolation: joint friction off, or sub-breakaway torque
    # never reaches it (which is the second assertion).
    p = replace(nominal_params(), drivetrain_stiffness=2.90,
                wheel_frictionloss=0.0)
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    d = mujoco.MjData(m)
    la = m.joint("lash").qposadr[0]
    tau = 0.05
    d.ctrl[0] = tau
    for _ in range(400):          # settle against the spring
        mujoco.mj_step(m, d)
    windup = abs(float(d.qpos[la]))
    expected = tau / p.drivetrain_stiffness
    assert abs(windup - expected) < 0.35 * expected, (
        f"wind-up {math.degrees(windup):.2f} deg, expected "
        f"{math.degrees(expected):.2f} deg for tau/k")
    # With the measured gearbox friction on the joint, the same
    # sub-breakaway torque winds up (almost) NOTHING -- run 17's stiction
    # finding, now a property of the model instead of a surprise.
    p2 = replace(p, wheel_frictionloss=0.09)
    m2 = mujoco.MjModel.from_xml_string(build_mjcf(p2))
    d2 = mujoco.MjData(m2)
    la2 = m2.joint("lash").qposadr[0]
    d2.ctrl[0] = tau
    for _ in range(400):
        mujoco.mj_step(m2, d2)
    assert abs(float(d2.qpos[la2])) < 0.3 * expected, (
        "sub-breakaway torque should stay stiction-locked (run 17)")


def test_wheel_never_collides_with_the_chassis():
    """The bug that made backlash look unsimulable, as a regression test.

    MuJoCo filters contacts between a body and its PARENT only. With no lash
    joint the wheel geom sits on `wheels`, a direct child of `chassis`, so the
    pair is filtered. Adding the lash joint moves the geom onto `tyre`, a
    GRANDCHILD, which is not filtered -- and the wheel cylinder overlaps the
    body box, so the wheel jams inside the robot at 32.5 mm penetration and
    never comes out. Every deadband width then behaves identically, which is
    exactly what was observed and wrongly read as "the deadband does not
    engage". Open loop it cost a factor of 365 in wheel speed.
    """
    from dataclasses import replace

    import mujoco

    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params

    for half in (0.0, 0.5, 1.0, 2.5):
        p = replace(nominal_params(), motor_backlash_deg=half)
        m = mujoco.MjModel.from_xml_string(build_mjcf(p))
        d = mujoco.MjData(m)
        # lifted well clear of the floor: any contact at all is self-collision
        d.qpos[int(m.joint("slide_z").qposadr[0])] = 0.5
        mujoco.mj_forward(m, d)
        assert d.ncon == 0, (
            f"backlash={half}: {d.ncon} self-collisions while airborne; the "
            "wheel is jammed inside the chassis")


def test_deadband_engages_in_proportion_to_its_width():
    """The test the old model.py docstring asked for and never got.

    It was written off with "0.1 deg and 2.0 deg of play give identical
    closed-loop results". Two things were wrong: the self-collision above, and
    MuJoCo's default solreflimit of 0.02 s, at which the limit is so soft that
    a 0.1 deg gap lets the encoder lead the tyre by 0.6 deg -- six times the
    gap. A gap that does not scale is not a gap.
    """
    from dataclasses import replace

    import mujoco
    import numpy as np

    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params

    # The torque must exceed the stiction breakaway (0.22 duty ~ 0.124 N*m) or
    # the gap correctly refuses to open at all -- that is the whole point of
    # modelling stiction, and it is why the robot tolerates the play it has.
    for half in (0.1, 0.25, 0.5, 1.0, 2.0):
        p = replace(nominal_params(), motor_backlash_deg=half)
        m = mujoco.MjModel.from_xml_string(build_mjcf(p))
        d = mujoco.MjData(m)
        d.qpos[int(m.joint("slide_z").qposadr[0])] = 0.5   # isolate the drivetrain
        d.ctrl[0] = 0.30
        for _ in range(40):
            mujoco.mj_step(m, d)
        lead = -np.degrees(d.qpos[int(m.joint("lash").qposadr[0])])
        assert 0.9 < lead / half < 1.15, (
            f"half-width {half} deg produced {lead:.3f} deg of encoder lead "
            f"(ratio {lead / half:.2f}); the deadband is not engaging as a gap")


def test_drivetrain_transmits_torque_with_backlash():
    """A gap changes WHEN torque arrives, never how much of it arrives.

    With the self-collision present this failed by a factor of 365, which is
    what made the sim unable to stand and produced the false conclusion that
    the measured play had to be removed from the model rather than fixed in it.
    """
    from dataclasses import replace

    import mujoco
    import numpy as np

    from lego_rl.model import build_mjcf
    from lego_rl.params import nominal_params

    speeds = []
    for half in (0.0, 1.0):
        p = replace(nominal_params(), motor_backlash_deg=half)
        m = mujoco.MjModel.from_xml_string(build_mjcf(p))
        d = mujoco.MjData(m)
        d.qpos[int(m.joint("slide_z").qposadr[0])] = 0.5
        d.ctrl[0] = 0.05
        for _ in range(200):
            mujoco.mj_step(m, d)
        speeds.append(abs(float(np.degrees(d.qvel[int(m.joint("wheel").dofadr[0])]))))
    rigid, gapped = speeds
    assert gapped > 0.9 * rigid, (
        f"rigid reaches {rigid:.0f} deg/s but the gapped drivetrain only "
        f"{gapped:.0f} deg/s -- torque is being eaten, not merely delayed")


def test_robot_stands_with_the_slop_it_actually_has():
    """Urs can feel the play by hand and the robot balances anyway, so any
    model in which measured slop prevents balancing is a broken model."""
    import numpy as np

    from lego_rl.classical import ClassicalController, pybricks_to_si
    from lego_rl.gains import GAINS_SIM_TUNED
    from lego_rl.env import BalancerEnv

    gains = pybricks_to_si(np.array(GAINS_SIM_TUNED))
    for half in (0.0, 1.0, 2.5):
        env = BalancerEnv(task="balance", randomize=False, max_seconds=6.0,
                          param_override={"motor_backlash_deg": half})
        ctrl = ClassicalController(gains_si=gains, friction_comp=0.0)
        for ep in range(3):
            obs, _ = env.reset(seed=1000 + ep)
            n = 0
            while True:
                obs, _, term, trunc, _ = env.step(
                    ctrl.act(obs, battery_v=env.p.battery_v))
                n += 1
                if term or trunc:
                    break
            assert n / env.p.control_hz >= 6.0 - 1e-6, (
                f"fell after {n / env.p.control_hz:.2f}s with {half} deg of "
                "backlash; the hardware does not")
