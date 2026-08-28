"""Generate the MJCF model from physical parameters.

Regenerated (and recompiled) per episode so domain randomization can touch
geometry, not just mjModel fields. Compile time at this size is ~1 ms.

Layout: planar chassis (slide x, slide z, hinge pitch about +y, all at axle
height) with a single lumped wheel cylinder on a hinge inside it. The wheel
hinge is exactly what the motor encoder reads: wheel angle RELATIVE to the
chassis. MuJoCo auto-excludes parent-child collisions, so the body box only
collides with the floor (needed for swing-up from lying down).

Plus a `hub` body on a stiff, lightly damped hinge, carrying the IMU. This is
not decoration: on hardware the IMU sits in the Technic Hub, the hub is pinned
to the chassis through LEGO connectors that flex, and the resulting ~11 Hz mode
defeated six software fixes before bracing the structure proved it mechanical.
A control loop closed around a sensor that is not rigidly attached to the body
it controls oscillates however it is tuned, and a simulator without this term
cannot teach a policy to reject it -- which is exactly how a policy that
balanced fine at 69 Hz came to clamp 47% of its steps and fall at 200 Hz.

The observation reads the HUB angle; reward and termination read the true
chassis angle. That asymmetry is the whole point.

STATUS: default OFF (hub_resonance_hz = 0), because it is not yet calibrated.

What works. It reproduces the failure the old sim could not: the policy trained
without it survives 10 s on raw gyro in the rigid model and 1.16 s with this
one, matching hardware run 16 where raw gyro clamped 47% of steps and fell at
4.0 s. The mechanism is visible in the traces -- 1 deg of flex at 60 Hz is
377 deg/s of gyro rate, which K_RATE = 0.87 turns into 328% of commanded duty.

What does not. It is too severe at every setting tried. The classical
controller falls in ~1 s where hardware runs it indefinitely, PPO cannot learn
in it at all (episode length 138 of 2000), and no frequency from 8 to 60 Hz nor
any IMU coupling from 1.0 down to 0.05 reconciles both hardware facts.

Two constraints found on the way, worth not rediscovering:

  * Flex amplitude is MASS-INDEPENDENT. Stiffness goes as I*w^2 and the driving
    torque is the hub's own inertial reaction, also proportional to I, so
    theta_flex ~ alpha/w^2 and hub_mass_frac cancels. Confirmed by sweeping it
    0.40 to 0.05 with no effect. Frequency is the only lever on amplitude.
  * The blocker is NOT this model. Calibrating it needs "classical survives" as
    a target, and the sim's classical controller is not the hardware one: the
    CEM gains were tuned filter-free and sit at the edge of stability in the
    rigid sim, so adding the 30 ms filter drops them from 10.00 s to 1.00 s
    with no compliance present at all. On hardware those same gains REQUIRE the
    filter. Until the gains are re-tuned with the filter in the loop, there is
    no usable calibration target.

Next step: CEM again, with the filter in the loop and compliance enabled, and
compare the gains it finds against the ones that actually work on hardware.

BACKLASH (also default off, also not yet credible)

Urs can feel a few degrees of play in the L motor gearbox, and a deadband in
the torque path is a better story for the hardware limit cycle than the mount
mode above: its frequency is set by the loop rather than a spring, which is why
eleven runs found an oscillation that barely moved with gain. Unlike mount
flex, whose amplitude provably cancels with mass, backlash amplitude is a real
free parameter.

The structure is right -- actuator and encoder on the motor-side `wheel` joint,
the tyre hanging off it through a free `lash` hinge that only transmits torque
at its limits, so while the play is taken up the encoder turns and the wheel
does not. Verified directly: 0.05 N*m for 10 ms moves the encoder 2.10 deg and
the tyre 0.03 deg.

What is NOT right: 0.1 deg and 2.0 deg of play give identical closed-loop
results, so the limit is not engaging as a deadband should. Suspect the soft
constraint parameters (solreflimit 2 ms, solimplimit width 0.001 rad = 0.057
deg, comparable to the deadband itself). Until a sweep of the play produces a
monotonic change in behaviour, this is not modelling backlash, it is just
adding a floppy joint.

A BUG WORTH REMEMBERING: the <inertial> override for the motor-side rotor was
first emitted unconditionally, which discards the wheel geom's mass. That
silently produced a MASSLESS-WHEEL plant whenever backlash was off (0.371 kg
instead of 0.429, wheel dof inertia 1e-9), and several comparisons were run
against it before the open-loop mass check caught it. Any <inertial> element
must be emitted only on the branch that needs it.
"""
import math

from .params import PhysicalParams


def body_lumps(p: PhysicalParams):
    """The body as five rigid lumps, from the 2026-08-27 part weighing and
    layout photo: two L motors at the axle, the hub riding high, the beam
    frame, a small head on top. Shared by the planar model (here) and the
    3D roomba model, so both plants carry the same mass distribution AND
    the same contact silhouette -- the head's height decides where a wall
    hit lands, which is load-bearing for the collision work.

    Returns (name, mass, (x, y, z), (hx, hy, hz)) per lump. Masses are the
    weighed parts normalized to body_mass (the whole-robot weighing wins
    over the part tally; kitchen scale disagrees with itself by ~3%).
    """
    y_m = max(0.0, p.axle_half_width - 0.03)   # motors just inside the wheels
    parts = [
        ("motor_l", p.motor_mass, (0, -y_m, p.motor_com_z), (0.015, 0.028, 0.015)),
        ("motor_r", p.motor_mass, (0, y_m, p.motor_com_z), (0.015, 0.028, 0.015)),
        # Technic Hub is 88 x 44 x 26 mm, long axis vertical when standing
        ("hub", p.hub_mass, (0, 0, p.hub_center_z), (0.013, 0.022, 0.044)),
        ("frame", p.frame_mass, (0, 0, p.frame_com_z), (0.004, 0.045, 0.060)),
        ("head", p.head_mass, (0, 0, p.head_com_z), (0.015, 0.025, 0.015)),
    ]
    s = p.body_mass / sum(m for _, m, _, _ in parts)
    return [(n, m * s, pos, size) for n, m, pos, size in parts]


def _lump_geoms(p: PhysicalParams, fr: str) -> str:
    return "".join(
        f"""
      <geom name="{n}" type="box" size="{sx} {sy} {sz}"
            pos="{x} {y} {z}" mass="{m:.12g}" friction="{fr}"/>"""
        for n, m, (x, y, z), (sx, sy, sz) in body_lumps(p))


def build_mjcf(p: PhysicalParams) -> str:
    body_half = 0.9 * p.com_height  # box spans ~0.1*com to ~1.9*com above the axle
    fr = f"{p.ground_friction} 0.005 0.0001"

    # Split the body mass: part rides on the compliant hub mount, the rest is
    # rigid chassis. hub_resonance_hz = 0 restores the old single-body model.
    if p.hub_resonance_hz and p.hub_resonance_hz > 0:
        hub_mass = max(1e-4, p.hub_mass_frac * p.body_mass)
        chassis_mass = max(1e-4, p.body_mass - hub_mass)
        hx, hy, hz = 0.03, 0.04, 0.02
        # box inertia about its own y axis, so the hinge frequency is what we asked for
        i_yy = hub_mass * (hx * hx + hz * hz) / 3.0
        w_n = 2.0 * math.pi * p.hub_resonance_hz
        stiff = i_yy * w_n * w_n
        damp = 2.0 * p.hub_damping_ratio * math.sqrt(stiff * i_yy)
        hub_xml = f"""
        <body name="hub" pos="0 0 {p.com_height}">
          <joint name="hub_flex" type="hinge" axis="0 1 0"
                 stiffness="{stiff:.6g}" damping="{damp:.6g}" range="-25 25" limited="true"/>
          <geom name="hub_geom" type="box" size="{hx} {hy} {hz}"
                mass="{hub_mass}" contype="0" conaffinity="0"/>
        </body>"""
    else:
        chassis_mass = p.body_mass
        hub_xml = ""

    # Drivetrain compliance, MEASURED (2.90 N*m/rad) but DEFAULT OFF, because
    # the story built on top of the measurement did not survive simulation.
    #
    # Run 17 measured the stiffness and computed a mode at sqrt(k/I_robot) =
    # 10.7 Hz, against 10.6 observed. That looked like the answer. It silently
    # assumed the motor side is ANCHORED -- and an anchored motor side cannot
    # accelerate the robot:
    #
    #     motor side     wheel accel        drivetrain mode
    #     light           861 rad/s^2            76 Hz
    #     heavy            80 rad/s^2          11.2 Hz
    #
    # Both requirements cannot hold at once, so compliance between MOTOR and
    # WHEEL cannot be the source of the ring. Simulated with a heavy motor
    # side, the robot saturates its duty for 1.4 s and turns its wheels 14
    # degrees: it simply cannot drive. The measured stiffness stands; the
    # claim that it explains the 11 Hz is WITHDRAWN.
    #
    # What would settle where the compliance actually sits: the run 17 probe
    # held the TYRE and drove the motor, so it measured everything between the
    # motor and a rubber tyre held in a hand -- gearbox, axle, rim AND tyre
    # sidewall together. Holding the BODY and twisting the wheel instead
    # measures the tyre alone. The difference localises it. If most of it is
    # tyre, the mode is the robot rocking on tyre springs with the rim held by
    # the motor, which gives sqrt(k/I_robot) honestly and starves nothing.
    # The actuator and encoder live on the motor-side `wheel` joint and the
    # tyre hangs off it through a `lash` hinge that is a SPRING, so the encoder
    # leads the wheel under load. Was first modelled as a backlash deadband;
    # measurement said otherwise -- the angle grew with torque (0.5 deg at 20%
    # duty, 2.0 at 30%, 3.25 at 45%), which is a spring winding up rather than
    # a gap being crossed.
    wheel_i = 0.5 * p.wheel_mass * p.wheel_radius ** 2
    i_load = (p.body_mass + p.wheel_mass) * p.wheel_radius ** 2 + wheel_i

    # Reflected rotor inertia goes on the joint as ARMATURE, not as an
    # <inertial> override on the body. A geared motor reflects its rotor as
    # N^2*J, which is a joint-space inertia and nothing else -- that is exactly
    # what armature is. It also makes the massless-wheel bug above structurally
    # impossible, because the wheel geom's mass is never displaced.
    armature = p.motor_inertia_mult * i_load

    has_lash = ((p.motor_backlash_deg and p.motor_backlash_deg > 0)
                or (p.drivetrain_stiffness and p.drivetrain_stiffness > 0))
    if has_lash:
        # STICTION. Run 17 measured that nothing deflects below ~22% duty while
        # the robot balances at ~12% mean duty, so the drivetrain sits
        # frictionally LOCKED in normal operation and only frees up on large
        # swings. MuJoCo frictionloss is exactly this: a torque that must be
        # exceeded before the joint moves at all. It is also what keeps a
        # deadband from rattling continuously.
        stick = (p.drivetrain_stiction_duty * p.stall_torque
                 * (p.battery_v / p.v_nominal) * p.n_motors)
        attrs = [f'damping="{p.lash_damping:.6g}"',
                 f'frictionloss="{stick:.6g}"']
        if p.motor_backlash_deg and p.motor_backlash_deg > 0:
            # THE GAP. Free travel within +-half, hard stops at the ends.
            #
            # solreflimit is load-bearing and its default is wrong for this. At
            # MuJoCo's default 0.02 s the limit is so soft that a 0.1 deg
            # deadband lets the encoder lead the tyre by 0.6 deg -- six times
            # the gap -- which is why an earlier sweep of the play changed
            # nothing and the deadband was wrongly written off as unmodellable.
            # It was not modelling a gap, it was modelling mush. At 1 ms the
            # measured lead/half-width is 1.11, 1.07, 1.02 over 0.1-1.0 deg.
            h = p.motor_backlash_deg
            attrs += [f'range="{-h:.6g} {h:.6g}"', 'limited="true"',
                      f'solreflimit="{p.backlash_solref_s:.6g} 1"',
                      'solimplimit="0.95 0.99 1e-5 0.5 2"']
        if p.drivetrain_stiffness and p.drivetrain_stiffness > 0:
            # engaged compliance, if it is switched on as well as the gap
            c = 2.0 * p.drivetrain_damping_ratio * math.sqrt(
                p.drivetrain_stiffness * i_load)
            attrs[0] = f'damping="{c:.6g}"'
            attrs.insert(0, f'stiffness="{p.drivetrain_stiffness:.6g}"')
        # the wheel geom moves onto `tyre`, so `wheels` needs its own token
        # inertia; the physical rotor inertia is the armature above.
        rotor_inertial = ('\n        <inertial pos="0 0 0" mass="1e-3" '
                          'diaginertia="1e-9 1e-9 1e-9"/>')
        lash_open = f"""
        <body name="tyre" pos="0 0 0">
          <joint name="lash" type="hinge" axis="0 1 0"
                 {' '.join(attrs)}/>"""
        lash_close = """
        </body>"""
        # THE BUG THAT MADE BACKLASH LOOK UNSIMULABLE. MuJoCo filters contacts
        # between a body and its PARENT, and nothing else. With no lash the
        # wheel geom sits on `wheels`, a direct child of `chassis`, so the pair
        # is filtered. Adding the lash joint moves the geom onto `tyre`, a
        # GRANDCHILD -- unfiltered. The wheel cylinder (r=0.0375 at the chassis
        # origin) overlaps the body box (z from 0.005 to 0.095), so the wheel
        # instantly collides with the robot's own body at 32.5 mm penetration
        # and stays jammed there, off the floor, forever.
        #
        # That is why every deadband width behaved identically and why the
        # earlier conclusion "the sim cannot stand with 2 deg of play" was
        # reached: it was not measuring backlash, it was measuring a wheel
        # welded into the chassis. Open loop it cost a factor of 365 in wheel
        # speed (6.8 deg/s against 2482 rigid).
        exclude = ("  <contact>\n"
                   '    <exclude body1="chassis" body2="tyre"/>\n'
                   "  </contact>\n")
    else:
        exclude = ""
        # No <inertial> override here: the wheel geom must supply the mass.
        rotor_inertial = ""
        lash_open = lash_close = ""

    # The lumped body replaces the uniform box (2026-08-27); the box remains
    # reachable for comparison and still carries the hub-flex machinery,
    # which predates the lumps and splits the box mass by hub_mass_frac.
    if p.lumped_body and not (p.hub_resonance_hz and p.hub_resonance_hz > 0):
        body_geoms = _lump_geoms(p, fr)
    else:
        body_geoms = f"""
      <geom name="body" type="box" size="0.03 0.045 {body_half}"
            pos="0 0 {p.com_height}" mass="{chassis_mass}" friction="{fr}"/>{hub_xml}"""

    return f"""
<mujoco model="lego_balancer">
  <option timestep="{p.physics_dt}" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" friction="{fr}"/>
    <body name="chassis" pos="0 0 {p.wheel_radius}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="pitch" type="hinge" axis="0 1 0"/>{body_geoms}
      <body name="wheels" pos="0 0 0">
        <joint name="wheel" type="hinge" axis="0 1 0" armature="{armature:.6g}"
               frictionloss="{p.wheel_frictionloss:.6g}"/>
{rotor_inertial}{lash_open}
        <geom name="wheel_geom" type="cylinder"
              size="{p.wheel_radius} {p.axle_half_width}" euler="90 0 0"
              mass="{p.wheel_mass}" friction="{fr}"/>{lash_close}
      </body>
    </body>
  </worldbody>
{exclude}  <actuator>
    <motor name="drive" joint="wheel" gear="1" ctrlrange="-5 5"/>
  </actuator>
</mujoco>
"""
