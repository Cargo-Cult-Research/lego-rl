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
"""
import math

from .params import PhysicalParams


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
    return f"""
<mujoco model="lego_balancer">
  <option timestep="{p.physics_dt}" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" friction="{fr}"/>
    <body name="chassis" pos="0 0 {p.wheel_radius}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="pitch" type="hinge" axis="0 1 0"/>
      <geom name="body" type="box" size="0.03 0.045 {body_half}"
            pos="0 0 {p.com_height}" mass="{chassis_mass}" friction="{fr}"/>{hub_xml}
      <body name="wheels" pos="0 0 0">
        <joint name="wheel" type="hinge" axis="0 1 0"/>
        <geom name="wheel_geom" type="cylinder"
              size="{p.wheel_radius} {p.axle_half_width}" euler="90 0 0"
              mass="{p.wheel_mass}" friction="{fr}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="wheel" gear="1" ctrlrange="-5 5"/>
  </actuator>
</mujoco>
"""
