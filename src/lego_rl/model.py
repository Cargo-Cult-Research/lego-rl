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
