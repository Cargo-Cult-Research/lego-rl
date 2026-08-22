"""Generate the MJCF model from physical parameters.

Regenerated (and recompiled) per episode so domain randomization can touch
geometry, not just mjModel fields. Compile time at this size is ~1 ms.

Layout: planar chassis (slide x, slide z, hinge pitch about +y, all at axle
height) with a single lumped wheel cylinder on a hinge inside it. The wheel
hinge is exactly what the motor encoder reads: wheel angle RELATIVE to the
chassis. MuJoCo auto-excludes parent-child collisions, so the body box only
collides with the floor (needed for swing-up from lying down).
"""
from .params import PhysicalParams


def build_mjcf(p: PhysicalParams) -> str:
    body_half = 0.9 * p.com_height  # box spans ~0.1*com to ~1.9*com above the axle
    fr = f"{p.ground_friction} 0.005 0.0001"
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
            pos="0 0 {p.com_height}" mass="{p.body_mass}" friction="{fr}"/>
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
