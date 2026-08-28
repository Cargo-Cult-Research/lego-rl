"""3D MJCF for roomba mode: free-floating chassis, independent wheels, arena.

The balancer's planar model (model.py) lumps both wheels into one cylinder
and cannot yaw. Locomotion needs turning, so this builds the same measured
robot in 3D: a free joint on the chassis, one hinge per wheel at
±axle_half_width, and four arena walls to bounce off.

Deliberately v1-simple relative to the balancer model: NO backlash/lash
joints and NO hub-flex body yet — the deadband machinery lives in the
torque-path parameters and can be ported once roomba balancing works at all.
Noted here so nobody mistakes absence for a claim.

Same provenance'd parameters as the 2D model (params.py); `arena` is the
half-width of the square arena in metres.
"""
from .model import _lump_geoms
from .params import PhysicalParams


def build_mjcf_3d(p: PhysicalParams, arena: float = 0.75) -> str:
    body_half = 0.9 * p.com_height
    fr = f"{p.ground_friction} 0.005 0.0001"
    # Walls must be TALLER than the head (0.35 vs head top ~0.16): run 35's
    # real wall stopped the whole silhouette. The first version's 0.08
    # walls would let the sim robot high-side over a wall its real twin
    # bounces off. priority=1 makes the wall's fitted solref win the
    # contact pair against the default-solref robot geoms.
    wall_h, wall_t = 0.35, 0.02
    wall_contact = (f'solref="{p.contact_timeconst:.6g} '
                    f'{p.contact_dampratio:.6g}" priority="1"')
    # Reflected rotor inertia per wheel: same reasoning as the 2D model,
    # half the robot's inertia reflected at each of two wheels.
    i_load_half = (p.body_mass + p.wheel_mass) * p.wheel_radius ** 2 / 2
    armature = p.motor_inertia_mult * i_load_half
    half_track = p.axle_half_width

    def wall(name, x, y, sx, sy):
        return (f'<geom name="{name}" type="box" pos="{x} {y} {wall_h / 2}" '
                f'size="{sx} {sy} {wall_h / 2}" {wall_contact} '
                f'rgba="0.6 0.6 0.65 1"/>')

    # Same five lumps as the planar model -- identical mass distribution AND
    # contact silhouette (a wall hit lands on the frame/head at real height).
    if p.lumped_body:
        body_geoms = _lump_geoms(p, fr)
    else:
        body_geoms = f"""
      <geom name="body" type="box" size="0.03 0.045 {body_half}"
            pos="0 0 {p.com_height}" mass="{p.body_mass}" friction="{fr}"/>"""

    return f"""
<mujoco model="lego_roomba">
  <option timestep="{p.physics_dt}" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="{arena * 2} {arena * 2} 0.1" friction="{fr}"/>
    {wall("wall_n", 0, arena, arena + wall_t, wall_t)}
    {wall("wall_s", 0, -arena, arena + wall_t, wall_t)}
    {wall("wall_e", arena, 0, wall_t, arena + wall_t)}
    {wall("wall_w", -arena, 0, wall_t, arena + wall_t)}
    <body name="chassis" pos="0 0 {p.wheel_radius}">
      <freejoint name="root"/>
      <site name="imu" pos="0 0 {p.hub_center_z if p.lumped_body else p.com_height}"/>{body_geoms}
      <body name="wheel_l" pos="0 {half_track} 0">
        <joint name="spin_l" type="hinge" axis="0 1 0" armature="{armature:.6g}"
               frictionloss="{p.wheel_frictionloss / 2:.6g}"/>
        <geom name="tyre_l" type="cylinder" size="{p.wheel_radius} 0.008"
              euler="90 0 0" mass="{p.wheel_mass / 2}" friction="{fr}"/>
      </body>
      <body name="wheel_r" pos="0 {-half_track} 0">
        <joint name="spin_r" type="hinge" axis="0 1 0" armature="{armature:.6g}"
               frictionloss="{p.wheel_frictionloss / 2:.6g}"/>
        <geom name="tyre_r" type="cylinder" size="{p.wheel_radius} 0.008"
              euler="90 0 0" mass="{p.wheel_mass / 2}" friction="{fr}"/>
      </body>
    </body>
  </worldbody>
  <contact>
    <exclude body1="chassis" body2="wheel_l"/>
    <exclude body1="chassis" body2="wheel_r"/>
  </contact>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
  <actuator>
    <motor name="drive_l" joint="spin_l" gear="1" ctrlrange="-5 5"/>
    <motor name="drive_r" joint="spin_r" gear="1" ctrlrange="-5 5"/>
  </actuator>
</mujoco>
"""
