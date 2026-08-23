"""Physical parameters for the 42124 balancer, with provenance.

Every number is MEASURED (you put it on a scale / timed it), DATASHEET
(community-measured motor data), or GUESS (placeholder for sysID). Training
against GUESS values only smoke-tests the pipeline; before trusting
sim-to-real, burn down the GUESS list — `unmeasured()` prints it.
"""
import math
from dataclasses import dataclass, replace

MEASURED, DATASHEET, GUESS = "MEASURED", "DATASHEET", "GUESS"


@dataclass
class PhysicalParams:
    # --- geometry / mass ---
    wheel_radius: float = 0.0375       # m, measured 75 mm dia 2026-08-22
    axle_half_width: float = 0.06      # m, half-span of the lumped wheel cylinder
    wheel_mass: float = 0.058          # kg, both wheels, measured 29 g each
    body_mass: float = 0.371           # kg, measured: 429 g total - 58 g wheels
                                       # (bracing added 2026-08-22: 410 -> 429 g)
    com_height: float = 0.05           # m, measured. LOW -> fast pendulum (~14 rad/s);
                                       # raising the hub would make everything easier.
    # --- motors (2x Technic L motor, device id 46, one per wheel, direct
    #     drive, electrically synced; values below are PER MOTOR) ---
    n_motors: int = 2
    stall_torque: float = 0.25         # N*m at v_nominal (L motor, weaker than XL)
    no_load_speed: float = 1443.0      # deg/s at v_nominal (measured 1632 @ 8.37 V, 2026-08-22)
    v_nominal: float = 7.4             # V, anchor from the Pybricks reference dc() scaling
    motor_friction_duty: float = 0.10  # measured: dead zone to ~10% duty, kinetic intercept 7.4%
    battery_v: float = 8.37            # V, measured 2026-08-22; 6xAA: ~9.5 fresh, ~6.5 dying
    # --- sensing / timing ---
    imu_angle_bias: float = 0.0        # deg, offset on integrated pitch
    imu_rate_bias: float = 0.0         # deg/s, gyro bias
    imu_angle_noise: float = 0.05      # deg, per-sample std
    imu_rate_noise: float = 0.2        # deg/s, per-sample std
    delay_ctrl_steps: int = 4          # ticks; measured 15-19 ms cmd->motion (incl. stiction)
    # --- hub mount compliance: the reason M0 took eleven runs ---
    # The IMU sits in the hub, and the hub is attached to the chassis through
    # LEGO pins that flex. So the gyro measures the hub twisting on its mount,
    # not the robot's true lean, and a loop closed around it oscillates however
    # it is tuned. Measured on hardware: bracing the structure cut the ring 81%
    # AND RAISED its frequency 10.6 -> 11.5 Hz. Adding mass lowers a resonance;
    # it rose, so stiffness grew faster than mass. That is the signature.
    # Set hub_resonance_hz = 0 to get the old rigid model back.
    # DEFAULT OFF pending calibration -- see the note in model.py. The model
    # reproduces the hardware failure mode but is quantitatively too severe:
    # the classical controller cannot survive it and PPO cannot learn in it
    # (episode length 138 of 2000). Set to 11.5 to enable.
    hub_resonance_hz: float = 0.0      # Hz; measured 11.5 braced, 10.6 unbraced
    hub_damping_ratio: float = 0.08    # GUESS; the ring persists, so lightly damped
    hub_mass_frac: float = 0.40        # GUESS; fraction of body_mass in the hub
    hub_imu_coupling: float = 1.0      # fraction of mount motion the gyro sees.
                                       # 1.0 assumes the mode is purely in the
                                       # pitch plane and the IMU is rigid to the
                                       # flexing element -- both unlikely. See
                                       # the calibration note in model.py.
    # --- loop / sim ---
    control_hz: float = 200.0          # DT=5 ms in the Pybricks loop
    physics_dt: float = 0.001
    ground_friction: float = 1.0


PROVENANCE = {
    "wheel_radius": MEASURED,        # calipers on the actual 42124 tire
    "axle_half_width": GUESS,
    "wheel_mass": MEASURED,          # kitchen scale
    "body_mass": MEASURED,           # kitchen scale, batteries in
    "com_height": MEASURED,          # balance the body (no wheels) on a straightedge
    "n_motors": MEASURED,
    "stall_torque": GUESS,        # community numbers vary; lever + kitchen scale
    "no_load_speed": MEASURED,    # sysid_motor.py 2026-08-22: 17.7 deg/s per duty%, linear above ~50%
    "v_nominal": DATASHEET,       # from the Pybricks reference battery scaling
    "motor_friction_duty": MEASURED,   # sysid_motor.py: no motion at 10% duty, intercept 7.4%
    "battery_v": MEASURED,        # 8366-8379 mV across bringup runs
    "imu_angle_noise": GUESS,     # robot/sysid_imu.py
    "imu_rate_noise": MEASURED,   # sysid_imu 2026-08-22: bias -0.03, drift ~1 deg/30 s
    "delay_ctrl_steps": MEASURED, # sysid_latency 2026-08-22: loop jitter <=1 ms, act 15-19 ms
    "ground_friction": GUESS,
    "hub_resonance_hz": MEASURED,   # run 12: braced 11.5 Hz, unbraced 10.6
    "hub_damping_ratio": GUESS,     # ring persists in closed loop -> lightly damped
    "hub_mass_frac": GUESS,         # weigh the hub separately to settle it
    "hub_imu_coupling": GUESS,      # calibrated, not measured -- see model.py
    "control_hz": MEASURED,       # we set it
}


def nominal_params() -> PhysicalParams:
    return PhysicalParams()


def unmeasured() -> list:
    return [k for k, v in PROVENANCE.items() if v == GUESS]


@dataclass
class DomainRandomization:
    """Per-episode ranges. Scales are multiplicative on the nominal value;
    the rest are absolute. The battery and friction entries exist because the
    Pybricks reference hand-compensates exactly those two domain gaps."""
    mass_scale: tuple = (0.85, 1.15)
    com_height_scale: tuple = (0.90, 1.10)
    wheel_radius_scale: tuple = (0.97, 1.03)
    stall_torque_scale: tuple = (0.70, 1.10)
    no_load_speed_scale: tuple = (0.90, 1.10)
    motor_friction_scale: tuple = (0.5, 2.0)
    ground_friction: tuple = (0.6, 1.4)
    battery_v: tuple = (6.5, 9.4)
    imu_angle_bias: tuple = (-1.0, 1.0)     # deg
    imu_rate_bias: tuple = (-1.0, 1.0)      # deg/s
    imu_angle_noise: tuple = (0.02, 0.10)   # deg
    imu_rate_noise: tuple = (0.05, 0.50)    # deg/s
    delay_ctrl_steps: tuple = (2, 6)        # ticks, i.e. 10-30 ms around measured ~19
    # Only the frequency is measured, and only on one build -- unbraced was
    # 10.6 Hz, braced 11.5, and a rebuild could land anywhere nearby. Damping
    # and mass split are guesses. So randomize widely: a policy that only works
    # at exactly 11.5 Hz has learned the wrong thing.
    hub_resonance_hz: tuple = (8.0, 16.0)
    hub_damping_ratio: tuple = (0.03, 0.20)
    hub_mass_frac: tuple = (0.25, 0.55)
    hub_imu_coupling: tuple = (0.10, 0.40)

    def sample(self, p: PhysicalParams, rng) -> PhysicalParams:
        u = lambda r: float(rng.uniform(r[0], r[1]))
        return replace(
            p,
            body_mass=p.body_mass * u(self.mass_scale),
            wheel_mass=p.wheel_mass * u(self.mass_scale),
            com_height=p.com_height * u(self.com_height_scale),
            wheel_radius=p.wheel_radius * u(self.wheel_radius_scale),
            stall_torque=p.stall_torque * u(self.stall_torque_scale),
            no_load_speed=p.no_load_speed * u(self.no_load_speed_scale),
            motor_friction_duty=p.motor_friction_duty * u(self.motor_friction_scale),
            ground_friction=u(self.ground_friction),
            battery_v=u(self.battery_v),
            imu_angle_bias=u(self.imu_angle_bias),
            imu_rate_bias=u(self.imu_rate_bias),
            imu_angle_noise=u(self.imu_angle_noise),
            imu_rate_noise=u(self.imu_rate_noise),
            hub_resonance_hz=u(self.hub_resonance_hz),
            hub_damping_ratio=u(self.hub_damping_ratio),
            hub_mass_frac=u(self.hub_mass_frac),
            hub_imu_coupling=u(self.hub_imu_coupling),
            delay_ctrl_steps=int(rng.integers(self.delay_ctrl_steps[0],
                                              self.delay_ctrl_steps[1] + 1)),
        )
