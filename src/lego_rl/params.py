"""Physical parameters for the 42124 balancer, with provenance.

Every number is MEASURED (you put it on a scale / timed it), DATASHEET
(community-measured motor data), or GUESS (placeholder for sysID). Training
against GUESS values only smoke-tests the pipeline; before trusting
sim-to-real, burn down the GUESS list — `unmeasured()` prints it.
"""
import math
from dataclasses import dataclass, replace

MEASURED, DATASHEET, GUESS = "MEASURED", "DATASHEET", "GUESS"
# INFERRED: not observed directly, derived from a fit or from another signal.
# Distinct from MEASURED on purpose -- an inferred number invites a wide
# randomization range, a measured one does not.
INFERRED = "INFERRED"


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
    # --- gearbox backlash ---
    # A few degrees of play in the L motor's gearbox, felt by hand. This is a
    # DEADBAND IN THE TORQUE PATH, which is a textbook limit-cycle generator:
    # frequency set by the loop rather than by a spring (so it barely moves
    # with gain, which is what eleven runs of hardware kept showing), and
    # amplitude set by the width of the play.
    #
    # The encoder is on the motor side of the gearbox, so while the play is
    # being taken up the encoder reports rotation the WHEEL is not doing. The
    # controller's wheel-angle and wheel-speed terms are therefore reading a
    # signal that is partly fiction, twice per oscillation.
    # RETIRED: measured 2026-08-23 and it is NOT backlash. A fixed gap reads
    # the same angle at every torque; this grew with duty (0.5 deg at 20%,
    # 2.0 at 30%, 3.25 at 45%), which is a spring being wound up, not a gap
    # being crossed. Kept at 0; drivetrain_stiffness below replaces it.
    motor_backlash_deg: float = 0.0    # half-width of the deadband at the wheel
    # Drivetrain compliance, MEASURED 2026-08-23 (robot/sysid_backlash.py):
    # hold a wheel, sweep the motor to both stops, read the encoder.
    #
    # 100 randomized trials (sysid_backlash_many.py), analysed on the p10 of
    # each duty band because slip can only ADD angle -- one-sided contamination,
    # so the lower tail is the clean estimate. Stiffness comes from the SLOPE
    # (0.112 deg per duty%), which is immune to however much duty is eaten
    # before anything deflects.
    #
    # It is a SPRING, decisively: linear fit residual 0.45 against 6.80 for a
    # constant. A backlash gap would read the same angle at every torque.
    #
    # This is very likely the ~11 Hz that eleven runs chased. Against the
    # robot's inertia reflected at the wheel (M*r^2 + wheel inertia = 6.4e-4)
    # it predicts 11.5 Hz, against 10.6 observed unbraced and 11.5 braced. It
    # also explains what mount flex could not: bracing stiffens the frame the
    # MOTORS sit in, raising k and raising the frequency, which is the
    # direction actually observed.
    #
    # Caveats. stall_torque is still a GUESS and frequency goes as its square
    # root (0.18 -> 9.1 Hz, 0.35 -> 12.6). Turned around, the frequency match
    # is weak evidence that stall_torque is near 0.22-0.28. And the two motors
    # measured nearly 2x apart (portA 3.86, portB 2.20 both-equivalent), which
    # may be unit variation or may be two hands gripping differently.
    #
    # SEPARATE FINDING, and it matters for the controller: nothing deflects
    # until ~22% duty. The motor dead zone is 10%, so ~12% more is absorbed by
    # static friction inside the gearbox. The robot balances at ~12% mean duty,
    # which means it normally sits STICTION-LOCKED and effectively rigid -- the
    # compliance only wakes up on large swings. That is plausibly why the real
    # robot tolerates a resonance the sim could not.
    drivetrain_stiction_duty: float = 0.22   # duty below which nothing winds up
                                             # -> MuJoCo frictionloss on the
                                             # lash joint, so the drivetrain is
                                             # RIGID in normal operation
    # DEFAULT OFF. The STIFFNESS is measured and solid; the claim that it is
    # the source of the ~11 Hz ring did NOT survive being simulated, and that
    # claim is withdrawn. See the note in model.py.
    drivetrain_stiffness: float = 0.0   # N*m/rad both motors; measured 2.90
    drivetrain_damping_ratio: float = 0.10   # GUESS
    lash_damping: float = 2e-5         # N*m*s/rad inside the deadband; small, and
                                       # only there to stop free-flight chatter
    # Motor-side inertia, as a MULTIPLE OF THE ROBOT'S INERTIA REFLECTED AT THE
    # WHEEL -- because that ratio is what sets the drivetrain mode, and getting
    # it wrong is what made three earlier compliance models unusable.
    #
    # f = sqrt(k*(1/Ir + 1/Il))/2pi. At the old parametrisation the rotor came
    # out 0.02x the load, putting the mode at 76 Hz instead of the measured 11
    # and turning the actuator into a velocity source. It needs to be >=10x,
    # i.e. effectively ANCHORED, which is physically right: a geared motor
    # reflects rotor inertia as N^2, and the 22% stiction measured in run 17
    # pins the motor side further.
    #
    #    ratio    mode Hz        ratio    mode Hz
    #     0.02       76.3           10       11.2
    #     1.00       15.1           30       10.9
    #     3.00       12.3        anchored    10.7
    motor_inertia_mult: float = 10.0   # x robot inertia reflected at the wheel
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
    # DEFAULT OFF pending calibration -- see the note in model.py.
    #
    # And treat the 11.5 Hz carefully: the FLEX WAS NEVER OBSERVED. What was
    # observed is a frequency in a closed-loop gyro signal, on one build, with
    # a particular controller and delay in the loop -- from which a mount mode
    # was inferred. The bracing result (81% quieter, 10.6 -> 11.5 Hz) is just
    # as consistent with bracing tightening the frame the MOTORS sit in, which
    # is slop rather than flex. So this is INFERRED, and it gets a wide range
    # over the shape of the model rather than a tight one around a number.
    hub_resonance_hz: float = 0.0      # Hz; 11.5 inferred braced, 10.6 unbraced
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
    "hub_resonance_hz": INFERRED,   # a closed-loop gyro frequency, not a mount
                                    # measurement -- the flex was never seen
    "hub_damping_ratio": GUESS,     # ring persists in closed loop -> lightly damped
    "hub_mass_frac": GUESS,         # weigh the hub separately to settle it
    "hub_imu_coupling": GUESS,      # calibrated, not measured -- see model.py
    "motor_backlash_deg": GUESS,    # retired: measured and it is not backlash
    "drivetrain_stiffness": MEASURED,   # 2.90 N*m/rad, sysid_backlash_many.py
                                        # 2026-08-23. The NUMBER is measured;
                                        # its role in the 11 Hz ring is not.
    "drivetrain_damping_ratio": GUESS,
    "drivetrain_stiction_duty": MEASURED,   # 100 trials 2026-08-23
    "lash_damping": GUESS,
    "motor_inertia_mult": INFERRED,  # from the measured mode, not weighed
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
    # Nothing here was observed directly -- a frequency was inferred from a
    # closed-loop gyro signal on ONE build. So randomize over the SHAPE of the
    # model, not around a number: the range reaches from effectively rigid to
    # far softer than inferred, and the coupling can be near zero. A policy
    # that only works at 11.5 Hz has learned the wrong thing, and so has one
    # that requires the mode to exist at all.
    # PINNED AT ZERO while the model is default-off. A range here overrides the
    # nominal default on every randomized episode, so leaving it open silently
    # trained against a plant known to be broken -- and made the CLASSICAL
    # controller score 2.52 s where it scores 10.00 s rigid. If a model is not
    # trusted enough to be a default, it is not trusted enough to randomize.
    hub_resonance_hz: tuple = (0.0, 0.0)
    hub_damping_ratio: tuple = (0.02, 0.40)
    hub_mass_frac: tuple = (0.15, 0.60)
    hub_imu_coupling: tuple = (0.0, 0.50)
    # Felt by hand as "a few degrees" -- span from almost none to a lot ONCE
    # the deadband is validated. Pinned at zero until then; training against an
    # unvalidated nonlinearity is worse than not modelling it.
    motor_backlash_deg: tuple = (0.0, 0.0)
    # Wide, because slip biases the measurement low and stall_torque is a
    # guess whose sqrt scales the resulting frequency. This spans roughly
    # 8-16 Hz of drivetrain mode.
    drivetrain_stiffness: tuple = (0.0, 0.0)   # pinned; see hub_resonance_hz
    drivetrain_damping_ratio: tuple = (0.03, 0.30)
    drivetrain_stiction_duty: tuple = (0.12, 0.30)
    motor_inertia_mult: tuple = (4.0, 40.0)   # 10-12 Hz across this range

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
            motor_backlash_deg=u(self.motor_backlash_deg),
            motor_inertia_mult=u(self.motor_inertia_mult),
            drivetrain_stiffness=u(self.drivetrain_stiffness),
            drivetrain_damping_ratio=u(self.drivetrain_damping_ratio),
            drivetrain_stiction_duty=u(self.drivetrain_stiction_duty),
            delay_ctrl_steps=int(rng.integers(self.delay_ctrl_steps[0],
                                              self.delay_ctrl_steps[1] + 1)),
        )
