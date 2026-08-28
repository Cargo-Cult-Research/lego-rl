"""Physical parameters for the 42124 balancer, with provenance.

Every number is MEASURED (you put it on a scale / timed it), DATASHEET
(community-measured motor data), or GUESS (placeholder for sysID). Training
against GUESS values only smoke-tests the pipeline; before trusting
sim-to-real, burn down the GUESS list — `unmeasured()` prints it.

Provenance is attached to each field's metadata, not kept in a side table:
a side table drifted (three fields ended up untagged and nothing noticed),
and when a second robot splits these params into shared motor/drivetrain
pieces and per-robot bodies, metadata travels with the field. An untagged
field fails at import; the test suite additionally checks that every GUESS
is either domain-randomized or explicitly exempted.
"""
from dataclasses import dataclass, field, fields, replace

MEASURED, DATASHEET, GUESS = "MEASURED", "DATASHEET", "GUESS"
# INFERRED: not observed directly, derived from a fit or from another signal.
# Distinct from MEASURED on purpose -- an inferred number invites a wide
# randomization range, a measured one does not.
INFERRED = "INFERRED"


def _p(prov: str, src: str = ""):
    return {"prov": prov, "src": src}


@dataclass
class PhysicalParams:
    # --- geometry / mass ---
    wheel_radius: float = field(default=0.0375, metadata=_p(
        MEASURED, "calipers on the actual 42124 tire: 75 mm dia, 2026-08-22"))
    axle_half_width: float = field(default=0.06, metadata=_p(
        GUESS, "half-span of the lumped wheel cylinder"))
    wheel_mass: float = field(default=0.056, metadata=_p(
        MEASURED, "kitchen scale, 28 g each (re-weighed 2026-08-27)"))
    body_mass: float = field(default=0.388, metadata=_p(
        MEASURED, "kitchen scale 2026-08-27: 444 g whole robot with head "
                  "- 56 g wheels. (History: 429 g braced no-head 2026-08-22.)"))
    com_height: float = field(default=0.05, metadata=_p(
        MEASURED, "balance the body (no wheels) on a straightedge; "
                  "pre-head. The lumped body must land near this -- "
                  "tests/test_env.py checks it"))
    # LOW com -> fast pendulum (~14 rad/s); raising the hub would make
    # everything easier.

    # --- body composition (5 lumps + head; part weighing + photo 2026-08-27)
    # The real body is 2 motors at the axle, the hub riding high, the beam
    # frame, and a small head on top -- not a uniform box. The box model got
    # the LOCKED topple inertia right by luck (run 36: lambda 7.74 vs 7.46
    # real) but its free-axle inertia and its contact silhouette (where the
    # head hits a wall, and how high) were never anchored to anything.
    # Lump masses are RATIOS: the model normalizes them to body_mass, because
    # the part weighings (459 g + head) and the whole-robot weighing (444 g)
    # disagree by ~3% on a kitchen scale, and the whole-robot number wins.
    # Positions are photo-derived (LEGO hole pitch 8 mm for scale) and closed
    # against the measured 5 cm body com -> INFERRED, randomize accordingly.
    hub_mass: float = field(default=0.250, metadata=_p(
        MEASURED, "kitchen scale 2026-08-27, batteries in"))
    motor_mass: float = field(default=0.054, metadata=_p(
        MEASURED, "kitchen scale 2026-08-27, per motor"))
    frame_mass: float = field(default=0.045, metadata=_p(
        MEASURED, "kitchen scale 2026-08-27: beams + pins + cables"))
    head_mass: float = field(default=0.015, metadata=_p(
        INFERRED, "total went 429 -> 444 g when the head was added"))
    hub_center_z: float = field(default=0.060, metadata=_p(
        INFERRED, "photo 2026-08-27 + closing the measured 5 cm body com"))
    motor_com_z: float = field(default=0.022, metadata=_p(
        INFERRED, "L-motor body extends inward/up from the axle"))
    frame_com_z: float = field(default=0.060, metadata=_p(
        INFERRED, "beams span axle to hub top"))
    head_com_z: float = field(default=0.145, metadata=_p(
        INFERRED, "photo; also sets where a head-on wall hit lands"))
    lumped_body: bool = field(default=True, metadata=_p(
        MEASURED, "use the 5-lump body; False restores the uniform box "
                  "(pre-2026-08-27 plant). Auto-falls-back to the box when "
                  "hub_resonance_hz > 0 (the flex model predates the lumps)"))

    # --- motors (2x Technic L motor, device id 46, one per wheel, direct
    #     drive, electrically synced; values below are PER MOTOR) ---
    n_motors: int = field(default=2, metadata=_p(MEASURED))
    stall_torque: float = field(default=0.25, metadata=_p(
        GUESS, "community numbers vary; lever + kitchen scale would settle it"))
    no_load_speed: float = field(default=1443.0, metadata=_p(
        MEASURED, "sysid_motor.py 2026-08-22: measured 1632 deg/s @ 8.37 V, "
                  "17.7 deg/s per duty%, linear above ~50%"))
    v_nominal: float = field(default=7.4, metadata=_p(
        DATASHEET, "anchor from the Pybricks reference dc() battery scaling"))
    motor_friction_duty: float = field(default=0.10, metadata=_p(
        MEASURED, "sysid_motor.py: no motion at 10% duty, kinetic intercept 7.4%"))
    battery_v: float = field(default=8.37, metadata=_p(
        MEASURED, "8366-8379 mV across bringup runs; 6xAA: ~9.5 fresh, ~6.5 dying"))

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
    # UN-RETIRED 2026-08-24. It was retired on the strength of a probe that
    # cannot support the conclusion:
    #
    #   * the entire signal is 0-6 deg at 1 deg encoder quantisation, so the
    #     "decisive" gap-vs-spring fit ran through five integers;
    #   * the probe reads the angle AFTER dc(0) and a 120 ms settle, by which
    #     time a wound-up spring has partly relaxed. What is left is the gap
    #     plus however much wind-up stiction holds -- and the held fraction
    #     grows with duty, which manufactures the rising line that was read as
    #     proof of a spring;
    #   * fitting gap + spring gives a NEGATIVE intercept (-2.46 deg), which is
    #     not a gap at all but the ~22% stiction delaying the onset.
    #
    # Against that, Urs can feel distinct play by hand and reports no
    # perceptible spring between the motors and the body. A hand resolves
    # backlash better than a 1 deg encoder does. The sim previously could not
    # stand with 2 deg of play -- but the robot does, so the sim was wrong; see
    # the solreflimit note in model.py for what was actually broken.
    motor_backlash_deg: float = field(default=1.0, metadata=_p(
        MEASURED, "by hand (half-width of the deadband at the wheel); the "
                  "1 deg encoder cannot see it"))
    # Contact time constant for the deadband's end stops. NOT a free knob: at
    # MuJoCo's 0.02 s default the stop is so soft that the gap width stops
    # mattering, which is exactly how the deadband got wrongly written off.
    backlash_solref_s: float = field(default=0.001, metadata=_p(
        GUESS, "solver contact stiffness, randomized"))
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
    drivetrain_stiction_duty: float = field(default=0.22, metadata=_p(
        MEASURED, "100 trials 2026-08-23; duty below which nothing winds up "
                  "-> MuJoCo frictionloss on the lash joint, so the "
                  "drivetrain is RIGID in normal operation"))
    # DEFAULT OFF. The STIFFNESS is measured and solid; the claim that it is
    # the source of the ~11 Hz ring did NOT survive being simulated, and that
    # claim is withdrawn. See the note in model.py.
    drivetrain_stiffness: float = field(default=0.0, metadata=_p(
        MEASURED, "2.90 N*m/rad both motors, sysid_backlash_many.py "
                  "2026-08-23. The NUMBER is measured; its role in the 11 Hz "
                  "ring is not, hence default 0.0"))
    drivetrain_damping_ratio: float = field(default=0.10, metadata=_p(GUESS))
    lash_damping: float = field(default=2e-5, metadata=_p(
        GUESS, "N*m*s/rad inside the deadband; small, only there to stop "
               "free-flight chatter"))
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
    #
    # REVISED 2026-08-24, downward and hard. The 10x above was inferred to make
    # a SPRING resonate at the measured 11 Hz, and that claim was withdrawn --
    # it also made the robot undriveable (80 rad/s^2, duty saturated for 1.4 s).
    # With a deadband instead of a spring there is no reason for it to be
    # large, and armature adds directly to the drive inertia: 10x means 11x
    # less wheel acceleration, which a robot that visibly balances does not
    # have. Modest default, wide randomization, driveability asserted in tests.
    motor_inertia_mult: float = field(default=0.3, metadata=_p(
        INFERRED, "x robot inertia reflected at the wheel; from the measured "
                  "mode, not weighed"))
    # Static + coulomb friction on the WHEEL joint itself (gearbox drag,
    # motor-to-chassis). Run 17 measured a ~22% duty breakaway and run 36
    # showed the drivetrain stays locked through a whole topple, so the
    # mechanism is real; its joint-level torque is a FIT quantity. Default
    # off until the sysid fit lands -- one single-knob sweep already showed
    # it does not, alone, explain the lag-fragility.
    wheel_frictionloss: float = field(default=0.0, metadata=_p(
        INFERRED, "N*m; sysid_fit.py fits it against runs 35/36"))

    # The hub's imu.rotation() is accel-fused, not a pure gyro integral --
    # the sim has always fed back the pure integral. A complementary filter
    # at this crossover frequency models the fusion (0 = off). Suspect in
    # the lag-fragility hunt: accel fusion changes the pitch signal's phase
    # exactly where the 30 ms rate filter hurts. Constrained from above by
    # runs 20/26: the reference measurably WALKS ~0.2 deg/s in closed loop,
    # so the real fusion is weak or gated -- a fit that wants it strong is
    # contradicting that measurement.
    imu_fusion_hz: float = field(default=0.0, metadata=_p(
        INFERRED, "sysid_fit.py fits it; Pybricks does not document it"))

    # --- sensing / timing ---
    # Pybricks reports motor.angle() and motor.speed() as INTEGER degrees, and
    # the sim had infinite encoder resolution, which mattered far more than it
    # looks. The quantum is the same size as the backlash gap, and the speed
    # term differentiates it: crossing a 2 deg gap in ~20 ms reads as 100 deg/s
    # from a wheel that has not moved, and K_SPEED=0.30 turns that into 30% of
    # duty built on fiction, twice per limit cycle.
    encoder_quantum_deg: float = field(default=1.0, metadata=_p(
        MEASURED, "Pybricks returns integer degrees"))
    encoder_speed_quantum_dps: float = field(default=1.0, metadata=_p(
        MEASURED, "Pybricks returns integer deg/s"))
    imu_angle_bias: float = field(default=0.0, metadata=_p(
        MEASURED, "deg; defined zero at arm time (the robot re-zeros while "
                  "held still); drift bounded by sysid_imu: ~1 deg / 30 s"))
    imu_rate_bias: float = field(default=0.0, metadata=_p(
        MEASURED, "deg/s; sysid_imu 2026-08-22: -0.03 dps stationary. NOTE "
                  "run 20: in closed loop the effective bias walks (~0.2 "
                  "deg/s) because Pybricks only re-zeros at rest -- the "
                  "randomization range matters more than this default"))
    imu_angle_noise: float = field(default=0.05, metadata=_p(
        GUESS, "deg per-sample std; robot/sysid_imu.py would settle it"))
    imu_rate_noise: float = field(default=0.2, metadata=_p(
        MEASURED, "deg/s per-sample std; sysid_imu 2026-08-22: 0.25 dps"))
    delay_ctrl_steps: int = field(default=4, metadata=_p(
        MEASURED, "sysid_latency 2026-08-22: loop jitter <=1 ms, "
                  "cmd->motion 15-19 ms (incl. stiction)"))
    # The hub low-passes the gyro rate before ANY controller sees it
    # (hubconfig RATE_TAU_MS = 30 — load-bearing since run 3, and the thing
    # that saved the 200 Hz policy in run 16). The sim fed policies the RAW
    # rate, so every policy trained here met ~15 ms of unmodeled phase lag at
    # deployment — discovered while hunting run 27's transfer failure.
    #
    # DEFAULT OFF, and the reason is the strongest open sim-real anchor we
    # have (run 28): with the filter modeled, EVERY controller falls in sim —
    # classical 6/6 even at ZERO actuation delay — while on hardware the same
    # filter is what makes every controller STAND (raw gyro fell at 4.0 s,
    # run 16). The real robot needs what kills the sim robot. Something
    # structural is miscalibrated: the sim plant is far more lag-fragile than
    # the hardware (suspects: no static friction at zero speed, stall_torque
    # GUESS setting the loop gain, pendulum time constant off via com_height
    # effective inertia). Until that hunt lands, enabling this "correct"
    # element makes the sim less like the robot, not more.
    rate_filter_tau_ms: float = field(default=0.0, metadata=_p(
        MEASURED, "the hub's value is 30 (hubconfig RATE_TAU_MS); modeled "
                  "but default-off pending the run 28 lag-fragility hunt"))

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
    hub_resonance_hz: float = field(default=0.0, metadata=_p(
        INFERRED, "a closed-loop gyro frequency (11.5 braced, 10.6 unbraced), "
                  "not a mount measurement -- the flex was never seen"))
    hub_damping_ratio: float = field(default=0.08, metadata=_p(
        GUESS, "the ring persists in closed loop -> lightly damped"))
    hub_mass_frac: float = field(default=0.40, metadata=_p(
        GUESS, "fraction of body_mass in the hub; weigh it to settle this"))
    hub_imu_coupling: float = field(default=1.0, metadata=_p(
        GUESS, "fraction of mount motion the gyro sees. 1.0 assumes the mode "
               "is purely in the pitch plane and the IMU is rigid to the "
               "flexing element -- both unlikely. See model.py"))

    # --- collision contacts (fit_contacts.py fits these to run 35) ---
    # Wall-impact contact behavior: MuJoCo solref (timeconst, dampratio) on
    # the arena walls. timeconst sets how stiff the wall feels, dampratio
    # how dead the impact is (<1 restores energy = bounce). Defaults are
    # MuJoCo's own; they are FICTION until fit to the run 35 traces
    # (wheels 850->0 in ~90 ms, -300 deg/s whip).
    contact_timeconst: float = field(default=0.02, metadata=_p(
        GUESS, "s; fit_contacts.py fits it against run 35"))
    contact_dampratio: float = field(default=1.0, metadata=_p(
        GUESS, "1 = dead contact; fit_contacts.py fits it against run 35"))

    # --- loop / sim ---
    control_hz: float = field(default=200.0, metadata=_p(
        MEASURED, "we set it: DT=5 ms in the Pybricks loop"))
    physics_dt: float = field(default=0.001, metadata=_p(
        MEASURED, "we set it: sim integrator step"))
    ground_friction: float = field(default=1.0, metadata=_p(
        GUESS, "coast-down test would settle it"))


def _assert_all_tagged():
    missing = [f.name for f in fields(PhysicalParams) if "prov" not in f.metadata]
    if missing:
        raise TypeError(
            f"PhysicalParams fields without provenance metadata: {missing} — "
            "every parameter carries its own tag; see the module docstring")


_assert_all_tagged()

# Derived views. PROVENANCE keeps its old shape for existing callers.
PROVENANCE = {f.name: f.metadata["prov"] for f in fields(PhysicalParams)}
SOURCES = {f.name: f.metadata["src"] for f in fields(PhysicalParams)}


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
    # Lump positions are INFERRED from a photo (hole-pitch scale) and the
    # com closure -- randomize over the honest uncertainty, ~+-1 cm, and
    # the head over more (its mass is a difference of two weighings).
    hub_center_z: tuple = (0.050, 0.072)
    motor_com_z: tuple = (0.015, 0.030)
    frame_com_z: tuple = (0.045, 0.075)
    head_com_z: tuple = (0.125, 0.165)
    head_mass: tuple = (0.008, 0.025)
    # Contact params get randomized around whatever fit_contacts.py lands
    # on -- a collision policy that only survives one wall stiffness has
    # overfit to the solver (same lesson as backlash_solref_s).
    contact_timeconst: tuple = (0.005, 0.05)
    contact_dampratio: tuple = (0.5, 1.2)
    wheel_radius_scale: tuple = (0.97, 1.03)
    stall_torque_scale: tuple = (0.70, 1.10)
    no_load_speed_scale: tuple = (0.90, 1.10)
    motor_friction_scale: tuple = (0.5, 2.0)
    ground_friction: tuple = (0.6, 1.4)
    battery_v: tuple = (6.5, 9.4)
    imu_angle_bias: tuple = (-1.0, 1.0)     # deg
    # Now that the bias INTEGRATES into the pitch measurement (env._obs),
    # this range is the reference-walk rate. Hardware measured 0.12-0.2 deg/s
    # (runs 20, 26); +-0.4 is 2x the worst observed. The old +-1.0 predates
    # the integration and would mean +-10 deg of reference error per episode.
    imu_rate_bias: tuple = (-0.4, 0.4)      # deg/s
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
    # LIVE as of 2026-08-24, and deliberately WIDE. The gap is the one bit of
    # drivetrain nonlinearity that is directly perceptible by hand, so it earns
    # a default; but the 1 deg encoder cannot pin its width, so the honest
    # response is to randomize hard over the whole plausible span rather than
    # pretend to a number. 0.3-2.5 deg half-width = 0.6-5.0 deg of total play.
    motor_backlash_deg: tuple = (0.3, 2.5)
    # How hard the teeth are when they meet. Also randomized: this is the
    # parameter whose default MuJoCo gets wrong for gearboxes, and a policy
    # that only works at one contact stiffness has overfit to the solver.
    backlash_solref_s: tuple = (0.0005, 0.004)
    # Wide, because slip biases the measurement low and stall_torque is a
    # guess whose sqrt scales the resulting frequency. This spans roughly
    # 8-16 Hz of drivetrain mode.
    drivetrain_stiffness: tuple = (0.0, 0.0)   # pinned; see hub_resonance_hz
    drivetrain_damping_ratio: tuple = (0.03, 0.30)
    drivetrain_stiction_duty: tuple = (0.12, 0.30)
    motor_inertia_mult: tuple = (0.05, 1.5)   # see the note on the default

    def sample(self, p: PhysicalParams, rng) -> PhysicalParams:
        u = lambda r: float(rng.uniform(r[0], r[1]))
        return replace(
            p,
            body_mass=p.body_mass * u(self.mass_scale),
            wheel_mass=p.wheel_mass * u(self.mass_scale),
            com_height=p.com_height * u(self.com_height_scale),
            hub_center_z=u(self.hub_center_z),
            motor_com_z=u(self.motor_com_z),
            frame_com_z=u(self.frame_com_z),
            head_com_z=u(self.head_com_z),
            head_mass=u(self.head_mass),
            contact_timeconst=u(self.contact_timeconst),
            contact_dampratio=u(self.contact_dampratio),
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
            backlash_solref_s=u(self.backlash_solref_s),
            motor_inertia_mult=u(self.motor_inertia_mult),
            drivetrain_stiffness=u(self.drivetrain_stiffness),
            drivetrain_damping_ratio=u(self.drivetrain_damping_ratio),
            drivetrain_stiction_duty=u(self.drivetrain_stiction_duty),
            delay_ctrl_steps=int(rng.integers(self.delay_ctrl_steps[0],
                                              self.delay_ctrl_steps[1] + 1)),
        )
