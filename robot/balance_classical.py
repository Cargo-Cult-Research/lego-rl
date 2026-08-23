"""Classical four-gain balancer, two-wheel two-motor build. Milestone 0.

Launch ritual: start the program, stand the robot upright on the floor and
hold it still at its balance point. LED: RED = waiting for you to get it
upright and still; GREEN = controller live, release gently. It stops and
prints stats when pitch exceeds FALL_DEG.

Gain sets in duty% per deg / deg/s on (pitch, rate, motor angle, motor speed):
reference = published Pybricks balancer (tall heavy robot); sim_tuned = CEM
in our measured-parameter sim (2026-08-22, held-out 10 s survival 100%).
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
from umath import copysign

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)  # sysid_directions 2026-08-22
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # probe: +CW rolled backward
right = Motor(Port.B, Direction.CLOCKWISE)        # probe: +CW rolled forward
PITCH_AXIS = -Axis.Y  # sign check 2026-08-22: +Y read toward-face tilt as negative

GAINS_REFERENCE = (88, 0.35, 0.72, 0.19)
GAINS_SIM_TUNED = (10.71, 0.87, 0.43, 0.30)
K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED   # <-- pick the set here

# Everything below is measured, not assumed. See data/run_NN_*/ for each.
RATE_TAU_MS = 30  # low-pass on the gyro term. Raw was the single biggest cause
                  # of the original 14 Hz shake (run 1); sweeping down is worse
                  # in both directions tested -- 15 ms and 0 ms both fell
                  # (runs 9, 10).
MAX_DUTY = 40     # clamping authority beat leaving it at 100: lowest pitch RMS
                  # and least drift of the gain sweep (run 5).
K_SYNC = 0        # yaw loop off. Proportional-only and undamped, but innocent:
                  # switching it off changes the pitch ring not at all (run 5).
FRICTION_COMP = 0  # the reference design adds +-10% duty in the direction of
                  # travel. On a body this light that step is a bang-bang
                  # oscillator (run 1); ramping it helped, and removing it
                  # outright halved the peak excursion, 11.7 deg -> 6.6, with
                  # drift still ~1 cm (run 8). Wrong compensation for this robot.
DT = 5          # ms -> 200 Hz
V_NOM = 7400    # mV
FALL_DEG = 45

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED,
      "tau:", RATE_TAU_MS, "max:", MAX_DUTY, "sync:", K_SYNC,
      "fc:", FRICTION_COMP)


def drive(duty, sync):
    scale = V_NOM / hub.battery.voltage()
    l = duty - sync
    r = duty + sync
    if FRICTION_COMP:
        l += copysign(FRICTION_COMP, l)
        r += copysign(FRICTION_COMP, r)
    left.dc(scale * l)
    right.dc(scale * r)


# RED: wait until held upright (accel ~ +g on robot Z) and still for 0.5 s
hub.light.on(Color.RED)
still = 0
while still < 50:
    ok = (hub.imu.acceleration(Axis.Z) > 8000
          and abs(hub.imu.angular_velocity(PITCH_AXIS)) < 2)
    still = still + 1 if ok else 0
    wait(10)

hub.imu.reset_heading(0)
pitch0 = hub.imu.rotation(PITCH_AXIS)
left.reset_angle(0)
right.reset_angle(0)
hub.light.on(Color.GREEN)   # live -- release gently

alpha = DT / (RATE_TAU_MS + DT) if RATE_TAU_MS else 1.0
watch = StopWatch()
n = 0
peak_duty = 0
peak_pitch = 0
sum_sq = 0.0
rate_f = 0.0
while True:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    rate_f += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
    if abs(pitch) > FALL_DEG:
        left.dc(0)
        right.dc(0)
        hub.light.on(Color.BLUE)
        print("fell after", watch.time() / 1000, "s, peak duty", peak_duty)
        break
    angle = (left.angle() + right.angle()) / 2
    speed = (left.speed() + right.speed()) / 2
    duty = (K_ANGLE * pitch
            + K_RATE * rate_f
            + K_MOTOR * angle
            + K_SPEED * speed)
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    if abs(duty) > peak_duty:
        peak_duty = abs(duty)
    if abs(pitch) > peak_pitch:
        peak_pitch = abs(pitch)
    sum_sq += pitch * pitch
    sync = K_SYNC * (left.angle() - right.angle())
    drive(duty, sync)
    n += 1
    # Progress report every 5 s, so a run that is quietly succeeding says so
    # instead of looking identical to a hung program.
    if n % 1000 == 0:
        print("t", watch.time() // 1000, "s  pitch rms",
              int(10 * (sum_sq / n) ** 0.5) / 10, "peak", int(peak_pitch),
              "wheel", int(angle))
    wait(max(0, DT * n - watch.time()))
