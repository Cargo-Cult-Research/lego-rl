"""Balance forever. For prodding the robot and getting a feel for it.

Runs until you stop it with the hub button. Falls are not fatal: it cuts the
motors, goes RED, and waits for you to stand it back up — so you can knock it
over, catch it, push it around, and it re-arms itself each time.

No telemetry buffer. The hub's heap is small and a long run cannot hold one, so
this keeps running scalars only and prints a line every REPORT_MS.

Two things this needs that a 10 s run does not:

1. **Drift trim.** Pitch is integrated gyro, and this IMU drifts ~0.56 deg/s
   (measured, run 1). Over a minute that is 34 degrees: the robot would come to
   believe level is a steep lean and drive off the table. The trim watches
   accumulated wheel travel — the signature of a wrong zero is that the machine
   keeps having to drive one way to stay up — and walks the zero back. It is
   deliberately far slower than the wheel-position gain so the two do not
   fight, and it is clamped, so a bad estimate cannot run away. Set
   K_TRIM = 0 to disable and watch the drift for yourself.
2. **Re-arming**, so a fall is an event rather than the end.

LED: RED waiting for you to stand it up and hold still · GREEN live ·
BLUE briefly on each fall.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

# The configuration M0 closed on. See README "Where it is right now".
K_ANGLE, K_RATE, K_MOTOR, K_SPEED = 10.71, 0.87, 0.43, 0.30
RATE_TAU_MS = 30
MAX_DUTY = 40
K_SYNC = 0
FRICTION_COMP = 0

K_TRIM = 0.005      # deg of zero-correction per second, per deg of wheel offset.
                    # Sized against the measured 0.56 deg/s gyro drift: holding
                    # that correction costs ~110 deg of steady wheel offset,
                    # about 7 cm. Bigger = tighter station-keeping but a faster
                    # integrator fighting the wheel-position gain.
TRIM_LIMIT = 8.0    # deg; a wrong zero can never exceed this
DT = 5
V_NOM = 7400
FALL_DEG = 45
REPORT_MS = 5000

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED, "trim:", K_TRIM)
print("stand it up on RED, release on GREEN. Hub button stops.")

alpha = DT / (RATE_TAU_MS + DT)
watch = StopWatch()
falls = 0
best_run = 0

while True:
    # ---- RED: wait to be stood up and held still ----------------------
    left.dc(0)
    right.dc(0)
    hub.light.on(Color.RED)
    still = 0
    while still < 50:
        ok = (hub.imu.acceleration(Axis.Z) > 8000
              and abs(hub.imu.angular_velocity(PITCH_AXIS)) < 2)
        still = still + 1 if ok else 0
        wait(10)

    pitch0 = hub.imu.rotation(PITCH_AXIS)
    left.reset_angle(0)
    right.reset_angle(0)
    hub.light.on(Color.GREEN)

    trim = 0.0
    rate_f = 0.0
    t0 = watch.time()
    n = 0
    win_n = 0
    win_sq = 0.0
    win_peak = 0.0
    next_report = REPORT_MS

    # ---- GREEN: balance until it falls --------------------------------
    while True:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0 - trim
        rate_f += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
        if abs(pitch) > FALL_DEG:
            break
        la = left.angle()
        ra = right.angle()
        angle = (la + ra) / 2
        speed = (left.speed() + right.speed()) / 2
        duty = (K_ANGLE * pitch + K_RATE * rate_f
                + K_MOTOR * angle + K_SPEED * speed)
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        scale = V_NOM / hub.battery.voltage()
        sync = K_SYNC * (la - ra)
        left.dc(scale * (duty - sync))
        right.dc(scale * (duty + sync))

        # A persistent lean shows up as the robot always driving the same way.
        # Walk the zero toward whatever makes that stop -- but only while it is
        # actually balancing. Mid-shove the wheel angle says nothing about the
        # zero, and integrating it there just injects garbage that has to unwind.
        if abs(pitch) < 10:
            trim += K_TRIM * angle * DT / 1000.0
        if trim > TRIM_LIMIT:
            trim = TRIM_LIMIT
        elif trim < -TRIM_LIMIT:
            trim = -TRIM_LIMIT

        win_sq += pitch * pitch
        win_n += 1
        if abs(pitch) > win_peak:
            win_peak = abs(pitch)

        n += 1
        elapsed = watch.time() - t0
        if elapsed > next_report:
            print("up", elapsed // 1000, "s  rms",
                  int(100 * (win_sq / win_n) ** 0.5) / 100.0,
                  "peak", int(10 * win_peak) / 10.0,
                  "wheel", int(angle), "trim", int(10 * trim) / 10.0,
                  "falls", falls)
            win_sq = 0.0
            win_n = 0
            win_peak = 0.0
            next_report += REPORT_MS
        wait(max(0, t0 + DT * n - watch.time()))

    # ---- fall: report and go round again ------------------------------
    left.dc(0)
    right.dc(0)
    hub.light.on(Color.BLUE)
    lasted = watch.time() - t0
    falls += 1
    if lasted > best_run:
        best_run = lasted
    print("fell after", lasted / 1000.0, "s  (fall", falls,
          ", best", best_run / 1000.0, "s)")
    wait(600)
