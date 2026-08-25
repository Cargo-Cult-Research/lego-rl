"""Balance forever with the CURRENT BEST controller. Prod the robot.

Runs until you stop it with the hub button. Falls are not fatal: it cuts the
motors, goes RED, and waits for you to stand it back up — knock it over,
catch it, push it around, it re-arms every time.

The controller is the deployed policy (`policy_fast.py` — robot/README.md's
manifest says which network that is), the quietest controller measured on
this robot two crossover sessions running (runs 26, 27). An earlier play.py
ran the classical gains and integrated its own pitch from the first arm, so
the reference walked ~0.2 deg/s for the whole session and long sessions got
visibly shaky (the run 20 mechanism). Now the pitch reference re-zeros at
EVERY re-arm, so drift is bounded per stand, not per session.

(A gyro-bias estimator was tried here and removed: driving the bias estimate
from accumulated wheel travel closes a double integrator with no phase
margin — every gain tested fell in sim while the no-estimator baseline
stood. The real fix is the accelerometer as an absolute tilt reference;
ax/az stay in the report so its sign can be MEASURED before any filter
trusts it. Full obituary: git log -- robot/play.py.)

LED: RED = stand me up · GREEN = live · BLUE = fall, resetting.
No telemetry buffer (the hub OOMs on long logs): running stats every
REPORT_MS — rms/peak are about the arm-time reference, sigma is the
drift-immune statistic.
"""
from pybricks.parameters import Axis, Color
from pybricks.tools import StopWatch, wait
from umath import radians

from hubconfig import (DT, FALL_DEG, MAX_DUTY, PITCH_AXIS, RATE_TAU_MS, V_NOM,
                       make_hub, make_motors, wait_until_upright)
from policy_fast import act

hub = make_hub()
left, right = make_motors()

REPORT_MS = 5000
ALPHA = DT / (RATE_TAU_MS + DT)

print("battery mV:", hub.battery.voltage())
print("controller: the deployed policy (policy_fast). Hub button stops.")
print("LED: RED = stand me up | GREEN = live | BLUE = fall, resetting")

watch = StopWatch()
falls = 0
best_run = 0

while True:
    # ---- RED: wait to be stood up and held still ----------------------
    left.dc(0)
    right.dc(0)
    hub.light.on(Color.RED)
    wait_until_upright(hub)

    hub.imu.reset_heading(0)
    pitch0 = hub.imu.rotation(PITCH_AXIS)   # fresh reference every stand
    left.reset_angle(0)
    right.reset_angle(0)
    hub.light.on(Color.GREEN)

    rate_filt = 0.0
    t0 = watch.time()
    n = 0
    win_n = 0
    win_p = 0.0
    win_sq = 0.0
    win_peak = 0.0
    next_report = REPORT_MS

    # ---- GREEN: balance until it falls --------------------------------
    while True:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        rate_filt += ALPHA * (hub.imu.angular_velocity(PITCH_AXIS) - rate_filt)
        if abs(pitch) > FALL_DEG:
            break
        left_deg = left.angle()
        right_deg = right.angle()
        angle = (left_deg + right_deg) / 2
        # Same 4-vector as balance_policy.py / the sim (see README "The state")
        duty = act([radians(pitch),
                    radians(rate_filt),
                    radians(angle),
                    radians((left.speed() + right.speed()) / 2)]) * 100
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        out = duty * V_NOM / hub.battery.voltage()
        out = max(-100, min(100, out))
        left.dc(out)
        right.dc(out)

        win_p += pitch
        win_sq += pitch * pitch
        win_n += 1
        if abs(pitch) > win_peak:
            win_peak = abs(pitch)

        n += 1
        elapsed = watch.time() - t0
        if elapsed > next_report:
            mean = win_p / win_n
            var = win_sq / win_n - mean * mean
            print("up", elapsed // 1000, "s  rms",
                  int(100 * (win_sq / win_n) ** 0.5) / 100.0,
                  "sigma", int(100 * (var if var > 0 else 0.0) ** 0.5) / 100.0,
                  "peak", int(10 * win_peak) / 10.0,
                  "wheel", int(angle),
                  "ax", int(hub.imu.acceleration(Axis.X)),
                  "az", int(hub.imu.acceleration(Axis.Z)),
                  "falls", falls)
            win_p = 0.0
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
