"""Balance forever. For prodding the robot and getting a feel for it.

Runs until you stop it with the hub button. Falls are not fatal: it cuts the
motors, goes RED, and waits for you to stand it back up — so you can knock it
over, catch it, push it around, and it re-arms itself each time.

No telemetry buffer. The hub's heap is small and a long run cannot hold one, so
this keeps running scalars only and prints a line every REPORT_MS.

Two things this needs that a 10 s run does not:

1. **Gyro bias estimation.** Pitch is integrated gyro, and this IMU drifts
   ~0.56 deg/s, so a long run slowly convinces the robot that level is a lean.
   A first attempt trimmed the ZERO instead of the bias and failed twice over,
   in ways the 16-fall play session made obvious: an unbounded quantity cannot
   be tracked by a clamped one (0.56 deg/s needs 34 deg of trim after a minute
   and 168 after five), and gating the integrator on |pitch| < 10 shut it off
   exactly when drift had made things bad — drift, worse oscillation, gate
   closes, more drift.

   So estimate the BIAS, which is constant and small, rather than the offset,
   which grows without limit. Pitch is integrated here rather than read from
   imu.rotation() so the estimate can be subtracted before integration; a
   converged bias removes the drift permanently instead of chasing it. The
   evidence it works is the wheel no longer walking one way at ~1 cm/s.
2. **Re-arming**, so a fall is an event rather than the end.

LED: RED waiting for you to stand it up and hold still · GREEN live ·
BLUE briefly on each fall.
"""
from pybricks.parameters import Axis, Color
from pybricks.tools import StopWatch, wait

from gains import GAINS_SIM_TUNED, K_SYNC
from hubconfig import (DT, FALL_DEG, MAX_DUTY, PITCH_AXIS, RATE_TAU_MS, V_NOM,
                       make_hub, make_motors, wait_until_upright)

hub = make_hub()
left, right = make_motors()

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED

# Sustained wheel travel in one direction is the signature of a wrong pitch
# zero, so it is the error signal for the bias estimate. Sized to converge on a
# 0.56 deg/s bias within ~10 s at a few hundred degrees of accumulated travel.
# DISABLED, and the reason is worth keeping. Driving the bias estimate from
# accumulated wheel travel is an unstable loop: wheel position is ALREADY an
# integral of the pitch error, so integrating it again closes a double
# integrator with no phase margin. Tested in closed-loop sim against an
# injected 0.56 deg/s bias, 60 s, six gain combinations spanning 15x -- the
# no-estimator baseline was the ONLY one that stayed up (60.0 s); every
# estimator saturated its clamp and fell between 11.6 and 38.5 s, and adding a
# damping term on wheel speed made it worse rather than better.
#
# The real fix is the accelerometer, which is an absolute tilt reference and
# cannot drift. What blocks it is knowing the sign of its relationship to
# pitch, and this project has already lost an afternoon to a pitch-sign bug --
# so ax/az are logged below and the sign gets MEASURED before any filter
# trusts it.
K_BIAS = 0.0        # deg/s of bias correction, per deg of wheel offset, per s
BIAS_LIMIT = 3.0    # deg/s; far above the measured 0.56, far below anything sane
BIAS_GATE_DEG = 20  # skip the update mid-fall, where wheel angle says nothing
REPORT_MS = 5000

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED, "k_bias:", K_BIAS)
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
    wait_until_upright(hub)

    # No pitch0 read: pitch is integrated from zero at this instant, which the
    # RED phase has just guaranteed is upright and still.
    left.reset_angle(0)
    right.reset_angle(0)
    hub.light.on(Color.GREEN)

    bias = 0.0          # deg/s, estimated gyro zero-rate error
    pitch = 0.0         # integrated here, so bias can be removed before it
    rate_filt = 0.0
    t0 = watch.time()
    n = 0
    win_n = 0
    win_sq = 0.0
    win_peak = 0.0
    next_report = REPORT_MS

    # ---- GREEN: balance until it falls --------------------------------
    while True:
        rate_raw = hub.imu.angular_velocity(PITCH_AXIS)
        pitch += (rate_raw - bias) * DT / 1000.0
        rate_filt += alpha * (rate_raw - rate_filt)
        if abs(pitch) > FALL_DEG:
            break
        left_deg = left.angle()
        right_deg = right.angle()
        angle = (left_deg + right_deg) / 2
        speed = (left.speed() + right.speed()) / 2
        duty = (K_ANGLE * pitch + K_RATE * rate_filt
                + K_MOTOR * angle + K_SPEED * speed)
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        scale = V_NOM / hub.battery.voltage()
        sync = K_SYNC * (left_deg - right_deg)
        left.dc(scale * (duty - sync))
        right.dc(scale * (duty + sync))

        # Driving one way forever means the zero is wrong, which means the bias
        # is wrong. Push the bias until that stops. Unlike a zero-offset trim
        # this converges to a constant and stays there.
        if abs(pitch) < BIAS_GATE_DEG:
            bias += K_BIAS * angle * DT / 1000.0
            if bias > BIAS_LIMIT:
                bias = BIAS_LIMIT
            elif bias < -BIAS_LIMIT:
                bias = -BIAS_LIMIT

        win_sq += pitch * pitch
        win_n += 1
        if abs(pitch) > win_peak:
            win_peak = abs(pitch)

        n += 1
        elapsed = watch.time() - t0
        if elapsed > next_report:
            # ax/az are logged, not used: this project has been bitten by sign
            # assumptions before, so the accelerometer's relationship to pitch
            # gets measured from real data before any complementary filter
            # trusts it.
            print("up", elapsed // 1000, "s  rms",
                  int(100 * (win_sq / win_n) ** 0.5) / 100.0,
                  "peak", int(10 * win_peak) / 10.0,
                  "wheel", int(angle),
                  "bias", int(1000 * bias) / 1000.0,
                  "pitch", int(10 * pitch) / 10.0,
                  "ax", int(hub.imu.acceleration(Axis.X)),
                  "az", int(hub.imu.acceleration(Axis.Z)),
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
