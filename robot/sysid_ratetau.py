"""SysID: sweep OUR gyro filter upward — how much rate lag can the robot take?

(Lives in robot/, not experimental/: hub programs can only import hubconfig/
gains from robot/ — pybricksdev resolves imports relative to the script's own
directory. And it feeds the sim calibration, which makes it sysID.)

Run 28 found the strongest sim-real inversion the project has: the hardware
NEEDS the 30 ms filter (raw gyro fell at 4.0 s, run 16) while the sim cannot
TOLERATE it (classical falls 6/6 even at zero actuation delay — the sim's
cliff sits between 15 and 30 ms of rate lag). This measures the REAL cliff:
same classical law, RATE_TAU stepped 30 → 45 → 60 → 90 ms, ABBA-ordered
around the 30 ms anchor, sigma + duty histogram per segment.

If the robot sails through 60–90 ms, the sim is lag-fragile by 3x or more
and the calibration hunt (static friction, stall_torque, time constant) has
its target number.

Safest-first per CLAUDE.md: the anchor (30) runs first, then increasing lag.
One continuous balance, ~100 s. LED steps through colors per segment.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from gains import GAINS_SIM_TUNED, K_SYNC
from hubconfig import (DT, FALL_DEG, MAX_DUTY, PITCH_AXIS, V_NOM,
                       make_hub, make_motors, wait_until_upright)

hub = make_hub()
left, right = make_motors()

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED

# ABBA-ish around the anchor: 30 first (safe), each candidate bracketed.
TAUS = (30, 45, 30, 60, 30, 90, 30)
COLORS = (Color.GREEN, Color.YELLOW, Color.GREEN, Color.ORANGE,
          Color.GREEN, Color.MAGENTA, Color.GREEN)
SEG_MS = 12000
SETTLE_MS = 1500     # discarded after each tau switch

print("battery mV:", hub.battery.voltage())
print("taus:", TAUS)
# The color legend prints BEFORE anything happens — the run-27 lesson: the
# operator goes by colors, so every session announces its own mapping.
print("LED: GREEN = 30 ms anchor | YELLOW = 45 | ORANGE = 60 | MAGENTA = 90"
      " | RED = stand me up")
print("seg,tau_ms,battery_mV,rms_x100,mean_x100,sigma_x100,peak_x10,dmax,wheel,fell,rate_hz")

hub.light.on(Color.RED)
wait_until_upright(hub)
hub.imu.reset_heading(0)
pitch0 = hub.imu.rotation(PITCH_AXIS)
left.reset_angle(0)
right.reset_angle(0)

watch = StopWatch()
rate_filt = 0.0      # carried across switches: no cold filter at a new tau
n = 0

for seg, tau in enumerate(TAUS):
    hub.light.on(COLORS[seg])
    alpha = DT / (tau + DT)
    batt = hub.battery.voltage()
    t0 = watch.time()
    k = 0
    sum_p = 0.0
    sum_sq = 0.0
    peak = 0.0
    dmax = 0.0
    m = 0
    fell = 0
    while watch.time() - t0 < SEG_MS:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        rate_filt += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_filt)
        if abs(pitch) > FALL_DEG:
            fell = 1
            break
        la = left.angle()
        ra = right.angle()
        duty = (K_ANGLE * pitch + K_RATE * rate_filt
                + K_MOTOR * (la + ra) / 2
                + K_SPEED * (left.speed() + right.speed()) / 2)
        ad = duty if duty > 0 else -duty
        if ad > dmax:
            dmax = ad
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        out = duty * V_NOM / hub.battery.voltage()
        out = max(-100, min(100, out))
        left.dc(out)
        right.dc(out)
        if watch.time() - t0 > SETTLE_MS:
            sum_p += pitch
            sum_sq += pitch * pitch
            if abs(pitch) > peak:
                peak = abs(pitch)
            m += 1
        k += 1
        n += 1
        wait(max(0, t0 + DT * k - watch.time()))
    if m:
        mean = sum_p / m
        var = sum_sq / m - mean * mean
        print(seg, ",", tau, ",", batt, ",",
              int(100 * (sum_sq / m) ** 0.5), ",", int(100 * mean), ",",
              int(100 * (var if var > 0 else 0.0) ** 0.5), ",",
              int(10 * peak), ",", int(dmax), ",",
              int((left.angle() + right.angle()) / 2), ",", fell, ",",
              (1000 * k // (watch.time() - t0)) if watch.time() > t0 else 0)
    if fell:
        # re-arm and continue the sweep rather than losing the session
        left.dc(0)
        right.dc(0)
        hub.light.on(Color.RED)
        wait_until_upright(hub)
        hub.imu.reset_heading(0)
        pitch0 = hub.imu.rotation(PITCH_AXIS)
        left.reset_angle(0)
        right.reset_angle(0)

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)
print("END")
