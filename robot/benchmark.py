"""One config, one fixed window, numbers comparable across builds.

The sweeps answered "which setting", but comparing across a mechanical change
needs a fixed measurement: same gains, same duration, same statistics. The
configuration comes from hubconfig.py / gains.py, held for BENCH_MS.

Statistics printed: rms (about the arm-time reference — contaminated by gyro
bias drift over long windows, see run 20), sigma (rms about the window mean —
drift-immune), peak, and the duty distribution.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from gains import GAINS_SIM_TUNED, K_SYNC
from hubconfig import (DT, FALL_DEG, MAX_DUTY, PITCH_AXIS, RATE_TAU_MS, V_NOM,
                       make_hub, make_motors, wait_until_upright)

hub = make_hub()
left, right = make_motors()

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED

BENCH_MS = 10000
LOG_EVERY = 4       # -> 50 Hz

N_LOG = BENCH_MS // (DT * LOG_EVERY) + 4
p_buf = [0] * N_LOG      # pre-allocated: the hub OOMs on grown lists
d_buf = [0] * N_LOG
w_buf = [0] * N_LOG
w = 0

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED,
      "tau:", RATE_TAU_MS, "max:", MAX_DUTY)

hub.light.on(Color.RED)
wait_until_upright(hub)

hub.imu.reset_heading(0)
pitch0 = hub.imu.rotation(PITCH_AXIS)
left.reset_angle(0)
right.reset_angle(0)
hub.light.on(Color.GREEN)

alpha = DT / (RATE_TAU_MS + DT)
watch = StopWatch()
t0 = watch.time()
n = 0
rate_filt = 0.0
fell_at = -1
sum_sq = 0.0
sum_p = 0.0
peak = 0.0

while watch.time() - t0 < BENCH_MS:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    rate_filt += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_filt)
    if abs(pitch) > FALL_DEG:
        fell_at = watch.time() - t0
        break
    left_deg = left.angle()
    right_deg = right.angle()
    speed = (left.speed() + right.speed()) / 2
    duty = (K_ANGLE * pitch + K_RATE * rate_filt
            + K_MOTOR * (left_deg + right_deg) / 2 + K_SPEED * speed)
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    scale = V_NOM / hub.battery.voltage()
    sync = K_SYNC * (left_deg - right_deg)
    left.dc(scale * (duty - sync))
    right.dc(scale * (duty + sync))
    sum_sq += pitch * pitch
    sum_p += pitch
    if abs(pitch) > peak:
        peak = abs(pitch)
    if n % LOG_EVERY == 0 and w < N_LOG:
        p_buf[w] = int(pitch * 10)
        d_buf[w] = int(duty)
        w_buf[w] = int((left_deg + right_deg) / 2)
        w += 1
    n += 1
    wait(max(0, t0 + DT * n - watch.time()))

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

elapsed = watch.time() - t0
print("fell_at_ms:", fell_at, "elapsed:", elapsed, "steps:", n)
print("rate:", (1000 * n // elapsed) if elapsed else 0, "Hz (want ~200)")
if n:
    mean = sum_p / n
    var = sum_sq / n - mean * mean
    print("pitch_rms_x100:", int(100 * (sum_sq / n) ** 0.5),
          "sigma_x100:", int(100 * (var if var > 0 else 0.0) ** 0.5),
          "peak_x10:", int(10 * peak))
print("t_ms,seg,pitch_x10,duty,wheel_mean")
for i in range(w):
    print(i * DT * LOG_EVERY, 0, p_buf[i], d_buf[i], w_buf[i], sep=",")
print("END")
