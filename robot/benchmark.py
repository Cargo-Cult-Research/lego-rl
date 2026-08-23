"""One config, one fixed window, numbers comparable across builds.

The sweeps answered "which setting", but comparing across a mechanical change
needs a fixed measurement: same gains, same duration, same statistics. This is
the best configuration found over eleven runs, held for BENCH_MS.

Baseline to beat (2026-08-22, 410 g unbraced, run 9 segment 1):
    pitch RMS 2.24 deg, peak 7.3 deg, ring ~10.6 Hz

If bracing the structure removes the ring, mechanical compliance was the cause
and the airborne tap test simply could not see it -- the tyres were unloaded,
so the one spring actually inside the control loop was never excited.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = 10.71, 0.87, 0.43, 0.30
RATE_TAU_MS = 30
MAX_DUTY = 40
K_SYNC = 0
FRICTION_COMP = 0

DT = 5
V_NOM = 7400
FALL_DEG = 45
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
hub.light.on(Color.GREEN)

alpha = DT / (RATE_TAU_MS + DT)
watch = StopWatch()
t0 = watch.time()
n = 0
rate_f = 0.0
fell_at = -1
sum_sq = 0.0
peak = 0.0

while watch.time() - t0 < BENCH_MS:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    rate_f += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
    if abs(pitch) > FALL_DEG:
        fell_at = watch.time() - t0
        break
    la = left.angle()
    ra = right.angle()
    speed = (left.speed() + right.speed()) / 2
    duty = (K_ANGLE * pitch + K_RATE * rate_f
            + K_MOTOR * (la + ra) / 2 + K_SPEED * speed)
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    scale = V_NOM / hub.battery.voltage()
    sync = K_SYNC * (la - ra)
    left.dc(scale * (duty - sync))
    right.dc(scale * (duty + sync))
    sum_sq += pitch * pitch
    if abs(pitch) > peak:
        peak = abs(pitch)
    if n % LOG_EVERY == 0 and w < N_LOG:
        p_buf[w] = int(pitch * 10)
        d_buf[w] = int(duty)
        w_buf[w] = int((la + ra) / 2)
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
    print("pitch_rms_x100:", int(100 * (sum_sq / n) ** 0.5),
          "peak_x10:", int(10 * peak))
print("t_ms,seg,pitch_x10,duty,wheel_mean")
for i in range(w):
    print(i * DT * LOG_EVERY, 0, p_buf[i], d_buf[i], w_buf[i], sep=",")
print("END")
