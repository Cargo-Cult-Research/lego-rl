"""Sweep controller configs inside ONE hardware run.

Each config runs SEG_MS with its own LED colour, and the hub computes summary
stats on the fly (no big logs, no memory pressure): limit-cycle frequency from
duty sign flips, pitch RMS and peak, duty saturation fraction, wheel drift.

Stops early if it actually falls, and reports which config killed it.

Hypothesis under test: the 9 Hz limit cycle is saturation-driven, so configs
that keep |duty| off the rail should ring less.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
from umath import sqrt

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

# name, k_angle, k_rate, k_motor, k_speed, max_duty, rate_tau_ms
CONFIGS = (
    ("baseline",   10.71, 0.87, 0.43, 0.30, 100, 30),
    ("half_gain",   5.36, 0.44, 0.22, 0.15, 100, 30),
    ("clamp40",    10.71, 0.87, 0.43, 0.30,  40, 30),
    ("soft_angle",  5.36, 0.87, 0.22, 0.30,  60, 30),
    ("nofilt_low",  5.36, 0.44, 0.22, 0.15,  60,  0),
    ("heavy_filt", 10.71, 0.87, 0.43, 0.30,  60, 60),
)
COLORS = (Color.GREEN, Color.YELLOW, Color.ORANGE, Color.MAGENTA,
          Color.CYAN, Color.WHITE)

K_SYNC = 0.15
FRICTION_COMP = 10
FC_RAMP = 4
DT = 5
V_NOM = 7400
FALL_DEG = 45
SEG_MS = 2500

print("battery mV:", hub.battery.voltage())


def comp(d):
    return d + FRICTION_COMP * max(-1, min(1, d / FC_RAMP))


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

results = []
fell_in = None
watch = StopWatch()
n = 0
rate_f = 0.0

for ci in range(len(CONFIGS)):
    name, ka, kr, km, ks, maxd, tau = CONFIGS[ci]
    alpha = DT / (tau + DT) if tau else 1.0
    hub.light.on(COLORS[ci])
    t_end = watch.time() + SEG_MS
    flips = 0
    prev_duty = 0.0
    sum_sq = 0.0
    peak = 0.0
    n_sat = 0
    n_seg = 0
    w_start = (left.angle() + right.angle()) / 2
    t_start = watch.time()
    while watch.time() < t_end:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        rate_f += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
        if abs(pitch) > FALL_DEG:
            fell_in = name
            break
        angle = (left.angle() + right.angle()) / 2
        speed = (left.speed() + right.speed()) / 2
        duty = ka * pitch + kr * rate_f + km * angle + ks * speed
        duty = max(-maxd, min(maxd, duty))
        sync = K_SYNC * (left.angle() - right.angle())
        scale = V_NOM / hub.battery.voltage()
        left.dc(scale * comp(duty - sync))
        right.dc(scale * comp(duty + sync))
        if duty * prev_duty < 0:
            flips += 1
        prev_duty = duty
        sum_sq += pitch * pitch
        if abs(pitch) > peak:
            peak = abs(pitch)
        if abs(duty) >= maxd - 0.5:
            n_sat += 1
        n_seg += 1
        n += 1
        wait(max(0, DT * n - watch.time()))
    span = (watch.time() - t_start) / 1000
    w_end = (left.angle() + right.angle()) / 2
    if n_seg:
        results.append((name, flips / 2 / span, sqrt(sum_sq / n_seg), peak,
                        100 * n_sat / n_seg, w_end - w_start))
    if fell_in:
        break

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

print("fell_in:", fell_in)
print("config,hz,pitch_rms_deg,pitch_peak_deg,sat_pct,wheel_drift_deg")
for r in results:
    print(r[0], round(r[1], 1), round(r[2], 2), round(r[3], 1),
          round(r[4]), round(r[5]), sep=",")
print("END")
