"""Instrumented balancer: same control law, logs telemetry, dumps it over BLE.

Runs RUN_MS after the GREEN launch (or until it falls), then stops and prints
one CSV row per LOG_EVERY control steps so we can see the oscillation
frequency and which term is driving it.

FRICTION_COMP: set to 0 to test whether the +-10 duty deadband compensation is
the thing causing a bang-bang limit cycle.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
from umath import copysign

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = (10.71, 0.87, 0.43, 0.30)
K_SYNC = 0.15
FRICTION_COMP = 10   # duty% added in the direction of travel; 0 disables
FC_RAMP = 4          # ramp the comp in over +-this duty instead of stepping; 0 = hard step
RATE_TAU_MS = 30     # low-pass on the gyro rate term; 0 = raw
MAX_DUTY = 100

DT = 5           # ms -> 200 Hz
V_NOM = 7400
FALL_DEG = 45
RUN_MS = 8000
LOG_EVERY = 4    # -> 50 Hz logging

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED,
      "fc:", FRICTION_COMP, "ramp:", FC_RAMP, "tau:", RATE_TAU_MS)


def comp(d):
    # Coulomb-friction feedforward. A hard copysign() step is a 2*FRICTION_COMP
    # discontinuity at d=0 -- on a body this light that is enough to sustain a
    # limit cycle by itself, so ramp it in linearly over +-FC_RAMP.
    if not FRICTION_COMP:
        return d
    if FC_RAMP:
        return d + FRICTION_COMP * max(-1, min(1, d / FC_RAMP))
    return d + copysign(FRICTION_COMP, d)


def drive(duty, sync):
    scale = V_NOM / hub.battery.voltage()
    left.dc(scale * comp(duty - sync))
    right.dc(scale * comp(duty + sync))


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

t_log = []
p_log = []
r_log = []
d_log = []
a_log = []

ALPHA = DT / (RATE_TAU_MS + DT) if RATE_TAU_MS else 1.0

watch = StopWatch()
n = 0
fell = -1
rate_f = 0.0
while watch.time() < RUN_MS:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    rate = hub.imu.angular_velocity(PITCH_AXIS)
    rate_f += ALPHA * (rate - rate_f)
    if abs(pitch) > FALL_DEG:
        fell = watch.time()
        break
    angle = (left.angle() + right.angle()) / 2
    speed = (left.speed() + right.speed()) / 2
    duty = K_ANGLE * pitch + K_RATE * rate_f + K_MOTOR * angle + K_SPEED * speed
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    sync = K_SYNC * (left.angle() - right.angle())
    drive(duty, sync)
    if n % LOG_EVERY == 0:
        t_log.append(watch.time())
        p_log.append(int(pitch * 10))
        r_log.append(int(rate_f))
        d_log.append(int(duty))
        a_log.append(int(angle))
    n += 1
    wait(max(0, DT * n - watch.time()))

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)
print("stopped, fell_at_ms:", fell, "steps:", n, "elapsed:", watch.time())
if max(abs(a) for a in a_log) < 3:
    print("NOTE: wheels never turned -- robot was held the whole run, not released")
print("t_ms,pitch_x10,rate_dps,duty,wheel_deg")
for i in range(len(t_log)):
    print(t_log[i], p_log[i], r_log[i], d_log[i], a_log[i], sep=",")
print("END")
