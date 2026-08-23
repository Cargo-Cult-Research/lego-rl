"""The other unfiltered differentiator: K_SPEED * motor.speed().

The gyro rate term was guilty -- a raw derivative feeding a 19 ms-delayed loop
on a very light body. motor.speed() is structurally the same thing on the
wheel channel, and no run so far has logged how big that term actually gets.

Three 2.5 s segments, everything else fixed:

  GREEN   K_SPEED = 0.30, raw          as shipped
  YELLOW  K_SPEED = 0                  term removed entirely
  CYAN    K_SPEED = 0.30, 30 ms filter same authority, no high-frequency path

Logs the speed TERM itself (duty% contributed), so its size stops being a
guess either way.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

K_ANGLE, K_RATE, K_MOTOR = 10.71, 0.87, 0.43
K_SPEED = 0.30
MAX_DUTY = 40
RATE_TAU_MS = 30
K_SYNC = 0.0        # exonerated by sweep_sync.py; off so it cannot confound

# (k_speed, speed_tau_ms)
SEGS = ((0.30, 0), (0.0, 0), (0.30, 30))
COLORS = (Color.GREEN, Color.YELLOW, Color.CYAN)

FRICTION_COMP = 10
FC_RAMP = 4
DT = 5
V_NOM = 7400
FALL_DEG = 45
SEG_MS = 2500
LOG_EVERY = 4

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

t_log = []
p_log = []
d_log = []
sp_log = []
st_log = []
s_log = []

rate_alpha = DT / (RATE_TAU_MS + DT)
watch = StopWatch()
n = 0
rate_f = 0.0
speed_f = 0.0
fell_in = None

for si in range(len(SEGS)):
    k_speed, speed_tau = SEGS[si]
    sp_alpha = DT / (speed_tau + DT) if speed_tau else 1.0
    hub.light.on(COLORS[si])
    t_end = watch.time() + SEG_MS
    while watch.time() < t_end:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        rate_f += rate_alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
        if abs(pitch) > FALL_DEG:
            fell_in = si
            break
        la = left.angle()
        ra = right.angle()
        speed_raw = (left.speed() + right.speed()) / 2
        speed_f += sp_alpha * (speed_raw - speed_f)
        speed_term = k_speed * speed_f
        duty = K_ANGLE * pitch + K_RATE * rate_f + K_MOTOR * (la + ra) / 2 + speed_term
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        sync = K_SYNC * (la - ra)
        scale = V_NOM / hub.battery.voltage()
        left.dc(scale * comp(duty - sync))
        right.dc(scale * comp(duty + sync))
        if n % LOG_EVERY == 0:
            t_log.append(watch.time())
            p_log.append(int(pitch * 10))
            d_log.append(int(duty))
            sp_log.append(int(speed_raw))
            st_log.append(int(speed_term))
            s_log.append(si)
        n += 1
        wait(max(0, DT * n - watch.time()))
    if fell_in is not None:
        break

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

print("fell_in_seg:", fell_in)
print("t_ms,seg,pitch_x10,duty,wheel_speed_dps,speed_term_duty")
for i in range(len(t_log)):
    print(t_log[i], s_log[i], p_log[i], d_log[i], sp_log[i], st_log[i], sep=",")
print("END")
