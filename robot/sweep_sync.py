"""Is the yaw (sync) loop driving the oscillation?

sync = K_SYNC * (left.angle() - right.angle()) is a proportional-only yaw
controller with no damping term, running through the same ~19 ms delay as the
pitch loop, on a body with very little yaw inertia. Every run so far logged
only the MEAN wheel angle, which cancels the differential channel exactly --
so this loop has never been observed.

Three 2.5 s segments, identical pitch gains (the best row of the gain sweep),
varying only K_SYNC. Logs the wheel DIFFERENCE alongside pitch and duty.

  GREEN   K_SYNC = 0.15   as shipped
  YELLOW  K_SYNC = 0      yaw loop off entirely
  CYAN    K_SYNC = 0.05   weak
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = (10.71, 0.87, 0.43, 0.30)
MAX_DUTY = 40          # best row of the gain sweep: lowest pitch RMS and drift
RATE_TAU_MS = 30
SYNCS = (0.15, 0.0, 0.05)
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
m_log = []
df_log = []
s_log = []

alpha = DT / (RATE_TAU_MS + DT)
watch = StopWatch()
n = 0
rate_f = 0.0
fell_in = None

for si in range(len(SYNCS)):
    k_sync = SYNCS[si]
    hub.light.on(COLORS[si])
    t_end = watch.time() + SEG_MS
    while watch.time() < t_end:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        rate_f += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
        if abs(pitch) > FALL_DEG:
            fell_in = k_sync
            break
        la = left.angle()
        ra = right.angle()
        angle = (la + ra) / 2
        speed = (left.speed() + right.speed()) / 2
        duty = K_ANGLE * pitch + K_RATE * rate_f + K_MOTOR * angle + K_SPEED * speed
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        sync = k_sync * (la - ra)
        scale = V_NOM / hub.battery.voltage()
        left.dc(scale * comp(duty - sync))
        right.dc(scale * comp(duty + sync))
        if n % LOG_EVERY == 0:
            t_log.append(watch.time())
            p_log.append(int(pitch * 10))
            d_log.append(int(duty))
            m_log.append(int(angle))
            df_log.append(int(la - ra))
            s_log.append(si)
        n += 1
        wait(max(0, DT * n - watch.time()))
    if fell_in is not None:
        break

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

print("fell_in_ksync:", fell_in)
print("t_ms,seg,pitch_x10,duty,wheel_mean,wheel_diff")
for i in range(len(t_log)):
    print(t_log[i], s_log[i], p_log[i], d_log[i], m_log[i], df_log[i], sep=",")
print("END")
