"""The friction compensation, third time: does its SLOPE set the limit cycle?

Ruled out so far: saturation, the yaw loop, and the wheel-speed term (the last
one is load-bearing -- removing it drops the robot). What survives every one of
those tests is a ~10 Hz ring that barely moves with gain.

Ramping the friction term removed its discontinuity but not its steepness. At
FRICTION_COMP=10 over FC_RAMP=4 the slope near zero is 1 + 10/4 = 3.5, so the
controller has 3.5x the loop gain for small commands -- and a limit cycle is
exactly the regime where the command hovers near zero. A nonlinearity with
high small-signal gain plus a fixed 19 ms delay oscillates at a frequency set
by the delay, with an amplitude that self-adjusts: gain-insensitive and
amplitude-stable, which is the signature in every run so far.

  GREEN   comp 10 over  4   slope 3.50  (current)
  YELLOW  comp 10 over 15   slope 1.67
  CYAN    comp  0           slope 1.00, no compensation at all

Expect a trade: less compensation means the ~10% motor dead zone goes
unhelped, so watch drift and RMS together, not RMS alone.
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
MAX_DUTY = 40
RATE_TAU_MS = 30
K_SYNC = 0.0

# (friction_comp, fc_ramp)
SEGS = ((10, 4), (10, 15), (0, 1))
COLORS = (Color.GREEN, Color.YELLOW, Color.CYAN)

DT = 5
V_NOM = 7400
FALL_DEG = 45
SEG_MS = 2500
LOG_EVERY = 4

print("battery mV:", hub.battery.voltage())

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
s_log = []

rate_alpha = DT / (RATE_TAU_MS + DT)
watch = StopWatch()
n = 0
rate_f = 0.0
fell_in = None

for si in range(len(SEGS)):
    fc, ramp = SEGS[si]
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
        speed = (left.speed() + right.speed()) / 2
        duty = (K_ANGLE * pitch + K_RATE * rate_f
                + K_MOTOR * (la + ra) / 2 + K_SPEED * speed)
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        sync = K_SYNC * (la - ra)
        scale = V_NOM / hub.battery.voltage()
        dl = duty - sync
        dr = duty + sync
        if fc:
            dl += fc * max(-1, min(1, dl / ramp))
            dr += fc * max(-1, min(1, dr / ramp))
        left.dc(scale * dl)
        right.dc(scale * dr)
        if n % LOG_EVERY == 0:
            t_log.append(watch.time())
            p_log.append(int(pitch * 10))
            d_log.append(int(duty))
            m_log.append(int((la + ra) / 2))
            s_log.append(si)
        n += 1
        wait(max(0, DT * n - watch.time()))
    if fell_in is not None:
        break

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

print("fell_in_seg:", fell_in)
print("t_ms,seg,pitch_x10,duty,wheel_mean")
for i in range(len(t_log)):
    print(t_log[i], s_log[i], p_log[i], d_log[i], m_log[i], sep=",")
print("END")
