"""Does the gyro FILTER set the ring frequency? (friction comp off throughout)

Run 1 rang at 14 Hz with the gyro term raw; every run since has rung at
9-10.6 Hz with a 30 ms low-pass on it. That is a frequency that MOVED when the
filter moved, which is not what a purely mechanical resonance does -- but those
two runs were confounded, because the friction term changed at the same time.

Run 8 settled that: friction compensation off is the best configuration
measured (peak 6.6 deg against 11.7), and the ring frequency did not care. So
the confound is gone and the filter can be tested alone.

Run 9 answered it, in the direction opposite to the guess: tau = 15 ms FELL
(peak 43 deg, 250 deg of drift) and tau = 0 never got to run. Less filtering is
worse. So the sweep goes UP from the known-good value, which is also the safe
direction to walk.

  GREEN   tau = 30 ms   known good: 2.24 deg RMS, +-7.3 deg peak
  YELLOW  tau = 45 ms
  CYAN    tau = 60 ms

If the frequency tracks tau, the ring is a loop mode -- delay plus filter lag
eating the damping term's phase -- and the fix is a better damping structure,
not a mechanical one. If it sits at ~10 Hz regardless, software is out of
suspects and it is the hardware.
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
RATE_TAU_MS = 30   # overridden per segment below
K_SYNC = 0.0

# (friction_comp, rate_tau_ms) -- comp stays 0 throughout
SEGS = ((0, 30), (0, 45), (0, 60))  # (friction_comp, rate_tau_ms); comp 0 throughout
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

rate_alpha = 1.0
watch = StopWatch()
n = 0
rate_f = 0.0
fell_in = None

for si in range(len(SEGS)):
    fc, rate_tau = SEGS[si]
    rate_alpha = DT / (rate_tau + DT) if rate_tau else 1.0
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
        # friction compensation is off for every segment of this run (run 8:
        # comp 0 gave the lowest peak of any config, 6.6 deg against 11.7)
        left.dc(scale * (duty - sync))
        right.dc(scale * (duty + sync))
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
