"""Classical four-gain balancer, two-wheel two-motor build. Milestone 0.

Launch ritual: start the program, stand the robot upright on the floor and
hold it still at its balance point. LED: RED = waiting for you to get it
upright and still; GREEN = controller live, release gently. It stops and
prints stats when pitch exceeds FALL_DEG.

Gain sets in duty% per deg / deg/s on (pitch, rate, motor angle, motor speed):
reference = published Pybricks balancer (tall heavy robot); sim_tuned = CEM
in our measured-parameter sim (2026-08-22, held-out 10 s survival 100%).
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
from umath import copysign

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)  # sysid_directions 2026-08-22
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # probe: +CW rolled backward
right = Motor(Port.B, Direction.CLOCKWISE)        # probe: +CW rolled forward
PITCH_AXIS = -Axis.Y  # sign check 2026-08-22: +Y read toward-face tilt as negative

GAINS_REFERENCE = (88, 0.35, 0.72, 0.19)
GAINS_SIM_TUNED = (10.71, 0.87, 0.43, 0.30)
K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED   # <-- pick the set here

K_SYNC = 0.15   # duty% per deg of left-right encoder difference
DT = 5          # ms -> 200 Hz
V_NOM = 7400    # mV
FALL_DEG = 45

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED)


def drive(duty, sync):
    scale = V_NOM / hub.battery.voltage()
    l = duty - sync
    r = duty + sync
    left.dc(scale * (l + copysign(10, l)))
    right.dc(scale * (r + copysign(10, r)))


# RED: wait until held upright (accel ~ +g on robot Z) and still for 0.5 s
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
hub.light.on(Color.GREEN)   # live -- release gently

watch = StopWatch()
n = 0
peak_duty = 0
while True:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    if abs(pitch) > FALL_DEG:
        left.dc(0)
        right.dc(0)
        hub.light.on(Color.BLUE)
        print("fell after", watch.time() / 1000, "s, peak duty", peak_duty)
        break
    angle = (left.angle() + right.angle()) / 2
    speed = (left.speed() + right.speed()) / 2
    duty = (K_ANGLE * pitch
            + K_RATE * hub.imu.angular_velocity(PITCH_AXIS)
            + K_MOTOR * angle
            + K_SPEED * speed)
    duty = max(-100, min(100, duty))
    if abs(duty) > peak_duty:
        peak_duty = abs(duty)
    sync = K_SYNC * (left.angle() - right.angle())
    drive(duty, sync)
    n += 1
    wait(max(0, DT * n - watch.time()))
