"""SysID: loop timing + actuation latency. Feeds delay_ctrl_steps in params.py.

Part 1: jitter of the DT=5 ms loop with the same sensor reads as the balancer.
Part 2: command a duty step and time until the encoder first moves --
command path + motor driver + electrical rise + STICTION/backlash takeup,
bundled. (The 2-deg detection threshold itself costs little: at duty 60 the
rotor crosses 2 deg within ~2 ms.)
Part 3: command a duty step and time until the GYRO first responds. The
body's angular reaction to motor torque is effectively instantaneous on this
robot (~30000 deg/s^2: a 5 dps threshold is crossed inside a millisecond), so
this delay is the command path + the SENSING chain (IMU sample rate, its
internal filter, Pybricks polling) -- the side part 2 cannot see, and the
side that was never measured before the run 28 lag-fragility question.
Setup for part 3: stand the robot upright on the ground and steady it with
two light fingertips; each trial gives a brief kick.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub()
left = Motor(Port.A)
right = Motor(Port.B)
watch = StopWatch()

# part 1: loop period with realistic per-tick work
DT = 5
worst = 0
watch.reset()
prev = 0
for n in range(1, 1001):
    hub.imu.rotation(Axis.Y)
    hub.imu.angular_velocity(Axis.Y)
    left.angle(); left.speed(); left.dc(0)
    right.angle(); right.speed(); right.dc(0)
    wait(max(0, DT * n - watch.time()))
    now = watch.time()
    worst = max(worst, now - prev)
    prev = now
print("1000 ticks, worst period ms:", worst)

# part 2: duty step -> first encoder movement
for trial in range(5):
    left.dc(0)
    wait(500)
    a0 = left.angle()
    watch.reset()
    left.dc(60)
    while abs(left.angle() - a0) < 2:
        pass
    print("actuation latency ms:", watch.time())
left.dc(0)

# part 3: duty step -> first gyro response (sensing-chain latency)
print("part 3: stand the robot UP on the ground, steady it lightly. 6 trials.")
wait(5000)
for trial in range(6):
    left.dc(0)
    right.dc(0)
    wait(1500)                      # settle; fingertips steady the robot
    still = 0
    while still < 30:               # quiet gyro before we trust the trigger
        still = still + 1 if abs(hub.imu.angular_velocity(Axis.Y)) < 2 else 0
        wait(5)
    watch.reset()
    left.dc(40)
    right.dc(40)
    t_gyro = -1
    while watch.time() < 200:       # timeout guard: never push for long
        if abs(hub.imu.angular_velocity(Axis.Y)) > 5:
            t_gyro = watch.time()
            break
    left.dc(0)
    right.dc(0)
    print("gyro response ms:", t_gyro)
print("part 3 done: gyro-response minus ~1 ms of body spin-up = sensing lag.")
print("compare with part 2: encoder time - gyro time ~= stiction/backlash takeup")
