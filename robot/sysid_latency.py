"""SysID: loop timing + actuation latency. Feeds delay_ctrl_steps in params.py.

Part 1: jitter of the DT=5 ms loop with the same sensor reads as the balancer.
Part 2: command a duty step and time until the encoder first moves --
that's motor driver + mechanical latency in ms.
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
