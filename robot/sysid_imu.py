"""SysID: gyro bias and noise, robot stationary for 30 s.

Feeds imu_rate_bias / imu_rate_noise, and the rotation() drift bounds the
angle-bias randomization range.

Measures about PITCH_AXIS from hubconfig — the same signed axis the balancer
integrates — so the bias sign matches what the control loop sees."""
from pybricks.tools import wait

from hubconfig import PITCH_AXIS, make_hub

hub = make_hub()
wait(2000)  # settle

r0 = hub.imu.rotation(PITCH_AXIS)
n, s, s2 = 0, 0.0, 0.0
for _ in range(6000):   # 30 s at 200 Hz
    v = hub.imu.angular_velocity(PITCH_AXIS)
    n += 1
    s += v
    s2 += v * v
    wait(5)
mean = s / n
var = s2 / n - mean * mean
print("rate bias deg/s:", mean)
print("rate noise std deg/s:", var ** 0.5)
print("rotation drift over 30 s, deg:", hub.imu.rotation(PITCH_AXIS) - r0)
