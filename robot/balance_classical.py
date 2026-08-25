"""Classical four-gain balancer, two-wheel two-motor build. Milestone 0.

Launch ritual: start the program, stand the robot upright on the floor and
hold it still at its balance point. LED: RED = waiting for you to get it
upright and still; GREEN = controller live, release gently. It stops and
prints stats when pitch exceeds FALL_DEG.

All configuration lives in hubconfig.py (build) and gains.py (control law) —
edit there, never here, so every program agrees on the numbers.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait
from umath import copysign

from gains import FRICTION_COMP, GAINS_SIM_TUNED, K_SYNC
from hubconfig import (DT, FALL_DEG, MAX_DUTY, PITCH_AXIS, RATE_TAU_MS, V_NOM,
                       make_hub, make_motors, wait_until_upright)

hub = make_hub()
left, right = make_motors()

K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED,
      "tau:", RATE_TAU_MS, "max:", MAX_DUTY, "sync:", K_SYNC,
      "fc:", FRICTION_COMP)


def drive(duty, sync):
    scale = V_NOM / hub.battery.voltage()
    l = duty - sync
    r = duty + sync
    if FRICTION_COMP:
        l += copysign(FRICTION_COMP, l)
        r += copysign(FRICTION_COMP, r)
    left.dc(scale * l)
    right.dc(scale * r)


hub.light.on(Color.RED)
wait_until_upright(hub)

hub.imu.reset_heading(0)
pitch0 = hub.imu.rotation(PITCH_AXIS)
left.reset_angle(0)
right.reset_angle(0)
hub.light.on(Color.GREEN)   # live -- release gently

alpha = DT / (RATE_TAU_MS + DT) if RATE_TAU_MS else 1.0
watch = StopWatch()
n = 0
peak_duty = 0
peak_pitch = 0
sum_sq = 0.0
rate_f = 0.0
while True:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    rate_f += alpha * (hub.imu.angular_velocity(PITCH_AXIS) - rate_f)
    if abs(pitch) > FALL_DEG:
        left.dc(0)
        right.dc(0)
        hub.light.on(Color.BLUE)
        print("fell after", watch.time() / 1000, "s, peak duty", peak_duty)
        break
    angle = (left.angle() + right.angle()) / 2
    speed = (left.speed() + right.speed()) / 2
    duty = (K_ANGLE * pitch
            + K_RATE * rate_f
            + K_MOTOR * angle
            + K_SPEED * speed)
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    if abs(duty) > peak_duty:
        peak_duty = abs(duty)
    if abs(pitch) > peak_pitch:
        peak_pitch = abs(pitch)
    sum_sq += pitch * pitch
    sync = K_SYNC * (left.angle() - right.angle())
    drive(duty, sync)
    n += 1
    # Progress report every 5 s, so a run that is quietly succeeding says so
    # instead of looking identical to a hung program.
    if n % 1000 == 0:
        print("t", watch.time() // 1000, "s  pitch rms",
              int(10 * (sum_sq / n) ** 0.5) / 10, "peak", int(peak_pitch),
              "wheel", int(angle))
    wait(max(0, DT * n - watch.time()))
