"""Orientation + direction probe for the assembled robot. Follow the LED:

  GREEN  (6 s): hold the robot UPRIGHT and still, in balancing pose.
  ORANGE (6 s): tilt the robot FORWARD ~30 deg and hold it there.
  RED    (2 s warning), then motor A pulses forward-duty, then motor B.
         Wheels free! Note for each pulse which way the wheel rolls
         (toward robot front or back).
  BLUE: done.

Prints gravity vectors, rotation deltas, and encoder deltas; Claude turns
those into top_side/front_side, PITCH_AXIS, and Direction settings.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Port
from pybricks.pupdevices import Motor
from pybricks.tools import wait

hub = TechnicHub()
AXES = (Axis.X, Axis.Y, Axis.Z)


def accel_avg(ms):
    n, acc = 0, [0.0, 0.0, 0.0]
    for _ in range(ms // 20):
        for i, ax in enumerate(AXES):
            acc[i] += hub.imu.acceleration(ax)
        n += 1
        wait(20)
    return [round(a / n) for a in acc]


print("battery mV:", hub.battery.voltage())

hub.light.on(Color.GREEN)
wait(1500)  # time to get into pose
g_up = accel_avg(4500)
print("UPRIGHT accel mm/s^2 [X Y Z]:", g_up)
rot_before = [hub.imu.rotation(ax) for ax in AXES]

hub.light.on(Color.ORANGE)
wait(2500)  # time to tilt forward
g_fwd = accel_avg(3500)
rot_after = [hub.imu.rotation(ax) for ax in AXES]
print("TILTED-FWD accel [X Y Z]:", g_fwd)
print("rotation delta deg [X Y Z]:",
      [round(a - b, 1) for a, b in zip(rot_after, rot_before)])

hub.light.on(Color.RED)
wait(2000)
for name, port in (("A", Port.A), ("B", Port.B)):
    m = Motor(port)
    m.reset_angle(0)
    m.dc(30)
    wait(700)
    m.dc(0)
    print("motor", name, "encoder delta at +30% duty:", m.angle())
    wait(800)

hub.light.on(Color.BLUE)
print("probe done")
