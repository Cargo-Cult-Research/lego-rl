"""Closed-loop sign check. Hold the robot IN THE AIR, wheels free.

Start it upright, then slowly tip it toward the LED face and back. For 30 s:
  LED GREEN  = firmware thinks it's leaning FORWARD (toward the face)
  LED RED    = thinks it's leaning BACKWARD
  LED WHITE  = near upright
And the wheels run a gentle catch-up: when tipped toward the face they must
roll TOWARD the face (as if chasing the fall). If LED color or wheel
direction contradicts what your hands are doing, we found the sign bug.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from hubconfig import PITCH_AXIS, make_hub, make_motors

hub = make_hub()
left, right = make_motors()

wait(2000)                     # get it upright-ish
hub.imu.reset_heading(0)
pitch0 = hub.imu.rotation(PITCH_AXIS)

watch = StopWatch()
last_print = 0
while watch.time() < 30000:
    pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
    if pitch > 5:
        hub.light.on(Color.GREEN)
    elif pitch < -5:
        hub.light.on(Color.RED)
    else:
        hub.light.on(Color.WHITE)
    duty = max(-40, min(40, 3 * pitch))
    left.dc(duty)
    right.dc(duty)
    if watch.time() - last_print > 1000:
        print("pitch %.1f duty %d" % (pitch, duty))
        last_print = watch.time()
    wait(20)

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)
print("sign check done")
