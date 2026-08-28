"""Backdrive breakaway: what does it take to turn the axle from outside?

The ~50:1 planetary ratio makes gearbox friction ASYMMETRIC: driving,
it reflects small (the 12-15% duty dead zone, runs 38/39); backdriving,
it works against you -- nothing attached to this axle ever falls back,
and rolling the robot by hand needs a real press (run 37 note). MuJoCo
frictionloss is symmetric, so the sim carries the DRIVE-side number and
this probe measures how wrong that is on the backdrive side.

SETUP: same lever as the stall probe (beam on the port-A axle, chassis
held/weighted), tip ABOVE the kitchen scale. Tare the scale.

RUN, per rep (3 reps), motors coasting throughout:
  CYAN     press the lever tip down onto the scale, SLOWLY and steadily
           increasing force -- watch the LED, not the scale
  MAGENTA  the encoder just moved past TRIG_DEG: FREEZE YOUR HAND and
           read the scale. Write the grams down. (Press slowly enough
           that the reading at the flip is the breakaway force, not an
           overshoot.)
  BLUE     3 s: push the lever back up out of the way (it stays where
           you leave it), then the next rep arms on CYAN.

breakaway torque = grams/1000 * 9.81 * lever_arm_m. Compare against the
drive-side wheel_frictionloss (~0.09 N*m for both motors, so ~0.045 per
motor): the ratio backdrive/drive is the asymmetry the sim ignores.
"""
from pybricks.parameters import Color
from pybricks.tools import wait

from hubconfig import make_hub, make_motors

hub = make_hub()
left, right = make_motors()

REPS = 3
TRIG_DEG = 3

print("battery mV:", hub.battery.voltage())
print(REPS, "reps: CYAN press slowly; MAGENTA = FREEZE, read grams; BLUE",
      "reposition.")
print("B,rep,trigger_deg")

for rep in range(REPS):
    left.stop()
    right.stop()
    hub.light.on(Color.CYAN)
    left.reset_angle(0)
    while abs(left.angle()) < TRIG_DEG:
        wait(10)
    hub.light.on(Color.MAGENTA)
    print("B,", rep, ",", left.angle())
    wait(4000)
    hub.light.on(Color.BLUE)
    wait(3000)

print("END")
