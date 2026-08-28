"""Stall-torque probe: lever + kitchen scale, the measurement params.py
has wanted since day one.

WHY NOW. stall_torque has been a GUESS (0.25 N*m) since bringup, and the
joint sysid fit pushed it to 0.10 -- the bottom of its search bound --
as one of the two load-bearing ingredients of the first sim config that
stands through the 30 ms filter. A guess that load-bearing needs to
become a measurement.

SETUP (one-time, ~5 min):
  1. Pull the LEFT wheel (port A) off its axle and fix a Technic beam to
     the axle in its place, as a lever.
  2. Weigh the robot down or hold it so the chassis CANNOT rotate.
  3. Rest the lever tip on the kitchen scale so that driving the motor
     presses the tip DOWN into the scale. Tare the scale with the motor
     idle.
  4. MEASURE AND WRITE DOWN the lever arm L: axle center to the contact
     point on the scale, in mm (hole pitch is 8 mm -- counting holes
     works: n-th hole center = n*8 mm from the axle hole).

RUN. Before every step the motor blips in REVERSE to unwind the gear
train (the lever hops off the scale -- let it fall back and settle
during the RED rest; nudge it flat if it lands crooked). Then GREEN =
the hold: read the scale, WRITE DOWN the grams next to the pass+duty it
prints. Without the unwind, steps below the ~22% breakaway (run 17)
just re-read whatever wind-up the previous step froze into the train --
the first session's 15/20/25% all reading ~50 g was exactly that.

The program verifies the motor is actually stalled: it prints how far
the encoder crept during each hold. A few degrees is settling; tens of
degrees means the lever slipped and the step is void.

torque per step = grams/1000 * 9.81 * L_mm/1000  [N*m]
Fitting a line through torque vs duty gives the effective stall torque
(slope * 100, at this battery) and the friction intercept -- the two
numbers the sim's motor model runs on.
"""
from pybricks.parameters import Color
from pybricks.tools import wait

from hubconfig import make_hub, make_motors

hub = make_hub()
left, right = make_motors()

# Denser at the low end: the dead zone is the load-bearing quantity now,
# and the first session showed 15-25% all reading the same ~50 g. That
# plateau was FROZEN WIND-UP: below the ~22% breakaway (run 17) the rotor
# does not move, so the scale keeps whatever the previous step left in
# the train. Every step now starts from a known-zero state: a reverse
# UNWIND pulse first (the lever will hop off the scale -- let it fall
# back and settle during the RED rest), then the target duty from zero.
# Two passes check repeatability.
DUTIES = (12, 15, 18, 22, 26, 30, 35, 40, 50, 60)
PASSES = 2
HOLD_MS = 4000
REST_MS = 2500
UNWIND_DUTY = -25
UNWIND_MS = 250

print("battery mV:", hub.battery.voltage())
print("stall probe, port A. Schedule:", DUTIES, "x", PASSES, "passes")
print("Each step: reverse blip (lever hops -- let it settle back on the")
print("scale), RED rest, then GREEN hold: READ THE SCALE, write grams.")
print("S,pass,duty,battery_mV,angle_crept_deg")

for p in range(PASSES):
    for duty in DUTIES:
        left.dc(UNWIND_DUTY)          # unwind the train; lever lifts
        wait(UNWIND_MS)
        left.dc(0)
        left.stop()
        hub.light.on(Color.RED)       # settle: lever back on the scale
        wait(REST_MS)
        hub.light.on(Color.GREEN)
        left.reset_angle(0)
        left.dc(duty)
        wait(HOLD_MS)
        batt = hub.battery.voltage()
        crept = left.angle()
        left.dc(0)
        left.stop()
        print("S,", p, ",", duty, ",", batt, ",", crept)

hub.light.on(Color.BLUE)
print("done. Now: reply with grams per duty step AND the lever arm L in mm.")
print("END")
