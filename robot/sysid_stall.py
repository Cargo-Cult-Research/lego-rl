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

RUN. The script steps duty through the schedule below, holding each for
4 s with the LED GREEN -- read the scale during the hold and WRITE DOWN
the grams next to the duty it prints. RED = resting between steps (2 s,
scale should return to ~0; if it does not, the lever is binding).

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

DUTIES = (15, 20, 25, 30, 40, 50, 60)
HOLD_MS = 4000
REST_MS = 2000

print("battery mV:", hub.battery.voltage())
print("stall probe, port A. Schedule:", DUTIES)
print("READ THE SCALE during each GREEN hold; write grams next to the duty.")
print("S,duty,battery_mV,angle_crept_deg")

for duty in DUTIES:
    hub.light.on(Color.GREEN)
    left.reset_angle(0)
    left.dc(duty)
    wait(HOLD_MS)
    batt = hub.battery.voltage()
    crept = left.angle()
    left.dc(0)
    left.stop()
    hub.light.on(Color.RED)
    print("S,", duty, ",", batt, ",", crept)
    wait(REST_MS)

hub.light.on(Color.BLUE)
print("done. Now: reply with grams per duty step AND the lever arm L in mm.")
print("END")
