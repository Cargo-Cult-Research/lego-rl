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
  3. Position the lever so its tip hangs ~1 cm ABOVE the kitchen scale,
     oriented so that driving the motor forward presses the tip DOWN
     into the scale. Tare the empty scale. The lever stays up on its own
     (this gearbox holds anything) and each step drives it down.
  4. MEASURE AND WRITE DOWN the lever arm L: axle center to the contact
     point on the scale, in mm (hole pitch is 8 mm -- counting holes
     works: n-th hole center = n*8 mm from the axle hole).

RUN -- hands-free (v3, Urs's protocol). Before every step the motor
blips in REVERSE, lifting the lever off the scale, where it STAYS (this
motor holds anything; nothing falls back). Each step then DRIVES the
lever down from above into the scale and stalls against it: read the
scale during the GREEN hold, write grams next to the pass+duty printed.
You never touch the lever between steps.

Two things are data, not failures: a step too weak to bring the lever
down to the scale at all reads ~0 g -- that is the dead-zone boundary
seen directly. And every step approaches from the same side with
freshly unwound backlash, which is the whole point: the first session's
15/20/25% all reading the same ~50 g was frozen wind-up re-read three
times (below breakaway the rotor cannot move, so the scale cannot
change).

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
print("stall probe v3 (hands-free), port A. Schedule:", DUTIES,
      "x", PASSES, "passes")
print("Lever starts ABOVE the scale and drives down into it each step.")
print("READ THE SCALE during each GREEN hold; write grams next to the")
print("pass+duty. ~0 g = the step could not reach the scale (dead zone).")
print("S,pass,duty,battery_mV,angle_moved_deg")

for p in range(PASSES):
    for duty in DUTIES:
        left.dc(UNWIND_DUTY)          # unwind + lift; the lever stays up
        wait(UNWIND_MS)
        left.dc(0)
        left.stop()
        hub.light.on(Color.RED)
        wait(REST_MS)
        hub.light.on(Color.GREEN)
        left.reset_angle(0)
        left.dc(duty)                 # drive down into the scale, stall
        wait(HOLD_MS)
        batt = hub.battery.voltage()
        moved = left.angle()          # lift-arc + wind-up; big and positive
        left.dc(0)                    # means it reached the scale
        left.stop()
        print("S,", p, ",", duty, ",", batt, ",", moved)

hub.light.on(Color.BLUE)
print("done. Now: reply with grams per duty step AND the lever arm L in mm.")
print("END")
