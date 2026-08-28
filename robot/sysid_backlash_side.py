"""Which side of the gearbox is the encoder on, and what does backdrive cost?

Two of Urs's questions, one session, motors coasting throughout.

PHASE A -- WIGGLE (encoder side of the free play). The sim's lash model
assumes the encoder sits on the MOTOR side of the play, so wiggling the
output within the free play should NOT register. That is an assumption,
never a measurement. Protocol: GREEN, 12 s -- grip the lever (or wheel)
and wiggle it gently WITHIN the free play, without forcing the gearbox
through breakaway; the script streams the encoder. If angle() follows
your wiggle, the encoder is output-side and the sim's lash arrangement
is backwards. If it stays flat while you clearly feel the play, the
motor-side assumption is confirmed (1 deg encoder quantum limits what
"flat" means -- wiggle the full play span).

PHASE B -- BACKDRIVE BREAKAWAY (the asymmetry). Driving, the ~50:1
ratio works FOR the motor, so gearbox friction shows up as only a
~10-20% duty dead zone. Backdriving, it works AGAINST you -- that is
why nothing ever falls back. MuJoCo frictionloss is symmetric, so we
need the real backdrive number to know how wrong that is. Protocol per
rep (3 reps): CYAN -- press the lever tip down onto the kitchen scale
SLOWLY, increasing force; the moment the encoder moves more than
TRIG_DEG the LED flips MAGENTA: FREEZE and read the scale. That reading
x lever arm = backdrive breakaway torque. Then BLUE 3 s to reposition
the lever back up (push it through the gearbox -- it will stay where
you leave it), and the next rep arms.

Output: phase A stream `A , i , angle_deg` at 50 Hz (only rows where the
angle changed, to keep BLE quiet), phase B rows `B , rep , trigger_deg`.
Write down: phase A -- whether you could feel play the encoder did not
show; phase B -- grams on the scale at each MAGENTA.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from hubconfig import make_hub, make_motors

hub = make_hub()
left, right = make_motors()

WIGGLE_S = 12
REPS = 3
TRIG_DEG = 3

print("battery mV:", hub.battery.voltage())
print("PHASE A: GREEN 12 s -- wiggle the lever gently WITHIN the play.")
print("A,i,angle_deg")

left.stop()
right.stop()
watch = StopWatch()
hub.light.on(Color.GREEN)
left.reset_angle(0)
t0 = watch.time()
last = None
for k in range(WIGGLE_S * 50):
    a = left.angle()
    if a != last:
        print("A ,", k, ",", a)
        last = a
    wait(max(0, t0 + 20 * (k + 1) - watch.time()))

print("PHASE B:", REPS, "reps. CYAN = press lever onto scale slowly;")
print("MAGENTA = encoder moved, FREEZE and read grams. BLUE = reposition.")
print("B,rep,trigger_deg")

for rep in range(REPS):
    left.stop()
    right.stop()
    hub.light.on(Color.CYAN)
    left.reset_angle(0)
    while abs(left.angle()) < TRIG_DEG:
        wait(10)
    hub.light.on(Color.MAGENTA)      # FREEZE -- read the scale
    print("B,", rep, ",", left.angle())
    wait(4000)
    hub.light.on(Color.BLUE)         # reposition the lever back up
    wait(3000)

print("END")
