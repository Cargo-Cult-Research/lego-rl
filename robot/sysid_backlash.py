"""Measure the gearbox play, and tell play apart from compliance.

Grip a wheel so it cannot turn, then drive its motor one way until it stops,
and the other way until it stops. The encoder is on the motor side of the
gearbox, so the angle it sweeps between those two stops IS the play.

The important part is doing it at several duty levels, because that separates
the two things it could be:

    BACKLASH    a fixed gap. The angle is the same at 15% duty and 45% duty,
                because once the teeth touch, they touch.
    COMPLIANCE  something springy. The angle grows with duty, because you are
                winding up a spring rather than crossing a gap.

Both would feel like "slop" in the hand. They need different models and they
behave differently in a control loop, so measure rather than assume.

Which wheel is which: "left" and "right" here are just names for Port A and
Port B. Their DIRECTIONS were verified during bringup; which physical side
each sits on never was. So the script does not tell you -- it SHOWS you, by
wiggling the wheel under test before each phase. Grab the one that moves.

The operator cannot see this terminal, and colour-coded LEDs turned out to be
one thing too many to remember. So the signal vocabulary is three states and
the wheel itself does the talking:

    a wheel WIGGLES ......... that is the one to hold. Grab it now.
    LED DARK ................ get your grip, no rush
    LED ON (steady) ......... measuring. Hold it still until the light goes out.
    LED DARK again .......... let go, switch to the other wheel

Timings are deliberately slow. A rushed grip is a wasted run.
"""
from pybricks.parameters import Color
from pybricks.tools import wait

from hubconfig import make_hub, make_motors

hub = make_hub()
left, right = make_motors()

DUTIES = (20, 30, 45)     # %, clear of the measured ~10% dead zone. 15% was
                          # too close to it -- the motor can stall short of the
                          # stop and report a small angle that looks like play.
MAX_PLAUSIBLE = 40        # deg. Beyond this the wheel turned instead of the
                          # gearbox taking up its play, so the number is not a
                          # measurement of anything. A first run reported
                          # 88-243 deg because the wheels were not held.
REPS = 2                  # keep the hold under ~15 s; gripping is tiring
PUSH_MS = 500             # long enough to settle against the stop
SETTLE_MS = 250

held_fail = 0
print("battery mV:", hub.battery.voltage())
print("motor,duty_pct,rep,play_deg,status")


def play_at(motor, duty):
    """Sweep to one stop, then the other. Returns the angle between them."""
    motor.dc(duty)
    wait(PUSH_MS)
    motor.dc(0)
    wait(SETTLE_MS)
    a = motor.angle()
    motor.dc(-duty)
    wait(PUSH_MS)
    motor.dc(0)
    wait(SETTLE_MS)
    b = motor.angle()
    motor.dc(duty)          # return to the first stop so reps start alike
    wait(PUSH_MS)
    motor.dc(0)
    wait(SETTLE_MS)
    return a - b


for name, motor, colour in (("portA", left, Color.GREEN),
                            ("portB", right, Color.CYAN)):
    # Identify the wheel physically rather than naming a side we never verified.
    print("--- WIGGLING", name, ": hold THAT wheel ---")
    hub.light.off()
    for _ in range(6):
        motor.dc(45)
        wait(150)
        motor.dc(-45)
        wait(150)
    motor.dc(0)
    print("    grip it now -- measuring starts in 6 s")
    wait(6000)              # unhurried: a rushed grip is a wasted run
    hub.light.on(colour)    # steady light = hold still
    print("    MEASURING", name, "- keep holding until the light goes out")
    wait(700)
    for duty in DUTIES:
        for rep in range(REPS):
            p = play_at(motor, duty)
            bad = 1 if abs(p) > MAX_PLAUSIBLE else 0
            print(name, ",", duty, ",", rep, ",", int(10 * p) / 10.0,
                  ", HELD" if not bad else ", WHEEL_TURNED")
            if bad:
                held_fail += 1
            # no flashing between reps: the light stays ON for the whole hold

    motor.dc(0)
    hub.light.off()
    print("    done with", name, "- you can let go")
    wait(4000)

left.dc(0)
right.dc(0)
hub.light.on(Color.RED)
if held_fail:
    print("INVALID:", held_fail, "of", 2 * len(DUTIES) * REPS,
          "reps had the wheel turn instead of hitting a stop.")
    print("The gearbox play is whatever the motor sweeps while the wheel CANNOT")
    print("move, so a spinning wheel measures nothing. Grip harder and repeat.")
else:
    print("all reps hit a stop -- numbers are usable")
print("END")
