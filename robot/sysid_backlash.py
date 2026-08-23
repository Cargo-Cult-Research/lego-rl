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

LED: GREEN = grip the LEFT wheel  ·  CYAN = grip the RIGHT wheel
     BLUE flashes between reps    ·  RED at the end
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)

DUTIES = (15, 25, 40)     # %, well above the ~10% dead zone
REPS = 3
PUSH_MS = 600             # long enough to settle against the stop
SETTLE_MS = 250

print("battery mV:", hub.battery.voltage())
print("motor,duty_pct,rep,play_deg")


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


for name, motor, colour in (("left ", left, Color.GREEN),
                            ("right", right, Color.CYAN)):
    hub.light.on(Color.RED)
    wait(2500)              # time to move your hand to the other wheel
    hub.light.on(colour)
    wait(1500)
    for duty in DUTIES:
        for rep in range(REPS):
            p = play_at(motor, duty)
            print(name, ",", duty, ",", rep, ",", int(10 * p) / 10.0)
            hub.light.on(Color.BLUE)
            wait(150)
            hub.light.on(colour)

left.dc(0)
right.dc(0)
hub.light.on(Color.RED)
print("END")
