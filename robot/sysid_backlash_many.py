"""100 randomized drivetrain-compliance probes, both wheels held throughout.

Hold BOTH wheels for the whole run. Each trial picks a motor and a torque at
random, sweeps that motor to its stop one way and then the other, and records
the angle the encoder swept. Randomizing the order matters: it decorrelates
grip fatigue from torque, so a tiring hand adds noise rather than a trend.

Slip is expected and does not ruin this. Slip can only ever ADD angle -- a
wheel that creeps lets the motor travel further than the compliance alone
would. So the contamination is ONE-SIDED, and the lower tail of the
distribution at each torque is the clean estimate. That is why 100 noisy
trials beat 6 careful ones: we do not need every trial held perfectly, we need
enough of them that the well-held ones show up in the bottom quantile.

LED: ON steady  = holding phase, keep gripping both wheels
     OFF (4 s)  = rest, shake out your hands, re-grip
     RED        = finished
"""
from pybricks.parameters import Color
from pybricks.tools import wait

from hubconfig import make_hub, make_motors

hub = make_hub()
motors = make_motors()

TRIALS = 100
REST_EVERY = 25
REST_MS = 4000
PUSH_MS = 250
SETTLE_MS = 120
DUTY_MIN = 15
DUTY_MAX = 50

# Tiny LCG rather than urandom: deterministic, so a rerun is comparable, and
# no dependency on which random module this firmware happens to ship.
# The multiplier is small ON PURPOSE -- Pybricks has no long ints, and the
# textbook 1103515245 constant overflows 30-bit small ints immediately.
# 75 * 65537 stays well inside range.
_seed = 12345


def rnd(n):
    global _seed
    _seed = (_seed * 75 + 74) % 65537
    return (_seed >> 3) % n


print("battery mV:", hub.battery.voltage())
print("trial,motor,duty_pct,play_deg")

hub.light.on(Color.GREEN)
wait(3000)

for i in range(TRIALS):
    if i and i % REST_EVERY == 0:
        motors[0].dc(0)
        motors[1].dc(0)
        hub.light.off()
        print("--- rest", i, "of", TRIALS, "---")
        wait(REST_MS)
        hub.light.on(Color.GREEN)
        wait(1200)

    mi = rnd(2)
    duty = DUTY_MIN + rnd(DUTY_MAX - DUTY_MIN + 1)
    m = motors[mi]

    m.dc(duty)
    wait(PUSH_MS)
    m.dc(0)
    wait(SETTLE_MS)
    a = m.angle()
    m.dc(-duty)
    wait(PUSH_MS)
    m.dc(0)
    wait(SETTLE_MS)
    b = m.angle()
    m.dc(0)
    print(i, ",", mi, ",", duty, ",", a - b)

motors[0].dc(0)
motors[1].dc(0)
hub.light.on(Color.RED)
print("END")
