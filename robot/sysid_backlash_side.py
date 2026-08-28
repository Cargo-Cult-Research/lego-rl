"""Which side of the gearbox free play is the encoder on?

The sim's lash model ASSUMED the encoder sits on the MOTOR side of the
play (encoder leads the wheel through the gap) -- an assumption that
shaped the whole backlash/quantization story, never a measurement.
ANSWERED 2026-08-27 (run 40): the encoder FOLLOWS a hand-wiggle within
the free play (155 events over a ~4 deg span in 12 s), so the encoder
rides the OUTPUT side and it is the ACTUATOR that acts through the gap.
The model was fixed the same day (env._true_state sums wheel + lash).
Kept for re-running on other motors/builds.

Protocol: motors coast. GREEN, 12 s -- grip the wheel or a lever on the
axle and wiggle it gently WITHIN the free play, without forcing the
gearbox through breakaway. The script streams every encoder change.
Note down what your hand felt vs what printed (a flat stream while you
feel play = motor-side encoder; a following stream = output-side).

Backdrive breakaway moved to robot/sysid_backdrive.py -- it needs the
lever-on-scale setup, which conflicts with free wiggling.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from hubconfig import make_hub, make_motors

hub = make_hub()
left, right = make_motors()

WIGGLE_S = 12

print("battery mV:", hub.battery.voltage())
print("GREEN 12 s -- wiggle the wheel/lever gently WITHIN the play.")
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

hub.light.on(Color.BLUE)
print("END")
