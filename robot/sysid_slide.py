"""Slide probe: does imu.rotation() lie under pure acceleration?

WHY. The joint sysid fit (data/sysid_fit_best.json) only stands with a
~4.6 Hz accel-fusion term in the pitch signal -- but true fusion at that
crossover would forbid the reference walk run 34 measured (-29 deg by
seg 16). The reconciliation would be that the fit wants fusion's
ACCELERATION CONTAMINATION: an accel-fused tilt estimate reads a
horizontal acceleration as a lean. That is directly measurable with the
robot in your hand -- no balancing, no wheels on the ground needed.

Protocol per rep (2 reps; motors coast the whole time):

  RED     hold the robot upright and still on the table
  GREEN   baseline: keep holding it still (2 s)
  CYAN    SLIDE: push the robot back and forth along its forward axis,
          briskly and rhythmically (~1-2 Hz), keeping it as upright as
          you can. Sliding on the table is ideal (wheels roll); in the
          air works too. 8 s.
  GREEN   still again (2 s)
  BLUE    printing (~1200 rows), then next rep

What the analysis looks for: if rotation() is contaminated, the pitch
reading will oscillate IN PHASE with the measured forward acceleration
while the gyro rate says the body is not actually pitching that much.
The regression pitch-vs-accel gives deg per m/s^2, which pins the sim's
imu_fusion model against a measurement instead of a fit preference.

Output rows at 100 Hz (10 ms):
  D , i , pitch_x100 , rate_x10 , acc
where pitch is imu.rotation(PITCH_AXIS) relative to the rep start, rate
the raw pitch rate (deg/s), acc the acceleration along the ROBOT's
forward axis in mm/s^2 (Axis.X in the mapped frame -- the same frame
where upright reads +g on Axis.Z, see hubconfig's arm check). When held
still but tilted it reads ~g*sin(pitch); the analysis separates that
tilt term (known from the gyro) from the slide contamination.
"""
from pybricks.parameters import Axis, Color
from pybricks.tools import StopWatch, wait

from hubconfig import PITCH_AXIS, make_hub, make_motors, wait_until_upright

hub = make_hub()
left, right = make_motors()

REPS = 2
DT = 10            # ms -> 100 Hz; this probe needs duration, not bandwidth
STILL_TICKS = 200  # 2 s baseline / tail
SLIDE_TICKS = 800  # 8 s of sliding
N = STILL_TICKS + SLIDE_TICKS + STILL_TICKS

b_pitch = [0] * N
b_rate = [0] * N
b_acc = [0] * N

print("battery mV:", hub.battery.voltage())
print("slide probe: 2s still / 8s slide / 2s still per rep,", REPS, "reps")
print("D,i,pitch_x100,rate_x10,acc_mm_s2")

watch = StopWatch()
for rep in range(REPS):
    hub.light.on(Color.RED)
    left.stop()
    right.stop()
    wait_until_upright(hub)
    pitch0 = hub.imu.rotation(PITCH_AXIS)
    hub.light.on(Color.GREEN)

    t0 = watch.time()
    for k in range(N):
        if k == STILL_TICKS:
            hub.light.on(Color.CYAN)      # slide now
        elif k == STILL_TICKS + SLIDE_TICKS:
            hub.light.on(Color.GREEN)     # hold still again
        b_pitch[k] = int((hub.imu.rotation(PITCH_AXIS) - pitch0) * 100)
        b_rate[k] = int(hub.imu.angular_velocity(PITCH_AXIS) * 10)
        b_acc[k] = int(hub.imu.acceleration(Axis.X))
        wait(max(0, t0 + DT * (k + 1) - watch.time()))

    hub.light.on(Color.BLUE)
    print("R,", rep, ",", hub.battery.voltage())
    for k in range(N):
        print("D ,", k, ",", b_pitch[k], ",", b_rate[k], ",", b_acc[k])
    wait(500)

print("END")
