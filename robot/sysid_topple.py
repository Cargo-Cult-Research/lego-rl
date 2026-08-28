"""Free-topple probe: measure the pendulum time constant, motors off.

WHY. The sim classical falls in ~3 s with the robot's own 30 ms rate
filter while the hardware stands indefinitely (run 28, reconfirmed by
scripts/sim_cruise_check.py after run 35). The suspect is the plant's
pendulum time constant: com_height is MEASURED at 5 cm, but the box
model derives the moment of inertia from it assuming uniform mass, and
the real robot carries the hub + 6 AAs high. This probe measures the
toppling dynamics directly; the fit happens OFFLINE by replicating this
exact experiment in sim and matching the traces (inertia + whatever
friction decides if the wheels roll or stay stiction-locked).

Protocol per rep (LED is the interface; put CUSHIONS on both sides):

  RED    hold the robot upright and still on the floor
  GREEN  motors are COASTING and recording runs at 200 Hz: let go,
         as close to balanced as you can -- it topples on its own.
         Alternate the lean slightly forward / backward across reps
         (the trace's pitch sign records which way it went).
  BLUE   printing the trimmed trace, then RED for the next rep

A rep where nothing topples within the window (still held, or a
miraculous balance) voids and repeats -- void reps do not count.

The wheels are the second measurement: wl/wr angles say whether the
wheels rolled during the topple or stayed stiction-locked (run 17: the
drivetrain holds below ~22% duty equivalent), which changes what inertia
the topple actually probes. Both columns ride along at native integer
resolution.

Output rows, 5 ms apart, from ~0.2 s before motion onset to |pitch|>70:
  D , i , pitch_x100 , rate_x10 , wl , wr , wls , wrs
(wl/wr wheel ANGLES deg, wls/wrs wheel SPEEDS deg/s)
Per-rep summary:
  R , rep , battery_mV , n_rows , t45_ms , roll_deg
(t45 = onset to |pitch|>45; roll = mean wheel angle change over that)
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from hubconfig import DT, PITCH_AXIS, make_hub, make_motors, wait_until_upright

hub = make_hub()
left, right = make_motors()

REPS = 8
WINDOW_S = 3          # recording window per rep
END_DEG = 70          # stop recording: it is on the cushion
VOID_DEG = 20         # never passed this = no topple happened
ONSET_RATE = 5        # deg/s sustained...
ONSET_TICKS = 3       # ...marks motion onset (for print trimming only)
PRE_ROWS = 40         # keep 0.2 s before onset

N = WINDOW_S * 1000 // DT
b_pitch = [0] * N
b_rate = [0] * N
b_wl = [0] * N
b_wr = [0] * N
b_wls = [0] * N
b_wrs = [0] * N

print("battery mV:", hub.battery.voltage())
print("topple probe: motors coasting, window", WINDOW_S, "s, reps", REPS)
print("R,rep,battery_mV,n_rows,t45_ms,roll_deg")
print("D,i,pitch_x100,rate_x10,wl,wr,wls,wrs")

watch = StopWatch()
rep = 0
while rep < REPS:
    hub.light.on(Color.RED)
    left.stop()
    right.stop()
    wait_until_upright(hub)
    pitch0 = hub.imu.rotation(PITCH_AXIS)
    left.reset_angle(0)
    right.reset_angle(0)
    batt = hub.battery.voltage()
    hub.light.on(Color.GREEN)

    t0 = watch.time()
    n = 0
    for k in range(N):
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        b_pitch[k] = int(pitch * 100)
        b_rate[k] = int(hub.imu.angular_velocity(PITCH_AXIS) * 10)
        b_wl[k] = int(left.angle())
        b_wr[k] = int(right.angle())
        b_wls[k] = int(left.speed())
        b_wrs[k] = int(right.speed())
        n = k + 1
        if abs(pitch) > END_DEG:
            break
        wait(max(0, t0 + DT * (k + 1) - watch.time()))

    peak = 0
    for k in range(n):
        a = abs(b_pitch[k])
        if a > peak:
            peak = a
    if peak < VOID_DEG * 100:
        print("void: no topple, redoing rep", rep)
        continue

    # motion onset, for trimming the printout
    onset = 0
    run = 0
    for k in range(n):
        run = run + 1 if abs(b_rate[k]) > ONSET_RATE * 10 else 0
        if run >= ONSET_TICKS:
            onset = k - ONSET_TICKS + 1
            break
    start = max(0, onset - PRE_ROWS)

    t45 = -1
    for k in range(onset, n):
        if abs(b_pitch[k]) > 4500:
            t45 = (k - onset) * DT
            break
    end45 = onset + t45 // DT if t45 >= 0 else n - 1
    roll = ((b_wl[end45] - b_wl[onset]) + (b_wr[end45] - b_wr[onset])) // 2

    hub.light.on(Color.BLUE)
    print("R,", rep, ",", batt, ",", n - start, ",", t45, ",", roll)
    for k in range(start, n):
        print("D ,", k - start, ",", b_pitch[k], ",", b_rate[k], ",",
              b_wl[k], ",", b_wr[k], ",", b_wls[k], ",", b_wrs[k])
    rep += 1
    wait(500)

print("END")
