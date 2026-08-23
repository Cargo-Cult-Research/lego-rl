"""Is the ~10 Hz ring mechanical? (open loop -- no feedback anywhere)

Six software hypotheses are dead: saturation, the yaw loop, the wheel-speed
term, filtering that term, the friction compensation's slope, and the gyro
filter. The oscillation is indifferent to all of them, which points at
compliance between the hub -- where the IMU sits -- and the wheels. A control
loop closed around a sensor that is not rigidly attached to the controlled body
oscillates no matter how it is tuned.

  CYAN + white flash   3 taps. Hold the robot, tap the top of the hub. Each
                       tap arms on |gyro| > 25 deg/s and records a ring-down.
                       Motors are off and passive throughout.
  MAGENTA              grip BOTH tyres so they cannot turn. Motor torque
                       pulses react straight into the frame; if the hub twists
                       and rings, the compliance is real and measurable.

A decaying oscillation near 10 Hz in either phase is the answer.

Three bugs this script has already been through, each of which destroyed a run
the user had already performed:

1. Recorded blind on a timer, so a run where nothing touched the robot came
   back as a flat 0 deg/s. It now TRIGGERS on motion and says so when a
   segment captures nothing.
2. MemoryError partway through phase B, from growing four parallel lists to
   1500 entries on a microcontroller. The buffer is now a single int list
   allocated UP FRONT, so an over-long capture fails before anyone touches the
   robot.
3. Sampling scheduled against a global counter (`DT * n`) while the
   arm-for-tap loops advanced the clock without advancing n -- so the deadline
   was already in the past, `wait()` returned instantly, and the first tap ate
   the whole buffer at an unknown rate. Each segment now keeps its own time
   base, and the hub reports its achieved sample rate per segment, so a broken
   schedule announces itself instead of producing plausible garbage.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

DT = 5              # 200 Hz, so anything to 100 Hz is visible
TAPS = 3
TRIG_DPS = 25       # what counts as a tap
CAPTURE_MS = 1200   # ring-down window after each tap
ARM_TIMEOUT_MS = 20000
PULSE_PHASE_MS = 2500
PULSE_DUTY = 40
PULSE_MS = 60
GAP_MS = 400

TAP_N = CAPTURE_MS // DT
PULSE_N = PULSE_PHASE_MS // DT
CAP_N = TAPS * TAP_N + PULSE_N
PERIOD = PULSE_MS + GAP_MS

print("battery mV:", hub.battery.voltage())

# Allocated before any user interaction: if the heap is short, this is where it
# fails, not halfway through a run someone has already done.
g_buf = [0] * CAP_N
segs = []           # (phase, start_index, count, elapsed_ms)
w = 0

for c in (Color.RED, Color.ORANGE, Color.YELLOW):
    hub.light.on(c)
    wait(800)

left.dc(0)
right.dc(0)
watch = StopWatch()
taps_got = 0

# --- Phase A: passive ring-down, triggered on each tap ------------------
for tap in range(TAPS):
    hub.light.on(Color.CYAN)
    armed_at = watch.time()
    triggered = False
    while watch.time() - armed_at < ARM_TIMEOUT_MS:
        if abs(hub.imu.angular_velocity(PITCH_AXIS)) > TRIG_DPS:
            triggered = True
            break
        wait(5)
    if not triggered:
        break
    hub.light.on(Color.WHITE)
    taps_got += 1

    seg_start = w
    t0 = watch.time()               # this segment's OWN time base
    k = 0
    while k < TAP_N and w < CAP_N:
        g_buf[w] = int(hub.imu.angular_velocity(PITCH_AXIS))
        w += 1
        k += 1
        wait(max(0, t0 + DT * k - watch.time()))
    segs.append((tap, seg_start, k, watch.time() - t0))

    hub.light.on(Color.BLUE)        # settle before arming again
    wait(700)

# --- Phase B: torque pulses into held wheels ---------------------------
hub.light.on(Color.MAGENTA)
wait(1500)                          # time to get a grip on both tyres
seg_start = w
t0 = watch.time()
k = 0
while k < PULSE_N and w < CAP_N:
    since = k * DT
    if since % PERIOD < PULSE_MS:
        duty = PULSE_DUTY if (since // PERIOD) % 2 == 0 else -PULSE_DUTY
    else:
        duty = 0
    left.dc(duty)
    right.dc(duty)
    g_buf[w] = int(hub.imu.angular_velocity(PITCH_AXIS))
    w += 1
    k += 1
    wait(max(0, t0 + DT * k - watch.time()))
segs.append((9, seg_start, k, watch.time() - t0))

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

print("taps_captured:", taps_got, "of", TAPS)
if taps_got == 0:
    print("NOTE: no tap exceeded", TRIG_DPS, "dps -- phase A captured nothing")
for ph, st, cnt, ms in segs:
    rate = (1000 * cnt // ms) if ms else 0
    tag = "pulse" if ph == 9 else "tap" + str(ph + 1)
    print("seg", tag, ":", cnt, "samples in", ms, "ms =", rate, "Hz")
    if ms and (rate < 150 or rate > 260):
        print("  WARNING: expected ~200 Hz, this segment's timing is broken")
    if ph == 9:
        pk = 0
        for i in range(st, st + cnt):
            if abs(g_buf[i]) > pk:
                pk = abs(g_buf[i])
        if pk < 5:
            print("  NOTE: almost no rotation -- were the wheels actually held?")

print("t_ms,phase,duty,gyro_dps")
for ph, st, cnt, ms in segs:
    for k in range(cnt):
        if ph == 9:
            since = k * DT
            d = (PULSE_DUTY if (since // PERIOD) % 2 == 0
                 else -PULSE_DUTY) if since % PERIOD < PULSE_MS else 0
        else:
            d = 0
        print(k * DT, ph, d, g_buf[st + k], sep=",")
print("END")
