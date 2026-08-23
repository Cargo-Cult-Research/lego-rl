"""Is the ~10 Hz oscillation structural rather than control?

The gain sweep showed a limit cycle whose frequency barely moves with gain and
which survives with the duty saturated only 3% of the time. That is not a
control-loop instability -- it smells like compliance between the hub (where
the IMU lives) and the wheels, i.e. the gyro measuring the hub twisting on its
mount instead of the robot's actual pitch.

Two open-loop phases, no feedback at all:

  CYAN    2.5 s, motors off. Tap the top of the hub. A decaying ring in the
          gyro = a structural mode, and its frequency is the suspect.
  MAGENTA 2.5 s, grip BOTH tyres so they cannot turn. Motor torque pulses
          react straight into the frame; if the hub twists and rings, the
          compliance is real and we can measure it.

Logs raw gyro at the full 200 Hz so anything up to 100 Hz is visible.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

hub = TechnicHub(top_side=-Axis.X, front_side=-Axis.Z)
left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
PITCH_AXIS = -Axis.Y

DT = 5
PHASE_MS = 2500
PULSE_DUTY = 40
PULSE_MS = 60      # on
GAP_MS = 400       # off

print("battery mV:", hub.battery.voltage())

# Countdown so there is time to get hands in position.
for c in (Color.RED, Color.ORANGE, Color.YELLOW):
    hub.light.on(c)
    wait(1000)

t_log = []
g_log = []
d_log = []
ph_log = []

watch = StopWatch()
n = 0

# --- Phase A: passive. Tap the hub. -------------------------------------
hub.light.on(Color.CYAN)
left.dc(0)
right.dc(0)
t_end = watch.time() + PHASE_MS
while watch.time() < t_end:
    t_log.append(watch.time())
    g_log.append(int(hub.imu.angular_velocity(PITCH_AXIS)))
    d_log.append(0)
    ph_log.append(0)
    n += 1
    wait(max(0, DT * n - watch.time()))

# --- Phase B: torque pulses into held wheels. ---------------------------
hub.light.on(Color.MAGENTA)
t0 = watch.time()
t_end = t0 + PHASE_MS
period = PULSE_MS + GAP_MS
while watch.time() < t_end:
    phase_t = (watch.time() - t0) % period
    if phase_t < PULSE_MS:
        k = (watch.time() - t0) // period
        duty = PULSE_DUTY if k % 2 == 0 else -PULSE_DUTY
    else:
        duty = 0
    left.dc(duty)
    right.dc(duty)
    t_log.append(watch.time())
    g_log.append(int(hub.imu.angular_velocity(PITCH_AXIS)))
    d_log.append(int(duty))
    ph_log.append(1)
    n += 1
    wait(max(0, DT * n - watch.time()))

left.dc(0)
right.dc(0)
hub.light.on(Color.BLUE)

print("samples:", len(t_log))
print("t_ms,phase,duty,gyro_dps")
for i in range(len(t_log)):
    print(t_log[i], ph_log[i], d_log[i], g_log[i], sep=",")
print("END")
