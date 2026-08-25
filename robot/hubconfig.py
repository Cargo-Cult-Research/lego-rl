"""This robot's hub, wiring, and loop constants — SINGLE SOURCE.

Everything here is what a DIFFERENT robot (same hub, same motors, other
physics) would edit, gathered in one file so it edits exactly one. Every hub
program imports it; pybricksdev bundles local imports into the upload
automatically (compile_multi_file), so this costs nothing at deploy time.

Orientation and directions are MEASURED, not assumed — robot/sysid_directions.py
is the probe that produces these constants, robot/sysid_signs.py the
closed-loop check. If the build changes, re-run both before trusting a sign.
"""
from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import wait

# --- orientation & wiring (sysid directions + signs probes, 2026-08-22) -----
TOP_SIDE = -Axis.X
FRONT_SIDE = -Axis.Z
PITCH_AXIS = -Axis.Y   # raw +Axis.Y reads a toward-the-LED-face tilt as
                       # negative; this sign bug cost an afternoon once.
LEFT_PORT, LEFT_DIR = Port.A, Direction.COUNTERCLOCKWISE    # +CW rolled backward
RIGHT_PORT, RIGHT_DIR = Port.B, Direction.CLOCKWISE         # +CW rolled forward

# --- control loop (measured; each number cites the run that set it) ---------
DT = 5             # ms -> 200 Hz
V_NOM = 7400       # mV; duty is battery-compensated to this. Sim mirror:
                   # v_nominal in src/lego_rl/params.py (volts) — a test
                   # cross-checks the two.
FALL_DEG = 45      # beyond this, cut motors: it is over
RATE_TAU_MS = 30   # gyro low-pass. Raw rate was the original 14 Hz shake
                   # (run 1) and fell the 200 Hz policy at 4.0 s (run 16);
                   # sweeping the filter DOWN is worse in both directions
                   # tested (runs 9, 10).
MAX_DUTY = 100     # i.e. no artificial clamp. The old 40 had no valid
                   # provenance (one pre-lab-book segment, config long gone)
                   # and run 20 found both controllers riding it, which made
                   # every comparison hypersensitive. Unclamped since run 20;
                   # run 22 (120 s, 20 segments) never needed the rail:
                   # clamp_pct 0 everywhere. Log the duty DISTRIBUTION
                   # instead of truncating it.

# --- launch ritual: RED until held upright and still --------------------------
ARM_ACCEL_Z = 8000   # mm/s^2: accel ~ +g on robot Z means upright
ARM_RATE_DPS = 2     # and not moving
ARM_STILL_TICKS = 50  # for 0.5 s (50 x 10 ms)


def make_hub():
    return TechnicHub(top_side=TOP_SIDE, front_side=FRONT_SIDE)


def make_motors():
    return (Motor(LEFT_PORT, LEFT_DIR), Motor(RIGHT_PORT, RIGHT_DIR))


def wait_until_upright(hub):
    """Block until the robot is held upright and still for ~0.5 s.

    The caller owns the LED (convention: RED while waiting, GREEN after)."""
    still = 0
    while still < ARM_STILL_TICKS:
        ok = (hub.imu.acceleration(Axis.Z) > ARM_ACCEL_Z
              and abs(hub.imu.angular_velocity(PITCH_AXIS)) < ARM_RATE_DPS)
        still = still + 1 if ok else 0
        wait(10)
