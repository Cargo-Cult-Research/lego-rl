"""The control law's constants — SINGLE SOURCE, imported everywhere.

Units: duty% per deg / deg/s on (pitch, gyro rate, motor angle, motor speed).

Every hub program imports this module (pybricksdev bundles local imports into
the upload), and the Mac side reads the same file through
src/lego_rl/gains.py. Do not paste these numbers anywhere else: the last
audit found them duplicated in 18 files.

To retune: scripts/tune_gains.py (CEM in the measured sim) prints a
Pybricks-unit line — it goes HERE and nowhere else.
"""

# CEM in the measured-parameter sim, 2026-08-22 (held-out 10 s survival 100%).
# vs the reference: 8x less angle stiffness, 2.5x more rate damping — what a
# short light pendulum with a ~70 ms time constant wants.
GAINS_SIM_TUNED = (10.71, 0.87, 0.43, 0.30)

# Published Pybricks balancer (tall heavy robot). Kept for context only: in
# the measured sim they survive 0% of episodes, and on hardware they fell in
# 0.755 s — exactly as the sim predicted.
GAINS_REFERENCE = (88, 0.35, 0.72, 0.19)

# Retired terms, each with the run that retired it. They stay as named
# constants so the control law reads the same everywhere.
K_SYNC = 0         # yaw loop: innocent — off changes the pitch ring not at
                   # all (run 5). Proportional-only and undamped anyway.
FRICTION_COMP = 0  # the reference design's +-10% duty kick. On a body this
                   # light it is a bang-bang oscillator (run 1); removing it
                   # halved the peak excursion, 11.7 -> 6.6 deg (run 8).
