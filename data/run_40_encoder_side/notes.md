# Run 40 — encoder side of the free play: output side. Model flipped.

Urs's question, Urs's experiment: with motors coasting, wiggle the
wheel/lever gently WITHIN the free play and watch whether angle()
follows. It does — 155 encoder changes in 12 s spanning ~4 deg,
tracking the wiggle.

## What flips

The lash model assumed a motor-side encoder: encoder+actuator on the
`wheel` joint, tyre hanging through the lash, encoder "leading" the
wheel through the gap — and the run-14-era narrative (crossing a 2 deg
gap in 20 ms reads as 100 deg/s of fictional wheel speed) was built on
that. Measured reality is the reverse: the encoder tells the truth
about the wheel, and it is the MOTOR TORQUE that arrives late, through
the gap. env._true_state now returns wheel+lash for the wheel channels
(actuator stays on the motor-side joint), so obs, reward, and the
controller all see the output side, as the hub does.

Note the observed ~4 deg total span vs the felt-play range (0.6–5 deg
total): consistent, upper half. Some of the 4 deg may be gear-mesh
elasticity rather than pure gap; the probe cannot distinguish.

## Still open

Backdrive breakaway (the asymmetry number) — now its own probe,
robot/sysid_backdrive.py, needs the lever-on-scale setup this session
did not have.
