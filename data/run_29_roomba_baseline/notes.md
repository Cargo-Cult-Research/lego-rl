# Run 29 — roomba mode exists: the arena, the task, and its known answer (sim-only)

2026-08-24. The next stage opens. New in the sim:

- **`src/lego_rl/model3d.py`** — the measured robot in 3D: free-floating
  chassis, one hinge per wheel at ±`axle_half_width` (that parameter is
  load-bearing for the first time), yaw, and a 1.5 × 1.5 m walled arena.
  Deliberately v1-simple: no backlash/lash joints yet, noted in the module
  so absence is not mistaken for a claim.
- **`src/lego_rl/roomba.py`** — the task. Observation is strictly
  hub-realizable (pitch with the walking-reference model, pitch rate, yaw
  rate, both encoder speeds — no world position). Action is two duties.
  Reward is *exploration*: each first visit to a cell of the 10×10 coverage
  grid pays 1.0, small alive bonus, no wall penalty — bouncing is the
  locomotion mechanic, not a failure. Termination at 30° tilt.
- **`scripts/roomba_baseline.py`** — the verifier analog: a hand-coded
  state machine (CRUISE / BACKOFF / TURN) on the classical balance law,
  detecting walls the way the hub will have to — encoder stall and
  sustained forward lean, no world knowledge.

## Baseline result: coverage ≈ 10–12% per 40 s episode

Mostly survives full episodes; occasionally falls at a wall. Overhead video
in this directory. This number is the bar a learned exploration policy must
beat, under identical observations.

## What building the baseline taught (three findings, each a run-6 echo)

1. **The speed term is load-bearing in 3D too**: pitch+rate alone falls
   5/5 within seconds; adding the wheel-speed term gives σ 0.18°. A version
   of the bouncer that clamped the speed term's authority to ±0.15 duty
   fell within 2 s every episode — the term is the loop's damping, and
   cruise thrust must come from slewing its *reference*, never from
   weakening it.
2. **Per-wheel traction is thin.** ~0.08 N·m per wheel before slip: an
   aggressive velocity loop breaks traction, the encoder spins toward
   no-load, and the speed loop whipsaws (observed ±25 rad/s with the robot
   nearly stationary). Real LEGO tyres will have their own µ — a sysID
   target when roomba goes to hardware.
3. **Speed tracking saturates low.** The commanded 6 rad/s reference yields
   ~1–2 rad/s of net travel; encoder speed is dominated by balance
   corrections. The scripted controller has no clean way through this —
   a learned policy coordinating lean and thrust is exactly what should.

## Next

- Train the explorer (2-output head; `make_fast_policy` needs a 2-output
  emitter before it can deploy).
- Port the deadband/backlash machinery into the 3D model.
- Hardware roomba program: `play.py`-style re-arming loop, two duties,
  the same stall/lean wall detection.
