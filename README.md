# lego-rl

Sim-to-real RL for a LEGO 42124 rebuilt as a two-wheeled inverted pendulum:
Technic Hub + both L angular motors, one direct-driving each wheel (no gears,
no slop), hub mounted low. Runs Pybricks 3.6.1 on the hub; MuJoCo (CPU) + PPO
on the Mac.

The point of the project is the **verifier**: get the classical four-gain
controller (Pybricks reference: ~88, 0.35, 0.72, 0.19 duty% per deg / deg/s
on gyro angle, gyro rate, motor angle, motor speed) working on hardware,
train PPO in sim, then linearize the learned policy at the upright
equilibrium and compare its Jacobian to those four gains. Rough agreement
validates the entire pipeline — model, sysID, domain randomization,
training, export — against a known answer.

## Milestones

- [x] **M0** classical balancer on hardware (`robot/balance_classical.py`) —
      it stands indefinitely and holds station, with a residual ~10 Hz ring
      (~2.2° pitch RMS) that is still unexplained
- [x] **M1** sim credibility — with *measured* parameters the published
      reference gains fail here, and CEM in-sim retuning gives
      `(10.71, 0.87, 0.43, 0.30)`; see "What the verifier caught" below
- [x] **M2** sysID pass: most GUESSes burned down (`robot/sysid_*.py`);
      `axle_half_width`, `stall_torque`, `imu_angle_noise`, `ground_friction`
      still outstanding
- [ ] **M3** PPO balancer in sim (`scripts/train_ppo.py`)
- [ ] **M4** verifier: policy Jacobian ≈ classical gains (`scripts/linearize.py`)
- [ ] **M5** deploy: export MLP, time it on the hub, balance on hardware
      (`scripts/export_policy.py`, `robot/balance_policy.py`)
- [ ] **M6** swing-up from lying flat — the task linear feedback cannot do
      (`scripts/train_ppo.py --task swingup`)

## Quickstart

```sh
uv venv --python 3.12 && uv pip install -e ".[dev]"
.venv/bin/pytest -q
.venv/bin/python scripts/verify_classical.py          # M1
.venv/bin/python scripts/tune_gains.py                # CEM retune in the measured sim
.venv/bin/python scripts/train_ppo.py                 # M3 (~2M steps)
.venv/bin/python scripts/linearize.py runs/ppo_balance_seed0.zip   # M4
.venv/bin/python scripts/export_policy.py runs/ppo_balance_seed0.zip  # M5
.venv/bin/python scripts/view.py [--policy runs/....zip]  # watch it
```

## Where it is right now (2026-08-22)

**It balances.** With the configuration in `robot/balance_classical.py` the
robot stands indefinitely, holds station to within a couple of centimetres,
and does not fall — while oscillating at ~10 Hz with a pitch RMS of about
2.2° and peaks around 7°. Compare the first hardware run: 14 Hz, ±12°, duty
pinned to the rail 58% of the time, and a fall into the furniture.

Eleven hardware runs are recorded in `data/run_NN_*/`, each with its raw
telemetry, the question it was meant to settle, and what it actually showed —
including the null runs and the six hypotheses that turned out wrong. The
write-up renders from those directories (`scripts/build_page.py`).

What was found guilty and fixed:

- **A raw gyro term feeding a 19 ms-delayed loop.** A 30 ms low-pass took the
  oscillation from 14 Hz to 9 and the peak gyro from 404 to 210 °/s. Sweeping
  the filter *down* is worse in both directions tested — 15 ms and raw both
  fell over (runs 9, 10).
- **The reference design's ±10% coulomb-friction compensation.** A 20-point
  discontinuity at every zero crossing; on a body with ~0.0009 kg·m² of
  inertia that step alone is worth ~130 °/s of gyro within one sample.
  Ramping it helped; **removing it entirely halved the peak excursion**
  (11.7° → 6.6°) with drift still ~1 cm (run 8).

What was ruled out, each with the run that killed it: duty saturation (rings
at 3% saturation), the yaw loop `K_SYNC` (switching it off changes nothing),
the wheel-speed term (**load-bearing** — remove it and the robot falls in
2.5 s), filtering that term (lag on a stabiliser costs exactly what it should),
and the friction term's small-signal slope.

The residual ~10 Hz ring is unexplained. An open-loop ring test found no
structural resonance — three hub taps decay to nothing in ~400 ms with only
12% of spectral mass in the 8–13 Hz band — but that test was run with the
robot held in the air, so the tyre-against-ground compliance that is actually
inside the control loop was never excited. That is the next measurement.

## What the verifier caught (2026-08-22)

The published Pybricks gains `(88, 0.35, 0.72, 0.19)` are for a tall, heavy
robot. Once `params.py` held *measured* values for this build — 410 g total,
CoM only 50 mm above the axle, ~19 ms actuation delay — those gains balanced
the sim for a mean of 4.6 s and survived 0% of full episodes. CEM retuning in
the measured sim gives `(10.71, 0.87, 0.43, 0.30)`: **8× less angle stiffness
and 2.5× more rate damping**, which is what a short, light pendulum with a
~70 ms time constant should want.

On hardware the reference gains fell in 0.755 s, exactly as the sim predicted.
That is the whole point of the verifier — the sim disagreed with the published
answer *before* the hardware did, and was right.

## Hardware gotchas

- **Pin firmware to Pybricks 3.6.1.** With v4.0.1 every released `pybricksdev`
  uploads to 100% and the program silently never starts.
- macOS: the hub is a BLE peripheral, so it never appears in the Bluetooth
  menu. Blinking blue = advertising, slow pulse = connected idle.
- `pybricksdev`'s scan window is ~10 s and the hub sleeps when idle — drive it
  from a retry loop, not a single call.
- Pressing the hub button re-runs the **last downloaded** program. If you are
  debugging over BLE, make sure you know which one that is.
- Every sign in the chain was verified empirically, not reasoned about, using
  LED-feedback probes (`robot/sysid_signs.py`, `robot/sysid_motor_pulse.py`).
  The one that bit us: raw `+Axis.Y` reads a tilt *toward* the LED face as
  negative, hence `PITCH_AXIS = -Axis.Y`.

## SysID checklist (feeds `src/lego_rl/params.py`)

| param | how to measure | status |
|---|---|---|
| wheel_radius, masses, com_height | calipers, kitchen scale, balance on a straightedge | measured (75 mm, 410 g, 50 mm) |
| no_load_speed, motor_friction_duty | `robot/sysid_motor.py` (wheels up) | measured (1632 dps @ 8.37 V, 10% dead zone) |
| delay_ctrl_steps | `robot/sysid_latency.py` | measured (15–19 ms) |
| imu rate noise | `robot/sysid_imu.py` | measured (0.25 dps) |
| hub orientation, motor directions, pitch sign | `robot/sysid_directions.py`, `sysid_motor_pulse.py`, `sysid_signs.py` | measured |
| battery_v range | `hub.battery.voltage()` fresh vs dying | measured |
| stall_torque | lever arm + kitchen scale at duty=100 | **guess** |
| axle_half_width, imu_angle_noise, ground_friction | calipers / still-hub log / coast-down | **guess** |

Update the number **and** its provenance tag in `PROVENANCE`.

## Design notes

- Observation = motor-encoder-faithful: wheel angle is measured relative to
  the chassis, exactly what `motor.angle()` reads.
- Motor model: DC motor with back-EMF (stall torque + no-load speed at
  7.4 V), scaled by battery voltage, minus coulomb friction. Battery scaling
  and the +10% friction duty are the two hand-derived compensations in the
  Pybricks reference — here they are domain randomization parameters.
- Actions pass through a FIFO delay (5–25 ms randomized) to model loop latency.
- Policy head is fixed at 4→8→8→1 tanh (~100 MACs) so it can run in
  MicroPython at 200 Hz; the value net is bigger because it never deploys.
  If the forward pass doesn't fit the 5 ms budget, distilling to the linear
  gains from `linearize.py` is legitimate — near equilibrium the optimal
  policy essentially *is* linear.
