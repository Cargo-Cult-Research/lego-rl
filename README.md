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
      stands indefinitely at 1.50° pitch RMS / 3.6° peak; the residual ring was
      traced to mechanical compliance and largely fixed by bracing
- [x] **M1** sim credibility — with *measured* parameters the published
      reference gains fail here, and CEM in-sim retuning gives
      `(10.71, 0.87, 0.43, 0.30)`; see "What the verifier caught" below
- [x] **M2** sysID pass: most GUESSes burned down (`robot/sysid_*.py`);
      `axle_half_width`, `stall_torque`, `imu_angle_noise`, `ground_friction`
      still outstanding
- [x] **M3** PPO balancer in sim (`scripts/train_ppo.py`) — 100% of full 10 s
      episodes under domain randomization, peak drift 7 cm, returns to 3 cm
- [x] **M4** verifier: policy Jacobian ≈ classical gains (`scripts/linearize.py`)
      — signs all agree; after removing a uniform 0.66× scale the feedback
      *shape* matches the CEM controller within 2% (pitch rate) and 6% (wheel
      rate), with wheel position 2× low. See "The verifier's verdict" below
- [x] **M5** deploy: the learned policy balances on hardware at a full 200 Hz
      (`scripts/export_policy.py`, `scripts/make_fast_policy.py`,
      `robot/balance_policy.py`) — **1.17° pitch RMS, zero duty clamping**,
      after an 11× inference speedup and one filter that the sim should have
      taught it
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
robot stands indefinitely and holds station to within a couple of centimetres,
at **1.50° pitch RMS and 3.6° peak** over a 10 s window. The first hardware
run was 14 Hz at ±12°, duty pinned to the rail 58% of the time, and a fall
into the furniture.

Twelve hardware runs are recorded in `data/run_NN_*/`, each with its raw
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

And then the part software could not fix. **The residual ring was
mechanical, and the robot's own hardware proved it.**
Bracing the structure (410 g → 429 g, run 12) cut the ring amplitude 81%,
1.75° → 0.34°, and **raised its frequency** from 10.6 to 11.5 Hz. Adding mass
alone lowers a resonance (ω ∝ √(k/m)); it rose, so stiffness grew faster than
mass — the signature of stiffening a compliant structure, and something no
control parameter ever did across eleven runs. The IMU sits on the hub, so the
loop was closed around a sensor that was not rigidly attached to the body being
controlled. Unfixable in software, which is exactly why six software hypotheses
died in a row and the residual outlived all of them.

## The verifier's verdict (2026-08-22)

The point of the project. A policy trained by PPO on a MuJoCo model built from
kitchen-scale measurements, linearised at the upright equilibrium, against the
four-gain controller CEM found by direct search in the same sim — two methods
that share a plant and nothing else:

| state | learned | sim-tuned | ratio |
|---|---|---|---|
| pitch | 7.057 | 10.710 | 0.66 |
| pitch rate | 0.563 | 0.870 | 0.65 |
| wheel | 0.148 | 0.430 | 0.35 |
| wheel rate | 0.186 | 0.300 | 0.62 |

Signs all agree, and three gains sit at a near-identical 0.62–0.66. A *uniform*
scale factor is not a pipeline fault: CEM maximised survival time alone while
PPO pays a quadratic cost on lean, drift and effort, so the two optimise
different objectives on the same plant and land on differently-scaled versions
of the same law. Normalising both by their own pitch gain isolates the
structure:

| state | learned/pitch | tuned/pitch | ratio |
|---|---|---|---|
| pitch rate | 0.0798 | 0.0812 | **0.98** |
| wheel rate | 0.0264 | 0.0280 | **0.94** |
| wheel | 0.0210 | 0.0401 | 0.52 |

Two of three within 6%. The residual is wheel position, 2× low, which is the
same axis the reward weights directly — a remaining objective difference rather
than a discovered bug, and left alone rather than tuned away.

### Two bugs this caught, one of them mine

The verifier's first run had a wheel-position gain **33× below** the classical
controller. The cause was in the reward, not the training: lateral position was
weighted at 0.1 on raw metres, making a 5 cm drift 111× cheaper than a 5° lean,
with position only mattering at half a metre. A policy cannot learn feedback it
is never rewarded for. Predicted from reading the code before the run that
confirmed it.

The first fix was worse. Raising the weight to 10 without bounding it let the
penalty reach 40 against an alive bonus of 1, so *falling over early scored
better than staying up* — and the agent learned exactly that (episode length
2000 → 674, return +1990 → −890). Both reward terms are now normalised by their
own termination limit with weights summing to 0.95, so a degree of lean and a
centimetre of drift cost the same and survival always dominates.

| | survival | full episodes | peak drift | ends at |
|---|---|---|---|---|
| before | 9.40 s | 63% | 40.3 cm | 40.3 cm |
| after | 10.00 s | 100% | 7.1 cm | 3.0 cm |

## The policy on the robot (M5)

The exported net cost **13–18 ms against a 5 ms budget**, so the first hardware
run went at 69 Hz — and balanced anyway, 2.43° RMS. 13 ms for 104
multiply-accumulates is ~125 µs per MAC on a 100 MHz Cortex-M4F with a hardware
FPU: four orders of magnitude off the metal, so essentially none of it was
arithmetic. Six implementations later (`robot/_bench_fast.py`):

| variant | µs/call | of budget |
|---|---|---|
| as exported (generators, `exp` tanh) | 18125 | 362% |
| unrolled floats | 10296 | 206% |
| unrolled + float tanh LUT | 11642 | 233% |
| fixed point Q10, lists and loops | 2758 | 55% |
| fully unrolled fixed point Q12 | 1241 | 25% |
| **+ interpolated tanh (deployed)** | **1607** | **32%** |

Two results that invert the usual instincts. **A float tanh lookup table is
slower than calling `exp()`** — `exp()` is one C call, the table lookup is ten
interpreted operations, each boxing a float. On this platform you count
interpreter dispatches, not FLOPs. And **inlining the weights as literals was
worth 2.2× on its own**, more than the float→integer conversion, by deleting
104 bounds-checked list subscripts per pass. Fixed point wins because small
ints are tagged immediates that never touch the heap, not because integer
arithmetic is faster.

Then the speedup made the robot *worse* — RMS 2.43° → 8.06°, fell at 5.5 s.
Running at 69 Hz had been acting as a low-pass filter nobody designed. At
200 Hz the policy chases the ~11 Hz structural mode, on raw gyro, having
trained in a sim with no resonance in it.

Run 16 settled it: four segments, one battery, classical repeated at both ends
to bracket drift.

| segment | battery | RMS | peak | clamp | lasted |
|---|---|---|---|---|---|
| classical | 7877 mV | 3.31° | 7.5° | 7% | 10 s |
| policy, raw gyro | 7874 mV | 7.35° | 43.5° | 47% | **fell at 4.0 s** |
| **policy, 30 ms filter** | 7872 mV | **1.17°** | **3.7°** | **0%** | 10 s |
| classical again | 7867 mV | 1.56° | 4.4° | 15% | 10 s |

Identical policy, identical rate, batteries two millivolts apart. The filter is
the only difference and it is worth 6.3×. Battery drift across the session was
10 mV, so the confound that made the previous run uninterpretable is gone.

**1.17° is the lowest pitch RMS this robot has produced.** But "learned beats
classical" is *not* established: classical scored 3.31° and 1.56° on identical
configurations at the same voltage, and 1.17 sits inside that 2× spread. What
sits far outside it is raw versus filtered. The filter is also a patch over a
known sim-to-real gap — the right fix is to put the resonance in the simulator
so the policy learns to reject it.

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
| wheel_radius, masses, com_height | calipers, kitchen scale, balance on a straightedge | measured (75 mm, 429 g braced, 50 mm) |
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
