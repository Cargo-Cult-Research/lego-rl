# lego-rl

Sim-to-real RL for a LEGO 42124 rebuilt as a two-wheeled inverted pendulum:
Technic Hub + both L angular motors, one direct-driving each wheel (no
external gears; the motors' internal gearboxes still contribute ~1 deg of
play each — run 17), hub mounted low. Runs Pybricks 3.6.1 on the hub; MuJoCo (CPU) + PPO
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
      stands indefinitely (sigma ~1.0° about the segment mean, run 22); the
      residual ring was cut 81% by bracing (run 12) and finally attributed to
      the gearbox deadband (runs 17, 24)
- [x] **M1** sim credibility — with *measured* parameters the published
      reference gains fail here, and CEM in-sim retuning gives
      `(10.71, 0.87, 0.43, 0.30)`; see "What the verifier caught" below
- [x] **M2** sysID pass: most GUESSes burned down (`robot/sysid_*.py`);
      `axle_half_width`, `stall_torque`, `imu_angle_noise`, `ground_friction`
      still outstanding
- [x] **M3** PPO balancer in sim (`scripts/train_ppo.py`) — 100% of full 10 s
      episodes under domain randomization, peak drift 7 cm, returns to 3 cm
      (trained before the wheel-contact and backlash sim fixes of 2026-08-24;
      a retrain on the fixed plant is pending)
- [x] **M4** verifier: policy Jacobian ≈ classical gains (`scripts/linearize.py`)
      — signs all agree; after removing a uniform 0.66× scale the feedback
      *shape* matches the CEM controller within 2% (pitch rate) and 6% (wheel
      rate), with wheel position 2× low. See "The verifier's verdict" below
- [x] **M5** deploy: the learned policy balances on hardware at a full 200 Hz
      (`scripts/export_policy.py`, `scripts/make_fast_policy.py`,
      `robot/balance_policy.py`), after an 11× inference speedup and one
      filter that the sim should have taught it. It does **not** beat the
      classical controller (run 18). Runs 22/24 found and fixed a +11% gain
      defect in the pipeline-CONTROL's fit (`linear_to_net.py`); the learned
      policy's own export chain measured clean, so run 18 stands
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
.venv/bin/python scripts/view.py [--policy runs/....zip]  # watch it live
.venv/bin/python scripts/render_rollout.py out.mp4 [--policy ...]  # film it
.venv/bin/python scripts/check_labbook.py             # lab book consistent?
```

## Where it is right now (2026-08-24)

**It balances.** With the configuration in `robot/hubconfig.py` +
`robot/gains.py` the robot stands indefinitely and holds station to within a
couple of centimetres, at **sigma ≈ 1.0° about the segment mean** (run 22,
120 s continuous; the drift-immune statistic run 20 forced — the older
"1.50° RMS" headline was RMS about a drifting reference and is not
reproducible from run 12's own hub output). The first hardware
run was 14 Hz at ±12°, duty pinned to the rail 58% of the time, and a fall
into the furniture.

Twenty-four runs are recorded in `data/run_NN_*/` (hardware and, since run
23, sim), each with its raw
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

And then the part software could not fix, which took two attempts to identify.

**The residual ring is drivetrain compliance, and it was measured, not
inferred.** Hold a wheel so it cannot turn and sweep its motor to the stops:
the encoder is on the motor side of the gearbox, so the angle it sweeps is the
slop. 100 randomized trials (run 17) say it is a **spring**, not a backlash gap
— a linear fit leaves residual 0.45 against 6.80 for a constant. Stiffness
**2.90 N·m/rad**, which against the robot's inertia reflected at the wheel
predicts a mode at **10.7 Hz**, against **10.6 Hz observed unbraced** and
11.5 braced. An encoder and a pair of hands landed on the frequency that
survived six software hypotheses, with no reference to any gyro signal.

It also explains the bracing result that started this: bracing stiffens the
frame the **motors** sit in, raising `k` and raising the frequency, which is the
direction the hardware actually moved.

An earlier story — flex in the hub's IMU mount — was *inferred* from the same
closed-loop frequency and is now believed wrong. It never explained the bracing
direction without strain, and its amplitude provably cancels with mass
(`θ ≈ α/ω²`), leaving frequency as its only free parameter. The lesson is in
`params.py` as a provenance category: `INFERRED` is not `MEASURED`, and an
inferred number deserves a wide randomization range, not a model built on top
of it.

Two things surfaced that nobody was looking for. **Nothing deflects until ~22%
duty** — the motor dead zone is 10%, so another ~12% goes to static friction in
the gearbox, meaning the robot at its usual ~12% mean duty runs
*stiction-locked and effectively rigid*. And **the two motors differ nearly 2×**
in compliance.

Current: **sigma ≈ 1.06°** (classical, run 22, drift-immune statistic).
The learned policy's honest standing: run 18 measured classical quieter in
5 of 6 paired cycles, and that stands — run 24 traced the +11% pipeline
defect to the pipeline-control's fit only; the policy's own export chain
measured clean. Its real deficiencies are the verifier-flagged weak position
gain and training on the pre-contact-fix sim.

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
(Caveat added after run 20: the RMS column in this table is taken about the
arm-time reference, which drifts with gyro bias — treat the *relative*
comparison as meaningful, not the absolute numbers; the drift-immune sigma
statistic exists from run 20 onward.)

**"Learned beats classical" was not established, and run 18 settled it: it
doesn't.** Switching the controller back and forth inside one continuous
balance — so release, floor position and battery hit both equally — classical
is quieter in **5 of 6 paired cycles** (2.66° vs 2.92°). Run 16's 1.17° was a
lucky segment inside 2× variance, exactly as the notes said at the time. In the
two cleanest cycles the gap is only 0.12–0.15°, so they are near-equivalent
when nothing is going wrong.

**The verifier's one flagged mismatch showed up on hardware.** The policy
travels **1.9× further** per segment (52° vs 28°) and drove into a wall. That
is precisely what the Jacobian predicted — `wheel: learned 0.148 vs sim-tuned
0.430, ratio 0.35` — a neural network's position feedback measured as 3× too
weak in simulation, cashing out as a robot that wanders off. A specific
real-world failure predicted from one number is what the project exists to
test.

They also fail differently in character: the policy commanded far less duty
than classical in run 18. (The duty-clamp statistics from that era are
tainted: MAX_DUTY=40 had no valid provenance and both controllers were found
riding it — run 20 — so the clamp was removed; run 22 ran unclamped with
clamp_pct 0 throughout.)

The gyro filter remains a patch over a known sim-to-real gap — the right fix is
to put the drivetrain compliance in the simulator so the policy learns to
reject it.

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
  LED-feedback probes (`robot/sysid_signs.py`, `robot/sysid_directions.py`).
  The one that bit us: raw `+Axis.Y` reads a tilt *toward* the LED face as
  negative, hence `PITCH_AXIS = -Axis.Y`.

## SysID checklist (feeds `src/lego_rl/params.py`)

| param | how to measure | status |
|---|---|---|
| wheel_radius, masses, com_height | calipers, kitchen scale, balance on a straightedge | measured (75 mm, 429 g braced, 50 mm) |
| no_load_speed, motor_friction_duty | `robot/sysid_motor.py` (wheels up) | measured (1632 dps @ 8.37 V, 10% dead zone) |
| delay_ctrl_steps | `robot/sysid_latency.py` | measured (15–19 ms) |
| imu rate noise | `robot/sysid_imu.py` | measured (0.25 dps) |
| hub orientation, motor directions, pitch sign | `robot/sysid_directions.py`, `sysid_signs.py` | measured |
| battery_v range | `hub.battery.voltage()` fresh vs dying | measured |
| stall_torque | lever arm + kitchen scale at duty=100 | **guess** |
| axle_half_width, imu_angle_noise, ground_friction | calipers / still-hub log / coast-down | **guess** |
| backlash_solref_s, lash_damping, drivetrain_damping_ratio, hub_* | solver/damping knobs — randomized wide, see `params.py` metadata | **guess** |

Update the number **and** its provenance tag — both live in the field's
`metadata` in `src/lego_rl/params.py`; an untagged field fails at import.

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
