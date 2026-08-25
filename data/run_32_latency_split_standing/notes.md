# Run 32 — latency split, standing (hardware)

2026-08-24, the valid rerun of run 31's part 3 (prompt streamed correctly
this time; the robot stood, steadied by two fingertips). Raw log: `raw.log`.

| measurement | ms |
|---|---|
| loop worst period (part 1) | 6 |
| duty step → encoder motion (part 2) | 18, 22, 22, 23, 19 |
| duty step → gyro response (part 3) | 23, 16, 19, 18, 31, 19 — median ~19 |

## The split

The body's reaction to torque is effectively instantaneous, so part 3 ≈
command path + electrical rise + **sensor pipeline**. Median 19 ms with
maybe 3–4 ms of command+electrical means `hub.imu.angular_velocity()` hands
back a value **~15 ms stale**. Part 2 − part 3 ≈ 3 ms: the
stiction/backlash takeup share is small — the '15–19 ms actuation latency'
of 2026-08-22 was mostly the sensing pipeline all along (the encoder path
evidently carries a similar reporting lag).

## Consequences

- **Sim calibration:** total hardware dead time ≈ 20 ms, which
  `delay_ctrl_steps = 4` already models (dead time is dead time regardless
  of which side of the loop it sits on). So run 28's lag-fragility is NOT
  explained by unmodeled latency — the static-friction and stall_torque
  suspects are now clearly ranked first.
- **Run 30 deepens:** with the sensing pipeline underneath, the 90 ms
  filter segment ran at ~105 ms of total rate-channel lag — and was
  *quieter* than the 30 ms anchors.
- **Latency levers, ranked by this split:** (1) the IMU pipeline — Pybricks
  exposes IMU settings; whether the ~15 ms is ODR, internal filtering, or
  driver-side averaging is checkable in the Pybricks source and possibly
  configurable; (2) the 5 ms control tick (0–5 ms quantization); (3)
  mechanics — barely worth touching at ~3 ms.

## Addendum (same day): the firmware says the gyro is NOT the slow part

Reading Pybricks 3.6.1's source (`lib/pbio/drv/imu/imu_lsm6ds3tr_c_stm32.c`,
`lib/pbio/src/imu.c`): the LSM6DS3TR-C runs at **833 Hz ODR** for both gyro
and accelerometer, and `angular_velocity_calibrated` is updated per frame —
the value `angular_velocity()` returns is ~1.2 ms fresh, with only bias/scale
applied on read. `imu.settings()` exposes thresholds and calibration only;
there is no rate or bandwidth knob, and nothing to slow it down.

So the headline above needs correcting: the ~15–19 ms that parts 2 and 3
SHARE cannot be sensing — it sits in the **command→torque path**
(`dc()` → pbio → PWM → electrical rise), possibly padded by the fingertip
steadying damping the initial kick (per-trial spread 16–31 ms supports some
of that). The gyro and encoder each add only a couple of ms on top.

Revised split: command→torque ≈ 15 ms (mechanism inside Pybricks/driver
unidentified — next firmware dive), mechanics ≈ 3 ms, sensing ≈ 1–2 ms.
The sim conclusion is unchanged: total dead time ~20 ms, already modeled.
The latency lever moves from "IMU pipeline" to "why does a duty command
take 15 ms to become torque" — a pbio/dcmotor question, not a sensor one.
