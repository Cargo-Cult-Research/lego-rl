# Run 37 — slide probe: no fast accel fusion in rotation()

`robot/sysid_slide.py`, one rep captured (the session ended before rep
2; one is decisive here). **Operator note (part of the record):** making
the wheels roll while sliding required pressing down slightly -- the
slide backdrives the sticky planetary gearbox -- done holding the robot
with two hands. Consistent with the 22%-duty stiction (run 17) and the
locked-wheel topples (run 36).

## Measurement

10 s window at 100 Hz: 2 s still, 8 s sliding at +-2 m/s^2, 2 s still.

- Pitch reading peak-to-peak 4.59 deg; pure gyro integral of the logged
  rate: 4.58 deg. The reading IS the gyro integral -- the body genuinely
  rocked +-2.3 deg in the operator's hands (that rocking is also why
  pitch correlates +0.5 deg per m/s^2 with acceleration: mechanics, not
  the sensor).
- Direct estimate of the fusion correction (reading minus gyro integral,
  regressed against accel-implied tilt error): crossover ~0.03 Hz, i.e.
  a ~30 s time constant. Still-window residual drift +-2 mdeg/s.

That 0.03 Hz is exactly the regime the reference-walk measurements
implied (runs 20/26; run 34 walked to -29 deg over ~2 min): the fusion
exists, but it is glacial -- invisible at control timescales.

## What this rules out, and what it opens

The joint fit's standing plant (data/sysid_fit_best.json) had two
load-bearing legs: the soft motor (stall 0.10 + armature 1.28) and
imu_fusion_hz = 4.6. This run kills the second leg as physics: whatever
4.6 Hz fusion was doing for the fit, the real hub does not do it. It was
a proxy for something still missing (candidates: tire contact patch
mechanics, the motor curve's true shape, the ring compliance still
default-off).

Next: the stall-torque probe (lever + scale) adjudicates the first leg;
refit with imu_fusion_hz capped at the measured 0.03 Hz once that
number is in. sysid_fit.py's search bound is updated to 0.05 Hz so the
optimizer can never again buy stability with unphysical fusion.
