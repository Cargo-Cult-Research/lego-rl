# Run 36 — free topple: the pendulum was never the problem

`robot/sysid_topple.py`: motors coasting, robot released near balance,
8 reps alternating direction, pitch/rate/wheel-angles at 200 Hz.
Cushions caught every fall; zero voids.

## The measurement

Energy fit (omega^2 linear in cos theta, slope -2 lambda^2 — exact for a
frictionless pendulum, release-independent), band 3–40 deg:

    rep  dir t45_ms roll lambda  rms
      0  fwd    310   -1   9.24  1.12   <- sloppy first release, excluded
      1 back   2160    1   6.97  0.38   <- near-perfect release
      2  fwd    525   -2   7.46  0.33
      3 back    545    1   7.49  0.35
      4  fwd    510   -2   7.61  0.35
      5 back    575    1   7.41  0.37
      6  fwd    440   -1   7.93  0.35
      7 back    565    1   7.45  0.34

lambda = 7.46 median (7.0–7.9 across reps 1–7). roll_deg = +-1–2:
the drivetrain stayed STICTION-LOCKED throughout every topple, exactly
as run 17's 22%-duty threshold predicts — so the topple probes the
whole-robot-rolling-about-the-contact mode.

## Sim replication: 4% agreement

Same experiment in MuJoCo (nominal box model, wheel joint locked with
frictionloss, zero actuation): lambda 7.74, t(2->45) 560 ms — inside
the real spread. The free-wheel topple gives 11.75. Both plants exist;
the box model gets the locked one RIGHT. The earlier "com must be
0.07–0.09" reading of the cruise-check sweep confused the two: raising
com slowed the FREE plant to ~7.5, i.e. it was fitting the locked
plant's lambda by bending a measured parameter. Inertia suspect:
ruled out.

## Three follow-up fixes for the lag-fragility, all refuted same day

The real question is unchanged: hardware stands with the 30 ms filter
+ ~19 ms delay; sim falls in ~3 s. Tried in sim, station-keeping,
filter on:

1. **Wheel-joint stiction** (the missing static friction, 0.05–0.2 N*m
   around the measured 22%-duty breakaway): still falls at every level.
2. **Delay double-counting** (measured delay includes breakaway, so
   model stiction + shorter FIFO): the cliff is at ~5 ms — sim falls
   with ONE tick of delay at any stiction; hardware carries ~19 ms.
3. **Pitch damping** (tire contact patch / carpet proxy on the pitch
   dof): survives only at 0.02 N*m*s/rad = 40% of critical damping,
   while burning 80% mean duty. Not physical.

## Where this leaves the hunt

Single-parameter stories are exhausted: inertia (this run), stiction,
delay, damping — each individually refuted. The gap between "sim
classical falls at 5 ms delay" and "hardware stands at 19 ms" is a
factor ~4 in lag tolerance that no one knob supplies. Next move (Urs
called it before this run): a JOINT multi-probe fit — parameterize the
uncertain model dimensions (motor curve, contact, damping, signal path)
and fit them together against all real traces at once: topple (run 36,
pins the locked inertia), closed-loop station-keep segments (sigma,
duty histograms, the ~10 Hz ring), the run-35 cruise limit cycle
(rail-to-rail at 9 Hz, target-independent 640–890 deg/s), and the
run-35 impact whip. The closed-loop traces are the discriminator,
because the mismatch IS closed-loop.

Also on the table: 5-lump rigid inertia from part masses (measured
2026-08-27: motor 54 g x2, wheel 28 g x2, hub 250 g, frame 45 g — note
the tally, 459 g, sits 30 g above the 429 g whole-robot weighing;
re-weigh before trusting either) — needs part positions; and a
suspicion worth a probe: the hub's `imu.rotation()` is accel-fused, so
the hardware pitch signal may not be the pure integrated gyro the sim
feeds back.
