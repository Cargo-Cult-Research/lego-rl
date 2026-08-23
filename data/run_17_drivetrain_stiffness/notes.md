Eleven runs chased a ~10-11 Hz oscillation through six software hypotheses and
one mechanical one. This run measured where it comes from, using an encoder and
a pair of hands, with no reference to any gyro signal at all.

Hold a wheel so it cannot turn and sweep its motor to the stop one way and then
the other. The encoder is on the motor side of the gearbox, so the angle it
sweeps is the slop. Do it at many torques, because that separates two things
that feel identical in the hand: a **gap** reads the same angle at every
torque, a **spring** reads more as you push harder.

A first attempt with 6 careful trials was too few and too contaminated — the
motors are strong and a hand cannot fully stop a wheel. Urs proposed the fix:
hold both wheels and run 100 trials with the motor and the torque randomized.
Randomizing decorrelates grip fatigue from torque, so a tiring hand adds noise
instead of a trend. And slip does not need to be eliminated, only understood —
**slip can only ADD angle**, never subtract it, so the contamination is
one-sided and the lower tail of each duty band is the clean estimate.

| duty band | n | min | p10 | median | p90 | max |
| --- | --- | --- | --- | --- | --- | --- |
| 15-22 | 19 | 0 | 0.0 | 1.0 | 2.0 | 2 |
| 23-29 | 18 | 0 | 0.0 | 1.0 | 3.0 | 4 |
| 30-36 | 15 | 1 | 1.0 | 2.0 | 4.0 | 4 |
| 37-43 | 27 | 1 | 2.0 | 3.0 | 5.0 | 6 |
| 44-50 | 21 | 2 | 3.0 | 4.0 | 5.0 | 6 |

**It is a spring.** A linear fit to the p10 points leaves residual 0.45; a
constant leaves 6.80. Fifteen times better. So the backlash deadband model was
the wrong shape and is retired.

Taking the stiffness from the SLOPE (0.112 deg per duty%) rather than from
absolute angles makes it immune to however much duty is eaten before anything
moves: **1.45 N*m/rad per motor, 2.90 for both.** Against the robot's inertia
reflected at the wheel (M r^2 + wheel inertia = 6.4e-4 kg m^2) that is a mode
at **10.7 Hz**, against **10.6 Hz observed unbraced** and 11.5 braced.

It also explains what the hub-mount story never could. Bracing stiffens the
frame the MOTORS sit in, which raises k and therefore raises the frequency —
which is the direction the hardware actually moved. The mount-flex model had to
strain for that, and its amplitude provably cancels with mass, leaving
frequency as its only free parameter.

Two things surfaced that nobody was looking for.

**Nothing deflects until ~22% duty.** The motor dead zone is 10%, so a further
~12% is absorbed by static friction inside the gearbox. The robot balances at
about 12% mean duty, which means it normally runs STICTION-LOCKED and
effectively rigid, and the compliance only wakes up on large swings. That is a
plausible reason the real robot tolerates a resonance that destabilised every
version of the simulator.

**The two motors differ nearly 2x** — portA 3.86 against portB 2.20 in
both-motor-equivalent stiffness. Either real unit variation or two hands
gripping differently, and worth knowing before trusting a single number.

Remaining caveat: `stall_torque` is still a GUESS and the predicted frequency
goes as its square root (0.18 N*m gives 9.1 Hz, 0.35 gives 12.6). Turned
around, the frequency match is weak evidence that stall_torque sits near
0.22-0.28 — the first real constraint on a number that has been a placeholder
since the project started.
