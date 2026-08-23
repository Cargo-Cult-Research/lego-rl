Open loop, no feedback anywhere. Three hub taps with the motors passive, then
torque pulses into wheels held by hand. This is the test the previous ten runs
had been pointing at, and it took three attempts to get clean data — the first
recorded blind on a timer and captured a flat 0 deg/s because nothing touched
the robot; the second died of MemoryError partway through, losing taps the
operator had already given it; the third scheduled its sampling against a
global counter that the arm-for-tap waits had already invalidated, so the first
tap ate the whole buffer at an unknown rate. The script now triggers on motion,
pre-allocates its buffer before anyone touches the robot, keeps a per-segment
time base, and prints its achieved sample rate for each segment. All four
segments came back at exactly 200 Hz.

| segment | RMS | peak | decay (1st -> 3rd third) | dominant |
| --- | --- | --- | --- | --- |
| tap 1 | 8.8 | 54 deg/s | 15 -> 1 | 3.5 Hz |
| tap 2 | 10.7 | 68 deg/s | 18 -> 1 | 4.0 Hz |
| tap 3 | 9.0 | 51 deg/s | 16 -> 1 | 4.8 Hz |
| pulses, wheels held | 10.6 | 49 deg/s | 13 -> 7 | 3.2 Hz |

Every tap decays to nothing inside about 400 ms with **no 10 Hz peak**: only
12% of the 3-80 Hz spectral mass falls in the 8-13 Hz band, and what energy
there is sits broadband and low. A lightly damped structural mode would ring at
one frequency and hang on. Nothing here does.

So the mechanical hypothesis is not supported — but this run cannot close it,
for two reasons that are the test's fault rather than the robot's.

The hand is inside the measurement. Someone has to hold the robot to tap it,
so hand damping is part of every number. Phase B makes that quantitative: 40%
duty into locked wheels should whip a 0.0009 kg m^2 body at roughly 760 deg/s
within a 60 ms pulse, and it measured 49 — fifteen times less. That is a grip
absorbing the reaction torque, so phase B largely measured a pair of hands.

And the taps happened in the air. Held aloft the tyres carry no load, so the
one compliance that is genuinely inside the control loop while balancing —
rubber tyre against desk — was never excited at all. A soft tyre is a spring
between the body and the ground, and that mode only exists with weight on it.

The refined version of this test taps the robot while it stands on the floor,
supported just enough not to fall.
