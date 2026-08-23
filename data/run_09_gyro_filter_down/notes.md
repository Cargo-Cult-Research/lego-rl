Run 1 rang at 14 Hz with the gyro term raw; everything since has rung at
9-10.6 Hz with a 30 ms low-pass on it. A frequency that moves when the filter
moves is not a mechanical resonance — but those two runs were confounded,
because the friction term changed at the same time. Run 8 removed that
confound, so the filter could finally be tested alone.

The answer came in the opposite direction to the guess. At 15 ms the robot
fell within its segment, peaking at 43 degrees with 250 degrees of wheel
drift, and the raw segment never got to run. **Less filtering is worse**, so
the gyro term's phase lag is not what is feeding the oscillation.

The consolation prize is the best configuration measured anywhere so far —
tau = 30 ms, friction compensation off, yaw loop off, duty clamped to 40 —
holding 2.24 degrees RMS at 7.3 degrees peak. Compare run 1's 12 degrees.

Segment ordering was right this time, safest-first, so the fall cost only the
segment that was already the least likely to survive.
