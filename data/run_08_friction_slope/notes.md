Ramping the friction term removed its discontinuity but not its steepness. At
`FRICTION_COMP=10` over `FC_RAMP=4` the slope near zero is 1 + 10/4 = 3.5, so
the controller carries 3.5x the loop gain for small commands -- and a limit
cycle is exactly the regime where the command hovers near zero. High
small-signal gain into a fixed delay was the one story that predicted all
three signatures at once: constant frequency, constant amplitude, and
indifference to every gain change.

It was wrong about the frequency. All three slopes ring at 10.4-10.6 Hz.

It was right that the term is hurting. Turning the compensation off entirely
halves the peak excursion, from 11.7 deg to 6.6, and gives the lowest pitch RMS
of any configuration to date -- with drift still about a centimetre, so the
uncompensated 10% dead zone costs far less than assumed. The reference
design's friction compensation is simply wrong for a robot this light.

Five software hypotheses are now dead as causes. But the series as a whole
says something the individual runs did not: run 1 rang at 14 Hz with the gyro
term raw, and everything since has rung at 9-10.6 Hz with a 30 ms filter on
it. That frequency MOVED when the filter moved, which a mechanical resonance
would not do. Those runs were confounded by the friction term changing at the
same time -- a confound this run has just removed.
