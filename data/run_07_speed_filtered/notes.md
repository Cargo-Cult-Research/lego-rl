Every segment keeps full authority; only the filter changes.

Lag on a stabilising term costs exactly what it should. Raw wins, and the ring
is indifferent to all three settings. That is four hypotheses dead --
saturation, the yaw loop, the speed term, and filtering the speed term -- and
the oscillation has been insensitive to every one of them.

The best configuration measured anywhere so far is this run's first segment:
raw speed term, 30 ms gyro filter, yaw loop off, duty clamped to 40, giving
**2.86 deg pitch RMS at +-8.8 deg peak**. A buzzing robot, but a standing
one.
