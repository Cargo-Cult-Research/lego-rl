`motor.speed()` is a differentiated encoder feeding the same
delayed loop as the gyro rate term that was already found guilty -- structurally
the identical bug on the other sensor, and never once logged.

It is not a bug. Set it to zero and the robot topples within 2.5 s at 2.8 Hz,
which is the bare pendulum mode: it stops controlling and just falls. The term
is what holds the thing up.

It is also the largest term in the controller -- mean 18.5, peak 94 duty%
against a clamp of 40, with the wheels swinging +-313 deg/s -- and the only
remaining unfiltered high-frequency path. So the question was never "remove it"
but "filter it", which is the segment this run never reached: the destructive
config was ordered second and ended the run early. Bad experiment design,
rewritten for the next one.
