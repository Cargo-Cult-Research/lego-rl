Same gains, same robot, both discontinuities removed: a 30 ms
low-pass on the rate term (leaving the ~2 Hz pendulum dynamics it exists to
damp) and the friction compensation ramped linearly over +-4% duty instead of
stepped.

Real, large improvement. It stopped running away and stopped falling. And it
is still a robot vibrating at +-12 deg, with duty on the rail 60% of the time
-- which raised the obvious next question, and the obvious next question turned
out to be wrong.
