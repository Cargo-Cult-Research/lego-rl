This run was an accident worth keeping. The intent was to sweep the gyro
filter *upward* (30 / 45 / 60 ms); the edit that was supposed to change the
segment list silently failed to match, so the hub re-ran run 9's exact
configuration instead. The script now asserts that its own edit matched before
anything reaches the robot.

The accident replicated run 9 and finished the job, because this time the
robot survived long enough to reach the raw-gyro segment that run 9 never got
to:

| gyro filter | run 9 | run 10 |
| --- | --- | --- |
| tau = 30 ms | 2.24 deg RMS, 7.3 peak | 3.20 RMS, 8.0 peak |
| tau = 15 ms | 7.56 RMS, 43.2 peak (fell) | 7.14 RMS, 12.5 peak |
| raw | never ran | 9.39 RMS, 43.6 peak, 236 deg drift |

Monotonic across two independent runs: less filtering is worse, and the raw
gyro term is catastrophic. So the filter's phase lag is not feeding the
oscillation — if it were, more filtering would hurt, and it plainly helps.
That is the sixth software hypothesis to die.

What survives everything now is the ~10 Hz ring itself, indifferent to gain
magnitude, duty clamping, the yaw loop, the speed term and its filtering, the
friction compensation's slope, and the gyro filter. The remaining candidate is
mechanical: compliance between the hub, where the IMU sits, and the wheels.
That test needs hands on the robot and has not been run properly yet.
