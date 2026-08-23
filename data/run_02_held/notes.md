Kept because a null run is still evidence about the rig, and
because it caught a real bug in the fix.

A free-standing two-wheeler falls within about half a second; this one sat
inside 0.6 deg of pitch with the wheels at exactly 0 deg for five seconds, so
it was supported the entire time. The lesson for the harness was to make the
hub say so itself, which later runs do.

The useful finding: duty sat at 2-6% and with `FC_RAMP=8` that becomes ~7% at
the motors, under the measured 10% dead zone, so the wheels could not even
twitch. The ramp was softening the compensation exactly where it was needed,
and was tightened to +-4% duty for the next run.
