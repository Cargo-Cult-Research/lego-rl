The first instrumented run, and the one that turned "it shakes"
into numbers. The controller carries the sim-tuned gains
`(10.71, 0.87, 0.43, 0.30)` with the gyro term raw and the reference design's
fixed +-10% coulomb-friction compensation.

The first 1.16 s is calm because a hand is holding it. The moment the wheels
are free to react against the body it detonates: duty pinned to +-100% for 58%
of samples, gyro to +-404 deg/s, pitch buzzing +-10 deg.

Two discontinuities feed a 19 ms-delayed loop here. The rate gain is a pure
differentiator. The friction compensation is a hard 20-point jump at every
zero crossing -- on a body with ~0.0009 kg m^2 of inertia that step alone is
worth ~130 deg/s of gyro within a single sample. It is a bang-bang oscillator
by construction.
