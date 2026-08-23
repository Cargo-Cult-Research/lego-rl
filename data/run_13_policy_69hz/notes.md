The first time the learned policy met the real robot. Everything checkable
without hardware had been checked first: the exported MicroPython matches
PyTorch to 1.9e-6 over 2000 random states, and the signs were verified against
the exported weights before anything moved (+5 deg of lean gives +0.34 duty,
-5 gives -0.34, both wheel terms carry the same sign as pitch).

It balanced the full 10 seconds — 2.43 deg RMS, 9.5 deg peak — against the
classical controller's 1.50 deg. But the forward pass cost **13.1 ms against a
5 ms budget**, so the loop ran at 69 Hz instead of 200, stacking roughly 14 ms
of extra actuation delay on the 19 ms already measured. A policy trained at
200 Hz staying upright on a third of its design rate is a better robustness
result than the RMS number suggests.

Two things surfaced that the desktop could not have:

`umath` has no `tanh`. Pybricks ships a subset of math and the first upload
died on ImportError. The exporter now emits its own from `exp` with a
saturation clamp, re-verified against PyTorch at 1.9e-6.

The duty clamp fired on 9.8% of steps. In sim this policy never exceeds 35.7%
(mean 12.5, p99 32.5), so the counter existed specifically to report if the
robot found states the sim never showed it. It did — the first quantitative
sign of the sim-to-real gap that runs 15 and 16 would pin down.

Process note against myself: this run's output was piped through `head`, so
only the first 700 ms of telemetry survived. The summary statistics are the
hub's own and are complete; the trace is not.
