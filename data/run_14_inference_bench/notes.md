13 ms for 104 multiply-accumulates is ~125 us per MAC, on a 100 MHz Cortex-M4F
with a hardware FPU that does a float multiply in one cycle. Four orders of
magnitude off the metal means almost none of that time is arithmetic. Six
implementations of the identical network, timed on the hub, each checked
against the float reference before its timing was believed.

| variant | us/call | of budget |
| --- | --- | --- |
| as exported (generators, exp tanh) | 18125 | 362% |
| unrolled floats | 10296 | 206% |
| unrolled + float tanh LUT | 11642 | 233% |
| fixed point Q10, lists and loops | 2758 | 55% |
| fully unrolled fixed point Q12 | 1241 | 25% |
| **+ interpolated tanh (deployed)** | **1607** | **32%** |

**The float tanh lookup table is slower than calling `exp()`.** That is the
classic optimisation inverted. `exp()` is a single C call — one interpreter
dispatch, then the FPU. The table lookup with interpolation is ten interpreted
operations, each boxing a float. On this platform you count interpreter
dispatches, not FLOPs: transcendentals are cheap and arithmetic is expensive.

**Inlining the weights as literals was worth 2.2x on its own**, more than the
entire float-to-integer conversion. Fixed point with lists and loops runs at
2758 us; fully unrolled with every weight a constant in the bytecode runs at
1241. That deletes 104 bounds-checked list subscripts per pass.

And fixed point does not win because integer math is faster per operation. It
wins because MicroPython small ints are **tagged immediates** — they never
touch the heap — while every float allocates an object that then has to be
collected.

Q12 is the largest scale keeping every intermediate under 2^30. Past that
MicroPython promotes to arbitrary precision and the entire advantage
evaporates, so the generator asserts the headroom (13.3x) at build time rather
than discovering it at 200 Hz on a falling robot. Interpolating the table cut
duty error 25x — 4.44% to 0.18% of scale — for 366 us.

The C and assembly routes are genuinely blocked rather than merely hard.
`pybricksdev` never passes `-march` to mpy-cross, so `@micropython.viper`
fails at cross-compile with "invalid arch" before anything reaches the hub.
Patching that through gets one step further before pybricksdev's vendored
`mpy_tool` rejects the native feature byte (0x1c against 0x00 for plain
bytecode), and Pybricks almost certainly does not build the Thumb emitter into
firmware anyway. Real C would mean building custom Pybricks firmware with a C
module — possible, since Pybricks is open source, but a different project.

At 32% of budget the remaining headroom is not worth chasing for this network.
Where it matters is that it reopens bigger ones: 4-32-32-1 would now fit, which
the genuinely nonlinear swing-up task is likely to want.
