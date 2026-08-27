# Run 34 — Q12 vs C-module crossover (same weights), classical anchor

First closed-loop test of `pybricks.experimental.MLP` (custom firmware,
run 33 benched it open-loop). Three-way ABBA crossover, 7 cycles x 6 s,
`experimental/ab/make_ab_cycle.py --kind1 policy_fast.py --kind2
policy_mlp.py`: GREEN classical direct, YELLOW Q12 fixed-point, CYAN the
same weights through the C module. Battery 7628 mV at start; one
stand-up, one fall (classical's).

## Result: the C module is validated under control

- Zero falls and 200 Hz in all 14 policy segments; duty histograms for
  YELLOW and CYAN are the same shape (cool, 0-30% bins), dmax <= 62 vs
  classical's 76-126 with up to 27% clamping and the run's only fall
  (seg 17). Operator report matches: "violent shaking in green".
- sigma medians: classical ~2.2 deg, Q12 1.17, C 1.28. Policies beat
  classical 6/7 (classical took cycle 4, the segment right after its
  battery-sag reading 6666 mV).
- Q12 vs C: YELLOW ahead 6/7 but by ~0.1 deg per cycle — below the noise
  floor this rig has ever resolved (classical once scored 3.31 vs 1.56 on
  identical configs), AND kind 1 enjoys a positional advantage: the ABBA
  mirror (0,1,2 / 2,1,0) keeps YELLOW in the middle slot of every cycle,
  never first after a switch storm nor last into accumulated drift.
  Equivalence is the expected result for a correct implementation; this
  run does not license "Q12 is quieter".

## Design notes for the next crossover

- Rotate the kind order (latin square), don't mirror it, so every kind
  visits every within-cycle position.
- The generator inlines module announcement banners (`policy_fast.py is a
  MODULE ...` printed at startup); strip prints when inlining.
- Reference walk as always: mean pitch -29 deg and wheel 1985 deg by seg
  16 — common-mode across conditions, which is the point of the design.

## What this closes

The C inference path is now: benched (run 33, 100 us), bit-checked
(host vs NumPy, virtualhub 10/10), and control-validated (this run).
Swing-up on 4-32-32-1 (365 us/call) and the 4-motor trotter can build on
it. Also replicated: policies > classical on this plant (run 27's old-
policy result, now with classical falling).
