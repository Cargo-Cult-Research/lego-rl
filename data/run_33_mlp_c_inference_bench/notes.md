# Run 33 — C inference benchmark on the hub

First hardware run of the custom firmware (fork `mlp-module` @ v3.6.1 +
`pybricks.experimental.MLP`; see firmware/README.md). Flash over BLE took
~60 s. Bench: 200 calls per topology, list inputs, StopWatch around the
loop — the same measurement style as run 14, so the comparison is
apples-to-apples.

| topology | run 14 (Q12 MicroPython) | this run (C module) | of 2 ms budget |
|---|---|---|---|
| 4-8-8-1 | 1607 us | **100 us** | 2% (was 32%) |
| 4-32-32-1 | ~18 ms extrapolated | **365 us** | 18% (was ~9x OVER) |
| 30-64-64-12 | — | MemoryError at construction | — |

Reading the numbers:

- **The floor is call overhead, not arithmetic.** 4-32-32-1 has ~11x the
  MACs of 4-8-8-1 but costs only 3.7x. Subtracting: ~75-90 us of fixed
  per-call machinery (converting the input list, boxing the result float,
  call dispatch) + ~0.25 us per MAC including tanh. Run 14's lesson —
  count interpreter dispatches, not FLOPs — still rules the residual: the
  remaining cost is the Python/C boundary, which is paid once per call
  instead of once per multiply.
- **MemoryError on 6924 params is the answer, not a failure.** 28K of
  float32 weights plus scratch does not fit next to the program in the
  hub's 64K heap. Trotter-scale networks take a SPIKE-class hub, which was
  already implied by the >4-motor-port requirement; now it is measured.
- The predicted "single-digit us" in the original bench docstring was the
  pure-math estimate; it ignored the boundary cost. Corrected in the file.

Next: A/B `robot/policy_mlp.py` (C, float) vs `robot/policy_fast.py`
(Q12) balancing — same policy weights, so any behavioural difference is
quantisation + the 1.5 ms of loop time handed back.
