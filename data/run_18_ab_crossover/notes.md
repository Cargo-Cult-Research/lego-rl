Run 16 hinted the learned policy beat the hand-tuned controller — 1.17 deg
against 1.56 — and the notes said plainly that this was not established,
because classical had scored 3.31 and 1.56 on identical configurations at the
same voltage. One number inside 2x run-to-run variance proves nothing.

Urs proposed the fix: stop standing the robot up between conditions. Balance
once and switch the controller underneath, over and over. Everything that
varies between stand-ups — release, floor position, wheel loading, battery —
then hits both conditions equally, and one measurement becomes a paired series.
The first 1.2 s of each segment is discarded because switching mid-flight
causes a transient belonging to neither controller, and the gyro filter state
carries across the switch so the policy is not handed a cold filter.

Mid-run the robot drove into a wall and off a table, so segments were flagged
from the data rather than by memory: a fall, or ending more than 250 deg from
centre, disqualifies a segment, and a cycle is only used when BOTH its segments
survive. That leaves 6 clean pairs of 10.

| cycle | classical | policy | diff | classical travel | policy travel |
| --- | --- | --- | --- | --- | --- |
| 0 | 1.19 | 1.34 | +0.15 | — | 16 |
| 1 | 1.12 | 1.24 | +0.12 | 1 | 41 |
| 2 | 1.75 | 2.84 | +1.09 | 69 | 120 |
| 3 | 3.71 | 4.67 | +0.96 | 54 | 64 |
| 4 | 6.75 | 4.32 | −2.43 | 0 | 28 |
| 9 | 1.44 | 3.13 | +1.69 | — | 127 |

**The policy does not beat the classical controller.** Classical is quieter in
5 of 6 paired cycles (2.66 deg against 2.92 on average). Run 16's result was a
lucky segment, and labelling it "not established" at the time was the right
call. In the two cleanest cycles the gap is only 0.15 and 0.12 deg, so the two
are close to equivalent when nothing is going wrong — the policy loses ground
as conditions degrade.

**The more valuable result is why it hit the wall.** The policy travels 1.9x
further per 6 s segment: median 52 deg against 28. That is the single mismatch
the verifier flagged when the policy's Jacobian was first compared to the
classical gains:

    wheel   learned 0.148   sim-tuned 0.430   ratio 0.35

A Jacobian taken from a neural network in simulation said the position feedback
was three times too weak. Hardware has now answered: it wanders off and drives
into things. The verifier predicted a specific real-world failure from a single
number, which is what the project exists to test.

The two controllers also fail differently in character. The policy never
saturates its duty limit (0% median) while classical rides it 18% of the time.
The policy is gentle and lets position drift; classical is aggressive and holds
station. Both ran at a confirmed 200 Hz throughout.

Caveats. Six pairs is not many, and the run was genuinely disturbed — the
degradation from cycle 2 onward tracks the robot getting further from centre
and into trouble rather than anything about the controllers. The station-keeping
result is the robust one: consistent in direction, predicted in advance, and
large enough to see through the noise.
