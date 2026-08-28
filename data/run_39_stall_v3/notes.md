# Run 39 — stall v3: the dead zone is solid, the slope is not

Hands-free protocol (Urs's design): unwind blip leaves the lever up,
each step drives it down into the scale. Operator record: the 12% steps
never reached the scale and were not logged (correct — that IS the dead
zone); the two consecutive no-shows backed the arm too far and it was
manually repositioned once.

## Solid

- **Drive-side breakaway 12–15% duty.** 12% moved the arm 0 deg in both
  passes; 15% barely traveled (25–29 deg) and pressed 55–65 g. This is
  the number wheel_frictionloss realizes (~0.09 N*m both motors) and it
  brackets run 38's 11.5% from above.
- **60% torque: 216–224 g here, 275 g in run 38** -> with a 13% dead
  zone, stall = 0.29–0.36 N*m across all sessions. Default 0.316 stands;
  DR widened.

## Not solid

Both passes plateau at ~150 g across 26–40% (battery-corrected: the
model predicts +95 g over that span), then rise again at 50–60%. Line
fits therefore disagree wildly across sessions (0.17–0.39 N*m).
Candidates: the lever meets the scale at an angle in this protocol
(vertical-force reading decays as the contact skids), or hub current
limiting in the mid range. The lever+scale method has hit its accuracy
floor — trust the invariants (dead zone, high-duty torque), not the
slopes. A better instrument (spring scale pulling tangentially, or a
current probe) would be the next step IF stall precision ever matters
more than ±15%.
