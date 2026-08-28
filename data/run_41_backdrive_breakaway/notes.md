# Run 41 — backdrive breakaway, by hand and scale

No script. Urs's protocol: press the lever tip onto the kitchen scale,
ramping by hand; the arm breaks free at some force and the scale's
reading at that moment is the measurement. Five reps: 120, 130, 140,
127, 135 g at 58 mm.

**Backdrive breakaway = 0.074 ± 0.004 N·m per motor.**

## What it settles

- **The asymmetry is real but modest: ~1.7×** over the drive side
  (0.038–0.047 N·m from the 12–15% dead zone × 0.316 stall). Despite
  the ~50:1 ratio, so the friction is mostly in the output-side gear
  stages, not the rotor — a rotor-dominated friction would have
  reflected enormously.
- **Run 36's creep is explained.** At 70° of topple the gravity torque
  per gearbox is ~0.089 N·m — just past this breakaway — which is why
  the encoders logged 1–2° of creep near the end of each topple and no
  more.
- **The sim's symmetric frictionloss is bounded, not broken:** at most
  ~2× wrong depending on direction. DR range widened to 0.06–0.15
  (total, both motors) to span drive-side to backdrive-side; a custom
  asymmetric friction callback is not worth building at this error
  size.

## Method note for the lab book

Two probe scripts were drafted for this number and both were wrong:
the first fired its encoder trigger inside the free play (run 40 made
that inevitable), the second assumed hand-applied force could be
frozen on an LED cue. The kitchen scale was the instrument all along —
it records the force at slip by itself. Instruments that integrate the
operator beat instruments that fight the operator.
