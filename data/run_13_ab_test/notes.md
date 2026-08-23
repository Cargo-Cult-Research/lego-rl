Making inference 11x faster took the learned policy from 69 Hz to its design
rate of 200 Hz — and made the robot worse, from 2.43 deg RMS surviving 10 s to
8.06 deg and a fall at 5.5 s. Two explanations were live and that run could not
separate them: the ~11 Hz structural resonance, or simply a flatter battery
(7918 mV against 8340-8472 for the classical baseline, with the duty clamp
firing on 22% of steps).

Four segments, one upload, one battery, auto-rearming between them so a fall
could not cost the later controllers.

| segment | battery | RMS | peak | clamp | lasted |
| --- | --- | --- | --- | --- | --- |
| classical | 7877 mV | 3.31 deg | 7.5 | 7% | 10 s |
| policy, raw gyro | 7874 mV | 7.35 deg | 43.5 | 47% | **fell at 4.0 s** |
| policy, 30 ms filter | 7872 mV | **1.17 deg** | **3.7** | **0%** | 10 s |
| classical again | 7867 mV | 1.56 deg | 4.4 | 15% | 10 s |

Segments 2 and 3 are the experiment. Identical policy, identical 200 Hz,
batteries two millivolts apart; the only difference is a 30 ms low-pass on the
gyro. Raw clamps nearly half its steps and falls. Filtered is 6.3x quieter,
never clamps once, and stays up. **The resonance is confirmed**, and the
battery is exonerated: drift across the entire session was 10 mV.

Why it matters beyond this robot: the policy was trained in a simulator with
no structural resonance in it, so nothing ever taught it not to chase an
11 Hz mode. Running at 69 Hz had been hiding that by accident — the slow loop
was acting as a low-pass filter nobody designed. Making the code faster did
not introduce the fault; it removed the accident that was concealing it.

Two honest caveats.

The prediction registered before this run was that policy-filtered would land
*between* raw and classical, still short of the hand-tuned controller because
it is being fed an input distribution it never saw in training. That was wrong
in the robot's favour: 1.17 deg is the lowest pitch RMS this robot has
produced, better than either classical segment and better than the 1.50 deg
braced benchmark.

But "the learned policy beats the classical controller" is NOT established
here. Classical scored 3.31 and 1.56 on identical configurations at the same
voltage — 2x run-to-run variance, presumably from release and floor position.
1.17 sits inside that spread. What sits far outside it is raw versus filtered.
Establishing the policy-versus-classical claim needs repeats on a charged
battery, not one good number.
