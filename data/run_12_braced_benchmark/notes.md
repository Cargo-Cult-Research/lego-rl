Eleven runs had killed six software hypotheses and left a ~10 Hz ring that was
indifferent to every one of them: gain magnitude, duty clamping, the yaw loop,
the wheel-speed term, filtering that term, the friction compensation's slope,
and the gyro filter. An open-loop tap test found no structural resonance, but
it was run holding the robot in the air — tyres unloaded, so the one spring
genuinely inside the control loop was never excited, and the run was recorded
`inconclusive` rather than `ruled out`.

Then the build changed: bracing added, 410 g -> 429 g. Same controller, same
gains, same 10 s window.

| | 410 g unbraced | 429 g braced |
| --- | --- | --- |
| ring amplitude | 1.75 deg | **0.34 deg** (-81%) |
| ring frequency | 10.6 Hz | **11.5 Hz** |
| pitch RMS | 2.24 deg | **1.50 deg** |
| peak | 6.3 deg (2.5 s window) | **3.6 deg** (10 s window) |

The amplitude collapse is the headline, but the **frequency shift is the
proof**. Adding 19 g of mass to an oscillator lowers its frequency, since
omega scales as sqrt(k/m). This one went up, which means the stiffness k grew
faster than the mass m — the definition of bracing a compliant structure. No
control parameter had ever moved that frequency more than a hertz or two, in
either direction, across eleven runs.

So the ring was mechanical compliance between the hub, where the IMU sits, and
the wheels: a control loop closed around a sensor not rigidly attached to the
body it is controlling. That is unfixable in software, which is precisely why
six software hypotheses died one after another, and the residual survived every
one of them.

The peak comparison understates the improvement: 3.6 deg is over a window four
times longer than the 6.3 deg it beats, so it had four times the opportunity to
find a worse excursion.

Method note. The failed test was still worth running — it was wrong about the
answer but right about its own limits, and saying so in writing (`inconclusive`,
with the airborne caveat spelled out) is what made this comparison legible the
moment the hardware changed. A test that had claimed to rule compliance out
would have sent the next week in the wrong direction.
