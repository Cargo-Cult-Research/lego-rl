# Run 38 — stall torque: 0.32 N*m, and the dead zone is the real story

`robot/sysid_stall.py`: 58 mm lever on the port-A axle, kitchen scale,
duty stepped 15-60% with 4 s holds. Encoder creep <= 10 deg at every
step, so the motor was genuinely stalled. Operator ran a few sessions
getting the mechanics right; the last one counts. Battery sagged
7.37 -> 7.03 V across the holds and is corrected per-step in the fit.

## Fit (tau = stall * duty * batt/7.4 - friction, stalled)

    duty%  grams  tau_mNm   pred   resid
      15     50     28.4    11.0   +17.5   <- stiction regime, sits high
      20     45     25.6    26.5    -0.9
      25     56     31.9    42.0   -10.1
      30    101     57.5    57.3    +0.2
      40    113     64.3    87.3   -23.0
      50    210    119.5   115.9    +3.6
      60    275    156.5   143.7   +12.8

**stall_torque = 0.316 N*m per motor** at 7.4 V (0.352 excluding the
15% point); **dead zone 11.5% duty** (14.9% excluding it) -- an
independent confirmation of the 10% motor_friction_duty measured by
sysid_motor in bringup. RMS residual 13 mN*m on a kitchen scale: fine.

## What this refutes, and what it teaches

The joint fit's standing plant ran on stall 0.10. Refuted: the real
motor is 3x stronger. Run 37 had already killed the other leg (4.6 Hz
fusion). So the first standing configuration was a compensation
artifact -- two unphysical values conspiring to fake something real.

The autopsy points somewhere specific: the sim's motor model applies
coulomb friction ONLY when the wheel is moving (`if |omega| > eps`), so
at stall the sim delivers full commanded torque where the real motor
delivers nothing below ~11.5% duty. The robot BALANCES at ~12% mean
duty -- half its commands live inside the dead zone. Scaling stall to
0.10 was the optimizer's only way to shrink small-signal torque; the
honest model is the dead zone itself, which this run measured. It goes
into the motor model as static friction applied at stall, and the refit
runs with stall pinned here and fusion capped at run 37's 0.05 Hz.
