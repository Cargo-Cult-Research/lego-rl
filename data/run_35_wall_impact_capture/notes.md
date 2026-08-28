# Run 35 — first real impact traces (wall collisions at cruise)

`robot/sysid_collision.py`, third field session (the first two false-
triggered and shaped the script: self-calibrated rate trigger, wall-press
backstop, pickup detection — see the script's git history). 12 segments,
4 speed targets x 3 reps ascending, ~2 m runway.

**Operator record (part of the run, the log cannot know this):** every
segment was the hard wall, near dead-on — except seg 11, deliberately
~45 deg glancing. Battery 7514 mV at start.

## Captures: 12/12, zero voids

Every segment triggered and printed its 1.5 s window (0.5 s pre / 1.0 s
post). The self-calibrated triggers landed at 260–390 deg/s — 3–5x the
old fixed 80 deg/s floor, which is why sessions 1–2 with fixed
thresholds false-triggered and quiet-hold arming never armed: cruise
wobble on this floor lives right around any threshold you'd pick blind.
The pickup detector ended every grab cleanly; the wall-press backstop
was never needed.

## Finding 1 — the speed sweep collapsed (cruise is a limit cycle)

Mean wheel speed in the pre-impact window is 640–890 deg/s **regardless
of the 300–750 target**, with duty slamming rail-to-rail (+100/-100) in
a ~9 Hz limit cycle and pitch swinging +-6 deg. The classical law with
GAINS_SIM_TUNED is a station-keeper pressed into speed tracking, and it
cannot do it: the speed term saturates the duty long before the lean is
regulated, and the robot barrels at whatever the limit cycle averages
(~0.27–0.38 m/s). Consequences:

- All 11 head-on impacts landed at roughly ONE speed. Fine for contact
  fitting (that needs impacts, not a sweep), but the "recovery vs speed"
  curve did not materialize.
- Speed control is itself an open problem on this plant — the
  speed-chaser policy is not just replacing recovery, it is replacing a
  cruise controller that does not work. Also why the robot was hard to
  aim: it arrives via limit cycle, not steady cruise.
- Sim check before trusting anything: does the same law in MuJoCo also
  limit-cycle at ~700 deg/s when asked for 300? If not, the friction/
  motor model is off before contacts even enter the picture.

## Finding 2 — the head-on impact signature (what sim contacts must fit)

Canonical example seg 1 (trace rows 98–118): wheels 850 -> 0 deg/s in
~90 ms while pitch whips from +6 deg to -18 deg peaking near -300 deg/s;
duty saturates in reverse and the wheels spin backward past -1000 deg/s
during recovery. peak_rate 533–689 deg/s (raw) across head-on segments,
peak pitch 30–45 deg. `v_at_trig` in the S rows is contaminated —
encoders measure wheel-vs-chassis, and the chassis is rotating at
hundreds of deg/s at the trigger — use the pre-window mean from the
trace instead.

## Finding 3 — glancing is visibly different, and gentler

Seg 11 (45 deg): the LEFT wheel decays ~190 ms before the right
(`analyze.py` divergence detector; head-on segments never diverge — the
small early flags on segs 1/5 are cruise yaw wobble, visible as the
+-100 deg/s wl/wr split during the limit cycle). Peak rate 292 vs
533–689 head-on, peak pitch 16.8 vs 30–45 deg, recovery sigma 8 vs
13–21 — the fastest target, and the easiest recovery of the run. For
the policy: glancing hits are survivable at speed; the one-sided wheel
decay + yaw transient is the observable that distinguishes them.

## Finding 4 — classical recovery baseline: 5/12 falls

Falls on segs 2, 5, 6, 7, 10 — all head-on, all at the (collapsed)
~0.3+ m/s impact speed. With impact speed roughly constant, classical
recovery from a hard head-on hit is ~58% — a coin flip. That is the
number the policy has to beat, at higher commanded speed.

## Next

1. Reproduce the cruise limit cycle in sim with the same law (motor/
   friction check) — before touching contacts.
   **ANSWERED same day, and the answer is no** (`scripts/sim_cruise_check.py`):
   with the robot's actual config (30 ms rate filter) the sim classical
   falls in ~3 s even station-keeping — run 28's lag-fragility anchor,
   reconfirmed. Filter off, it stands but the tracking failure is
   target-DEPENDENT (433 @ 300, ~1000 @ 750), unlike the real robot's
   target-independent 640–890. A suspect sweep (same script's probe)
   points at the pendulum time constant: delay 0 survives, and
   com_height 0.07–0.09 survives WITH the filter — the box model's
   inertia from the measured 5 cm com is likely underestimating I (the
   hub + batteries ride high). Contact fitting is BLOCKED behind fixing
   that: the plan is a free-topple probe (motors off, IMU at 200 Hz),
   replicated exactly in sim, fitting effective inertia + stiction.
2. Fit MuJoCo contact params (solref/solimp/restitution + wall) to the
   head-on signature: wheels-to-zero in ~90 ms, -300 deg/s pitch whip,
   24 deg excursion.
3. Then the speed-chaser env: obstacle angle randomization anchored by
   the seg 11 glancing signature.

## Addendum 2026-08-27 (later): the contacts are fit

`scripts/fit_contacts.py` replayed this run's protocol in the 3D model
(fitted plant, fused pitch) and fit the wall contact to these traces:
timeconst 2.7 ms (MuJoCo's 20 ms default took 210 ms to stop the wheels
where the real wall took 90), dampratio 1.3 (dead, no bounce),
wall_friction 0.75. Head-on collapse time, whip-per-speed, excursion,
and the glancing hit's gentleness all match; the open residual is the
190 ms between wheel impacts on the 45-deg hit (sim yaw-snaps in ~25 ms
-- normal-impulse geometry, not reachable with these knobs). Fitted
values are now the PhysicalParams defaults, randomized around
(data/contacts_fit_best.json).
