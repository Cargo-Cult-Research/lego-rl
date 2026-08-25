# Run 31 — latency split: actuation confirmed, sensing probe voided (hardware)

2026-08-24. `robot/sysid_latency.py` with the new part 3. Raw log: `raw.log`
(`run31.log` as recorded).

- **Part 1** (loop jitter): worst period 5 ms — the 200 Hz loop holds.
- **Part 2** (duty step → encoder motion): **14, 19, 20, 18, 18 ms** —
  reconfirms the 2026-08-22 measurement. Now documented as a bundle:
  command path + electrical rise + stiction/backlash takeup + a ~2 ms
  detection threshold.
- **Part 3** (duty step → gyro response): **VOID.** The stand-it-up prompt
  never reached the operator — `run_on_hub.sh` captured all output and
  printed it only at exit, so the robot lay flat on the table through all
  six trials. The recorded 26–69 ms are reaction kicks damped by the table
  and measure nothing cleanly. (Fixed the same day: the runner now streams
  output live.)

Even tainted, the part-3 numbers hint the sensing chain is not fast — worth
the clean rerun, standing, since (part 2 − part 3) splits stiction from
sensing and ranks the latency levers.

**Rerun when convenient:** `scripts/run_on_hub.sh robot/sysid_latency.py`
— prompts now appear live; stand the robot upright for part 3 and steady it
with two fingertips.
