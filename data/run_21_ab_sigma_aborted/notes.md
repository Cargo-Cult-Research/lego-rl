# Run 21 — sigma-instrumented ABBA, two failed starts

2026-08-24 morning. **Retro-filed 2026-08-24** — same contract violation as
run 20, fixed the same day. Two launches of the corrected protocol
(`sigma` column, ABBA ordering, `MAX_DUTY = 100`, duty histograms), neither
of which completed:

- `raw_upload_failed.log` — the upload itself never went through: every BLE
  attempt timed out (`UPLOAD_FAILED_ALL_ATTEMPTS`), zero telemetry.
- `raw_stopped.log` — ran to segment 14 of 20, then stopped by hand
  (`SystemExit`), battery 8088 mV at start.

No verdict is taken from a partial crossover — a truncated ABBA loses its
order balance, which was the whole point of the design. Worth noting only
that the drift signature run 20 diagnosed is plainly visible in the partial
data (`mean_x100` walks monotonically from +444 to −1825 while `sigma_x100`
stays in its band), and the new `clamp_pct` column reads 0 nearly everywhere
at `MAX_DUTY = 100`.

The completed run is run 22.
