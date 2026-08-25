# lego-rl

On-device RL for LEGO: a Technic 42124 rebuilt as a two-wheeled inverted
pendulum (hub + both L motors, one direct-driving each wheel, hub mounted
low). MuJoCo + PPO on a Mac; the trained policy runs at 200 Hz *on the LEGO
Technic Hub itself* (Pybricks/MicroPython, quantised to fixed point).

The project's spine is a **verifier**: a published four-gain classical
controller balances this class of robot, so the learned policy is linearised
at the upright equilibrium and its Jacobian compared to those gains
(`scripts/linearize.py`). Near the equilibrium the optimal policy *is*
approximately linear — so rough agreement validates the whole pipeline
(model, sysID, domain randomization, training, export) against a known
answer, and every mismatch so far has been a real bug with a name.

## The state (what the network sees)

One 4-vector, identical in sim (`src/lego_rl/env.py`, the sign-convention
source of truth) and on the hub (`robot/balance_policy.py`):

| # | signal | unit | measured by |
|---|---|---|---|
| 0 | `pitch` — body lean, + = toward the front face | rad | hub IMU, integrated gyro |
| 1 | `pitch_rate` — lean rate, low-passed at 30 ms | rad/s | hub gyro |
| 2 | `wheel_angle` — mean of both encoders, **relative to the chassis** | rad | motor encoders |
| 3 | `wheel_speed` — mean encoder speed | rad/s | motor encoders |

The network input is `OBS_SCALE * state` (`OBS_SCALE = [3, 0.3, 0.03,
0.03]`, folded into the exported weights); the output is one number, wheel
duty in [-1, 1], the same for both motors. The policy head is fixed at
4→8→8→1 tanh (~100 MACs) so it fits the hub's 5 ms loop budget.

## Repo map

| where | what |
|---|---|
| `src/lego_rl/` | the sim: env, procedural MuJoCo model, measured params **with provenance** (`params.py` — every number tagged MEASURED/DATASHEET/GUESS/INFERRED), classical controller |
| `robot/` | everything that runs on the hub — **see `robot/README.md` for the file map and which policy module is current** |
| `policies/` | float weight exports (Mac-side intermediates; `robot/` holds the Q12 deployables) |
| `scripts/` | train, verify, export, quantise, render videos, build the lab book |
| `data/run_NN_*/` | the lab book: one directory per run — hardware and sim, including nulls and retracted results. Renders to a web page; `scripts/check_labbook.py` keeps code and record consistent (runs in the test suite) |
| `experimental/` | one-off per-experiment code, quarantined; records, not tooling |
| `docs/findings.md` | the long-form story of what the runs found |
| `tests/` | 25 tests incl. the export round-trip (SB3 → float → Q12 < 0.003 duty) and the sim's named-bug regressions |

## Quickstart

```sh
uv venv --python 3.12 && uv pip install -e ".[dev]"
.venv/bin/pytest -q
.venv/bin/python scripts/verify_classical.py          # sim vs both gain sets
.venv/bin/python scripts/train_ppo.py                 # PPO (~2M steps)
.venv/bin/python scripts/linearize.py runs/<model>.zip     # THE verifier
.venv/bin/python scripts/export_policy.py runs/<model>.zip # float export
.venv/bin/python scripts/make_fast_policy.py          # -> robot/ Q12 module
.venv/bin/python scripts/view.py [--policy runs/<model>.zip]   # watch live
.venv/bin/python scripts/render_rollout.py out.mp4    # film it
scripts/run_on_hub.sh robot/balance_policy.py         # deploy (BLE retry loop)
```

## Milestones

- [x] **M0** classical balancer on hardware — stands indefinitely
- [x] **M1** sim credibility — the sim rejected the published gains before the
      hardware did, and was right (fell in 0.755 s, as predicted)
- [x] **M2** sysID pass — most GUESSes measured; `unmeasured()` prints the rest
- [x] **M3** PPO balances in sim under domain randomization
- [x] **M4** verifier: policy Jacobian ≈ classical gains (signs exact, shape
      within a few %, position gain flagged weak — later confirmed on hardware)
- [x] **M5** deployed at a full 200 Hz after an 11× MicroPython inference
      speedup (float 18 ms → Q12 1.6 ms per pass)
- [ ] **M6** swing-up from lying flat — the task linear feedback cannot do

## Where it stands (2026-08-24)

The robot balances at **σ ≈ 1.0–1.4°** about the segment mean. Under clean
instrumentation (drift-immune σ, no duty clamp) the learned policy is
modestly **quieter than the classical law at half the duty effort** (run 26,
n=6 cycles — a lead, not a theorem). The plant sits near two instability
cliffs — limit-cycle ignition at ~+3–6% loop gain, and a latency cliff at
20–25 ms — which is where the current work lives. A policy trained on the
fixed plant (gearbox deadband modelled) doubles the position gain and cuts
sim drift 10×; its hardware test is underway. Full story: `docs/findings.md`;
run-by-run: the lab book.

## Hardware gotchas

- **Pin firmware to Pybricks 3.6.1.** With v4.0.1 every released `pybricksdev`
  uploads to 100% and the program silently never starts.
- macOS: the hub is a BLE peripheral — it never appears in the Bluetooth
  menu. Blinking blue = advertising, slow pulse = connected idle.
- `pybricksdev`'s scan window is ~10 s and the hub sleeps when idle:
  `scripts/run_on_hub.sh` is the retry loop.
- Pressing the hub button re-runs the **last downloaded** program.
- Every sign in the chain was verified empirically with LED-feedback probes
  (`robot/sysid_directions.py`, `robot/sysid_signs.py`), not reasoned about.
  The one that bit: raw `+Axis.Y` reads a toward-the-face tilt as negative,
  hence `PITCH_AXIS = -Axis.Y` in `robot/hubconfig.py`.

## SysID

Every parameter in `src/lego_rl/params.py` carries its provenance and
measurement note in the field's own metadata; an untagged field fails at
import, and the test suite checks every GUESS is either domain-randomized or
explicitly exempted. `scripts/verify_classical.py` prints the outstanding
GUESS list before every credibility run; the `robot/sysid_*.py` docstrings
say which parameter each tool measures and how.
