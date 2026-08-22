# lego-rl

Sim-to-real RL for a LEGO 42124 rebuilt as a two-wheeled inverted pendulum:
Technic Hub + XL angular motor, both wheels rigid on one axle, hub mounted
high, L motor off. Runs Pybricks on the hub; MuJoCo (CPU) + PPO on the Mac.

The point of the project is the **verifier**: get the classical four-gain
controller (Pybricks reference: ~88, 0.35, 0.72, 0.19 duty% per deg / deg/s
on gyro angle, gyro rate, motor angle, motor speed) working on hardware,
train PPO in sim, then linearize the learned policy at the upright
equilibrium and compare its Jacobian to those four gains. Rough agreement
validates the entire pipeline — model, sysID, domain randomization,
training, export — against a known answer.

## Milestones

- [ ] **M0** classical balancer on hardware (`robot/balance_classical.py`)
- [ ] **M1** sim credibility: reference gains balance the sim (`scripts/verify_classical.py`)
- [ ] **M2** sysID pass: replace every GUESS in `src/lego_rl/params.py` (`robot/sysid_*.py`)
- [ ] **M3** PPO balancer in sim (`scripts/train_ppo.py`)
- [ ] **M4** verifier: policy Jacobian ≈ classical gains (`scripts/linearize.py`)
- [ ] **M5** deploy: export MLP, time it on the hub, balance on hardware
      (`scripts/export_policy.py`, `robot/balance_policy.py`)
- [ ] **M6** swing-up from lying flat — the task linear feedback cannot do
      (`scripts/train_ppo.py --task swingup`)

## Quickstart

```sh
uv venv --python 3.12 && uv pip install -e ".[dev]"
.venv/bin/pytest -q
.venv/bin/python scripts/verify_classical.py          # M1
.venv/bin/python scripts/train_ppo.py                 # M3 (~2M steps)
.venv/bin/python scripts/linearize.py runs/ppo_balance_seed0.zip   # M4
.venv/bin/python scripts/export_policy.py runs/ppo_balance_seed0.zip  # M5
.venv/bin/python scripts/view.py [--policy runs/....zip]  # watch it
```

## SysID checklist (feeds `src/lego_rl/params.py`)

| param | how to measure |
|---|---|
| wheel_radius, masses, com_height | calipers, kitchen scale, balance on a straightedge |
| no_load_speed, motor_friction_duty | `robot/sysid_motor.py` (wheels up) |
| stall_torque | lever arm + kitchen scale at duty=100 |
| delay_ctrl_steps | `robot/sysid_latency.py` |
| imu_*_bias / noise | `robot/sysid_imu.py` |
| battery_v range | `hub.battery.voltage()` fresh vs dying |

Update the number **and** its provenance tag in `PROVENANCE`.

## Design notes

- Observation = motor-encoder-faithful: wheel angle is measured relative to
  the chassis, exactly what `motor.angle()` reads.
- Motor model: DC motor with back-EMF (stall torque + no-load speed at
  7.4 V), scaled by battery voltage, minus coulomb friction. Battery scaling
  and the +10% friction duty are the two hand-derived compensations in the
  Pybricks reference — here they are domain randomization parameters.
- Actions pass through a FIFO delay (5–25 ms randomized) to model loop latency.
- Policy head is fixed at 4→8→8→1 tanh (~100 MACs) so it can run in
  MicroPython at 200 Hz; the value net is bigger because it never deploys.
  If the forward pass doesn't fit the 5 ms budget, distilling to the linear
  gains from `linearize.py` is legitimate — near equilibrium the optimal
  policy essentially *is* linear.
