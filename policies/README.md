# policies/ — float policy weights (Mac-side build artifacts)

Pure-Python float exports of trained/fitted networks. The hub never runs
these (13–18 ms per pass, 3× the loop budget); they are the INPUT to
`scripts/make_fast_policy.py`, which emits the Q12 fixed-point modules in
`robot/` that actually deploy. Tracked so a fresh clone can regenerate and
verify every deployed policy without a GPU or a training run.

| file | produced by | quantised form |
|---|---|---|
| `policy_weights.py` | `scripts/export_policy.py runs/ppo_balance_seed0.zip` | `robot/policy_fast.py` |
| `policy_weights_8m.py` | `scripts/export_policy.py runs/ppo_fixedplant_8m_seed1.zip` | `robot/policy_fast_8m.py` |
| `policy_linear_weights.py` | `scripts/linear_to_net.py` (the classical law as a net — a pipeline control, not a policy) | `robot/policy_linear_fast.py` |

`tests/test_contracts.py` checks the chain end to end: SB3 → float export →
Q12 all agree to < 0.003 duty.
