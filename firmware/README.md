# firmware/ — custom Pybricks firmware with C neural-network inference

Scaling past the 4-8-8-1 balancer net means leaving MicroPython for
inference: the Q12 fixed-point path (run 14) costs ~15 µs per
multiply-accumulate because every op is an interpreter dispatch, so a
4-32-32-1 swing-up net would take ~18 ms — nine times the 2 ms loop budget.
The same math in C on the hub's Cortex-M4F FPU is ~3 orders of magnitude
faster. Viper/native code is structurally blocked (run 14: pybricksdev never
passes `-march`, its vendored mpy_tool rejects native .mpy, and viper has no
float types anyway), so the path is a custom firmware build.

The module lives in a fork of pybricks-micropython at
`~/code/pybricks-micropython`, branch `mlp-module`, based on the v3.6.1 tag
(v4.0.1 uploads silently fail, see repo CLAUDE.md). It adds
`pybricks.experimental.MLP`:

```python
from pybricks.experimental import MLP
net = MLP((4, 8, 8, 1), WEIGHTS, output_activation="linear")  # or "tanh"
duty = net([pitch, pitch_rate, wheel_angle, wheel_rate])
```

Weights are **data, not firmware**: a little-endian float32 buffer (per
layer: weight matrix row-major, then biases) uploaded with the user program,
so the firmware is flashed once and policies iterate at normal
`pybricksdev run` speed. Hidden layers are tanh; construction validates the
buffer length against the topology; calls allocate nothing but the result
object. Design goal: clean enough to PR upstream (pure-C core
`pybricks/experimental/pb_mlp.c` + thin binding, virtualhub test included in
the fork at `tests/virtualhub/experimental/mlp.py`).

## Files here

| file | what |
|---|---|
| `pybricks-technichub-v3.6.1.zip` | stock firmware, the rollback |
| `pybricks-technichub-v3.6.1-mlp.zip` | custom build with the MLP module |
| `export_mlp_blob.py` | policies/*.py float net → robot/ deployable with weight blob |
| `test_mlp_kernel.py` | host test of the C core against NumPy (random nets, 8 topologies) |
| `mlp_host_driver.c` | C harness that test uses |
| `hub_bench_mlp.py` | on-hub timing benchmark, run when hardware is back |

## Build and flash

```sh
export PATH=~/opt/xpack-arm-none-eabi-gcc-13.2.1-1.1/bin:$PATH   # homebrew arm-gcc lacks newlib
make -C ~/code/pybricks-micropython/bricks/technichub -j8
# flash (hub off, then hold button until it starts blinking):
pybricksdev flash ~/code/pybricks-micropython/bricks/technichub/build/firmware.zip
```

Runtime tests (Linux container; virtualhub needs python3.10 + libffi):

```sh
docker run --rm -v ~/code/pybricks-micropython:/work ubuntu:22.04 bash -c '...'
# see the fork's test-virtualhub.sh; test is tests/virtualhub/experimental/mlp.py
```

## Budgets (technichub, STM32L431: 208K firmware flash, 16K user storage, 64K RAM)

- Stock v3.6.1 firmware: 208,512 bytes text. With MLP module: 210,104 of
  212,992 — the module costs **1,592 bytes**, 2.8K headroom left.
- User storage caps the weight blob: 4-32-32-1 float32 is ~5K, fits with the
  program; trotter-scale (30-64-64-12, ~28K) does not — that robot needs a
  SPIKE-class hub anyway (>4 motor ports, 320K RAM).
