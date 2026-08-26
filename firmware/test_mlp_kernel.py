#!/usr/bin/env python3
"""Test the pb_mlp.c inference core against a NumPy reference.

Compiles firmware/mlp_host_driver.c against the pybricks-micropython
checkout with the host C compiler, then feeds it randomly generated networks
and inputs and compares every output with a NumPy forward pass.

Usage: python firmware/test_mlp_kernel.py [path-to-pybricks-micropython]
"""

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PYBRICKS = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parent.parent / "pybricks-micropython"

TOPOLOGIES = [
    # (sizes, tanh_output) — the deployed balancer head, the planned swing-up
    # net, a trotter-scale net, and edge cases: single layer, width-1 hidden.
    ((4, 8, 8, 1), True),
    ((4, 32, 32, 1), True),
    ((30, 64, 64, 12), True),
    ((30, 64, 64, 12), False),
    ((3, 5), False),
    ((3, 5), True),
    ((2, 1, 2), True),
    ((7, 16, 3), False),
]
TESTS_PER_TOPOLOGY = 20


def reference_forward(sizes, params, tanh_output, x):
    """NumPy forward pass, float32 throughout, matching pb_mlp.h layout."""
    x = x.astype(np.float32)
    offset = 0
    for layer in range(len(sizes) - 1):
        n_in, n_out = sizes[layer], sizes[layer + 1]
        w = params[offset:offset + n_out * n_in].reshape(n_out, n_in)
        offset += n_out * n_in
        b = params[offset:offset + n_out]
        offset += n_out
        x = (w @ x + b).astype(np.float32)
        if layer < len(sizes) - 2 or tanh_output:
            x = np.tanh(x).astype(np.float32)
    assert offset == len(params)
    return x


def main():
    rng = np.random.default_rng(42)

    with tempfile.TemporaryDirectory() as tmp:
        driver = Path(tmp) / "mlp_host_driver"
        subprocess.run(
            ["cc", "-O2", "-Wall", "-Werror", "-I", str(PYBRICKS),
             str(HERE / "mlp_host_driver.c"),
             str(PYBRICKS / "pybricks/experimental/pb_mlp.c"),
             "-lm", "-o", str(driver)],
            check=True,
        )

        worst = 0.0
        for sizes, tanh_output in TOPOLOGIES:
            num_params = sum((sizes[i] + 1) * sizes[i + 1] for i in range(len(sizes) - 1))
            # pb_mlp layout: W then b per layer. He-ish scale keeps
            # activations in a realistic range.
            params = []
            for i in range(len(sizes) - 1):
                w = rng.standard_normal((sizes[i + 1], sizes[i])) / np.sqrt(sizes[i])
                b = rng.standard_normal(sizes[i + 1]) * 0.1
                params.extend([w.ravel(), b])
            params = np.concatenate(params).astype(np.float32)
            assert len(params) == num_params

            inputs = rng.standard_normal((TESTS_PER_TOPOLOGY, sizes[0])).astype(np.float32)

            feed = [str(len(sizes)), *map(str, sizes), str(int(tanh_output))]
            feed += [repr(float(p)) for p in params]
            feed += [str(TESTS_PER_TOPOLOGY)]
            feed += [repr(float(v)) for v in inputs.ravel()]

            result = subprocess.run(
                [str(driver)], input=" ".join(feed), capture_output=True,
                text=True, check=True,
            )
            got = np.array([[float(v) for v in line.split()]
                            for line in result.stdout.strip().splitlines()])

            want = np.stack([reference_forward(sizes, params, tanh_output, x) for x in inputs])
            err = np.max(np.abs(got - want) / (np.abs(want) + 1e-3))
            worst = max(worst, err)
            status = "ok" if err < 1e-4 else "FAIL"
            print(f"{status}  {sizes} tanh_output={tanh_output}  max rel err {err:.2e}")
            if err >= 1e-4:
                sys.exit(1)

        print(f"all topologies pass, worst relative error {worst:.2e}")


if __name__ == "__main__":
    main()
