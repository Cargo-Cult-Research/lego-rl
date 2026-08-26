"""Firmware-path contract tests for pybricks.experimental.MLP.

The C inference core lives in the pybricks-micropython fork (see
firmware/README.md). These tests guard the two host-checkable links of that
chain: the C kernel agrees with NumPy, and the exported blob agrees with the
policy module it was generated from. The on-hub timing and behaviour are
hardware runs (firmware/hub_bench_mlp.py), not tests.
"""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
PYBRICKS = ROOT.parent / "pybricks-micropython"

needs_checkout = pytest.mark.skipif(
    not (PYBRICKS / "pybricks/experimental/pb_mlp.c").exists(),
    reason="pybricks-micropython checkout with the mlp-module branch not found",
)
needs_cc = pytest.mark.skipif(shutil.which("cc") is None, reason="no C compiler")


@needs_checkout
@needs_cc
def test_kernel_matches_numpy():
    subprocess.run(
        [sys.executable, str(ROOT / "firmware/test_mlp_kernel.py"), str(PYBRICKS)],
        check=True,
    )


def test_export_self_check(tmp_path):
    # The exporter asserts blob-vs-act() agreement internally; failure exits
    # nonzero. Writes to robot/policy_mlp.py, which is generated anyway.
    subprocess.run(
        [sys.executable, str(ROOT / "firmware/export_mlp_blob.py")],
        check=True,
    )
