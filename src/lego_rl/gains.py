"""Mac-side view of robot/gains.py — the control law's single source.

The hub cannot import this package (robot/ code must stay MicroPython-only),
so the source of truth lives on the robot's side and the package reads it by
path. Anything on the Mac that needs the gains — the verifier
(scripts/linearize.py), the pipeline control (scripts/linear_to_net.py), the
lab-book page — imports them from here, never as a pasted literal.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

_GAINS_PY = Path(__file__).resolve().parents[2] / "robot" / "gains.py"


def _load():
    if not _GAINS_PY.exists():
        raise FileNotFoundError(
            f"robot/gains.py not found at {_GAINS_PY} — lego_rl.gains needs "
            "the repo checkout (robot/gains.py is the single source; it is "
            "not shipped inside the installed package)")
    spec = importlib.util.spec_from_file_location("_robot_gains", _GAINS_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_m = _load()
GAINS_SIM_TUNED = tuple(_m.GAINS_SIM_TUNED)
GAINS_REFERENCE = tuple(_m.GAINS_REFERENCE)
K_SYNC = _m.K_SYNC
FRICTION_COMP = _m.FRICTION_COMP
