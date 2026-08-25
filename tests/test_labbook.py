"""The lab book is part of the test surface: a run directory that does not
parse, a run number cited in code with no directory behind it, or a meta.json
plotting a column the telemetry does not have breaks the suite — not silently
the published page. See scripts/check_labbook.py for the history that made
this necessary (runs 20-22)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_labbook_is_consistent():
    proc = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "check_labbook.py")],
        capture_output=True, text=True)
    assert proc.returncode == 0, (
        f"lab book inconsistent:\n{proc.stdout}\n{proc.stderr}")
