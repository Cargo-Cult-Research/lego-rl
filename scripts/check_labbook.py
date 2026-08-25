#!/usr/bin/env python3
"""Lab-book lint: fail when the run record and the code disagree.

Exists because the contract ("every hardware run gets a directory, no
exceptions") was broken three ways before anything noticed: run 20 was cited
in four source files but had no directory, run 21 left no trace, and run 22
sat invisible on the published page for lack of a meta.json. Each check below
corresponds to a way that actually happened.

    .venv/bin/python scripts/check_labbook.py

Exit 0 clean, 1 on any error. Warnings print but do not fail. The test suite
runs this (tests/test_labbook.py), so `pytest -q` breaks when the lab book
does.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_page  # noqa: E402  (shares the run-directory contract)

ROOT = Path(__file__).resolve().parent.parent

# Where prose and code may cite runs. Generated policy files excluded.
CITATION_GLOBS = ("robot/*.py", "scripts/*.py", "experimental/**/*.py",
                  "src/lego_rl/*.py", "tests/*.py", "README.md", "CLAUDE.md", "docs/*.md",
                  "data/*/notes.md")
CITE = re.compile(r"\bruns?[ _](\d{1,2}(?:\s*,\s*\d{1,2})*(?:\s*(?:and|&)\s*\d{1,2})?)\b",
                  re.IGNORECASE)


def check() -> int:
    errors: list[str] = []
    warnings: list[str] = []

    # 1. Every run directory is well-formed (same validation the page build
    #    enforces: meta/notes present, verdict known, integer telemetry,
    #    no duplicate or gapped numbering).
    try:
        runs = build_page.load_runs()
    except SystemExit:
        print("check_labbook: run directories failed build_page validation "
              "(errors above)", file=sys.stderr)
        return 1

    have = {r["meta"]["n"] for r in runs}

    # 2. Every run number cited anywhere resolves to a directory. This is the
    #    check that would have caught balance_classical.py citing run 20 (and,
    #    an iteration earlier, a nonexistent 'run 5' sweep).
    for pattern in CITATION_GLOBS:
        for f in sorted(ROOT.glob(pattern)):
            rel = f.relative_to(ROOT)
            for m in CITE.finditer(f.read_text(errors="replace")):
                for num in re.findall(r"\d{1,2}", m.group(1)):
                    n = int(num)
                    if n and n not in have:
                        errors.append(f"{rel}: cites run {n}, which has no "
                                      f"data/run_{n:02d}_* directory")

    # 3. Plotted series must exist in the telemetry they plot.
    for r in runs:
        for spec in r["meta"].get("series", []):
            col = spec[0]
            if r["cols"] and col not in r["cols"]:
                errors.append(f"{r['dir']}: meta.json plots column {col!r}, "
                              f"telemetry has {r['cols']}")
        if r["cols"] and "t_ms" in r["cols"] and len(r["rows"]) < 40:
            warnings.append(f"{r['dir']}: only {len(r['rows'])} telemetry rows "
                            "— truncated capture (runs 13/15 lost theirs to a "
                            "`head` pipe) or a per-segment table wearing a "
                            "t_ms header (its charts' time axis is fiction)")

    for w in warnings:
        print(f"  warning: {w}")
    for e in sorted(set(errors)):
        print(f"  ERROR: {e}", file=sys.stderr)
    print(f"check_labbook: {len(runs)} runs, {len(set(errors))} errors, "
          f"{len(warnings)} warnings")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(check())
