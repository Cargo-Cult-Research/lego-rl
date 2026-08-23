#!/usr/bin/env python3
"""Turn a raw hub log into a lab-book run directory.

    .venv/bin/python scripts/record_run.py RAW.log SLUG \
        --title "..." --question "..." --headline "..." \
        --script robot/sweep_x.py --verdict ruled-out \
        --series pitch_x10:"pitch (deg)":0.1 --series duty:"duty (%)":1 \
        --segment-labels "K=0.15" "K=0" "K=0.05" \
        --notes notes.md

Splits the hub's stdout into the CSV block (-> telemetry.csv) and everything
else (-> meta.hub_output), numbers the run from the existing directories, and
writes meta.json. Notes can come from a file or be added afterwards by hand;
the page renders whatever is in notes.md.

See CLAUDE.md ("Recording a run") for what each field is for.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
VERDICTS = ("guilty", "ruled-out", "progress", "void", "open")


def split_log(text: str) -> tuple[str, list[str], list[str]]:
    """-> (csv_text, hub_chatter, warnings)"""
    lines = text.splitlines()
    hdr = next((i for i, l in enumerate(lines) if l.startswith("t_ms,")), None)
    if hdr is None:
        raise SystemExit("no 't_ms,...' header in the log — did the run dump?")
    header = lines[hdr]
    ncol = header.count(",") + 1
    pat = re.compile(r"^" + ",".join([r"-?\d+"] * ncol) + r"$")
    body = [l.strip() for l in lines[hdr + 1:] if pat.match(l.strip())]
    chatter = [l for l in lines[:hdr] + lines[hdr + 1:]
               if l.strip() and not pat.match(l.strip())
               and not l.startswith(("Searching", "END", "[exited"))]
    warn = []
    if not body:
        warn.append("telemetry.csv has ZERO rows")
    return header + "\n" + "\n".join(body) + "\n", chatter, warn


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("raw", type=Path)
    ap.add_argument("slug", help="short kebab name, e.g. friction_slope")
    ap.add_argument("--title", required=True)
    ap.add_argument("--question", default="")
    ap.add_argument("--headline", default="")
    ap.add_argument("--script", default="")
    ap.add_argument("--verdict", default="open", choices=VERDICTS)
    ap.add_argument("--date", default="2026-08-22")
    ap.add_argument("--series", action="append", default=[],
                    help="col:label:scale, repeatable")
    ap.add_argument("--segment-labels", nargs="*", default=None)
    ap.add_argument("--extra-charts", nargs="*", default=None)
    ap.add_argument("--notes", type=Path, default=None)
    args = ap.parse_args()

    existing = sorted(d for d in DATA.glob("run_*") if d.is_dir())
    n = 1 + max((int(d.name.split("_")[1]) for d in existing), default=0)
    d = DATA / f"run_{n:02d}_{args.slug}"
    if d.exists():
        raise SystemExit(f"{d} already exists")

    csv_text, chatter, warn = split_log(args.raw.read_text())

    series = []
    for spec in args.series:
        col, label, scale = spec.rsplit(":", 2)
        series.append([col, label, float(scale)])
    if not series:
        series = [["pitch_x10", "pitch (deg)", 0.1], ["duty", "duty (%)", 1]]

    meta = {
        "n": n, "title": args.title, "date": args.date,
        "script": args.script, "question": args.question,
        "verdict": args.verdict, "headline": args.headline,
        "series": series, "hub_output": chatter,
    }
    if args.segment_labels:
        meta["segment_labels"] = args.segment_labels
    if args.extra_charts:
        meta["extra_charts"] = args.extra_charts

    d.mkdir(parents=True)
    (d / "telemetry.csv").write_text(csv_text)
    (d / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    (d / "notes.md").write_text(
        args.notes.read_text() if args.notes else
        "TODO: what was tested, what the data showed, what it means.\n")

    rows = csv_text.count("\n") - 1
    print(f"wrote {d.name}: {rows} rows")
    for w in warn:
        print(f"  WARNING: {w}", file=sys.stderr)
    if not args.notes:
        print(f"  now write {d / 'notes.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
