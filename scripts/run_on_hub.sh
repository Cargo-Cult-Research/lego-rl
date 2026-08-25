#!/bin/bash
# Retry pybricksdev run over BLE: the scan window is ~10 s and the hub sleeps.
# Usage: run_on_hub.sh <script.py> [max_tries]
#
# Output STREAMS LIVE (PYTHONUNBUFFERED; stderr captured separately for the
# retry decision). An earlier version captured everything and printed it only
# at exit — which silently ate every interactive prompt: one sysid session
# ran its stand-the-robot-up protocol with the robot flat on the table, and
# no crossover session ever showed its color legend. If a program talks to
# the operator, the operator has to hear it DURING the run.
cd /Users/token/code/lego-rl || exit 1
PBD=/Users/token/code/lego-rl/.venv/bin/pybricksdev
[ -x "$PBD" ] || { echo "pybricksdev missing at $PBD"; exit 3; }
SCRIPT="$1"
TRIES="${2:-30}"
ERR=$(mktemp)
trap 'rm -f "$ERR"' EXIT
for i in $(seq 1 "$TRIES"); do
  PYTHONUNBUFFERED=1 "$PBD" run ble "$SCRIPT" 2>"$ERR"
  RC=$?
  if [ $RC -eq 0 ] && ! grep -qE 'TimeoutError|BleakError|disconnected' "$ERR"; then
    exit 0
  fi
  echo "try $i: rc=$RC $(grep -oE 'TimeoutError|BleakError|disconnected' "$ERR" | head -1)" >&2
  sleep 2
done
echo "gave up after $TRIES tries; last stderr:" >&2
cat "$ERR" >&2
exit 1
