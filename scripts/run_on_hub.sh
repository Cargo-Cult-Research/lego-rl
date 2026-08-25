#!/bin/bash
# Retry pybricksdev run over BLE: the scan window is ~10 s and the hub sleeps.
# Usage: run_on_hub.sh <script.py> [max_tries]
cd /Users/token/code/lego-rl || exit 1
PBD=/Users/token/code/lego-rl/.venv/bin/pybricksdev
[ -x "$PBD" ] || { echo "pybricksdev missing at $PBD"; exit 3; }
SCRIPT="$1"
TRIES="${2:-30}"
for i in $(seq 1 "$TRIES"); do
  OUT=$("$PBD" run ble "$SCRIPT" 2>&1)
  RC=$?
  if [ $RC -eq 0 ] && ! echo "$OUT" | grep -qE 'TimeoutError|BleakError|disconnected'; then
    echo "$OUT" | sed -e 's/\r.*//' | grep -v '^$'
    exit 0
  fi
  echo "try $i: rc=$RC $(echo "$OUT" | grep -oE 'TimeoutError|BleakError|disconnected' | head -1)" >&2
  sleep 2
done
echo "gave up after $TRIES tries"
exit 1
