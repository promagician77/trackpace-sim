#!/usr/bin/env bash
# Measured Gazebo runs: a calibration lap (no drag term), drag identified from its log,
# then the paced laps with the identified value.
# usage: GZ_PREFIX=... bench/run_gz.sh /path/to/PX4-Autopilot [runs-folder]
set -u
cd "$(dirname "$0")/.."
PX4=$1; OUT=${2:-runs}
PY=${PYTHON:-python3}
LOGDIR=$PX4/build/px4_sitl_default/rootfs/log

start_sim() {
  setsid scripts/sitl_gz.sh "$PX4" > "$1/px4.log" 2>&1 &
  SIM_PGID=$!
  for _ in $(seq 120); do
    grep -q "Startup script returned successfully" "$1/px4.log" && { sleep 8; return 0; }
    sleep 1
  done
  echo "PX4/Gazebo did not start"; return 1
}
stop_sim() { kill -INT -- "-$SIM_PGID" 2>/dev/null; sleep 4; kill -KILL -- "-$SIM_PGID" 2>/dev/null; sleep 1; }

run() {
  local name=$1; shift
  mkdir -p "$OUT/$name"
  echo "== $name: $*"
  start_sim "$OUT/$name" || return 1
  "$PY" -m trackpace.fly config/gazebo_59.toml "$@" --out "$OUT/$name" --ulog-dir "$LOGDIR" > "$OUT/$name/fly.log" 2>&1
  echo "   flight exit $?"
  stop_sim
  "$PY" -m trackpace.analyze "$OUT/$name" > "$OUT/$name/analyze.txt" 2>&1
  echo "   analyze exit $?"
}

if [ ! -f "$OUT/gz_cal/drag.txt" ] || [ -n "${RECAL:-}" ]; then
  run gz_cal --laps 1 --drag 0
  DRAG=$("$PY" -c "import json,sys; from trackpace.identify import identify; print(identify(sys.argv[1], truth=False)['drag_per_s'])" "$OUT/gz_cal/flight.ulg")
  echo "$DRAG" > "$OUT/gz_cal/drag.txt"
fi
DRAG=$(cat "$OUT/gz_cal/drag.txt")
echo "   drag_per_s = $DRAG (identified from the calibration lap's EKF estimate)"
run gz59 --laps 3 --drag "$DRAG"
