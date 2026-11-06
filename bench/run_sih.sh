#!/usr/bin/env bash
# Measured SIH runs behind the review site. Each run starts a fresh PX4 (airframe defaults),
# flies, lands, copies its .ulg and is scored from it.
# usage: bench/run_sih.sh /path/to/PX4-Autopilot [runs-folder] [run names...]
set -u
cd "$(dirname "$0")/.."
PX4=$1; OUT=${2:-runs}; shift 2 || true
PY=${PYTHON:-python3}
LOGDIR=$PX4/build/px4_sitl_default/rootfs/log
RUNS=${@:-ff59 nodrag59 pos59 plan mission59 lane2}

start_px4() {
  scripts/sitl_sih.sh "$PX4" > "$1/px4.log" 2>&1 &
  PX4_PID=$!
  for _ in $(seq 60); do
    grep -q "Startup script returned successfully" "$1/px4.log" && { sleep 4; return 0; }
    sleep 1
  done
  echo "PX4 did not start"; return 1
}
stop_px4() { kill -INT "$PX4_PID" 2>/dev/null; wait "$PX4_PID" 2>/dev/null; sleep 2; }

run() {
  local name=$1; shift
  mkdir -p "$OUT/$name"
  echo "== $name: $*"
  start_px4 "$OUT/$name" || return 1
  "$PY" -m "$@" --out "$OUT/$name" --ulog-dir "$LOGDIR" > "$OUT/$name/fly.log" 2>&1
  echo "   flight exit $?"
  stop_px4
  "$PY" -m trackpace.analyze "$OUT/$name" > "$OUT/$name/analyze.txt" 2>&1
  echo "   analyze exit $?"
}

for r in $RUNS; do
  case $r in
    ff59)      run ff59      trackpace.fly config/pace_59.toml ;;
    nodrag59)  run nodrag59  trackpace.fly config/pace_59.toml --laps 3 --drag 0 ;;
    pos59)     run pos59     trackpace.fly config/pace_59.toml --laps 3 --mode pos ;;
    plan)      run plan      trackpace.fly config/pace_plan.toml --drag 0.99 ;;
    lane2)     run lane2     trackpace.fly config/pace_lane2_offset.toml --drag 0.99 ;;
    mission59) run mission59 trackpace.mission config/pace_59.toml --laps 2 ;;
  esac
done
