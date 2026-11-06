#!/usr/bin/env bash
# Start PX4 SITL with the built-in SIH quadcopter (no external simulator), headless.
# usage: scripts/sitl_sih.sh /path/to/PX4-Autopilot
set -euo pipefail
PX4=${1:-${PX4_DIR:-$HOME/PX4-Autopilot}}
B=$PX4/build/px4_sitl_default
[ -x "$B/bin/px4" ] || { echo "build first: cd $PX4 && make px4_sitl_default"; exit 1; }
cd "$B/rootfs"
rm -f parameters.bson parameters_backup.bson          # start from airframe defaults every time
exec env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim HEADLESS=1 ../bin/px4 -d
