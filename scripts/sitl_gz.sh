#!/usr/bin/env bash
# PX4 SITL + Gazebo Harmonic, x500 quadcopter, on the 400 m track world.
# usage: scripts/sitl_gz.sh /path/to/PX4-Autopilot [--gui]
# Gazebo comes from apt (Ubuntu 22.04/24.04: gz-harmonic) or a conda env: export GZ_PREFIX=/path/to/env
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
PX4=${1:-${PX4_DIR:-$HOME/PX4-Autopilot}}
B=$PX4/build/px4_sitl_default
[ -x "$B/bin/px4" ] || { echo "build first: cd $PX4 && make px4_sitl_default"; exit 1; }
if [ -n "${GZ_PREFIX:-}" ]; then export PATH=$GZ_PREFIX/bin:$PATH; fi
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-} GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
# shellcheck disable=SC1091
source "$B/rootfs/gz_env.sh"            # PX4 models, plugins and server config
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HERE/gazebo/worlds"

gz sim -r -s -v 1 "$HERE/gazebo/worlds/track400.sdf" &   # world server; PX4 spawns x500 into it
GZ_PID=$!
GUI_PID=
if [ "${2:-}" = "--gui" ]; then gz sim -g & GUI_PID=$!; fi
trap 'kill -INT $GZ_PID $GUI_PID 2>/dev/null || true' EXIT INT TERM
for _ in $(seq 30); do gz topic -l 2>/dev/null | grep -q "^/world/track400/clock" && break; sleep 1; done

cd "$B/rootfs"
rm -f parameters.bson parameters_backup.bson
PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=track400 HEADLESS=1 ../bin/px4 -d
