#!/usr/bin/env bash
# Start a recorded DualSense session from the HOST (normal terminal).
#   bash scripts/drive.sh [assisted|direct] [friction N] [drawer|door] [more launch args]
#   bash scripts/drive.sh stop        # end the session; the recording is saved
# Each run first ends the previous session (and saves its recording).
# Sessions go to results/sessions/<UTC time>_<mode>_<kind>_f<friction>/.
set -euo pipefail
cd "$(dirname "$0")/.."
in_c() { docker exec $1 sce571_dev bash -c "source /opt/ros/humble/setup.bash && source ~/SCE_571_ws/ros2_ws/install/setup.bash && $2"; }
xhost +local:docker > /dev/null 2>&1 || true
docker compose up -d > /dev/null 2>&1
in_c "" "bash ~/SCE_571_ws/scripts/stop_sim.sh" > /dev/null
latest=$(ls -td results/sessions/*/ 2>/dev/null | head -1 || true)
[ -n "$latest" ] && echo "previous session saved: $latest"
[ "${1:-}" = "stop" ] && exit 0
mode=${1:-assisted}; friction=${2:-3.0}; kind=${3:-drawer}; shift $(( $# < 3 ? $# : 3 ))
in_c "-d" "ros2 launch sc_teleop teleop.launch.py input:=dualsense mode:=$mode friction:=$friction kind:=$kind $* > /tmp/teleop.log 2>&1"
echo -n "starting ($mode, friction $friction N, $kind) "
for _ in $(seq 60); do
  sleep 2; echo -n .
  if in_c "" "grep -q READY /tmp/teleop.log"; then echo " READY: drive now (hold R2)."; break; fi
done
echo "live numbers:  docker exec sce571_dev tail -f /tmp/teleop.log"
echo "end session:   bash scripts/drive.sh stop"
