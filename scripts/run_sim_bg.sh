#!/usr/bin/env bash
# Start sim.launch.py in the background inside the container and wait for
# /sim_ready (run on the HOST):  bash scripts/run_sim_bg.sh [launch args]
set -euo pipefail
in_c() { sg docker -c "docker exec $1 sce571_dev bash -c 'source /opt/ros/humble/setup.bash && source ~/SCE_571_ws/ros2_ws/install/setup.bash && $2'"; }
in_c "" "bash ~/SCE_571_ws/scripts/stop_sim.sh" > /dev/null
in_c "-d" "ros2 launch sc_gazebo sim.launch.py $* > /tmp/sim.log 2>&1"
for _ in $(seq 120); do
  sleep 2
  if in_c "" "grep -q \"simulation ready\" /tmp/sim.log"; then echo "sim ready ($*)"; exit 0; fi
  if in_c "" "grep -q \"sim_startup.py-.*process has died\" /tmp/sim.log"; then
    in_c "" "grep -E \"ERROR|start pose\" /tmp/sim.log | cut -c1-200"; exit 1; fi
done
echo "timed out"; exit 1
