#!/usr/bin/env bash
# One scripted shared-control trial in a fresh simulation (run on the HOST).
#   bash scripts/run_teleop_trial.sh mode:=direct heading:=10 [kind:=door] [out:=/path] [log:=/path]
# Prints the metrics line and leaves the simulation stopped.
set -uo pipefail
in_c() { sg docker -c "docker exec $1 sce571_dev bash -c 'source /opt/ros/humble/setup.bash && source ~/SCE_571_ws/ros2_ws/install/setup.bash && $2'"; }
in_c "" "bash ~/SCE_571_ws/scripts/stop_sim.sh" > /dev/null
in_c "-d" "ros2 launch sc_teleop teleop.launch.py gui:=false input:=script $* > /tmp/teleop.log 2>&1"
for _ in $(seq 90); do
  sleep 2
  line=$(in_c "" "grep -h -o \"{\\\"opening_time_s.*\" /tmp/teleop.log" 2>/dev/null)
  if [ -n "$line" ]; then echo "$line"; break; fi
  if in_c "" "grep -q \"process has died\" /tmp/teleop.log"; then
    in_c "" "grep -E \"ERROR|died\" /tmp/teleop.log | cut -c1-200"; break; fi
done
in_c "" "bash ~/SCE_571_ws/scripts/stop_sim.sh" > /dev/null
