#!/usr/bin/env bash
# Headless Gazebo smoke test (run inside the container):
#   bash ~/SCE_571_ws/scripts/smoke_gazebo.sh [kind:=door ...]
# Starts gazebo.launch.py without GUI, then reports controllers, hardware
# interfaces and one sample of robot / mechanism joint states.
source /opt/ros/humble/setup.bash
source ~/SCE_571_ws/ros2_ws/install/setup.bash
LOG=/tmp/smoke_gazebo.log
ros2 daemon stop > /dev/null 2>&1   # a stale CLI daemon makes every `ros2` call fail

# Own process group, so the whole launch (gzserver, robot_state_publishers,
# spawners) can be stopped together; leftovers would keep publishing a stale
# robot_description that the next launch would spawn.
setsid ros2 launch sc_description gazebo.launch.py gui:=false "$@" > "$LOG" 2>&1 &
LAUNCH_PGID=$!
trap 'kill -INT -- -$LAUNCH_PGID 2>/dev/null; sleep 5; kill -9 -- -$LAUNCH_PGID 2>/dev/null' EXIT

for _ in $(seq 60); do
  ros2 control list_controllers 2>/dev/null | grep -q "gripper_controller.*active" && break
  sleep 1
done

echo "== controllers";      timeout 10 ros2 control list_controllers
echo "== command interfaces"; timeout 10 ros2 control list_hardware_interfaces | sed -n '/command/,/state/p'
echo "== /joint_states (positions)"
timeout 10 ros2 topic echo --once --field position /joint_states
echo "== /mechanism/joint_states"
timeout 10 ros2 topic echo --once /mechanism/joint_states | grep -A2 -E "name|position"
echo "== errors in launch log"
grep -E "\[ERROR\]|parser error" "$LOG" | cut -c1-200 | head -10 || true
