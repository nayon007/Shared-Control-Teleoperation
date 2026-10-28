#!/usr/bin/env bash
# Stop every simulation process in the container (run inside the container).
# Leftover robot_state_publishers would keep publishing a stale
# robot_description that the next launch spawns.
pkill -INT -f 'ros2 launch' 2>/dev/null; sleep 3
pkill -9 -f 'gzserver|gzclient|robot_state_publisher|spawner|spawn_entity|ros2 launch' 2>/dev/null
ros2 daemon stop > /dev/null 2>&1
echo "simulation stopped"
