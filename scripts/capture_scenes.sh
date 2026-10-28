#!/usr/bin/env bash
# Regenerate the README images and GIFs (run on the HOST with the container up):
#   bash scripts/capture_scenes.sh
# For each scene: launch Gazebo (GUI), wait until the arm holds its start pose,
# save a scene-camera frame and a Gazebo window screenshot, then record a GIF
# while a test effort opens the mechanism.
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=${DISPLAY:-:1}
xhost +local:docker > /dev/null

in_container() {
  sg docker -c "docker exec $1 sce571_dev bash -c 'source /opt/ros/humble/setup.bash && source ~/SCE_571_ws/ros2_ws/install/setup.bash && $2'"
}

capture() {  # name, launch args, effort, seconds
  in_container "" "bash ~/SCE_571_ws/scripts/stop_sim.sh" > /dev/null
  in_container "-d" "ros2 launch sc_description gazebo.launch.py $2 > /tmp/gz_$1.log 2>&1"
  sleep 40   # Gazebo start + controllers + go_to_start
  in_container "" "python3 ~/SCE_571_ws/scripts/snapshot.py --keep-running ~/SCE_571_ws/results/images/$1_camera.png"
  bash scripts/screenshot.sh Gazebo "results/images/$1_gui.png"
  in_container "" "python3 ~/SCE_571_ws/scripts/record_gif.py ~/SCE_571_ws/results/images/$1_open.gif --effort $3 --seconds $4"
}

mkdir -p results/images
capture drawer "kind:=drawer" 4.0 4             # 4 N vs 3 N friction  -> ~0.2 m/s
capture door   "kind:=door radius:=0.5" 5.0 5   # 5 Nm vs 3 Nm friction -> ~0.4 rad/s
