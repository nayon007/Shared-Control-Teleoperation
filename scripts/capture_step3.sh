#!/usr/bin/env bash
# Step 3 images and GIFs (run on the HOST with the container up):
#   bash scripts/capture_step3.sh
# For each scene: start sim.launch.py (GUI) and wait for /sim_ready, save a
# scene-camera frame of the welded grasp, then record a GIF while the arm
# opens the mechanism along its path (weld_check.py --demo).
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=${DISPLAY:-:1}
xhost +local:docker > /dev/null
OUT=results/images

in_container() {
  sg docker -c "docker exec $1 sce571_dev bash -c 'source /opt/ros/humble/setup.bash && source ~/SCE_571_ws/ros2_ws/install/setup.bash && $2'"
}

capture() {  # name, launch args
  bash scripts/run_sim_bg.sh $2
  in_container "" "python3 ~/SCE_571_ws/scripts/snapshot.py --keep-running ~/SCE_571_ws/$OUT/step3_$1.png"
  [[ $1 == drawer ]] && bash scripts/screenshot.sh Gazebo "$OUT/step3_drawer_gui.png"
  in_container "-d" "python3 ~/SCE_571_ws/scripts/record_gif.py ~/SCE_571_ws/$OUT/step3_$1_open.gif --seconds 9"
  in_container "" "ros2 run sc_gazebo weld_check.py --demo --seconds 8"
  sleep 2
}

mkdir -p "$OUT"
capture drawer        "kind:=drawer"
capture drawer_tilted "kind:=drawer yaw:=2.7416"
capture door          "kind:=door"
in_container "" "bash ~/SCE_571_ws/scripts/stop_sim.sh" > /dev/null
ls -la $OUT/step3_*
