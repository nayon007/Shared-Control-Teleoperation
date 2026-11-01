#!/usr/bin/env bash
# Phase 3 pilot experiments (run on the HOST with the container up):
#   bash scripts/run_pilot.sh
# Each trial starts from a full reset (stop + relaunch, headless) and runs the
# direct baseline with a scripted operator. Writes results/phase3/trials/.
set -uo pipefail
cd "$(dirname "$0")/.."
OUT=results/phase3/trials
mkdir -p "$OUT"
HEADINGS=(0 0.25 0.5 0.75 1 2 3 5 10 20)
YAWS=(3.14159 2.74159 3.54159)

trial() {  # yaw heading repetition
  local name="yaw${1}_h${2}_r${3}"
  [[ -f "$OUT/$name.json" ]] && return 0      # resumable
  local t0=$(date +%s.%N)
  bash scripts/run_sim_bg.sh gui:=false ideal_gravity_comp:=true yaw:=$1 > /dev/null || { echo "$name: reset failed"; return 1; }
  local t1=$(date +%s.%N)
  sg docker -c "docker exec sce571_dev bash -c 'source /opt/ros/humble/setup.bash && source ~/SCE_571_ws/ros2_ws/install/setup.bash && timeout 60 ros2 run sc_gazebo baseline_direct.py --heading $2 --out ~/SCE_571_ws/$OUT/$name.json --log ~/SCE_571_ws/$OUT/$name.csv'" > /dev/null
  echo "$name reset $(python3 -c "print('%.1f' % ($t1 - $t0))") s: $(python3 -c "import json;m=json.load(open('$OUT/$name.json'));print(m['status'], 'open %.3f m, peak %.1f N' % (m['final_opening_m'], m['peak_F_perp_N']))")"
}

for yaw in "${YAWS[@]}"; do
  for h in "${HEADINGS[@]}"; do trial "$yaw" "$h" 1; done
done
# repeatability: same trial again from a fresh reset
for h in 0 0.5 0.75 2 3 5; do
  for r in 2 3 4; do trial 3.14159 "$h" "$r"; done
done
sg docker -c "docker exec sce571_dev bash -c 'source /opt/ros/humble/setup.bash; bash ~/SCE_571_ws/scripts/stop_sim.sh'" > /dev/null
echo finished
