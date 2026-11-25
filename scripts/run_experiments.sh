#!/usr/bin/env bash
# Step 7: scripted direct vs assisted trials with the impedance controller
# (run on the HOST, container up, no simulation running). Resumable: trials
# with a JSON in results/step7/trials are skipped.
#   bash scripts/run_experiments.sh
set -uo pipefail
cd "$(dirname "$0")/.."
OUT=results/step7/trials
mkdir -p "$OUT"
run() {   # name, launch args
  local name=$1; shift
  [ -f "$OUT/$name.json" ] && return
  echo "== $name"
  bash scripts/run_teleop_trial.sh "$@" out:=/home/dev/SCE_571_ws/$OUT/$name.json \
       log:=/home/dev/SCE_571_ws/$OUT/$name.csv | cut -c1-160
}
for rep in 1 2; do
  for h in 0 5 10 15 20 30; do
    for m in direct assisted; do
      run "drawer_yaw3.14159_${m}_h${h}_r${rep}" kind:=drawer mode:=$m heading:=$h
    done
  done
done
for yaw in 2.74159 3.54159; do
  for h in 0 10 20; do
    for m in direct assisted; do
      run "drawer_yaw${yaw}_${m}_h${h}_r1" kind:=drawer yaw:=$yaw mode:=$m heading:=$h
    done
  done
done
for h in 0 10 20; do
  for m in direct assisted; do
    run "door_yaw3.14159_${m}_h${h}_r1" kind:=door mode:=$m heading:=$h
  done
done
for h in 10 20; do   # paper baseline: assisted without the off-axis leak
  run "drawer_yaw3.14159_assistednoleak_h${h}_r1" kind:=drawer mode:=assisted heading:=$h perp_leak:=0.0
  run "door_yaw3.14159_assistednoleak_h${h}_r1" kind:=door mode:=assisted heading:=$h perp_leak:=0.0
done
echo "all trials done"
