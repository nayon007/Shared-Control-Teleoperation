#!/usr/bin/env bash
# Friction study: direct vs assisted on the drawer with different joint
# friction between the drawer and its cabinet (run on the HOST, container up,
# no simulation running). Every trial is also video-recorded. Resumable.
#   bash scripts/run_friction_study.sh
set -uo pipefail
cd "$(dirname "$0")/.."
OUT=results/friction
mkdir -p "$OUT/trials"
for f in 1.0 3.0 10.0 20.0 30.0; do
  for h in 0 10 20; do
    for m in direct assisted; do
      name="f${f}_${m}_h${h}"
      [ -f "$OUT/trials/$name.json" ] && continue
      echo "== $name"
      bash scripts/run_teleop_trial.sh kind:=drawer mode:=$m heading:=$h friction:=$f \
        record:=true sessions_dir:=~/SCE_571_ws/$OUT/videos \
        out:=/home/dev/SCE_571_ws/$OUT/trials/$name.json \
        log:=/home/dev/SCE_571_ws/$OUT/trials/$name.csv | cut -c1-160
    done
  done
done
echo "all trials done"
