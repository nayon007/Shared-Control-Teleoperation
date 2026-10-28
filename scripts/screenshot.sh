#!/usr/bin/env bash
# Save a PNG of an X window by title (run on the HOST, not in the container):
#   bash scripts/screenshot.sh "Gazebo" results/images/gazebo_drawer.png
set -euo pipefail
TITLE=$1
OUT=$2
export DISPLAY=${DISPLAY:-:1}
# pick the largest matching window (Gazebo/RViz create small helper windows too)
ID=$(xwininfo -root -tree | grep -E "\"[^\"]*$TITLE[^\"]*\"" | \
     awk '{for(i=1;i<=NF;i++) if($i ~ /^[0-9]+x[0-9]+\+/){split($i,a,"[x+]"); print a[1]*a[2], $1}}' | \
     sort -n | tail -1 | awk '{print $2}')
[[ -n "$ID" ]] || { echo "no window matching '$TITLE'" >&2; exit 1; }
TMP=$(mktemp --suffix .xwd)
xwd -id "$ID" -silent -out "$TMP"
python3 "$(dirname "$0")/xwd2png.py" "$TMP" "$OUT"
rm -f "$TMP"
echo "saved $OUT (window $ID)"
