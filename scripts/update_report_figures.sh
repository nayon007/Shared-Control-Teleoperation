#!/usr/bin/env bash
# Copy the result figures into the self-contained LaTeX project (run on the host):
#   bash scripts/update_report_figures.sh
set -euo pipefail
cd "$(dirname "$0")/.."
F=docs/phase3_report/figures
mkdir -p "$F"
cp results/phase3/scene_start.png results/phase3/pilot_outcomes.png results/phase3/pilot_traces.png "$F/"
cp results/phase3/pilot_table.tex docs/phase3_report/
cp results/images/step3_drawer_tilted.png "$F/scene_tilted.png"
cp results/images/step3_door.png "$F/scene_door.png"
python3 - <<'PY'
from PIL import Image
F = 'docs/phase3_report/figures/'
for k in ('drawer', 'door'):   # drop the plot title
    im = Image.open('results/phase3/validation/check_%s.png' % k)
    im.crop((0, 42, im.width, im.height)).save(F + 'validation_%s.png' % k)
im = Image.open('results/images/step3_drawer_gui.png')
w, h = im.size
im.crop((int(.30 * w), int(.25 * h), int(.80 * w), int(.75 * h))).save(F + 'gazebo_window.png')
for h, name in ((0, 'baseline_success'), (2, 'baseline_failure')):
    g = Image.open('results/phase3/baseline_h%d.gif' % h)
    g.seek(g.n_frames - 1)
    g.convert('RGB').crop((80, 100, 560, 390)).save(F + name + '.png')
PY
