#!/bin/zsh
# Auto-route the placed board with scripts/router.py, then DRC.
set -e
cd "$(dirname "$0")/.."
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KPY scripts/export_geom.py
python3 scripts/router.py --iters 8 "$@"
$KPY scripts/apply_routes.py
$KPY scripts/finish_board.py
$KC pcb drc --schematic-parity --refill-zones --save-board -o build/drc.rpt MiniDrone_FC.kicad_pcb
