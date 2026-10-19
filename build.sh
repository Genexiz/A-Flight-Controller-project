#!/bin/zsh
# Regenerate schematic + PCB placement from scripts/design.py
set -e
cd "$(dirname "$0")"
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
python3 scripts/gen_schematic.py
$KC sch erc --severity-error --exit-code-violations -o build/erc.rpt MiniDrone_FC.kicad_sch
$KC sch export netlist --format kicadsexpr -o build/MiniDrone_FC.net MiniDrone_FC.kicad_sch
$KPY scripts/gen_pcb.py
