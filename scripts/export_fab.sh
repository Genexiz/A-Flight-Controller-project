#!/bin/zsh
# Produce fabrication + documentation outputs.
set -e
cd "$(dirname "$0")/.."
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
mkdir -p fab docs
rm -f fab/*
$KC pcb export gerbers --layers "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts" \
    --subtract-soldermask --use-drill-file-origin -o fab/ MiniDrone_FC.kicad_pcb
$KC pcb export drill --format excellon --excellon-separate-th --generate-map --map-format gerberx2 -o fab/ MiniDrone_FC.kicad_pcb
(cd fab && rm -f ../docs/MiniDrone_FC_gerbers.zip && zip -q ../docs/MiniDrone_FC_gerbers.zip *)
$KC pcb export pos --format csv --units mm --side front -o docs/CPL_top.csv MiniDrone_FC.kicad_pcb
$KC sch export bom --fields "Reference,Value,Footprint,MPN,\${QUANTITY}" --labels "Designator,Value,Footprint,MPN,Qty" \
    --group-by "Value,Footprint,MPN" --exclude-dnp -o docs/BOM.csv MiniDrone_FC.kicad_sch
$KC sch export pdf -o docs/MiniDrone_FC_schematic.pdf MiniDrone_FC.kicad_sch
$KC pcb export pdf --layers "F.Cu,F.Silkscreen,Edge.Cuts" --mode-single -o docs/MiniDrone_FC_top.pdf MiniDrone_FC.kicad_pcb
$KC pcb render --side top --quality high --width 1600 --height 1200 -o docs/render_top.png MiniDrone_FC.kicad_pcb
$KC pcb render --side bottom --quality high --width 1600 --height 1200 -o docs/render_bottom.png MiniDrone_FC.kicad_pcb
$KC pcb render --rotate "-45,0,30" --quality high --width 1600 --height 1200 -o docs/render_3d.png MiniDrone_FC.kicad_pcb
