"""Dump board geometry (outline, keep-outs, pads) to build/geom.json for router.py.
Run with KiCad's Python."""
import json, os
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
BOARD = os.path.join(ROOT, "MiniDrone_FC.kicad_pcb")
OUT = os.path.join(ROOT, "build", "geom.json")
MM = 1e-6

def poly_pts(spoly):
    out = []
    for i in range(spoly.OutlineCount()):
        o = spoly.Outline(i)
        out.append([(o.CPoint(k).x * MM, o.CPoint(k).y * MM) for k in range(o.PointCount())])
        for h in range(spoly.HoleCount(i)):
            hh = spoly.Hole(i, h)
            out.append([(hh.CPoint(k).x * MM, hh.CPoint(k).y * MM) for k in range(hh.PointCount())])
    return out

def main():
    b = pcbnew.LoadBoard(BOARD)
    outline = pcbnew.SHAPE_POLY_SET()
    b.GetBoardPolygonOutlines(outline, False)
    keepouts = []
    for z in b.Zones():
        if z.GetIsRuleArea() and z.GetDoNotAllowTracks():
            keepouts.append(poly_pts(z.Outline()))
    for fp in b.GetFootprints():
        for z in fp.Zones():
            if z.GetIsRuleArea() and z.GetDoNotAllowTracks():
                keepouts.append(poly_pts(z.Outline()))
    pads = []
    for fp in b.GetFootprints():
        for p in fp.Pads():
            layers = []
            if p.IsOnLayer(pcbnew.F_Cu):
                layers.append(0)
            if p.IsOnLayer(pcbnew.B_Cu):
                layers.append(1)
            if not layers:
                continue
            sp = pcbnew.SHAPE_POLY_SET()
            p.TransformShapeToPolygon(sp, pcbnew.F_Cu if 0 in layers else pcbnew.B_Cu, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
            pos = p.GetPosition()
            bb = p.GetBoundingBox()
            pads.append(dict(ref=fp.GetReference(), num=p.GetNumber(), net=p.GetNetname(),
                             layers=layers, x=pos.x * MM, y=pos.y * MM,
                             minsize=min(p.GetSize(pcbnew.F_Cu).x, p.GetSize(pcbnew.F_Cu).y) * MM,
                             drill=p.GetDrillSize().x * MM if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) else 0,
                             npth=p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH,
                             poly=poly_pts(sp)))
    # NPTH holes / board holes are in the outline already (motor holes); NPTH pads become obstacles
    json.dump(dict(outline=poly_pts(outline), keepouts=keepouts, pads=pads), open(OUT, "w"))
    print("pads:", len(pads), "outline polys:", outline.OutlineCount())

if __name__ == "__main__":
    main()
