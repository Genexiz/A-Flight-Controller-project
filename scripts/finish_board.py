"""Post-route touches that don't need re-routing: silkscreen ref text thickness for
JLCPCB (>= 0.15 mm) and 3 fiducials. Run with KiCad's Python after route.sh."""
import os
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
BOARD = os.path.join(ROOT, "MiniDrone_FC.kicad_pcb")
FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
SILK_THICK = 0.15
# board-centre-relative (mm), checked to be clear of copper/courtyards (asymmetric on purpose)
FIDUCIALS = [(-30.75, -26.75), (-6.5, 24.75), (16.75, -16.5)]
CX, CY = 150.0, 100.0

def main():
    b = pcbnew.LoadBoard(BOARD)
    mm = pcbnew.FromMM
    for fp in b.GetFootprints():
        if fp.GetReference().startswith("FID"):
            b.Delete(fp)
    for fp in b.GetFootprints():
        for t in [fp.Reference(), fp.Value()]:
            if t.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) and t.GetTextThickness() < mm(SILK_THICK):
                t.SetTextThickness(mm(SILK_THICK))
    for d in b.GetDrawings():
        if isinstance(d, pcbnew.PCB_TEXT) and d.GetTextThickness() < mm(SILK_THICK):
            d.SetTextThickness(mm(SILK_THICK))
    for i, (x, y) in enumerate(FIDUCIALS, 1):
        fp = pcbnew.FootprintLoad(os.path.join(FPDIR, "Fiducial.pretty"), "Fiducial_1mm_Mask2mm")
        fp.SetReference("FID%d" % i)
        fp.SetFPIDAsString("Fiducial:Fiducial_1mm_Mask2mm")
        fp.Reference().SetVisible(False)
        fp.SetBoardOnly(True)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
        fp.SetPosition(pcbnew.VECTOR2I(mm(CX + x), mm(CY + y)))
        b.Add(fp)
    pcbnew.SaveBoard(BOARD, b)
    import shutil
    shutil.copy(os.path.join(HERE, "project_template.kicad_pro"), os.path.join(ROOT, "MiniDrone_FC.kicad_pro"))
    print("silk thickness >= %.2f mm, fiducials: %d" % (SILK_THICK, len(FIDUCIALS)))

if __name__ == "__main__":
    main()
