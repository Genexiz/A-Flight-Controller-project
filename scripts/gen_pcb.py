"""Generate MiniDrone_FC.kicad_pcb: X-frame outline, footprint placement, nets.

Must run with KiCad's bundled Python (has the pcbnew module):
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 gen_pcb.py
Needs ../build/MiniDrone_FC.net (kicad-cli sch export netlist) - see build.sh.
"""
import math, os, sys, uuid
import pcbnew
sys.path.insert(0, os.path.dirname(__file__))
from sexp import parse, find, first
import design

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
OUT = os.path.join(ROOT, "MiniDrone_FC.kicad_pcb")
NETFILE = os.path.join(ROOT, "build", "MiniDrone_FC.net")
FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"

# ---------------- frame geometry (mm, origin = board centre, +y = rear) -------------
CX, CY = 150.0, 100.0          # where the board centre sits on the page
MOTOR_DIAG = 90.0              # motor-to-motor diagonal distance
MOTOR_HOLE_D = 7.0             # 7.0 for 716 motors, 8.5 for 820 motors
ARM_W = 9.0                    # arm width
MOTOR_RING = 3.0               # FR4 ring around the motor hole
BODY = (-18.0, -20.0, 18.0, 27.0)   # x0, y0, x1, y1
BODY_R = 3.0
ARM_LEN = MOTOR_DIAG / 2.0

# arm i: direction (screen coords). 1=front-right, 2=rear-right, 3=rear-left, 4=front-left
ARMS = {1: (1, -1), 2: (1, 1), 3: (-1, 1), 4: (-1, -1)}

def mm(v):
    return pcbnew.FromMM(v)

def V(x, y):
    return pcbnew.VECTOR2I(mm(CX + x), mm(CY + y))

def arm_xy(i, s, t):
    dx, dy = ARMS[i]
    n = math.hypot(dx, dy)
    ux, uy = dx / n, dy / n
    vx, vy = -uy, ux
    return ux * s + vx * t, uy * s + vy * t

def arm_angle(i):
    dx, dy = ARMS[i]
    return math.degrees(math.atan2(-dy, dx))

# ---------------- placement: ref -> (x, y, rot_deg) ----------------
P = {
    # MCU: antenna overhangs the front edge (module y=-6.75 is the antenna boundary)
    "U1": (0.0, BODY[1] + 6.75, 0),
    "C7": (-12.6, -17.4, 90), "C6": (-12.6, -13.5, 90),
    "R1": (-12.6, -9.6, 90), "C8": (-12.6, -5.7, 90),
    "R17": (12.6, -17.4, 90), "D6": (12.6, -13.5, 90),
    "R18": (12.6, -9.6, 90), "D7": (12.6, -5.7, 90),
    "R2": (12.6, -1.8, 90),
    # IMU at the centre of gravity
    "U3": (0.6, 4.6, 0),
    # IMU caps next to the pins they serve (VLOGIC/REGOUT below, VDD/CPOUT right)
    "C11": (-2.6, 9.85, 0), "C12": (1.4, 9.95, 0),
    "C10": (5.4, 6.2, 270), "C13": (5.4, 2.3, 270),
    # rear corners: buttons + battery divider
    "SW1": (-11.2, 20.0, 0), "SW2": (11.2, 20.0, 0),
    "R15": (-14.2, 23.4, 0), "R16": (-9.8, 23.4, 0), "C15": (-9.8, 25.5, 0),
    "R5": (8.6, 3.0, 90), "R6": (8.6, 6.9, 90),
    "J3": (15.6, 4.6, 0),
    # rear band (y ~13): USB side on the left, battery side on the right
    "J2": (0, 5.6, 270),           # USB-C on the left edge (x snapped to the edge)
    "J1": (0.0, 0, 0),             # battery plug centre-rear (y snapped to the edge)
}
# rear band (y ~13), packed left->right at run time: USB side left, battery side right
BAND_Y = 13.4
BAND = [("C9", 90), ("R3", 90), ("R4", 90), ("R20", 90), ("D1", 90), ("Q5", 90),
        ("C4", 90), ("U2", 90), ("C5", 90), ("C2", 90), ("C3", 90), ("C1", 90)]
PLACE_BY_ORIGIN = {"U1"}

# motor drivers sit on the arms
for i in range(1, 5):
    a = arm_angle(i)
    def at(s, t, rot=0):
        x, y = arm_xy(i, s, t)
        return (x, y, a + rot)
    P["Q%d" % i] = at(26.9, 1.7)
    P["R%d" % (6 + i)] = at(26.5, -2.3)
    P["R%d" % (10 + i)] = at(30.5, -2.3)
    P["D%d" % (1 + i)] = at(31.8, 1.5)
    P["M%d" % i] = at(37.0, 0, 90)

def symbol_fields(c):
    from kicadlib import get_symbol
    out = {}
    for p in find(get_symbol(c["lib"]), "property"):
        if p[1] not in ("Reference", "Value", "Footprint"):
            out[p[1]] = p[2]
    if c.get("mpn"):
        out["MPN"] = c["mpn"]
    return out

def load_netlist():
    t = parse(open(NETFILE).read())
    pinnet = {}
    for n in find(first(t, "nets"), "net"):
        name = first(n, "name")[1]
        for nd in find(n, "node"):
            pinnet[(first(nd, "ref")[1], first(nd, "pin")[1])] = name
    return pinnet

def outline(board):
    poly = pcbnew.SHAPE_POLY_SET()
    # body (rounded rectangle approximated via polygon with arcs)
    x0, y0, x1, y1 = BODY
    body = pcbnew.SHAPE_POLY_SET()
    body.NewOutline()
    pts = []
    r = BODY_R
    for (cx, cy, a0) in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for k in range(0, 10):
            ang = math.radians(a0 + k * 10)
            pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
    for x, y in pts:
        body.Append(mm(CX + x), mm(CY + y))
    poly.BooleanAdd(body)
    for i in ARMS:
        arm = pcbnew.SHAPE_POLY_SET()
        arm.NewOutline()
        for s, t in ((0, -ARM_W / 2), (ARM_LEN, -ARM_W / 2), (ARM_LEN, ARM_W / 2), (0, ARM_W / 2)):
            x, y = arm_xy(i, s, t)
            arm.Append(mm(CX + x), mm(CY + y))
        poly.BooleanAdd(arm)
        ring = pcbnew.SHAPE_POLY_SET()
        ring.NewOutline()
        mx, my = arm_xy(i, ARM_LEN, 0)
        R = MOTOR_HOLE_D / 2 + MOTOR_RING
        for k in range(72):
            ang = 2 * math.pi * k / 72
            ring.Append(mm(CX + mx + R * math.cos(ang)), mm(CY + my + R * math.sin(ang)))
        poly.BooleanAdd(ring)
    poly.Simplify()
    o = poly.Outline(0)
    n = o.PointCount()
    for k in range(n):
        a, b = o.CPoint(k), o.CPoint((k + 1) % n)
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(a); seg.SetEnd(b)
        seg.SetLayer(pcbnew.Edge_Cuts); seg.SetWidth(mm(0.1))
        board.Add(seg)
    # motor holes
    for i in ARMS:
        mx, my = arm_xy(i, ARM_LEN, 0)
        c = pcbnew.PCB_SHAPE(board)
        c.SetShape(pcbnew.SHAPE_T_CIRCLE)
        c.SetCenter(V(mx, my)); c.SetEnd(V(mx + MOTOR_HOLE_D / 2, my))
        c.SetLayer(pcbnew.Edge_Cuts); c.SetWidth(mm(0.1))
        board.Add(c)
        txt = pcbnew.PCB_TEXT(board)
        lx, ly = arm_xy(i, ARM_LEN - 7.5, 0)
        txt.SetText("M%d" % i); txt.SetPosition(V(lx, ly)); txt.SetLayer(pcbnew.F_SilkS)
        txt.SetTextSize(pcbnew.VECTOR2I(mm(1.2), mm(1.2))); txt.SetTextThickness(mm(0.18))
        board.Add(txt)
    return poly

def rule_area(board, name, pts, layers=(pcbnew.F_Cu, pcbnew.B_Cu)):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetZoneName(name)
    ls = pcbnew.LSET()
    for l in layers:
        ls.AddLayer(l)
    z.SetLayerSet(ls)
    z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(True); z.SetDoNotAllowZoneFills(True)
    z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
    o = z.Outline(); o.NewOutline()
    for x, y in pts:
        o.Append(mm(CX + x), mm(CY + y))
    board.Add(z)

def gnd_zone(board, layer, net, prio=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer); z.SetNet(net)
    z.SetAssignedPriority(prio)
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if layer == pcbnew.B_Cu else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3)); z.SetThermalReliefSpokeWidth(mm(0.4))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    o = z.Outline(); o.NewOutline()
    for x, y in ((-60, -60), (60, -60), (60, 60), (-60, 60)):
        o.Append(mm(CX + x), mm(CY + y))
    board.Add(z)

def courtyard_w(fp, rot):
    fp.SetOrientationDegrees(rot)
    bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox() if fp.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else fp.GetBoundingBox(False)
    return pcbnew.ToMM(bb.GetWidth())

def pack_band(fps):
    widths = [courtyard_w(fps[r], rot) for r, rot in BAND]
    x0, x1 = BODY[0] + 0.6, BODY[2] - 0.6
    gap = (x1 - x0 - sum(widths)) / (len(BAND) - 1)
    if gap < 0:
        raise SystemExit("rear band does not fit: %.2f mm short" % -gap)
    x = x0
    for (r, rot), w in zip(BAND, widths):
        P[r] = (x + w / 2, BAND_Y, rot)
        x += w + gap

def place_fp(fp, x, y, rot, by_origin=False):
    """Put the pad centroid (or the footprint origin) at (x, y) after rotating."""
    fp.SetOrientationDegrees(rot)
    if by_origin:
        fp.SetPosition(V(x, y))
        return
    fp.SetPosition(V(0, 0))
    pads = [p for p in fp.Pads() if p.GetNumber()]
    px = sum(p.GetPosition().x for p in pads) / len(pads)
    py = sum(p.GetPosition().y for p in pads) / len(pads)
    target = V(x, y)
    fp.SetPosition(pcbnew.VECTOR2I(int(target.x - (px - mm(CX))), int(target.y - (py - mm(CY)))))

def main():
    pinnet = load_netlist()
    board = pcbnew.CreateEmptyBoard() if hasattr(pcbnew, "CreateEmptyBoard") else pcbnew.BOARD()
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(mm(1.6))

    nets = {}
    def net(name):
        if name not in nets:
            ni = pcbnew.NETINFO_ITEM(board, name)
            board.Add(ni)
            nets[name] = ni
        return nets[name]

    fps = {}
    for c in design.C:
        if c["ref"].startswith("#"):
            continue
        lib, name = c["fp"].split(":")
        fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + ".pretty"), name)
        if fp is None:
            raise SystemExit("footprint not found: " + c["fp"])
        fp.SetReference(c["ref"]); fp.SetValue(c["value"])
        fp.SetFPIDAsString(c["fp"])
        sym_uuid = str(uuid.uuid5(uuid.NAMESPACE_DNS, "minidrone/sym/" + c["ref"]))
        fp.SetPath(pcbnew.KIID_PATH("/" + sym_uuid))
        if c["ref"] == "U1":
            # drop the library's oversized antenna keep-out, we add our own below
            for z in list(fp.Zones()):
                fp.Remove(z)
            for g in list(fp.GraphicalItems()):
                if g.GetLayer() == pcbnew.F_CrtYd:
                    fp.Remove(g)
            crt = pcbnew.PCB_SHAPE(fp)
            crt.SetShape(pcbnew.SHAPE_T_RECT)
            crt.SetStart(pcbnew.VECTOR2I(mm(-9.75), mm(-12.95)))
            crt.SetEnd(pcbnew.VECTOR2I(mm(9.75), mm(13.45)))
            crt.SetLayer(pcbnew.F_CrtYd); crt.SetWidth(mm(0.05))
            fp.Add(crt)
            # EP via holes 0.2 -> 0.3 mm (standard fab drill, lets you solder the pad from the back)
            for p in fp.Pads():
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and p.GetDrillSize().x < mm(0.3):
                    p.SetDrillSize(pcbnew.VECTOR2I(mm(0.3), mm(0.3)))
                    p.SetSize(pcbnew.VECTOR2I(mm(0.55), mm(0.55)))
        for k, v in symbol_fields(c).items():
            fp.SetField(k, v)
        for f in fp.GetFields():
            if f.GetName() not in ("Reference", "Value"):
                f.SetVisible(False)
        if c["ref"].startswith("M"):
            fp.SetAttributes(fp.GetAttributes() & ~pcbnew.FP_EXCLUDE_FROM_BOM)
        board.Add(fp)
        fps[c["ref"]] = fp
        for p in fp.Pads():
            n = pinnet.get((c["ref"], p.GetNumber()))
            if n:
                p.SetNet(net(n))
    pack_band(fps)
    for ref, fp in fps.items():
        x, y, rot = P[ref]
        place_fp(fp, x, y, rot, ref in PLACE_BY_ORIGIN)
        if ref == "J1":
            # battery plug sits on the rear edge
            bb = fp.GetBoundingBox(False)
            fp.Move(pcbnew.VECTOR2I(0, mm(CY + BODY[3] - 0.4) - bb.GetBottom()))
        if ref == "J2":
            # USB-C shell overhangs the left edge by 0.6 mm
            bb = fp.GetBoundingBox(False)
            fp.Move(pcbnew.VECTOR2I(mm(CX + BODY[0] - 0.6) - bb.GetLeft(), 0))
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        fp.Reference().SetTextThickness(mm(0.12))

    outline(board)
    # antenna keep-out: overhanging part of the module + margin
    x0, y0, x1, y1 = BODY
    rule_area(board, "ANTENNA_KEEPOUT", ((-13, y0 - 8), (13, y0 - 8), (13, y0 + 1.0), (-13, y0 + 1.0)))
    gnd_zone(board, pcbnew.B_Cu, net("/GND"))
    # top layer: no pour (every GND pad is tied to the B.Cu plane by the router)

    # silkscreen info
    for txt, x, y, size in (("MiniDrone FC v1.0", 0, -1.6, 1.0), ("FRONT ^", 0, BODY[1] + 4.0, 1.5)):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt); t.SetPosition(V(x, y)); t.SetLayer(pcbnew.B_SilkS)
        t.SetMirrored(True)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(0.15))
        board.Add(t)

    pcbnew.SaveBoard(OUT, board)
    # SaveBoard rewrites the project file with defaults: restore our rules + net classes
    import shutil
    shutil.copy(os.path.join(HERE, "project_template.kicad_pro"), os.path.join(ROOT, "MiniDrone_FC.kicad_pro"))
    print("saved", OUT, "footprints:", len(board.GetFootprints()), "nets:", len(nets))

if __name__ == "__main__":
    main()
