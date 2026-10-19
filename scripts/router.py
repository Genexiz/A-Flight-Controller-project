"""Small 2-layer grid maze router (A*) for the MiniDrone board.

Reads build/geom.json (export_geom.py), writes build/routes.json (apply_routes.py).
Plain python3 + numpy.

Rules it honours (keep in sync with project_template.kicad_pro):
  copper clearance 0.2 mm, board-edge clearance 0.3 mm, via 0.6/0.3 mm.
B.Cu is the ground plane, so the bottom layer is expensive and GND pads are
just dropped to it with a via.
"""
import heapq, json, math, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "..", "build")

G = 0.05            # grid pitch (mm)
CLR = 0.2           # copper clearance
EDGE = 0.3          # copper to board edge
MARGIN = 0.04       # rasterisation safety (half a cell + rounding)
VIA_D, VIA_DRILL = 0.6, 0.3
VIA_COST = 25       # in cells
BOTTOM_MULT = 2.2   # bottom layer is the GND plane: prefer top
BOTTOM_MULT_NET = {"/VBAT": 1.6}   # VBAT may hop along the board edge underneath
TURN45, TURN90 = 0.4, 1.5

WIDTH = {"/VBAT": 0.8, "/GND": 0.5, "/VIN": 0.5, "/VBUS": 0.5, "/+3V3": 0.3}
for i in range(1, 5):
    WIDTH["/MOT%d_N" % i] = 0.8
SIG_W = 0.2
# boxes (x0, y0, x1, y1 relative to board centre 150,100) the big power nets should
# avoid so they run along the board edges and leave the middle to signals
# (x0, y0, x1, y1, layers)
AVOID = {"/VBAT": [(-12.5, -1.0, 12.5, 11.6, (0, 1))]}
AVOID_MULT = 8.0
CENTRE = (150.0, 100.0)
# Hand-made escape for the 0.5 mm-pitch USB-C data pins (offsets in mm from pad A6
# centre, connector pads running top->bottom, pad ends pointing +x). D- (A7) dives
# to B.Cu inside the D+ loop and comes back up next to B7's track.
# Points are (dx, dy) mm from the anchor pad centre, or ("pad", ref, num).
PREROUTE = [
    # USB-C: D- (A7) dives to B.Cu inside the D+ loop and comes back up beside B7.
    dict(net="/USB_DN", ref="J2", anchor="A6", tracks=[
        (0, 0.2, [(0, -0.5), (4.5, -0.5)]),
        (0, 0.2, [(0, 0.5), (1.95, 0.5), (2.2, 0.75)]),
        (1, 0.2, [(2.2, 0.75), (4.5, -0.5)])],
        vias=[(2.2, 0.75), (4.5, -0.5)]),
    dict(net="/USB_DP", ref="J2", anchor="A6", tracks=[
        (0, 0.2, [(0, 0), (3.6, 0), (3.6, 1.5)]),
        (0, 0.2, [(0, 1.0), (1.1, 1.0), (1.6, 1.5), (3.6, 1.5)])],
        vias=[]),
    # MPU-6050 bottom row (pins 8..12 at 0.5 mm pitch), anchored on pin 10.
    dict(net="/+3V3", ref="U3", anchor="10", tracks=[
        (0, 0.2, [(-1.0, 0), (-1.0, 0.6), (-1.85, 1.45)])], vias=[]),
    dict(net="/GND", ref="U3", anchor="10", tracks=[
        (0, 0.2, [(-0.5, 0), (-0.5, 0.7), (-1.1, 1.3), (-1.1, 1.6)]),
        (0, 0.2, [(0.5, 0), (0.5, 1.8)])],
        vias=[(-1.1, 1.6), (0.5, 1.8)]),
    dict(net="/GND", ref="U3", anchor="18", tracks=[
        (0, 0.25, [(0, 0), ("pad", "C13", "2")])], vias=[]),
    dict(net="/IMU_REGOUT", ref="U3", anchor="10", tracks=[
        (0, 0.2, [(0, 0), (0, 0.85), (-0.45, 1.3), ("pad", "C12", "1")])], vias=[]),
    dict(net="/IMU_INT", ref="U3", anchor="10", tracks=[
        (0, 0.2, [(1.0, 0), (1.0, 0.8), (1.7, 1.5)])], vias=[]),
]
PROTECTED = {"/VBAT", "/MOT1_N", "/MOT2_N", "/MOT3_N", "/MOT4_N"}   # never ripped for others
POWER_ORDER = ["/VBAT", "/MOT1_N", "/MOT2_N", "/MOT3_N", "/MOT4_N", "/VIN", "/VBUS", "/+3V3"]

VERBOSE = "-v" in sys.argv
MAX_EXPAND = 2_500_000

DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

# ------------------------------------------------------------------ geometry
def raster_polys(polys, x0, y0, H, W):
    """Even-odd fill of polygons onto the grid (cell centres)."""
    m = np.zeros((H, W), bool)
    allpts = [p for poly in polys for p in poly]
    if not allpts:
        return m
    xs = [p[0] for p in allpts]; ys = [p[1] for p in allpts]
    c0 = max(0, int(math.floor((min(xs) - x0) / G)) - 1); c1 = min(W, int(math.ceil((max(xs) - x0) / G)) + 2)
    r0 = max(0, int(math.floor((min(ys) - y0) / G)) - 1); r1 = min(H, int(math.ceil((max(ys) - y0) / G)) + 2)
    if c1 <= c0 or r1 <= r0:
        return m
    px = x0 + (np.arange(c0, c1) + 0.5) * G
    py = y0 + (np.arange(r0, r1) + 0.5) * G
    X, Y = np.meshgrid(px, py)
    inside = np.zeros(X.shape, bool)
    for poly in polys:
        n = len(poly)
        for k in range(n):
            ax, ay = poly[k]; bx, by = poly[(k + 1) % n]
            if ay == by:
                continue
            cond = (Y >= min(ay, by)) & (Y < max(ay, by))
            xc = ax + (Y - ay) * (bx - ax) / (by - ay)
            inside ^= cond & (X < xc)
    m[r0:r1, c0:c1] = inside
    return m

def hdilate(mask, k):
    if k <= 0:
        return mask.copy()
    H, W = mask.shape
    cs = np.zeros((H, W + 2 * k + 1), np.int32)
    cs[:, k + 1:k + 1 + W] = mask
    cs = np.cumsum(cs, axis=1)
    return (cs[:, 2 * k + 1:2 * k + 1 + W] - cs[:, 0:W]) > 0

def dilate(mask, r):
    """Disk dilation: marks cells closer than r cells (strict) to a set cell."""
    if r <= 0:
        return mask.copy()
    H, W = mask.shape
    out = np.zeros_like(mask)
    cache = {}
    R = int(math.ceil(r)) - 1 if float(r).is_integer() else int(math.floor(r))
    for dy in range(-R, R + 1):
        rem = r * r - dy * dy
        if rem <= 0:
            continue
        k = int(math.ceil(math.sqrt(rem))) - 1
        if k not in cache:
            cache[k] = hdilate(mask, k)
        h = cache[k]
        if dy >= 0:
            out[dy:, :] |= h[:H - dy, :]
        else:
            out[:H + dy, :] |= h[-dy:, :]
    return out

def rcells(mm):
    """Clearance radius in (fractional) cells."""
    return mm / G

# ------------------------------------------------------------------ router
class Board:
    def __init__(self, geom):
        allp = [p for poly in geom["outline"] for p in poly]
        self.x0 = math.floor(min(p[0] for p in allp)) - 1
        self.y0 = math.floor(min(p[1] for p in allp)) - 1
        self.W = int(math.ceil((max(p[0] for p in allp) + 1 - self.x0) / G))
        self.H = int(math.ceil((max(p[1] for p in allp) + 1 - self.y0) / G))
        H, W = self.H, self.W
        inside = raster_polys(geom["outline"], self.x0, self.y0, H, W)
        self.outside = ~inside
        keep = np.zeros((H, W), bool)
        for k in geom["keepouts"]:
            keep |= raster_polys(k, self.x0, self.y0, H, W)
        self.keepout = keep
        self.netid = {"": -1}
        self.netname = {}
        self.owner = np.zeros((2, H, W), np.int32)
        self.padmask = np.zeros((2, H, W), bool)
        self.pads = []
        for p in sorted(geom["pads"], key=lambda q: (q["ref"], q["num"], q["x"], q["y"])):
            net = p["net"] if p["net"] and not p["net"].startswith("unconnected") else ""
            if p.get("npth") or not p["net"]:
                nid = -1
                if p["drill"] > 0:   # grow holes so vias keep the 0.25 mm hole-to-hole gap
                    p = dict(p, poly=[[(p["x"] + (x - p["x"]) * (1 + 0.3 / max(p["drill"], 0.1)), p["y"] + (y - p["y"]) * (1 + 0.3 / max(p["drill"], 0.1))) for x, y in poly] for poly in p["poly"]])
            elif net == "":
                nid = self._nid("NC:%s:%s" % (p["ref"], p["num"]))
            else:
                nid = self._nid(net)
            m = raster_polys(p["poly"], self.x0, self.y0, H, W)
            if not m.any():  # tiny pad: mark centre cell
                c, r = self.cell(p["x"], p["y"])
                m[r, c] = True
            for L in p["layers"]:
                self.owner[L][m] = nid
                self.padmask[L][m] = True
            if p["drill"] > 0:   # hole: block both layers for other nets
                for L in (0, 1):
                    hm = m & (self.owner[L] == 0)
                    self.owner[L][hm] = nid
            p["nid"] = nid
            p["mask"] = m
            self.pads.append(p)
        self.edge_cache = {}
        self.tracks, self.vias = [], []
        self.center = {}
        self.fixed_cells = {}

    def _nid(self, name):
        if name not in self.netid:
            self.netid[name] = len(self.netid)
            self.netname[self.netid[name]] = name
        return self.netid[name]

    def cell(self, x, y):
        return int((x - self.x0) / G), int((y - self.y0) / G)

    def xy(self, c, r):
        return round(self.x0 + (c + 0.5) * G, 4), round(self.y0 + (r + 0.5) * G, 4)

    def edge_block(self, w):
        key = round(w, 3)
        if key not in self.edge_cache:
            e = dilate(self.outside, rcells(EDGE + w / 2 + MARGIN))
            e |= dilate(self.keepout, rcells(w / 2 + MARGIN))
            self.edge_cache[key] = e
        return self.edge_cache[key]

    def free_map(self, nid, w, win, ignore_tracks=False):
        """passable[L] for net nid with track width w inside window win."""
        r0, r1, c0, c1 = win
        rad = rcells(CLR + w / 2 + MARGIN)
        pd = int(math.ceil(rad))
        pr0, pr1, pc0, pc1 = max(0, r0 - pd), min(self.H, r1 + pd), max(0, c0 - pd), min(self.W, c1 + pd)
        out = []
        eb = self.edge_block(w)[r0:r1, c0:c1]
        for L in (0, 1):
            o = self.owner[L, pr0:pr1, pc0:pc1]
            foreign = (o != 0) & (o != nid)
            if ignore_tracks:
                foreign &= self.padmask[L, pr0:pr1, pc0:pc1] | (o < 0)
            blk = dilate(foreign, rad)[r0 - pr0:r1 - pr0, c0 - pc0:c1 - pc0]
            own = self.owner[L, r0:r1, c0:c1] == nid
            out.append((~blk & ~eb) | (own & self.padmask[L, r0:r1, c0:c1]))
        return out

    def via_map(self, nid, win, ignore_tracks=False):
        r0, r1, c0, c1 = win
        rad = rcells(CLR + VIA_D / 2 + MARGIN)
        pd = int(math.ceil(rad))
        pr0, pr1, pc0, pc1 = max(0, r0 - pd), min(self.H, r1 + pd), max(0, c0 - pd), min(self.W, c1 + pd)
        ok = ~self.edge_block(VIA_D)[r0:r1, c0:c1]
        for L in (0, 1):
            o = self.owner[L, pr0:pr1, pc0:pc1]
            foreign = (o != 0) & (o != nid)
            if ignore_tracks:
                foreign &= self.padmask[L, pr0:pr1, pc0:pc1] | (o < 0)
            ok &= ~dilate(foreign, rad)[r0 - pr0:r1 - pr0, c0 - pc0:c1 - pc0]
            ok &= ~self.padmask[L, r0:r1, c0:c1]          # no via-in-pad
        # keep vias apart from other vias (hole to hole)
        return ok

    # ---------------------------------------------------------------- A*
    def astar(self, nid, w, sources, target_mask, target_xy, win, gnd_via=False, ignore_tracks=False):
        r0, r1, c0, c1 = win
        H, W = r1 - r0, c1 - c0
        free = self.free_map(nid, w, win, ignore_tracks)
        viaok = self.via_map(nid, win, ignore_tracks)
        fr = [bytearray(f.astype(np.uint8).tobytes()) for f in free]
        av = np.zeros((2, H, W), bool)
        for (ax0, ay0, ax1, ay1, lys) in AVOID.get(self.netname.get(nid, ""), []):
            cA, rA = self.cell(CENTRE[0] + ax0, CENTRE[1] + ay0)
            cB, rB = self.cell(CENTRE[0] + ax1, CENTRE[1] + ay1)
            for ly in lys:
                av[ly, max(0, rA - r0):max(0, rB - r0), max(0, cA - c0):max(0, cB - c0)] = True
        avb = [bytearray(a.astype(np.uint8).tobytes()) for a in av]
        bmult = BOTTOM_MULT_NET.get(self.netname.get(nid, ""), BOTTOM_MULT)
        vo = bytearray(viaok.astype(np.uint8).tobytes())
        tg = [bytearray(t[r0:r1, c0:c1].astype(np.uint8).tobytes()) for t in target_mask] if target_mask is not None else None
        N = H * W
        INF = 1e18
        gbest = {}
        prev = {}
        heap = []
        tx, ty = (target_xy[0] - c0, target_xy[1] - r0) if target_xy else (0, 0)

        def h(x, y):
            if gnd_via or target_xy is None:
                return 0.0
            dx, dy = abs(x - tx), abs(y - ty)
            return (dx + dy) + (1.41421356 - 2) * min(dx, dy)

        for (L, r, c) in sources:
            if r0 <= r < r1 and c0 <= c < c1:
                i = (r - r0) * W + (c - c0)
                s = (L * N + i) * 9 + 8
                gbest[s] = 0.0
                heapq.heappush(heap, (h(c - c0, r - r0), 0.0, s))
        if not heap:
            return None
        cnt = 0
        while heap:
            f, g, s = heapq.heappop(heap)
            if gbest.get(s, INF) < g:
                continue
            cnt += 1
            self.cnt = cnt
            if cnt > getattr(self, "limit", MAX_EXPAND):
                return None
            d = s % 9; li = s // 9; L = li // N; i = li % N
            y, x = divmod(i, W)
            if gnd_via:
                if vo[i] and g > 0:
                    return self._unwind(prev, s, N, W, r0, c0, via_end=True)
            elif tg[L][i]:
                return self._unwind(prev, s, N, W, r0, c0)
            mult = 1.0 if L == 0 else bmult
            for nd, (dx, dy) in enumerate(DIRS):
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= W or ny >= H:
                    continue
                j = ny * W + nx
                if not fr[L][j]:
                    continue
                if dx and dy and not (fr[L][y * W + nx] and fr[L][ny * W + x]):
                    continue
                step = 1.41421356 if dx and dy else 1.0
                if d != 8:
                    t = (nd - d) % 8
                    if t in (3, 4, 5):
                        continue
                    step += TURN45 if t in (1, 7) else (TURN90 if t in (2, 6) else 0)
                ng = g + step * (mult * AVOID_MULT if avb[L][j] else mult)
                ns = (L * N + j) * 9 + nd
                if ng < gbest.get(ns, INF):
                    gbest[ns] = ng; prev[ns] = s
                    heapq.heappush(heap, (ng + h(nx, ny), ng, ns))
            if vo[i] and not gnd_via:
                OL = 1 - L
                if fr[OL][i]:
                    ns = (OL * N + i) * 9 + 8
                    ng = g + VIA_COST
                    if ng < gbest.get(ns, INF):
                        gbest[ns] = ng; prev[ns] = s
                        heapq.heappush(heap, (ng + h(x, y), ng, ns))
        return None

    def _unwind(self, prev, s, N, W, r0, c0, via_end=False):
        path = []
        while True:
            li = s // 9; L = li // N; i = li % N
            y, x = divmod(i, W)
            path.append((L, y + r0, x + c0))
            if s not in prev:
                break
            s = prev[s]
        path.reverse()
        return path, via_end

    # ---------------------------------------------------------------- commit
    def lay(self, nid, w, path, via_end):
        segs, vias = [], []
        pts = path
        k = 0
        cur = [pts[0]]
        for p in pts[1:]:
            if p[0] != cur[-1][0]:
                if len(cur) > 1:
                    segs.append(cur)
                vias.append((p[1], p[2]))
                cur = [p]
            else:
                cur.append(p)
        if len(cur) > 1:
            segs.append(cur)
        if via_end:
            vias.append((pts[-1][1], pts[-1][2]))
        new_cells = []
        for run in segs:
            L = run[0][0]
            # merge collinear
            verts = [run[0]]
            for a, b, c in zip(run, run[1:], run[2:]):
                if (b[1] - a[1], b[2] - a[2]) != (c[1] - b[1], c[2] - b[2]):
                    verts.append(b)
            verts.append(run[-1])
            for a, b in zip(verts, verts[1:]):
                self.tracks.append(dict(net=self.netname[nid], layer=L, w=w,
                                        a=self.xy(a[2], a[1]), b=self.xy(b[2], b[1])))
                self._paint_seg(L, a, b, w / 2, nid)
            new_cells += [(L, r, c) for (_, r, c) in run]
        have = {tuple(v["at"]) for v in self.vias}
        for (r, c) in vias:
            if self.xy(c, r) in have:
                continue
            self.vias.append(dict(net=self.netname[nid], at=self.xy(c, r)))
            for L in (0, 1):
                self._paint_disk(L, r, c, VIA_D / 2, nid)
                new_cells.append((L, r, c))
        self.center.setdefault(nid, []).extend(new_cells)
        return new_cells

    def _paint_seg(self, L, a, b, rad, nid):
        R = int(math.ceil(rcells(rad))) + 1
        rmin, rmax = min(a[1], b[1]) - R, max(a[1], b[1]) + R + 1
        cmin, cmax = min(a[2], b[2]) - R, max(a[2], b[2]) + R + 1
        rr, cc = np.mgrid[rmin:rmax, cmin:cmax]
        ax, ay, bx, by = a[2], a[1], b[2], b[1]
        dx, dy = bx - ax, by - ay
        ll = dx * dx + dy * dy
        t = np.clip(((cc - ax) * dx + (rr - ay) * dy) / ll, 0, 1) if ll else np.zeros(rr.shape)
        dist = np.hypot(cc - (ax + t * dx), rr - (ay + t * dy)) * G
        m = dist <= rad + 1e-6
        sub = self.owner[L, rmin:rmax, cmin:cmax]
        sub[m & (sub == 0)] = nid

    def rip(self, nid):
        name = self.netname[nid]
        for L in (0, 1):
            m = (self.owner[L] == nid) & ~self.padmask[L]
            self.owner[L][m] = 0
        self.center[nid] = list(self.fixed_cells.get(nid, []))
        self.tracks = [t for t in self.tracks if t["net"] != name or t.get("fixed")]
        self.vias = [v for v in self.vias if v["net"] != name or v.get("fixed")]
        for t in self.tracks:
            if t["net"] == name:
                ca, ra = self.cell(*t["a"]); cb, rb = self.cell(*t["b"])
                self._paint_seg(t["layer"], (t["layer"], ra, ca), (t["layer"], rb, cb), t["w"] / 2, nid)
        for v in self.vias:
            if v["net"] == name:
                c, r = self.cell(*v["at"])
                for L in (0, 1):
                    self._paint_disk(L, r, c, VIA_D / 2, nid)
        # restore drill-hole ownership of this net's THT pads
        for p in self.pads:
            if p["nid"] == nid and p["drill"] > 0:
                for L in (0, 1):
                    self.owner[L][p["mask"] & (self.owner[L] == 0)] = nid

    def _paint_disk(self, L, r, c, rad, nid):
        self._paint_seg(L, (L, r, c), (L, r, c), rad, nid)

# ------------------------------------------------------------------ driver
def window(cells, pad, H, W):
    rs = [c[1] for c in cells]; cs = [c[2] for c in cells]
    m = int(pad / G)
    return max(0, min(rs) - m), min(H, max(rs) + m + 1), max(0, min(cs) - m), min(W, max(cs) + m + 1)

def pad_cells(b, p):
    rr, cc = np.nonzero(p["mask"])
    return [(L, int(r), int(c)) for L in p["layers"] for r, c in zip(rr, cc) if b.owner[L, r, c] == p["nid"]]

def log(*a):
    if VERBOSE:
        print(*a, flush=True)

class Router:
    def __init__(self, geom):
        self.b = b = Board(geom)
        self.by_net = {}
        for p in b.pads:
            if p["nid"] > 0 and not b.netname[p["nid"]].startswith("NC:"):
                self.by_net.setdefault(b.netname[p["nid"]], []).append(p)
        nets = [n for n in self.by_net if n != "/GND" and len(self.by_net[n]) > 1]
        hp = lambda n: (max(p["x"] for p in self.by_net[n]) - min(p["x"] for p in self.by_net[n]) +
                        max(p["y"] for p in self.by_net[n]) - min(p["y"] for p in self.by_net[n]))
        self.order = [n for n in POWER_ORDER if n in nets] + sorted([n for n in nets if n not in POWER_ORDER], key=hp) + ["/GND"]
        self.fails = {}
        self.pre_cells = {}
        self.pre_ends = {}
        self.pre_vias = {}
        self.apply_preroutes()
        for net, cells in self.pre_cells.items():
            nid = self.by_net[net][0]["nid"]
            b.fixed_cells[nid] = list(cells)
            b.center[nid] = list(cells)

    def apply_preroutes(self):
        b = self.b
        padat = {(p["ref"], p["num"]): p for p in b.pads}
        for d in PREROUTE:
            net = d["net"]
            anc = padat[(d["ref"], d["anchor"])]
            nid = self.by_net[net][0]["nid"]
            cells = self.pre_cells.setdefault(net, [])
            ends = self.pre_ends.setdefault(net, [])
            vcells = self.pre_vias.setdefault(net, [])
            def cellof(pt):
                if pt[0] == "pad":
                    q = padat[(pt[1], pt[2])]
                    c, r = b.cell(q["x"], q["y"])
                else:
                    c, r = b.cell(anc["x"] + pt[0], anc["y"] + pt[1])
                return r, c
            for (L, w, pts) in d["tracks"]:
                for a, bb in zip(pts, pts[1:]):
                    ra, ca = cellof(a); rb, cb = cellof(bb)
                    b.tracks.append(dict(net=net, layer=L, w=w, a=b.xy(ca, ra), b=b.xy(cb, rb), fixed=True))
                    b._paint_seg(L, (L, ra, ca), (L, rb, cb), w / 2, nid)
                    n = max(abs(rb - ra), abs(cb - ca), 1)
                    cells += [(L, ra + round((rb - ra) * k / n), ca + round((cb - ca) * k / n)) for k in range(n + 1)]
                re_, ce_ = cellof(pts[-1])
                ends.append((L, re_, ce_))
            for off in d["vias"]:
                r, c = cellof(off)
                b.vias.append(dict(net=net, at=b.xy(c, r), fixed=True))
                for L in (0, 1):
                    b._paint_disk(L, r, c, VIA_D / 2, nid)
                    cells.append((L, r, c))
                    ends.append((L, r, c))
                    vcells.append((L, r, c))

    # -------------------------------------------------------------- one net
    def route_net(self, net):
        if net == "/GND":
            return self.route_gnd()
        b = self.b
        pads = self.by_net[net]
        nid = pads[0]["nid"]
        wnet = WIDTH.get(net, SIG_W)
        tree = [pads[0]]
        rest = pads[1:]
        tree_cells = pad_cells(b, pads[0])
        if net in self.pre_cells:
            pre = set(self.pre_cells[net])
            tree = [p for p in pads if any(q in pre for q in pad_cells(b, p))]
            rest = [p for p in pads if p not in tree]
            # grow only from the open ends of the hand-routed stubs (no dangling stubs)
            tree_cells = list(self.pre_ends[net]) + [q for p in tree for q in pad_cells(b, p)
                                                      if not any(x in pre for x in pad_cells(b, p))]
        failed = []
        while rest:
            tx = sorted((min(math.hypot(p["x"] - q["x"], p["y"] - q["y"]) for q in tree), k) for k, p in enumerate(rest))
            p = rest.pop(tx[0][1])
            w0 = min(wnet, max(SIG_W, p["minsize"]))
            tmask = [p["mask"] if L in p["layers"] else np.zeros_like(p["mask"]) for L in (0, 1)]
            tc, trr = b.cell(p["x"], p["y"])
            res = None
            narrow = [] if net == "/VBAT" or net.endswith("_N") else sorted({min(w0, 0.3), SIG_W} - {w0}, reverse=True)
            near = min(tree_cells, key=lambda q: (q[1] - trr) ** 2 + (q[2] - tc) ** 2)
            for w, padmm in [(w0, 5), (w0, 12), (w0, 1000)] + [(x, pm) for x in narrow for pm in (5, 1000)]:
                win = window([near, (0, trr, tc)], padmm, b.H, b.W)
                srcs = [q for q in tree_cells if win[0] <= q[1] < win[1] and win[2] <= q[2] < win[3]]
                b.limit = MAX_EXPAND if padmm >= 1000 else 300_000
                t0 = time.time()
                res = b.astar(nid, w, srcs, tmask, (tc, trr), win)
                log("  %-12s -> %s.%s w=%.2f win=%d exp=%d %.1fs %s" % (net, p["ref"], p["num"], w, padmm, b.cnt, time.time() - t0, "ok" if res else "FAIL"))
                if res:
                    break
            tree.append(p)
            if not res:
                failed.append((net, p))
                continue
            tree_cells += b.lay(nid, w, res[0], False)
            tree_cells += pad_cells(b, p)
        return failed

    def route_gnd(self):
        b = self.b
        gp = self.by_net.get("/GND", [])
        tht = {(p["ref"], p["num"]) for p in gp if 1 in p["layers"]}
        failed = []
        pre = set(self.pre_vias.get("/GND", []))
        via_pads = {("U3", "9"), ("U3", "11")}
        for p in gp:
            if 1 in p["layers"] or (p["ref"], p["num"]) in tht or p["minsize"] < 0.2:
                continue
            if (p["ref"], p["num"]) in via_pads:
                continue
            nid = p["nid"]
            w = min(WIDTH["/GND"], max(SIG_W, p["minsize"]))
            cells = pad_cells(b, p)
            res = None
            for padmm in (3, 6):
                b.limit = 300_000
                res = b.astar(nid, w, cells, None, None, window(cells, padmm, b.H, b.W), gnd_via=True)
                if res:
                    break
            if res:
                b.lay(nid, w, res[0], True)
                continue
            tmask = [(b.owner[0] == nid) & ~p["mask"], np.zeros_like(p["mask"])]
            for w2 in sorted({min(w, 0.3), SIG_W}, reverse=True):
                for padmm in (3, 8):
                    b.limit = 300_000
                    res = b.astar(nid, w2, cells, tmask, None, window(cells, padmm, b.H, b.W))
                    if res:
                        break
                if res:
                    break
            if not res:
                failed.append(("/GND", p))
                continue
            b.lay(nid, w2, res[0], False)
        return failed

    def route_missing(self, net, missing):
        """Connect the given pads to the copper already laid for net (keeps it)."""
        b = self.b
        nid = self.by_net[net][0]["nid"]
        wnet = WIDTH.get(net, SIG_W)
        mset = {(p["ref"], p["num"]) for p in missing}
        tree_cells = list(b.center.get(nid, [])) + [q for p in self.by_net[net] if (p["ref"], p["num"]) not in mset for q in pad_cells(b, p)]
        failed = []
        for p in missing:
            w0 = min(wnet, max(SIG_W, p["minsize"]))
            tmask = [p["mask"] if L in p["layers"] else np.zeros_like(p["mask"]) for L in (0, 1)]
            tc, trr = b.cell(p["x"], p["y"])
            near = min(tree_cells, key=lambda q: (q[1] - trr) ** 2 + (q[2] - tc) ** 2)
            res = None
            for padmm in (5, 12, 1000):
                win = window([near, (0, trr, tc)], padmm, b.H, b.W)
                srcs = [q for q in tree_cells if win[0] <= q[1] < win[1] and win[2] <= q[2] < win[3]]
                b.limit = MAX_EXPAND if padmm >= 1000 else 300_000
                res = b.astar(nid, w0, srcs, tmask, (tc, trr), win)
                if res:
                    break
            if not res:
                failed.append((net, p))
                continue
            tree_cells += b.lay(nid, w0, res[0], False) + pad_cells(b, p)
        return failed

    # -------------------------------------------------------------- rip-up
    def blockers(self, net, p):
        """Nets whose tracks sit on the ideal (tracks-ignored) path of a failed connection."""
        b = self.b
        nid = p["nid"]
        others = [q for q in self.by_net[net] if q is not p] if net != "/GND" else []
        cells = pad_cells(b, p)
        if net == "/GND":
            res = b.astar(nid, SIG_W, cells, None, None, window(cells, 4, b.H, b.W), gnd_via=True, ignore_tracks=True)
        else:
            q = min(others, key=lambda q: math.hypot(p["x"] - q["x"], p["y"] - q["y"]))
            tmask = [q["mask"] if L in q["layers"] else np.zeros_like(q["mask"]) for L in (0, 1)]
            b.limit = MAX_EXPAND
            res = b.astar(nid, WIDTH.get(net, SIG_W) if net == "/VBAT" else SIG_W, cells, tmask,
                          b.cell(q["x"], q["y"]), window(cells + pad_cells(b, q), 6, b.H, b.W), ignore_tracks=True)
        if not res:
            return set()
        rad = int(math.ceil(rcells(CLR + WIDTH.get(net, SIG_W) / 2 + MARGIN))) + 2
        found = set()
        for (L, r, c) in res[0]:
            o = b.owner[L, max(0, r - rad):r + rad + 1, max(0, c - rad):c + rad + 1]
            pm = b.padmask[L, max(0, r - rad):r + rad + 1, max(0, c - rad):c + rad + 1]
            for v in np.unique(o[(o > 0) & ~pm & (o != nid)]):
                found.add(b.netname[int(v)])
        return found

    def total_fails(self):
        return sum(len(v) for v in self.fails.values())

    def snapshot(self):
        b = self.b
        return (b.owner.copy(), list(b.tracks), list(b.vias), {k: list(v) for k, v in self.fails.items()},
                {k: list(v) for k, v in b.center.items()})

    def restore(self, snap):
        b = self.b
        b.owner, b.tracks, b.vias, self.fails = snap[0].copy(), list(snap[1]), list(snap[2]), {k: list(v) for k, v in snap[3].items()}
        b.center = {k: list(v) for k, v in snap[4].items()}

    def run(self, iters):
        for net in self.order:
            self.fails[net] = self.route_net(net)
        print("initial: %d failed %s" % (self.total_fails(), self.fail_list()), flush=True)
        best = self.snapshot()
        history = {}
        for it in range(iters):
            bad = [n for n in self.order if self.fails.get(n)]
            if not bad:
                break
            for net in bad:
                if not self.fails.get(net):
                    continue
                bl = set()
                for (_, p) in self.fails[net]:
                    bl |= self.blockers(net, p)
                bl.discard(net)
                bl -= PROTECTED
                nid = self.by_net[net][0]["nid"]
                history[net] = history.get(net, 0) + 1
                log("rip-up for %s: blockers %s" % (net, sorted(bl)))
                for x in bl:
                    self.b.rip(self.by_net[x][0]["nid"])
                if net in PROTECTED:
                    self.fails[net] = self.route_missing(net, [p for (_, p) in self.fails[net]])
                else:
                    self.b.rip(nid)
                    self.fails[net] = self.route_net(net)
                for x in sorted(bl, key=lambda n: self.order.index(n)):
                    self.fails[x] = self.route_net(x)
            print("iter %d: %d failed %s" % (it, self.total_fails(), self.fail_list()), flush=True)
            if self.total_fails() < sum(len(v) for v in best[3].values()):
                best = self.snapshot()
        if self.total_fails() > sum(len(v) for v in best[3].values()):
            self.restore(best)

    def fail_list(self):
        return [(n, p["ref"], p["num"]) for n, v in self.fails.items() for (_, p) in v]

def main():
    geom = json.load(open(os.path.join(BUILD, "geom.json")))
    iters = int(sys.argv[sys.argv.index("--iters") + 1]) if "--iters" in sys.argv else 6
    t = time.time()
    R = Router(geom)
    R.run(iters)
    b = R.b
    if "--png" in sys.argv:
        import debug_png
        debug_png.write_png(os.path.join(BUILD, "route.png"), debug_png.owner_image(b, 2))
    failed = R.fail_list()
    json.dump(dict(tracks=b.tracks, vias=b.vias, failed=failed, via=[VIA_D, VIA_DRILL]),
              open(os.path.join(BUILD, "routes.json"), "w"), indent=0)
    print("written routes.json: %d tracks, %d vias, failed %s (%.0fs)" % (len(b.tracks), len(b.vias), failed, time.time() - t))

if __name__ == "__main__":
    main()
