import pcbnew, math, itertools
T=pcbnew.ToMM; F=pcbnew.FromMM
b=pcbnew.LoadBoard('MiniDrone_FC.kicad_pcb')
outline=pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(outline, False)
o=outline.Outline(0)
poly=[(T(o.CPoint(k).x),T(o.CPoint(k).y)) for k in range(o.PointCount())]
holes=[]
for i in range(outline.HoleCount(0)):
    h=outline.Hole(0,i); holes.append([(T(h.CPoint(k).x),T(h.CPoint(k).y)) for k in range(h.PointCount())])
def inside(px,py,pl):
    c=False
    for (ax,ay),(bx,by) in zip(pl,pl[1:]+pl[:1]):
        if (ay>py)!=(by>py) and px < ax+(py-ay)*(bx-ax)/(by-ay): c=not c
    return c
def dpoly(px,py,pl):
    return min(dseg(px,py,(ax,ay,bx,by,0)) for (ax,ay),(bx,by) in zip(pl,pl[1:]+pl[:1]))
segs=[]
for t in b.GetTracks():
    if isinstance(t,pcbnew.PCB_VIA):
        p=t.GetPosition(); segs.append((T(p.x),T(p.y),T(p.x),T(p.y),0.3))
    elif t.GetLayer()==pcbnew.F_Cu:
        segs.append((T(t.GetStart().x),T(t.GetStart().y),T(t.GetEnd().x),T(t.GetEnd().y),T(t.GetWidth())/2))
boxes=[]
for fp in b.GetFootprints():
    c=fp.GetCourtyard(pcbnew.F_CrtYd)
    if c.OutlineCount():
        bb=c.BBox(); boxes.append((T(bb.GetX()),T(bb.GetY()),T(bb.GetRight()),T(bb.GetBottom())))
    for p in fp.Pads():
        pb=p.GetBoundingBox(); boxes.append((T(pb.GetX()),T(pb.GetY()),T(pb.GetRight()),T(pb.GetBottom())))
for z in b.Zones():
    if z.GetIsRuleArea():
        bb=z.GetBoundingBox(); boxes.append((T(bb.GetX()),T(bb.GetY()),T(bb.GetRight()),T(bb.GetBottom())))
def dseg(px,py,s):
    x1,y1,x2,y2,r=s; dx,dy=x2-x1,y2-y1; L=dx*dx+dy*dy
    t=0 if L==0 else max(0,min(1,((px-x1)*dx+(py-y1)*dy)/L))
    return math.hypot(px-(x1+t*dx),py-(y1+t*dy))-r
def dbox(px,py,bx):
    x0,y0,x1,y1=bx; dx=max(x0-px,0,px-x1); dy=max(y0-py,0,py-y1)
    return math.hypot(dx,dy) if (dx or dy) else -1
pts=[]
x=110.0
while x<190:
    y=60.0
    while y<140:
        if inside(x,y,poly) and not any(inside(x,y,h) for h in holes):
            de=min([dpoly(x,y,poly)]+[dpoly(x,y,h) for h in holes])
            if de>=1.6:
                ds=min(dseg(x,y,s) for s in segs); db=min(dbox(x,y,bx) for bx in boxes)
                if ds>=1.3 and db>=1.1:
                    pts.append((round(x,2),round(y,2)))
        y+=0.25
    x+=0.25
print('free', len(pts))
# pick 3 maximizing triangle area (spread) - restrict to body region for camera
body=[p for p in pts if 132<=p[0]<=168 and 80<=p[1]<=127]
print('body', len(body))
cand=pts[::4]
best=(0,None)
for a,b2,c in itertools.combinations(cand,3):
    area=abs((b2[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b2[1]-a[1]))/2
    if area>best[0]: best=(area,(a,b2,c))
print(best)
