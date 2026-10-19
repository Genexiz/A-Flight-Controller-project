"""Write build/routes.json tracks/vias into the board, refill zones. KiCad Python."""
import json, os, shutil
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
BOARD = os.path.join(ROOT, "MiniDrone_FC.kicad_pcb")

def main():
    r = json.load(open(os.path.join(ROOT, "build", "routes.json")))
    b = pcbnew.LoadBoard(BOARD)
    for t in list(b.GetTracks()):
        b.Delete(t)
    cache = {}
    class Nets(dict):
        def __missing__(self, k):
            self[k] = b.FindNet(k)
            return self[k]
    nets = Nets()
    for t in r["tracks"]:
        tr = pcbnew.PCB_TRACK(b)
        tr.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(t["a"][0]), pcbnew.FromMM(t["a"][1])))
        tr.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(t["b"][0]), pcbnew.FromMM(t["b"][1])))
        tr.SetWidth(pcbnew.FromMM(t["w"]))
        tr.SetLayer(pcbnew.F_Cu if t["layer"] == 0 else pcbnew.B_Cu)
        tr.SetNet(nets[t["net"]])
        b.Add(tr)
    d, drill = r["via"]
    for v in r["vias"]:
        via = pcbnew.PCB_VIA(b)
        via.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(v["at"][0]), pcbnew.FromMM(v["at"][1])))
        via.SetWidth(pcbnew.FromMM(d)); via.SetDrill(pcbnew.FromMM(drill))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetNet(nets[v["net"]])
        b.Add(via)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(BOARD, b)
    shutil.copy(os.path.join(HERE, "project_template.kicad_pro"), os.path.join(ROOT, "MiniDrone_FC.kicad_pro"))
    print("applied", len(r["tracks"]), "tracks", len(r["vias"]), "vias")

if __name__ == "__main__":
    main()
