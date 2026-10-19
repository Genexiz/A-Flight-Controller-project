"""Generate MiniDrone_FC.kicad_sch from design.py.

Style: every pin gets a net label right at its end (no wires), unused pins get
a no-connect flag. Components are grouped into titled blocks.
Run with any python3:  python3 gen_schematic.py
"""
import os, uuid
from sexp import Sym, dump, find, first
from kicadlib import get_symbol, pins
import design

OUT = os.path.join(os.path.dirname(__file__), "..", "MiniDrone_FC.kicad_sch")
PROJECT = "MiniDrone_FC"
GRID = 1.27
ROOT = str(uuid.uuid5(uuid.NAMESPACE_DNS, "minidrone-fc-root"))

def uid(*k):
    return str(uuid.uuid5(uuid.NAMESPACE_DNS, "minidrone/" + "/".join(map(str, k))))

def snap(v):
    return round(round(v / GRID) * GRID, 4)

# label angle for a pin, by pin rotation (lib) -> direction the label grows
LABEL_ANG = {0: 180, 180: 0, 90: 270, 270: 90}
DIRV = {0: (1, 0), 180: (-1, 0), 90: (0, -1), 270: (0, 1)}  # schematic coords (y down)

def label_len(net):
    return 1.1 * len(net) + 2.5

# block origins (mm) and wrap width on an A2 sheet (594 x 420)
BLOCKS = {
    "power":  (25, 40, 250),
    "mcu":    (300, 40, 270),
    "usb":    (25, 190, 120),
    "imu":    (160, 190, 120),
    "motors": (300, 190, 270),
}

def visible_pins(sym):
    seen, out = set(), []
    for p in pins(sym):
        if p["type"] == "no_connect" and p["hidden"]:
            continue
        key = (p["x"], p["y"])
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out

def all_pin_numbers(sym):
    return {p["number"]: p for p in pins(sym)}

def extents(comp, sym):
    xs, ys = [], []
    for p in visible_pins(sym):
        x, y = p["x"], -p["y"]
        xs.append(x); ys.append(y)
        net = comp["pins"].get(p["number"])
        dx, dy = DIRV[LABEL_ANG[p["rot"]]]
        L = label_len(net) if net else 2
        xs.append(x + dx * L); ys.append(y + dy * L)
    return min(xs) - 4, max(xs) + 12, min(ys) - 3, max(ys) + 3

def build():
    lib_syms, items = {}, []
    # check pin coverage
    for c in design.C:
        sym = get_symbol(c["lib"])
        lib_syms[c["lib"]] = sym
        allp = all_pin_numbers(sym)
        need = {p["number"] for p in visible_pins(sym)}
        # stacked hidden pins share position with a visible pin -> same net
        missing = need - set(c["pins"])
        extra = set(c["pins"]) - set(allp)
        if missing or extra:
            raise SystemExit("%s: missing pins %s, unknown pins %s" % (c["ref"], sorted(missing), sorted(extra)))

    flg = 0
    for blk, (ox, oy, width) in BLOCKS.items():
        comps = [c for c in design.C if c["block"] == blk]
        items.append([Sym("text"), design.BLOCK_TITLES[blk], [Sym("exclude_from_sim"), Sym("no")],
                      [Sym("at"), ox, oy - 12, 0],
                      [Sym("effects"), [Sym("font"), [Sym("size"), 2.54, 2.54], [Sym("bold"), Sym("yes")]],
                       [Sym("justify"), Sym("left"), Sym("bottom")]],
                      [Sym("uuid"), uid("title", blk)]])
        cx, cy, rowh = ox, oy, 0
        for c in comps:
            sym = lib_syms[c["lib"]]
            x0, x1, y0, y1 = extents(c, sym)
            w, h = x1 - x0, y1 - y0
            if cx + w > ox + width and cx > ox:
                cx, cy, rowh = ox, cy + rowh + 6, 0
            X, Y = snap(cx - x0), snap(cy - y0)
            cx += w + 4
            rowh = max(rowh, h)
            items.extend(place(c, sym, X, Y))
    return lib_syms, items

def place(c, sym, X, Y):
    ref = c["ref"]
    is_flag = ref.startswith("#FLG")
    if is_flag:
        flags = [d["ref"] for d in design.C if d["ref"].startswith("#FLG")]
        ref = "#FLG%02d" % (flags.index(c["ref"]) + 1)
    out = []
    node = [Sym("symbol"), [Sym("lib_id"), c["lib"]], [Sym("at"), X, Y, 0], [Sym("unit"), 1],
            [Sym("exclude_from_sim"), Sym("no")],
            [Sym("in_bom"), Sym("no" if is_flag else "yes")],
            [Sym("on_board"), Sym("no" if is_flag else "yes")],
            [Sym("dnp"), Sym("no")], [Sym("uuid"), uid("sym", c["ref"])]]
    vals = {"Reference": ref, "Value": c["value"], "Footprint": c["fp"]}
    for p in find(sym, "property"):
        name = p[1]
        at = first(p, "at")
        eff = first(p, "effects")
        val = vals.get(name, p[2])
        prop = [Sym("property"), name, val, [Sym("at"), round(X + float(at[1]), 4), round(Y - float(at[2]), 4), at[3] if len(at) > 3 else 0]]
        if eff is not None:
            prop.append(eff)
        out_hide = first(p, "hide")
        if out_hide is not None:
            prop.insert(4, out_hide)
        node.append(prop)
    if c.get("mpn"):
        node.append([Sym("property"), "MPN", c["mpn"], [Sym("at"), X, Y, 0], [Sym("hide"), Sym("yes")],
                     [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]])
    for p in pins(sym):
        node.append([Sym("pin"), p["number"], [Sym("uuid"), uid("pin", c["ref"], p["number"])]])
    node.append([Sym("instances"), [Sym("project"), PROJECT,
                 [Sym("path"), "/" + ROOT, [Sym("reference"), ref], [Sym("unit"), 1]]]])
    out.append(node)
    # labels / no-connects
    for p in visible_pins(sym):
        px, py = round(X + p["x"], 4), round(Y - p["y"], 4)
        net = c["pins"].get(p["number"])
        if net is None:
            out.append([Sym("no_connect"), [Sym("at"), px, py], [Sym("uuid"), uid("nc", c["ref"], p["number"])]])
            continue
        ang = LABEL_ANG[p["rot"]]
        just = [Sym("justify"), Sym("left" if ang in (0, 90) else "right"), Sym("bottom")]
        out.append([Sym("label"), net, [Sym("at"), px, py, ang], [Sym("fields_autoplaced"), Sym("yes")],
                    [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], just],
                    [Sym("uuid"), uid("lbl", c["ref"], p["number"])]])
    return out

def main():
    lib_syms, items = build()
    tree = [Sym("kicad_sch"), [Sym("version"), 20231120], [Sym("generator"), "eeschema"],
            [Sym("generator_version"), "8.0"], [Sym("uuid"), ROOT], [Sym("paper"), "A2"],
            [Sym("title_block"), [Sym("title"), "MiniDrone Flight Controller (ESP32-S3, brushed 1S)"],
             [Sym("rev"), "1.0"], [Sym("company"), "DIY"],
             [Sym("comment"), 1, "Generated by scripts/gen_schematic.py - edit design.py and regenerate"]],
            [Sym("lib_symbols")] + list(lib_syms.values())]
    tree += items
    tree.append([Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]])
    with open(OUT, "w") as f:
        f.write(dump(tree) + "\n")
    print("wrote", os.path.abspath(OUT), "components:", len(design.C))

if __name__ == "__main__":
    main()
