"""Load + flatten symbols from KiCad stock libraries."""
import copy, os
from sexp import parse, find, first, Sym

SYMDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"
_cache = {}

def _lib(name):
    if name not in _cache:
        with open(os.path.join(SYMDIR, name + ".kicad_sym")) as f:
            tree = parse(f.read())
        _cache[name] = {s[1]: s for s in find(tree, "symbol")}
    return _cache[name]

def get_symbol(lib_id):
    """Return flattened symbol s-expr named 'Lib:Name'."""
    lib, name = lib_id.split(":")
    syms = _lib(lib)
    s = copy.deepcopy(syms[name])
    ext = first(s, "extends")
    if ext:
        parent = get_symbol(lib + ":" + ext[1])
        pname = parent[1].split(":")[1]
        child_props = {p[1]: p for p in find(s, "property")}
        out = [Sym("symbol"), name]
        for item in parent[2:]:
            if isinstance(item, list) and item[0] == "property":
                out.append(child_props.pop(item[1], item))
            elif isinstance(item, list) and item[0] == "symbol":
                sub = copy.deepcopy(item)
                sub[1] = sub[1].replace(pname, name, 1)
                out.append(sub)
            else:
                out.append(item)
        # insert remaining child props after other props
        idx = max(i for i, x in enumerate(out) if isinstance(x, list) and x[0] == "property") + 1
        for p in child_props.values():
            out.insert(idx, p); idx += 1
        s = out
    s[1] = lib_id
    return s

def pins(sym):
    """List of dicts: number, name, type, x, y (lib coords, y-up), rot, hidden."""
    res = []
    for sub in find(sym, "symbol"):
        for p in find(sub, "pin"):
            at = first(p, "at")
            res.append(dict(
                type=str(p[1]),
                number=first(p, "number")[1],
                name=first(p, "name")[1],
                x=float(at[1]), y=float(at[2]), rot=int(float(at[3])) if len(at) > 3 else 0,
                length=float(first(p, "length")[1]),
                hidden=any(x == "hide" for x in p) or bool(first(p, "hide") and first(p, "hide")[1] == "yes"),
                unit=sub[1],
            ))
    return res
