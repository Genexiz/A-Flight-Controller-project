"""Minimal S-expression parser/serializer for KiCad files."""
import re

class Sym(str):
    """Unquoted atom."""

_tok = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)

def parse(text):
    stack, cur = [], []
    pos = 0
    n = len(text)
    while pos < n:
        m = _tok.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise ValueError("parse error at %d" % pos)
        pos = m.end()
        if m.group(1):
            stack.append(cur); cur = []
        elif m.group(2):
            done = cur; cur = stack.pop(); cur.append(done)
        elif m.group(3) is not None:
            cur.append(m.group(3).replace('\\"', '"').replace("\\\\", "\\"))
        else:
            cur.append(Sym(m.group(4)))
    return cur[0]

def q(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'

def dump(node, indent=0):
    if isinstance(node, list):
        if not node:
            return "()"
        simple = all(not isinstance(x, list) for x in node)
        if simple and len(node) < 12:
            return "(" + " ".join(dump(x) for x in node) + ")"
        head = dump(node[0])
        parts = [head]
        for x in node[1:]:
            if isinstance(x, list):
                parts.append("\n" + "\t" * (indent + 1) + dump(x, indent + 1))
            else:
                parts.append(" " + dump(x))
        return "(" + "".join(parts) + "\n" + "\t" * indent + ")"
    if isinstance(node, Sym):
        return str(node)
    if isinstance(node, bool):
        return "yes" if node else "no"
    if isinstance(node, (int, float)):
        s = ("%.4f" % node).rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return q(node)

def find(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]

def first(node, key):
    r = find(node, key)
    return r[0] if r else None
