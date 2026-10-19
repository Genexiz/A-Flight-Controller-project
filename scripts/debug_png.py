"""Tiny PNG writer for router debugging: owner map -> image."""
import zlib, struct
import numpy as np

def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].astype(np.uint8).tobytes() for y in range(h))
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + \
        chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    open(path, "wb").write(png)

def owner_image(b, step=2):
    o0 = b.owner[0, ::step, ::step]; o1 = b.owner[1, ::step, ::step]
    img = np.full(o0.shape + (3,), 255, np.uint8)
    img[b.outside[::step, ::step]] = (40, 40, 40)
    img[o1 > 0] = (120, 160, 255)
    img[o0 > 0] = (220, 60, 60)
    img[(o0 > 0) & (o1 > 0)] = (200, 120, 0)
    img[(o0 < 0) | (o1 < 0)] = (0, 0, 0)
    pm = b.padmask[0, ::step, ::step]
    img[pm & (o0 > 0)] = (255, 170, 170)
    return img
