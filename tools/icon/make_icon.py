"""Draws the SlimeVR-lite icon (a green slime) and writes overlay/resources/slimevr_lite.ico
with PNG entries at the standard Windows sizes.

Usage: python tools/icon/make_icon.py [preview.png]
"""
import io
import os
import struct
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, FancyBboxPatch, PathPatch
from matplotlib.path import Path

SIZES = [256, 128, 64, 48, 32, 24, 16]
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "overlay", "resources", "slimevr_lite.ico")


def render(size):
    fig = plt.figure(figsize=(1, 1), dpi=size)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    fig.patch.set_alpha(0)
    # Rounded dark tile
    ax.add_patch(FancyBboxPatch((0.04, 0.04), 0.92, 0.92, boxstyle="round,pad=0,rounding_size=0.2",
                                fc="#1d2433", ec="none"))
    # Slime body: dome with a flat, slightly wavy base
    verts = [(0.16, 0.22), (0.14, 0.62), (0.32, 0.84), (0.50, 0.84), (0.68, 0.84), (0.86, 0.62),
             (0.84, 0.22), (0.72, 0.17), (0.62, 0.24), (0.50, 0.20), (0.38, 0.16), (0.28, 0.24), (0.16, 0.22)]
    codes = [Path.MOVETO, Path.CURVE4, Path.CURVE4, Path.CURVE4, Path.CURVE4, Path.CURVE4, Path.CURVE4,
             Path.CURVE4, Path.CURVE4, Path.CURVE4, Path.CURVE4, Path.CURVE4, Path.CURVE4]
    ax.add_patch(PathPatch(Path(verts, codes), fc="#5fe08f", ec="#2f9e5c", lw=max(0.5, size / 64)))
    # Highlight and eyes (simplified at tiny sizes)
    if size >= 32:
        ax.add_patch(Ellipse((0.34, 0.68), 0.12, 0.07, angle=30, fc="white", alpha=0.55, ec="none"))
    for x in (0.39, 0.61):
        ax.add_patch(Ellipse((x, 0.47), 0.10, 0.15, fc="#1d2433", ec="none"))
        if size >= 48:
            ax.add_patch(Ellipse((x + 0.02, 0.51), 0.035, 0.05, fc="white", ec="none"))
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=size, transparent=True)
    plt.close(fig)
    return buf.getvalue()


def main():
    images = [render(s) for s in SIZES]
    header = struct.pack("<HHH", 0, 1, len(images))
    offset = 6 + 16 * len(images)
    entries, data = b"", b""
    for size, png in zip(SIZES, images):
        dim = 0 if size >= 256 else size
        entries += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(png), offset + len(data))
        data += png
    with open(OUT, "wb") as f:
        f.write(header + entries + data)
    print(f"wrote {os.path.normpath(OUT)} ({len(header + entries + data)} bytes)")
    if len(sys.argv) > 1:
        with open(sys.argv[1], "wb") as f:
            f.write(images[0])


if __name__ == "__main__":
    main()
