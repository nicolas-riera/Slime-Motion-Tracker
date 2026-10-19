"""Draws the Slime Motion Tracker icon (a purple rounded tile with "SMT", like the
SteamVR dashboard thumbnail drawn in overlay/src/main.cpp) and writes
overlay/resources/smt.ico with PNG entries at the standard Windows sizes.

Usage: python tools/icon/make_icon.py [preview.png]
"""
import io
import os
import struct
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

SIZES = [256, 128, 64, 48, 32, 24, 16]
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "overlay", "resources", "smt.ico")
ACCENT = "#5a4cd8"


def render(size):
    fig = plt.figure(figsize=(1, 1), dpi=size)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    fig.patch.set_alpha(0)
    margin = 0.02 if size <= 24 else 0.04
    ax.add_patch(FancyBboxPatch((margin, margin), 1 - 2 * margin, 1 - 2 * margin,
                                boxstyle="round,pad=0,rounding_size=0.18", fc=ACCENT, ec="none"))
    # The text fills the tile width; points = fraction of the 1-inch figure * 72
    ax.text(0.5, 0.47, "SMT", ha="center", va="center", color="white", fontweight="bold",
            fontsize=72 * (0.36 if size >= 32 else 0.40), family="DejaVu Sans")
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
