"""Draws the user's Kinect point cloud in SteamVR space next to the engine
outputs recorded at the same time (side and top views), for a few instants.

Usage: python tools/kinect_gt/preview.py rec.depth scene.npz calib.npz session.jsonl out.png
           [--times T1,T2,...] [--count N]
Times are host clock seconds (as in the recordings); default: N evenly spaced.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from calibrate import FloorFrame, user_points  # noqa: E402
from kinect_depth import DepthRecording  # noqa: E402


class KinectToWorld:
    def __init__(self, calib, floor_y=0.0):
        c = np.load(calib)
        self.frame = FloorFrame(c["floor_n"], float(c["floor_d"]))
        self.yaw = float(c["yaw"])
        self.t = np.array(c["t"], dtype=float)
        self.dt = float(c["dt"])
        self.floor_y = floor_y

    def points(self, camera_points):
        f = self.frame.coords(camera_points)
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        a, h, c = f[:, 0], f[:, 1], f[:, 2]
        return np.stack([cy * a + sy * c + self.t[0], h + self.floor_y, -sy * a + cy * c + self.t[1]], axis=1)


def load_outputs(session):
    rows, hmd = [], []
    with open(session, encoding="utf-8") as fh:
        for line in fh:
            if '"k":"out"' in line:
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                rows.append([d["t"], *d["waist"][1:4], *d["lf"][1:4], *d["rf"][1:4], *d["kn"]])
            elif '"k":"dev"' in line and '"cls":1,' in line:
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                hmd.append([d["t"], *d["p"]])
    return np.array(rows), np.array(hmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("depth")
    ap.add_argument("scene")
    ap.add_argument("calib")
    ap.add_argument("session")
    ap.add_argument("out")
    ap.add_argument("--times")
    ap.add_argument("--count", type=int, default=6)
    args = ap.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rec = DepthRecording(args.depth)
    scene = np.load(args.scene)
    k2w = KinectToWorld(args.calib)
    outs, hmd = load_outputs(args.session)
    if args.times:
        wanted = [float(x) for x in args.times.split(",")]
    else:
        t0, t1 = max(outs[0, 0], hmd[0, 0]), outs[-1, 0]
        wanted = list(np.linspace(t0 + 30, t1 - 5, args.count))

    picked = []
    it = iter(sorted(wanted))
    target = next(it, None)
    for t, rt, depth in rec.frames():
        if target is None:
            break
        if t + k2w.dt >= target:
            pts = user_points(rec, depth, scene["background"])
            if pts is not None:
                picked.append((t + k2w.dt, k2w.points(pts)))
            target = next(it, None)

    fig, axes = plt.subplots(2, len(picked), figsize=(5 * len(picked), 10), squeeze=False)
    for col, (t, w) in enumerate(picked):
        o = outs[np.argmin(np.abs(outs[:, 0] - t))]
        h = hmd[np.argmin(np.abs(hmd[:, 0] - t))]
        # Side view along the user's facing direction is unknown: use x/y and z/y
        for row, (i, name) in enumerate(((0, "x"), (2, "z"))):
            ax = axes[row, col]
            sub = w[:: max(1, len(w) // 4000)]
            ax.scatter(sub[:, i], sub[:, 1], s=1, c="0.6")
            ax.scatter([h[1 + i]], [h[2]], c="k", marker="s", label="hmd")
            ax.scatter([o[1 + i]], [o[2]], c="C0", s=60, label="waist")
            ax.scatter([o[4 + i]], [o[5]], c="C1", s=60, label="left ankle")
            ax.scatter([o[7 + i]], [o[8]], c="C2", s=60, label="right ankle")
            ax.axhline(0, color="k", lw=0.5)
            ax.set_aspect("equal")
            ax.set_xlabel(name)
            ax.set_title(f"t={t:.1f}  knees {o[10]:.0f}/{o[11]:.0f}")
        axes[0, col].legend(loc="upper right", fontsize=7)
    plt.tight_layout()
    plt.savefig(args.out, dpi=55)


if __name__ == "__main__":
    main()
