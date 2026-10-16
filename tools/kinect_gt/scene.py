"""Static scene model of a Kinect depth recording: background depth and floor plane.

The user moves, so each pixel sees the room behind them most of the time: the
background is a high percentile of the depth over frames spread across the
recording. The floor is the largest plane of the background whose normal is
roughly the camera's up axis (RANSAC).

Usage: python tools/kinect_gt/scene.py rec.depth scene.npz [--preview out.png]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from kinect_depth import DepthRecording  # noqa: E402


def sample_frames(rec, count):
    """Every n-th frame, about `count` frames over the whole recording."""
    n_total = 0
    for _ in rec.frames():
        n_total += 1
    step = max(1, n_total // count)
    out = []
    for i, (t, rt, d) in enumerate(rec.frames()):
        if i % step == 0:
            out.append(d.copy())
    return np.stack(out), n_total


def background(depths):
    d = depths.astype(np.float32)
    d[d == 0] = np.nan
    with np.errstate(all="ignore"):
        bg = np.nanpercentile(d, 90, axis=0)
        seen = np.mean(~np.isnan(d), axis=0)
    bg[(seen < 0.3) | np.isnan(bg)] = 0
    return bg.astype(np.uint16)


def fit_floor(points, rng, iterations=3000, tol=0.02):
    """RANSAC plane n.p + d = 0 with n pointing up (camera y is roughly up)."""
    best, best_count = None, 0
    n_pts = len(points)
    for _ in range(iterations):
        p = points[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[1] < 0:
            n = -n
        if n[1] < 0.7:  # within ~45 deg of the camera up axis
            continue
        d = -n @ p[0]
        count = np.count_nonzero(np.abs(points @ n + d) < tol)
        # The floor is the lowest large plane: prefer it over a table top
        if count > best_count:
            best, best_count = (n, d), count
    n, d = best
    inliers = points[np.abs(points @ n + d) < tol]
    # Least squares refinement on the inliers
    c = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - c)
    n = vt[2]
    if n[1] < 0:
        n = -n
    return n, -n @ c, len(inliers)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("depth")
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--preview")
    args = ap.parse_args()

    rec = DepthRecording(args.depth)
    depths, n_total = sample_frames(rec, args.frames)
    bg = background(depths)
    pts = rec.points(bg, step=2)
    rng = np.random.default_rng(0)
    n, d, count = fit_floor(pts, rng)
    # Camera height above the floor, camera tilt
    print(f"{n_total} frames; floor normal {np.round(n, 3)}, camera {d:.3f} m above it, {count} inliers, "
          f"tilt {np.degrees(np.arccos(n[1])):.1f} deg")
    np.savez(args.out, background=bg, floor_n=n, floor_d=d)

    if args.preview:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        full = rec.points(bg)
        h = np.full(bg.shape, np.nan, np.float32)
        valid = bg > 0
        h[valid] = full @ n + d
        fig, ax = plt.subplots(1, 2, figsize=(16, 6))
        ax[0].imshow(np.where(valid, bg, np.nan) / 1000.0, cmap="turbo")
        ax[0].set_title("background depth (m)")
        im = ax[1].imshow(h, cmap="coolwarm", vmin=-0.5, vmax=2.0)
        ax[1].set_title("height above the fitted floor (m)")
        plt.colorbar(im, ax=ax[1])
        plt.savefig(args.preview, dpi=60)


if __name__ == "__main__":
    main()
