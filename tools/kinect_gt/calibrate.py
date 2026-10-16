"""Kinect -> SteamVR calibration from a depth recording and the session recording.

The floor plane (scene.py) gives the vertical axis and the height. The remaining
yaw, horizontal translation and time offset come from the head: the top of the
user's silhouette, tracked in depth, follows the HMD. Solved by least squares
over the whole recording, with an unknown constant offset between the HMD and
the top-of-head centroid (in the HMD frame) and a grid search on the time offset.

Usage: python tools/kinect_gt/calibrate.py rec.depth scene.npz session.jsonl calib.npz
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from kinect_depth import DepthRecording  # noqa: E402

FOREGROUND_MM = 60  # closer than the background by this much: the user


class FloorFrame:
    """Right-handed frame on the Kinect floor: e1, up n, e3 = e1 x n."""

    def __init__(self, n, d):
        self.n, self.d = n, d
        e1 = np.array([1.0, 0.0, 0.0]) - n[0] * n
        self.e1 = e1 / np.linalg.norm(e1)
        self.e3 = np.cross(self.e1, n)

    def coords(self, p):
        """(N, 3) camera points -> (a, h, c): floor coordinates, h height above the floor."""
        return np.stack([p @ self.e1, p @ self.n + self.d, p @ self.e3], axis=1)


def user_points(rec, depth, background, min_pixels=800):
    """Camera-space points of the largest foreground blob (the user), or None."""
    valid = (depth > 0) & (background > 0) & (depth.astype(np.int32) < background.astype(np.int32) - FOREGROUND_MM)
    valid = ndimage.binary_opening(valid, iterations=1)
    labels, count = ndimage.label(valid)
    if count == 0:
        return None
    sizes = ndimage.sum(valid, labels, index=np.arange(1, count + 1))
    best = int(np.argmax(sizes)) + 1
    if sizes[best - 1] < min_pixels:
        return None
    mask = labels == best
    d = depth[mask].astype(np.float32) * 0.001
    ray = rec.table[mask]
    return np.stack([ray[:, 0] * d, ray[:, 1] * d, d], axis=1)


def head_track(rec, scene, frame, step):
    """Per sampled frame: host time and the top-of-head centroid in floor coordinates."""
    out = []
    for i, (t, rt, depth) in enumerate(rec.frames()):
        if i % step:
            continue
        pts = user_points(rec, depth, scene["background"])
        if pts is None:
            continue
        f = frame.coords(pts)
        top = f[:, 1].max()
        if top < 0.8:  # not standing or sitting upright: the top is not the head
            continue
        head = f[f[:, 1] > top - 0.20]
        if len(head) < 50:
            continue
        out.append([t, head[:, 0].mean(), top, head[:, 2].mean()])
    return np.array(out)


def load_hmd(session):
    rows = []
    with open(session, encoding="utf-8") as fh:
        for line in fh:
            if '"k":"dev"' not in line or '"cls":1,' not in line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if d["ok"]:
                rows.append([d["t"], *d["p"], *d["q"]])
    return np.array(rows)


def interp_hmd(hmd, t):
    p = np.stack([np.interp(t, hmd[:, 0], hmd[:, k]) for k in (1, 2, 3)], axis=1)
    i = np.clip(np.searchsorted(hmd[:, 0], t), 0, len(hmd) - 1)
    q = hmd[i, 4:8]  # nearest is fine at 90 Hz
    return p, Rotation.from_quat(q[:, [1, 2, 3, 0]])


def solve(head, hmd, dt):
    """Least squares on yaw, translation (x, z) and the HMD-frame offset."""
    p, rot = interp_hmd(hmd, head[:, 0] + dt)
    a, c = head[:, 1], head[:, 3]

    def residuals(x):
        yaw, tx, tz, ox, oy, oz = x
        cy, sy = np.cos(yaw), np.sin(yaw)
        # Rotation about +Y applied to (a, h, c)
        wx = cy * a + sy * c + tx
        wz = -sy * a + cy * c + tz
        target = p + rot.apply([ox, oy, oz])
        return np.concatenate([wx - target[:, 0], wz - target[:, 2]])

    best = None
    for yaw0 in np.linspace(-np.pi, np.pi, 12, endpoint=False):
        r = least_squares(residuals, [yaw0, 0, 0, 0, 0, 0.05], loss="soft_l1", f_scale=0.05)
        if best is None or r.cost < best.cost:
            best = r
    res = residuals(best.x).reshape(2, -1)
    err = np.hypot(res[0], res[1])
    return best.x, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("depth")
    ap.add_argument("scene")
    ap.add_argument("session")
    ap.add_argument("out")
    ap.add_argument("--step", type=int, default=3)
    args = ap.parse_args()

    rec = DepthRecording(args.depth)
    scene = np.load(args.scene)
    frame = FloorFrame(scene["floor_n"], float(scene["floor_d"]))
    head = head_track(rec, scene, frame, args.step)
    hmd = load_hmd(args.session)
    overlap = (head[:, 0] > hmd[0, 0]) & (head[:, 0] < hmd[-1, 0])
    head = head[overlap]
    print(f"{len(head)} head samples, {len(hmd)} HMD poses")

    results = []
    for dt in np.arange(-0.30, 0.31, 0.03):
        x, err = solve(head, hmd, dt)
        results.append((np.median(err), dt, x, err))
        print(f"  dt {dt:+.2f} s: median {np.median(err) * 100:.1f} cm")
    med, dt, x, err = min(results, key=lambda r: r[0])
    # Refine the time offset around the best
    for d2 in np.arange(dt - 0.03, dt + 0.031, 0.005):
        x2, e2 = solve(head, hmd, d2)
        if np.median(e2) < med:
            med, dt, x, err = np.median(e2), d2, x2, e2
    yaw, tx, tz, ox, oy, oz = x
    # Head height check: top of the silhouette vs HMD height
    p, _ = interp_hmd(hmd, head[:, 0] + dt)
    print(f"best: dt {dt:+.3f} s, yaw {np.degrees(yaw):.1f} deg, t ({tx:.3f}, {tz:.3f}), "
          f"HMD offset ({ox:.3f}, {oy:.3f}, {oz:.3f}); horizontal error median {med * 100:.1f} cm, "
          f"p90 {np.percentile(err, 90) * 100:.1f} cm; head top - HMD y median "
          f"{np.median(head[:, 2] - p[:, 1]) * 100:.1f} cm")
    np.savez(args.out, floor_n=frame.n, floor_d=frame.d, e1=frame.e1, e3=frame.e3, yaw=yaw, t=[tx, tz], dt=dt,
             hmd_offset=[ox, oy, oz], median_error=med)


if __name__ == "__main__":
    main()
