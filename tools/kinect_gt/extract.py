"""Ground-truth ankle positions from the Kinect point cloud and the IMU shin axes.

For each depth frame (in SteamVR space, see calibrate.py), each shin's direction
is known from its IMU. Only the points below the hip (along the pelvis axis,
from the waist IMU and the engine's hip position) and away from the controllers
(hands hanging below the hips look like shins) are used. Seen along that direction, the shin's points collapse into
a small dense disc: the densest disc near the expected side of the pelvis (and
near the previous ground truth) locates the shin axis. The surface the Kinect
sees is the front of the leg, so the axis is pushed back by the leg radius. The
ankle is the lowest end of the points along the axis, minus the ankle height.
Frames where the leg is hidden (too few points, too short) are marked invalid.

Output CSV: t (session clock), then per leg (l, r): valid, x, y, z, points, extent

Usage: python tools/kinect_gt/extract.py rec.depth scene.npz calib.npz session.jsonl out.csv
           [--step N] [--from T] [--to T] [--height H]
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from calibrate import user_points  # noqa: E402
from kinect_depth import DepthRecording  # noqa: E402
from preview import KinectToWorld, load_outputs  # noqa: E402

LEG_RADIUS = 0.045  # m, shin surface to axis
DISC = 0.06  # m, radius around the axis counted as this shin
MIN_POINTS = 40
HAND_RADIUS = 0.22  # m around each controller: hands and forearms hanging below the hips look like shins
MIN_EXTENT = 0.15  # m of shin visible along the axis
FLOOR_CONTACT = 0.09  # m, lowest visible point below this: touching the floor


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qrot(q, v):
    w, u = q[0], q[1:]
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def axis_angle(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.concatenate([[np.cos(a / 2)], axis * np.sin(a / 2)])


FW_TO_WORLD = axis_angle([1, 0, 0], -np.pi / 2)


class Segments:
    """Segment orientations by role over time, as the engine computes them."""

    def __init__(self, session):
        cfg, heading = None, {}
        self.series = {"waist": ([], []), "left_ankle": ([], []), "right_ankle": ([], [])}
        with open(session, encoding="utf-8") as fh:
            for line in fh:
                if '"k":"rot"' not in line and '"k":"trk"' not in line and '"k":"cfg"' not in line:
                    continue
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if d["k"] == "cfg":
                    cfg = d["config"]
                elif d["k"] == "trk":
                    heading[d["mac"]] = (d["hh"], d["hdg"])
                elif cfg and d["mac"] in cfg["trackers"] and d.get("s", 0) == 0:
                    tc = cfg["trackers"][d["mac"]]
                    if not tc.get("mountingValid") or tc["role"] not in self.series:
                        continue
                    q = qmul(FW_TO_WORLD, np.array(d["q"]))
                    if tc.get("yawInMagFrame"):
                        hh, h = heading.get(d["mac"], (0, 0))
                        if not hh:
                            continue
                        q = qmul(axis_angle([0, 1, 0], -h), q)
                    q = qmul(qmul(axis_angle([0, 1, 0], tc["yawAlign"]), q), np.array(tc["mounting"]))
                    ts, qs = self.series[tc["role"]]
                    ts.append(d["t"])
                    qs.append(q)
        self.series = {k: (np.array(ts), np.array(qs)) for k, (ts, qs) in self.series.items()}

    def at(self, role, t):
        ts, qs = self.series[role]
        if len(ts) == 0:
            return None
        i = int(np.clip(np.searchsorted(ts, t), 0, len(ts) - 1))
        if abs(ts[i] - t) > 0.1:
            return None
        return qs[i]


def load_controllers(session):
    """Controller positions over time: {role: (times, positions)} (1 left, 2 right)."""
    rows = {1: [], 2: []}
    with open(session, encoding="utf-8") as fh:
        for line in fh:
            if '"k":"dev"' not in line or '"cls":2,' not in line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if d["ok"] and d["role"] in rows:
                rows[d["role"]].append([d["t"], *d["p"]])
    return {r: (np.array(v)[:, 0], np.array(v)[:, 1:]) for r, v in rows.items() if v}


def find_shin(points, down, side_pref, prev, camera):
    """Locates one shin in the cloud; returns (ankle, n_points, extent) or None."""
    # Coordinates in the plane perpendicular to the shin
    u = np.cross(down, [0.0, 1.0, 0.0] if abs(down[1]) < 0.9 else [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(down, u)
    pu, pv, ps = points @ u, points @ v, points @ down
    # Density on a 1.5 cm grid, smoothed over a shin's width
    res = 0.015
    iu = np.floor((pu - pu.min()) / res).astype(int)
    iv = np.floor((pv - pv.min()) / res).astype(int)
    grid = np.zeros((iu.max() + 1, iv.max() + 1))
    np.add.at(grid, (iu, iv), 1)
    grid = ndimage.gaussian_filter(grid, 2.0)
    peaks = (grid == ndimage.maximum_filter(grid, size=7)) & (grid > 3)
    cand = np.argwhere(peaks)
    if len(cand) == 0:
        return None
    best, best_score = None, -np.inf
    for ci, cj in cand:
        cu, cv = pu.min() + (ci + 0.5) * res, pv.min() + (cj + 0.5) * res
        near = (pu - cu) ** 2 + (pv - cv) ** 2 < DISC ** 2
        n = np.count_nonzero(near)
        if n < MIN_POINTS:
            continue
        s = ps[near]
        extent = np.percentile(s, 98) - np.percentile(s, 2)
        center = cu * u + cv * v
        score = n * min(extent, 0.5)
        # Prefer the expected side of the pelvis, and continuity with the last frame
        score *= np.exp(side_pref(center))
        if prev is not None:
            dist = np.linalg.norm((prev - (prev @ down) * down) - center)
            score *= np.exp(-dist / 0.15)
        if score > best_score:
            best, best_score = (cu, cv, near, n, extent), score
    if best is None:
        return None
    cu, cv, near, n, extent = best
    if extent < MIN_EXTENT:
        return None
    # The Kinect sees the near surface: push the axis away from the camera
    axis = cu * u + cv * v
    view = np.mean(points[near], axis=0) - camera
    view -= (view @ down) * down
    if np.linalg.norm(view) > 1e-6:
        axis = axis + view / np.linalg.norm(view) * LEG_RADIUS
    bottom = np.percentile(ps[near], 99)
    return axis + bottom * down, n, extent


def clean_spikes(path, window=3, tol=0.15):
    """Invalidates single-frame jumps: a valid ankle farther than `tol` from the
    median of its valid neighbors (+-window frames). Real lifts last longer and
    survive. Rewrites the CSV, returns the number of invalidated leg samples."""
    with open(path) as fh:
        header = fh.readline()
        lines = [ln.rstrip("\n").split(",") for ln in fh]
    dropped = 0
    for base in (1, 7):  # l_valid, r_valid columns
        valid = np.array([ln[base] == "1" for ln in lines])
        pos = np.array([[float(ln[base + k]) if valid[i] else np.nan for k in (1, 2, 3)]
                        for i, ln in enumerate(lines)]).reshape(-1, 3)
        bad = []
        for i in np.flatnonzero(valid):
            lo, hi = max(0, i - window), min(len(lines), i + window + 1)
            neigh = np.delete(pos[lo:hi], i - lo, axis=0)
            neigh = neigh[~np.isnan(neigh[:, 0])]
            if len(neigh) >= 2 and np.linalg.norm(pos[i] - np.median(neigh, axis=0)) > tol:
                bad.append(i)
        for i in bad:
            lines[i][base:base + 6] = ["0", "", "", "", "0", "0"]
        dropped += len(bad)
    with open(path, "w") as fh:
        fh.write(header)
        for ln in lines:
            fh.write(",".join(ln) + "\n")
    return dropped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("depth")
    ap.add_argument("scene")
    ap.add_argument("calib")
    ap.add_argument("session")
    ap.add_argument("out")
    ap.add_argument("--step", type=int, default=1)
    ap.add_argument("--from", dest="t_from", type=float, default=-np.inf)
    ap.add_argument("--to", dest="t_to", type=float, default=np.inf)
    ap.add_argument("--height", type=float, default=1.7, help="user height (m), for the ankle height")
    args = ap.parse_args()

    rec = DepthRecording(args.depth)
    scene = np.load(args.scene)
    k2w = KinectToWorld(args.calib)
    camera = k2w.points(np.zeros((1, 3)))[0]
    segs = Segments(args.session)
    outs, _ = load_outputs(args.session)
    controllers = load_controllers(args.session)
    ankle_height = 0.039 * args.height  # same ratio as the body solver

    prev = [None, None]
    rows = 0
    with open(args.out, "w") as out:
        out.write("t,l_valid,l_x,l_y,l_z,l_n,l_extent,r_valid,r_x,r_y,r_z,r_n,r_extent\n")
        for i, (t_host, rt, depth) in enumerate(rec.frames(args.t_from - 1, args.t_to + 1)):
            if i % args.step:
                continue
            t = t_host + k2w.dt
            if t < args.t_from or t > args.t_to:
                continue
            waist = segs.at("waist", t)
            shins = [segs.at("left_ankle", t), segs.at("right_ankle", t)]
            if waist is None or shins[0] is None or shins[1] is None:
                continue
            cam_pts = user_points(rec, depth, scene["background"])
            if cam_pts is None:
                continue
            pts = k2w.points(cam_pts)
            # Legs only: below the hip, along the pelvis axis (works lying down too)
            o = outs[int(np.clip(np.searchsorted(outs[:, 0], t), 0, len(outs) - 1))]
            hip = o[1:4]
            pelvis_up = qrot(waist, np.array([0.0, 1.0, 0.0]))
            pts = pts[(pts - hip) @ pelvis_up < -0.08]
            for ts, ps in controllers.values():
                i = int(np.clip(np.searchsorted(ts, t), 0, len(ts) - 1))
                if abs(ts[i] - t) < 0.1:
                    pts = pts[np.linalg.norm(pts - ps[i], axis=1) > HAND_RADIUS]
            if len(pts) < 2 * MIN_POINTS:
                continue
            right = qrot(waist, np.array([1.0, 0.0, 0.0]))
            center = pts.mean(axis=0)
            cols = [f"{t:.4f}"]
            for leg in (0, 1):
                down = qrot(shins[leg], np.array([0.0, -1.0, 0.0]))
                sign = -1.0 if leg == 0 else 1.0

                def side_pref(c, sign=sign, down=down):
                    lateral = right - (right @ down) * down
                    ref = center - (center @ down) * down
                    return 3.0 * np.tanh(sign * ((c - ref) @ lateral) / 0.05)

                found = find_shin(pts, down, side_pref, prev[leg], camera)
                if found is None:
                    prev[leg] = None
                    cols += ["0", "", "", "", "0", "0"]
                    continue
                ankle, n, extent = found
                # Background subtraction drops what is within FOREGROUND_MM of the
                # floor: a lowest visible point that close means the foot is on it
                if ankle[1] - k2w.floor_y < FLOOR_CONTACT:
                    ankle = ankle.copy()
                    ankle[1] = k2w.floor_y
                ankle = ankle - down * ankle_height
                prev[leg] = ankle
                cols += ["1", f"{ankle[0]:.4f}", f"{ankle[1]:.4f}", f"{ankle[2]:.4f}", str(n), f"{extent:.3f}"]
            out.write(",".join(cols) + "\n")
            rows += 1
    dropped = clean_spikes(args.out)
    print(f"{rows} frames written, {dropped} isolated spikes invalidated")


if __name__ == "__main__":
    main()
