"""Dataset for the learned foot model: causal features from the body solver
inputs, Kinect ground-truth ankle targets.

Inputs come from `slimelite_ml_dump session.jsonl out.bin` (the solver inputs as
the engine builds them, one row per engine frame). Everything is expressed in a
heading-normalized frame (yaw of the pelvis, taken from its left-right axis so
that bending forward or leaning back does not change it), with the horizontal
origin at the current HMD position and the vertical origin at the floor.

Features at time t: for each lag in LAGS (s), the frame at or before t - lag:
  HMD position (x, z relative to the HMD now, y above the floor), the up (Y) and
  back (Z) axes of the HMD, pelvis and both shins, both shins' world linear
  accelerations (clipped, causal low-pass) and angular speeds.
The same computation is implemented in core/src/foot_model.cpp (FootModel).

Targets: both ankles, x/z relative to the HMD, y above the floor, in the
heading frame (meters).
"""
import json
import os

import numpy as np

REC = os.path.join(os.environ.get("LOCALAPPDATA", ""), "SlimeVR-lite", "recordings")
ML = os.path.join(REC, "ml")

ACC_CLIP = 40.0  # m/s^2, per axis, before the low-pass (impacts reach 60-90)
ACC_TAU = 0.04  # s, accelerometer low-pass


def default_lags():
    # 0.8 s at 30 Hz, then a sparse tail up to 2 s
    return [round(k / 30, 4) for k in range(24)] + [0.9, 1.0, 1.2, 1.4, 1.7, 2.0]


def load_dump(path=os.path.join(ML, "s232318.bin")):
    meta = json.load(open(path + ".json"))
    cols = meta["columns"]
    d = np.fromfile(path).reshape(-1, len(cols))
    return {c: d[:, i] for i, c in enumerate(cols)}, meta


def load_gt(paths):
    rows = []
    for p in paths:
        if os.path.exists(p):
            g = np.genfromtxt(p, delimiter=",", names=True, invalid_raise=False)
            rows.append(g)
    g = np.concatenate(rows)
    g = g[np.argsort(g["t"], kind="stable")]
    keep = np.concatenate([[True], np.diff(g["t"]) > 1e-3])
    return g[keep]


def quat_to_matrix(w, x, y, z):
    """(N,) arrays -> (N, 3, 3)"""
    R = np.empty((len(w), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def ema(x, t, tau):
    """Causal exponential low-pass with dt-dependent gain, per frame (as C++)."""
    out = np.empty_like(x)
    acc = x[0].copy()
    out[0] = acc
    dts = np.diff(t, prepend=t[0])
    for i in range(1, len(x)):
        dt = min(max(dts[i], 0.0), 0.1)
        acc += (x[i] - acc) * (1.0 - np.exp(-dt / tau))
        out[i] = acc
    return out


class Frames:
    """Per-frame quantities in world space (computed once per session)."""

    def __init__(self, d):
        self.t = d["t"]
        n = len(self.t)
        self.valid = (d["hmd_valid"] > 0) & (d["has_waist"] > 0) & (d["l_valid"] > 0) & (d["r_valid"] > 0)
        self.hmd = np.stack([d["hx"], d["hy"] - d["floor"], d["hz"]], axis=1)
        self.R = {
            "hmd": quat_to_matrix(d["hqw"], d["hqx"], d["hqy"], d["hqz"]),
            "pelvis": quat_to_matrix(d["wqw"], d["wqx"], d["wqy"], d["wqz"]),
            "l": quat_to_matrix(d["l_qw"], d["l_qx"], d["l_qy"], d["l_qz"]),
            "r": quat_to_matrix(d["r_qw"], d["r_qx"], d["r_qy"], d["r_qz"]),
        }
        self.acc = {}
        for s in ("l", "r"):
            a = np.clip(np.stack([d[f"{s}_ax"], d[f"{s}_ay"], d[f"{s}_az"]], axis=1), -ACC_CLIP, ACC_CLIP)
            self.acc[s] = ema(a, self.t, ACC_TAU)
        self.gyro = {"l": d["l_gyro"], "r": d["r_gyro"]}
        # Heading from the pelvis right axis: forward = Y x right = (rz, 0, -rx)
        right = self.R["pelvis"][:, :, 0]
        self.heading = np.arctan2(-right[:, 2], right[:, 0])
        self.floor = d["floor"]
        self.n = n

    def index_at(self, times):
        """Last frame at or before each time (-1 if none)."""
        return np.searchsorted(self.t, times, side="right") - 1


DR_STILL_GYRO = 0.3  # rad/s
DR_STILL_ACC = 0.6  # m/s^2
DR_ZUPT_AFTER = 0.15  # s still before the velocity is zeroed
DR_LEAK = 20.0  # s, height memory


def dead_reckoning(fr):
    """Per frame and leg, vertical dead reckoning with zero-velocity resets:
    (height change memory py, still time saturated at 3 s) -> (n, 4):
    py_l, py_r, still_l, still_r. Long memory a short window cannot have (a foot
    raised onto a bed seconds ago)."""
    out = np.zeros((fr.n, 4))
    dts = np.clip(np.diff(fr.t, prepend=fr.t[0]), 0.0, 0.1)
    for k, leg in enumerate("lr"):
        a = fr.acc[leg]
        ay = a[:, 1]
        anorm = np.linalg.norm(a, axis=1)
        g = fr.gyro[leg]
        v = py = still = 0.0
        for i in range(fr.n):
            dt = dts[i]
            if g[i] < DR_STILL_GYRO and anorm[i] < DR_STILL_ACC:
                still += dt
            else:
                still = 0.0
            if still >= DR_ZUPT_AFTER:
                v = 0.0
            else:
                v += ay[i] * dt
            py += v * dt - py * dt / DR_LEAK
            py = min(max(py, -1.0), 1.0)
            out[i, k] = py
            out[i, 2 + k] = min(still, 3.0)
    return out


def to_heading(v, c, s):
    """World vectors (N, 3) into the heading frame given cos/sin of the heading:
    v_h = Ry(heading)^T v"""
    x = c * v[:, 0] - s * v[:, 2]
    z = s * v[:, 0] + c * v[:, 2]
    return np.stack([x, v[:, 1], z], axis=1)


def from_heading(v, c, s):
    x = c * v[:, 0] + s * v[:, 2]
    z = -s * v[:, 0] + c * v[:, 2]
    return np.stack([x, v[:, 1], z], axis=1)


FEATS_PER_LAG = 35


def features(fr, idx, lags):
    """Features for the frames idx (int array). Returns X (N, F) and ok (N,):
    every lag frame exists and is valid."""
    N = len(idx)
    t_now = fr.t[idx]
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    hmd_now = fr.hmd[idx]
    X = np.empty((N, len(lags) * FEATS_PER_LAG), np.float64)
    ok = fr.valid[idx].copy()
    for li, lag in enumerate(lags):
        j = fr.index_at(t_now - lag + 1e-9) if lag > 0 else idx
        ok &= j >= 0
        j = np.maximum(j, 0)
        ok &= fr.valid[j]
        cols = []
        p = to_heading(fr.hmd[j] - hmd_now * np.array([1, 0, 1]), c, s)
        cols.append(p)
        for seg in ("hmd", "pelvis", "l", "r"):
            R = fr.R[seg][j]
            cols.append(to_heading(R[:, :, 1], c, s))
            cols.append(to_heading(R[:, :, 2], c, s))
        for side in ("l", "r"):
            cols.append(to_heading(fr.acc[side][j], c, s))
        cols.append(np.stack([fr.gyro["l"][j], fr.gyro["r"][j]], axis=1))
        X[:, li * FEATS_PER_LAG:(li + 1) * FEATS_PER_LAG] = np.concatenate(cols, axis=1)
    return X, ok


def targets(fr, idx, gt_rows):
    """Ankle targets in the heading frame of frames idx; (N, 6) and masks (N, 2)."""
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    hmd_now = fr.hmd[idx]
    Y = np.zeros((len(idx), 6))
    M = np.zeros((len(idx), 2), bool)
    for k, leg in enumerate(("l", "r")):
        g = np.stack([gt_rows[f"{leg}_x"], gt_rows[f"{leg}_y"] - fr.floor[idx], gt_rows[f"{leg}_z"]], axis=1)
        m = gt_rows[f"{leg}_valid"] > 0
        g = np.where(m[:, None], g, 0.0)
        Y[:, 3 * k:3 * k + 3] = to_heading(g - hmd_now * np.array([1, 0, 1]), c, s)
        M[:, k] = m
    return Y, M


def to_world(fr, idx, Yh):
    """Heading-frame predictions (N, 6) back to world (y above the floor)."""
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    off = fr.hmd[idx] * np.array([1, 0, 1])
    return np.concatenate([from_heading(Yh[:, 0:3], c, s) + off, from_heading(Yh[:, 3:6], c, s) + off], axis=1)


def mirror_features(X, lags):
    """Left/right mirror (x -> -x, swap legs), for augmentation."""
    X = X.reshape(len(X), len(lags), FEATS_PER_LAG).copy()
    out = X.copy()
    # layout: p(3) hmdY hmdZ pelY pelZ lY lZ rY rZ (each 3) accL accR (3) gyroL gyroR
    vec_slots = [(0, 3)] + [(3 + 3 * i, 3 + 3 * i + 3) for i in range(10)]
    for a, b in vec_slots:
        out[:, :, a] = -X[:, :, a]
    # swap shins (slots 3..6 of the 10 axis vectors -> offsets 15..26), accel, gyro
    sl = lambda a, b: slice(a, b)  # noqa: E731
    for (a, b), (c, d) in (((15, 21), (21, 27)), ((27, 30), (30, 33)), ((33, 34), (34, 35))):
        out[:, :, sl(a, b)], out[:, :, sl(c, d)] = out[:, :, sl(c, d)].copy(), out[:, :, sl(a, b)].copy()
    return out.reshape(len(X), -1)


def mirror_targets(Y, M):
    Y2 = np.concatenate([Y[:, 3:6], Y[:, 0:3]], axis=1)
    Y2[:, 0] *= -1
    Y2[:, 3] *= -1
    return Y2, M[:, ::-1].copy()


# Re-extracted ground truth (controllers masked out, spikes invalidated); the
# first extraction (_gt.csv + _gt_part2.csv) picked hands as shins at times
GT_V2 = os.path.join(REC, "gt", "kinect_20261016_235326_gt_v2.csv")
GT_V1 = [os.path.join(REC, "gt", "kinect_20261016_235326_gt.csv"),
         os.path.join(REC, "gt", "kinect_20261016_235326_gt_part2.csv")]
GT_FILES = [GT_V2] if os.path.exists(GT_V2) else GT_V1


def split_of(t, t0, block=300.0):
    """Contiguous 5 min blocks: 0 train, 1 val, 2 test (pattern T T V T Te)."""
    b = np.floor((t - t0) / block).astype(int)
    pattern = np.array([0, 0, 1, 0, 2])
    sp = pattern[b % 5]
    # 3 s guard at block edges (temporal correlation of the windows)
    edge = ((t - t0) % block < 3.0) | ((t - t0) % block > block - 3.0)
    sp[edge] = -1
    return sp


def build(lags=None, dump=None, gt_files=None):
    gt_files = gt_files or GT_FILES
    lags = lags or default_lags()
    d, meta = load_dump(dump) if dump else load_dump()
    fr = Frames(d)
    gt = load_gt(gt_files)
    idx = fr.index_at(gt["t"])
    keep = (idx >= 0) & (np.abs(fr.t[np.maximum(idx, 0)] - gt["t"]) < 0.05)
    gt, idx = gt[keep], idx[keep]
    X, ok = features(fr, idx, lags)
    Y, M = targets(fr, idx, gt)
    M &= ok[:, None]
    Mc = clean_mask(gt["t"], Y, M)
    Mc &= kinematic_mask(fr, idx, Y, meta)
    cam = camera_dirs(fr, idx, gt)
    DR = dead_reckoning(fr)[idx]
    rec = {"l": np.stack([d["rec_lx"], d["rec_ly"] - d["floor"], d["rec_lz"]], axis=1)[idx],
           "r": np.stack([d["rec_rx"], d["rec_ry"] - d["floor"], d["rec_rz"]], axis=1)[idx]}
    return dict(fr=fr, d=d, meta=meta, gt=gt, idx=idx, X=X, Y=Y, M=M, Mc=Mc, cam=cam, rec=rec, lags=lags, DR=DR)


# Kinect camera position in SteamVR space (recordings/gt/*_calib.npz via
# tools/kinect_gt/preview.py KinectToWorld.points(0))
CAMERA = np.array([1.46257241, 1.92838919, 1.48353078])

OUTLIER = 0.15  # m from the local median of the ground truth track
MAX_HEIGHT = 1.0  # m, ankle above the floor
MAX_REACH = 1.2  # m, horizontal ankle distance from the HMD


def clean_mask(t, Y, M, half_window=0.35):
    """Ground-truth outliers: isolated jumps (far from the median of the samples
    within +-half_window s) and implausible positions."""
    Mc = M.copy()
    for k in range(2):
        m = M[:, k]
        y = Y[:, 3 * k:3 * k + 3]
        Mc[:, k] &= (y[:, 1] < MAX_HEIGHT) & (np.hypot(y[:, 0], y[:, 2]) < MAX_REACH)
        vi = np.flatnonzero(Mc[:, k])
        tv = t[vi]
        lo = np.searchsorted(tv, tv - half_window)
        hi = np.searchsorted(tv, tv + half_window, side="right")
        bad = np.zeros(len(vi), bool)
        for n in range(len(vi)):
            if hi[n] - lo[n] < 4:
                bad[n] = True  # too sparse to tell
                continue
            med = np.median(y[vi[lo[n]:hi[n]]], axis=0)
            bad[n] = np.linalg.norm(y[vi[n]] - med) > OUTLIER
        Mc[vi[bad], k] = False
    return Mc


def kinematic_mask(fr, idx, Y, meta, margin=0.10):
    """Ground-truth ankles the body cannot reach: the knee (ankle + shin along the
    IMU shin axis) above the highest possible hip joint (HMD height minus the
    standing eye-to-hip distance, scaled by the pelvis tilt) by more than margin,
    and both feet high while the head is at standing height."""
    h = meta["userHeight"]
    eye = meta.get("eyeHeight", 0.936 * h)
    eye_to_hip = eye - (0.039 + 0.246 + 0.245) * h  # same ratios as BodyDimensions
    pel_up_y = fr.R["pelvis"][idx][:, 1, 1]
    hip_max = fr.hmd[idx, 1] - eye_to_hip * np.maximum(pel_up_y, 0.3)
    ok = np.ones((len(idx), 2), bool)
    for k, leg in enumerate("lr"):
        knee = Y[:, 3 * k + 1] + 0.246 * h * fr.R[leg][idx][:, 1, 1]
        ok[:, k] = knee < hip_max + margin
    both = (Y[:, 1] > 0.2) & (Y[:, 4] > 0.2) & (fr.hmd[idx, 1] > eye - 0.2)
    ok[both] = False
    return ok


def camera_dirs(fr, idx, gt):
    """Horizontal unit vectors from each ground-truth ankle to the camera, heading
    frame (N, 6): the extraction is biased toward the camera."""
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    out = []
    for leg in "lr":
        v = CAMERA - np.stack([np.nan_to_num(gt[f"{leg}_x"]), 0 * c, np.nan_to_num(gt[f"{leg}_z"])], axis=1)
        v[:, 1] = 0
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        out.append(to_heading(v, c, s))
    return np.concatenate(out, axis=1)


def first_time(session):
    with open(session, encoding="utf-8") as fh:
        for line in fh:
            try:
                return json.loads(line)["t"]
            except (json.JSONDecodeError, KeyError):
                continue


def replay_at(replay_csv, session, fr, idx):
    """Replayed solver ankles (slimelite_body_replay CSV) at frames idx, heading frame (N, 6)."""
    rp = np.genfromtxt(replay_csv, delimiter=",", names=True, usecols=("t", "lx", "ly", "lz", "rx", "ry", "rz"))
    t_rp = rp["t"] + first_time(session)
    tt = fr.t[idx]
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    off = fr.hmd[idx] * np.array([1, 0, 1]) + np.stack([0 * tt, fr.floor[idx], 0 * tt], axis=1)
    out = []
    for leg in "lr":
        p = np.stack([np.interp(tt, t_rp, rp[f"{leg}{a}"]) for a in "xyz"], axis=1)
        p[(tt < t_rp[0]) | (tt > t_rp[-1])] = np.nan
        out.append(to_heading(p - off, c, s))
    return np.concatenate(out, axis=1)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Builds the training set (features, targets, solver baselines)")
    ap.add_argument("--name", default="v1")
    ap.add_argument("--replay", default=os.path.join(ML, "replay_0149.csv"))
    ap.add_argument("--session", default=os.path.join(REC, "session_20261016_232318.jsonl"))
    ap.add_argument("--dump", default=os.path.join(ML, "s232318.bin"))
    ap.add_argument("--extra-lags", default="", help="comma separated lags (s) added to the default ones")
    args = ap.parse_args()
    lags = default_lags() + [float(v) for v in args.extra_lags.split(",") if v]
    D = build(lags=lags, dump=args.dump)
    fr, idx = D["fr"], D["idx"]
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    off = fr.hmd[idx] * np.array([1, 0, 1])
    rec = np.concatenate([to_heading(D["rec"][leg] - off, c, s) for leg in "lr"], axis=1)
    rep = replay_at(args.replay, args.session, fr, idx) if os.path.exists(args.replay) else rec * np.nan
    np.savez_compressed(os.path.join(ML, f"ds_{args.name}.npz"), X=D["X"].astype(np.float32), Y=D["Y"], M=D["M"],
                        t=D["gt"]["t"], idx=idx, lags=np.array(D["lags"]), rec=rec, replay=rep,
                        Mc=D["Mc"], cam=D["cam"], DR=D["DR"])
    print(f"{len(idx)} samples, {D['M'].sum(0)} valid legs, {D['Mc'].sum(0)} after cleaning, t {D['gt']['t'][0]:.0f}-{D['gt']['t'][-1]:.0f}")


if __name__ == "__main__":
    main()
