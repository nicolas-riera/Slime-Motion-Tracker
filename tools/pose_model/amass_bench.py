"""Benchmark of the body solver on AMASS motion converted to our sensors.

  python tools/pose_model/amass_bench.py amass.npz BODY_MODELS work_dir
         [--per-cat N] [--variants JSON] [--pose-model model.bin] [--ankles]

Segments of the packed AMASS (amass_prep.py) are picked by what the body does (a
foot held up while standing, seated, kneeling, lying, walking). Each is converted to what our hardware sees (HMD,
controllers, shin and waist tracker orientations and accelerations, as in
train_pose.py) in the body_replay --dump layout, run through the C++ solver
(slimelite_dump_solve, optionally with SL_TUNE variants and the pose model), and
scored against the exact SMPL ankles: median 3D error, and the error while a foot
is raised (> 15 cm above its floor height).
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import sys

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import train_pose as tp  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
EXE = os.path.join(ROOT, "build", "Release", "slimelite_dump_solve.exe")
M = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], np.float64)  # Rx90^T: AMASS -> ours
RY180 = np.diag([-1.0, 1.0, -1.0])
HEAD_OFFSET = np.array([0.0, -0.08, 0.10])  # features.py: head joint in the HMD frame
WRIST_OFFSET = np.array([0.0, 0.0, 0.08])
COLS = ["t", "hmd_valid", "hx", "hy", "hz", "hqw", "hqx", "hqy", "hqz", "floor", "eye_h", "height",
        "w_valid", "wqw", "wqx", "wqy", "wqz", "wax", "way", "waz",
        "l_valid", "lqw", "lqx", "lqy", "lqz", "lax", "lay", "laz",
        "r_valid", "rqw", "rqx", "rqy", "rqz", "rax", "ray", "raz",
        "c0_valid", "c0x", "c0y", "c0z", "c0qw", "c0qx", "c0qy", "c0qz",
        "c1_valid", "c1x", "c1y", "c1z", "c1qw", "c1qx", "c1qy", "c1qz",
        "out_valid", "o_wx", "o_wy", "o_wz", "o_lx", "o_ly", "o_lz", "o_rx", "o_ry", "o_rz"]


def find_segments(amass_npz, per_cat, seconds=12.0):
    """Segments of the packed AMASS (amass_prep.py) by what the body does."""
    z = np.load(amass_npz)
    poses, trans, rest = z["poses"], z["trans"], z["rest"]
    starts, lens = z["seq_start"], z["seq_len"]
    found = {k: [] for k in ("raised_hold", "seated", "kneel", "lying", "walking", "running")}
    rng = np.random.default_rng(0)
    order = rng.permutation(len(starts))
    half = int(seconds * 30)
    for si in order:
        a, n = int(starts[si]), int(lens[si])
        if n < 2 * half:
            continue
        with torch.no_grad():
            G = tp.fk_rot(tp.aa2mat(torch.from_numpy(poses[a:a + n].astype(np.float64)).reshape(n, 22, 3)))
            P = (tp.fk_pos(G, torch.from_numpy(rest[si].astype(np.float64))[None])
                 + torch.from_numpy(trans[a:a + n].astype(np.float64))[:, None]).numpy()
        floor = np.percentile(np.minimum(P[:, 7, 2], P[:, 8, 2]), 2)
        stand_head = rest[si][15, 1] - rest[si][7, 1]
        head = (P[:, 15, 2] - floor) / stand_head
        pelvis = (P[:, 0, 2] - floor) / (rest[si][0, 1] - rest[si][7, 1])
        ank = np.stack([P[:, 7, 2], P[:, 8, 2]], 1) - floor
        knee = np.stack([P[:, 4, 2], P[:, 5, 2]], 1) - floor
        speed = np.r_[0, np.linalg.norm(np.diff(P[:, 0, :2], axis=0), axis=1) * 60]
        masks = {
            "raised_hold": (head > 0.85) & (ank.max(1) > 0.25) & (ank.min(1) < 0.06),
            "seated": (head > 0.45) & (head < 0.8) & (pelvis < 0.65) & (knee.min(1) > 0.25),
            "kneel": (knee.min(1) < 0.12) & (head > 0.4),
            "lying": head < 0.35,
            "walking": (head > 0.9) & (speed > 0.8) & (speed < 1.8),
            "running": (head > 0.85) & (speed > 1.8),
        }
        need = {"raised_hold": 120, "seated": 180, "kneel": 120, "lying": 180, "walking": 180, "running": 60}
        for cat, m in masks.items():
            if len(found[cat]) >= per_cat:
                continue
            # longest run of the condition
            idx = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
            runs = list(zip(idx[::2], idx[1::2]))
            if not runs:
                continue
            b, e = max(runs, key=lambda r: r[1] - r[0])
            if e - b < need[cat]:
                continue
            mid = (b + e) // 2
            lo, hi = max(0, mid - half), min(n, mid + half)
            found[cat].append((int(si), int(a + lo), int(a + hi)))
        if all(len(v) >= per_cat for v in found.values()):
            break
    return found, z


def quat_wxyz(m):
    q = R.from_matrix(m).as_quat()
    return np.concatenate([q[:, 3:], q[:, :3]], 1)


def convert(poses, trans, rest, models, rng):
    """60 Hz poses (T, 66), translations, rest joints -> dump rows and ground truth."""
    poses, trans, rest = (np.asarray(a, np.float64) for a in (poses, trans, rest))
    T = len(poses)
    with torch.no_grad():
        G = tp.fk_rot(tp.aa2mat(torch.from_numpy(poses).reshape(T, 22, 3)))
        P = (tp.fk_pos(G, torch.from_numpy(rest)[None]) + torch.from_numpy(trans)[:, None]).numpy()
        G = G.numpy()
    # Body size: head-to-ankle rest distance, as a fraction of the height (neutral template)
    nm = models["neutral"]
    nJ = nm["J_regressor"][:22] @ nm["v_template"]
    ratio = (nJ[15, 1] - nJ[7, 1]) / (nm["v_template"][:, 1].max() - nm["v_template"][:, 1].min())
    height = (rest[15, 1] - rest[7, 1]) / ratio
    ankle_h = 0.039 * height
    floor = np.percentile(np.minimum(P[:, 7, 2], P[:, 8, 2]), 2) - ankle_h
    eye_h = rest[15, 1] - rest[7, 1] + ankle_h + 0.08

    def pos(p):
        q = p @ M.T
        q[:, 1] -= floor
        return q

    def rot(Gj):
        return M @ Gj @ RY180

    head_r = rot(G[:, 15])
    # The solver's calibration puts the hip under the head pivot; the SMPL head
    # joint sits a few cm in front of the pelvis at rest: shift the HMD back by as
    # much so a still standing body is solved where it is (the user's own geometry
    # was fitted on Kinect ground truth)
    lean = rest[15, 2] - rest[0, 2]  # SMPL +Z forward
    hmd = pos(P[:, 15]) - np.einsum("nij,j->ni", head_r, HEAD_OFFSET - np.array([0.0, 0.0, lean]))
    cols = {c: np.zeros(T) for c in COLS}
    cols["t"] = np.arange(T) / 60.0
    cols["hmd_valid"][:] = 1
    cols["hx"], cols["hy"], cols["hz"] = hmd.T
    for k, c in zip(range(4), "wxyz"):
        cols["hq" + c] = quat_wxyz(head_r)[:, k]
    cols["floor"][:] = 0
    cols["eye_h"][:] = eye_h
    cols["height"][:] = height

    def acc(p):  # world linear acceleration (ours), 60 Hz central difference + noise
        a = np.zeros_like(p)
        a[1:-1] = (p[2:] + p[:-2] - 2 * p[1:-1]) * 3600.0
        return a + rng.normal(0, 0.3, a.shape)

    for name, knee, ankle in (("l", 4, 7), ("r", 5, 8)):
        off = 0.8 * (rest[ankle] - rest[knee]) + np.array([0, 0, 0.04])
        sp = P[:, knee] + np.einsum("nij,j->ni", G[:, knee], off)
        cols[name + "_valid"][:] = 1
        q = quat_wxyz(rot(G[:, knee]))
        for k, c in enumerate("wxyz"):
            cols[name + "q" + c] = q[:, k]
        cols[name + "ax"], cols[name + "ay"], cols[name + "az"] = acc(pos(sp)).T
    wp = P[:, 0] + np.einsum("nij,j->ni", G[:, 0], np.array([0, 0.05, 0.11]))
    cols["w_valid"][:] = 1
    q = quat_wxyz(rot(G[:, 0]))
    for k, c in enumerate("wxyz"):
        cols["wq" + c] = q[:, k]
    cols["wax"], cols["way"], cols["waz"] = acc(pos(wp)).T
    for name, j in (("c0", 20), ("c1", 21)):
        cr = np.einsum("ij,njk->nik", M, G[:, j])  # controller = wrist (calibration identity)
        cp = pos(P[:, j]) - np.einsum("nij,j->ni", cr, WRIST_OFFSET)
        cols[name + "_valid"][:] = 1
        cols[name + "x"], cols[name + "y"], cols[name + "z"] = cp.T
        q = quat_wxyz(cr)
        for k, c in enumerate("wxyz"):
            cols[name + "q" + c] = q[:, k]
    X = np.stack([cols[c] for c in COLS], 1).astype(np.float64)
    gt = {"l": pos(P[:, 7]), "r": pos(P[:, 8]), "pelvis": pos(P[:, 0]), "lknee": pos(P[:, 4]), "rknee": pos(P[:, 5]),
          "ankle_h": ankle_h}
    return X, gt


def score(out_path, gt, warm=60):
    cols = json.load(open(out_path + ".json"))["columns"]
    o = np.fromfile(out_path, np.float64).reshape(-1, len(cols))
    c = {n: o[:, k] for k, n in enumerate(cols)}
    ok = c["valid"] > 0
    ok[:warm] = False
    errs, raised = [], []
    for k, leg in enumerate("lr"):
        est = np.stack([c[leg + "x"], c[leg + "y"], c[leg + "z"]], 1)
        e = np.linalg.norm(est - gt[leg], axis=1)
        errs.append(e[ok])
        up = ok & (gt[leg][:, 1] > gt["ankle_h"] + 0.15)
        raised.append(e[up])
    e = np.concatenate(errs)
    r = np.concatenate(raised)
    return float(np.median(e)) if len(e) else np.nan, float(np.median(r)) if len(r) > 30 else np.nan, len(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("body_models")
    ap.add_argument("work")
    ap.add_argument("--per-cat", type=int, default=20)
    ap.add_argument("--variants", default='[{}]', help="JSON list of SL_TUNE dicts")
    ap.add_argument("--pose-model")
    ap.add_argument("--ankles", action="store_true")
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()
    os.makedirs(args.work, exist_ok=True)
    models = {g: dict(np.load(os.path.join(args.body_models, "smplh", g, "model.npz"), allow_pickle=True))
              for g in ("male", "female", "neutral")}
    rng = np.random.default_rng(0)
    items = []
    index = os.path.join(args.work, "segments.json")
    if os.path.exists(index):
        sel = json.load(open(index))
        z = np.load(args.root)
    else:
        sel, z = find_segments(args.root, args.per_cat)
        json.dump(sel, open(index, "w"))
    for cat, segs in sel.items():
        for si, a, b in segs:
            name = f"{cat}_{si}_{a}"
            dump = os.path.join(args.work, name + ".bin")
            gtp = os.path.join(args.work, name + "_gt.npz")
            if not os.path.exists(gtp):
                X, gt = convert(z["poses"][a:b], z["trans"][a:b], z["rest"][si], models, rng)
                X.tofile(dump)
                json.dump({"columns": COLS}, open(dump + ".json", "w"))
                np.savez(gtp, **gt)
            items.append((cat, name, dump, gtp))
    print(f"{len(items)} motions: " + ", ".join(f"{c} {len(v)}" for c, v in sel.items()), flush=True)
    variants = json.loads(args.variants)

    def run(job):
        vi, v, (cat, f, dump, gtp) = job
        env = dict(os.environ)
        env["SL_TUNE"] = ",".join(f"{k}={x}" for k, x in v.items())
        out = dump[:-4] + f"_out{vi}.bin"
        cmd = [EXE, dump, out] + (["--ankles"] if args.ankles else [])
        if args.pose_model:
            cmd += ["--pose-model", args.pose_model, "--wrist-identity"]
        subprocess.run(cmd, env=env, check=True)
        z = np.load(gtp)
        r = score(out, {"l": z["l"], "r": z["r"], "ankle_h": float(z["ankle_h"])})
        os.remove(out)
        os.remove(out + ".json")
        return vi, cat, r

    jobs = [(vi, v, it) for vi, v in enumerate(variants) for it in items]
    results = {}
    with ThreadPoolExecutor(args.workers) as pool:
        for vi, cat, r in pool.map(run, jobs):
            results.setdefault(vi, {}).setdefault(cat, []).append(r)
    for vi, v in enumerate(variants):
        res = results[vi]
        line = []
        for cat, rs in res.items():
            a = np.array([r[0] for r in rs])
            b = np.array([r[1] for r in rs])
            line.append(f"{cat} {np.nanmedian(a) * 100:.1f}" + (f"/up {np.nanmedian(b) * 100:.1f}" if np.isfinite(b).any() else ""))
        print(f"{v or 'default'}: " + "  ".join(line), flush=True)


if __name__ == "__main__":
    main()
