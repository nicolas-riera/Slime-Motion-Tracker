"""Packs AMASS (SMPL+H G, *_poses.npz) into compact training arrays at 60 Hz.

  python tools/pose_model/amass_prep.py AMASS_ROOT BODY_MODELS OUT.npz [--datasets A B ...]

Per frame: the 22 body joint axis-angles (66) and the root translation (3).
Per sequence: start, length, dataset index, and the rest joint positions (22, 3) of
the subject (gender-specific SMPL+H model with the sequence betas), so the training
script can run the forward kinematics and synthesize the sensors on the GPU.
"""
import argparse
import glob
import os

import numpy as np
from scipy.spatial.transform import Rotation as R

HZ = 60


def rest_joints(model, betas):
    v = model["v_template"] + model["shapedirs"][:, :, :len(betas)] @ betas
    return (model["J_regressor"][:22] @ v).astype(np.float32)


def resample(poses, trans, fr):
    """Body poses (N, 66) and translations to 60 Hz (stride or slerp-like nlerp)."""
    if abs(fr / HZ - round(fr / HZ)) < 1e-3 and round(fr / HZ) >= 1:
        s = int(round(fr / HZ))
        return poses[::s], trans[::s]
    n = len(poses)
    tq = np.arange(0, (n - 1) / fr, 1.0 / HZ)
    x = tq * fr
    i0 = np.clip(np.floor(x).astype(int), 0, n - 2)
    a = (x - i0)[:, None]
    q = R.from_rotvec(poses.reshape(-1, 3)).as_quat().reshape(n, 22, 4)
    q0, q1 = q[i0], q[i0 + 1]
    q1 = np.where((q0 * q1).sum(-1, keepdims=True) < 0, -q1, q1)
    qi = q0 * (1 - a[:, None]) + q1 * a[:, None]
    qi /= np.linalg.norm(qi, axis=-1, keepdims=True)
    p = R.from_quat(qi.reshape(-1, 4)).as_rotvec().reshape(len(tq), 66)
    t = trans[i0] * (1 - a) + trans[i0 + 1] * a
    return p, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("body_models")
    ap.add_argument("out")
    ap.add_argument("--datasets", nargs="*")
    args = ap.parse_args()
    models = {g: dict(np.load(os.path.join(args.body_models, "smplh", g, "model.npz"), allow_pickle=True))
              for g in ("male", "female", "neutral")}
    names = args.datasets or sorted(d for d in os.listdir(args.root) if os.path.isdir(os.path.join(args.root, d)))
    poses, trans, seq = [], [], []
    start = 0
    for di, name in enumerate(names):
        files = sorted(glob.glob(os.path.join(args.root, name, "**", "*_poses.npz"), recursive=True))
        kept = 0
        for f in files:
            d = np.load(f, allow_pickle=True)
            if "mocap_framerate" not in d or "poses" not in d:
                continue
            p, t = resample(d["poses"][:, :66].astype(np.float64), d["trans"].astype(np.float64),
                            float(d["mocap_framerate"]))
            if len(p) < 30:
                continue
            g = str(d["gender"])
            g = g if g in models else "neutral"
            J = rest_joints(models[g], d["betas"][:16].astype(np.float64))
            poses.append(p.astype(np.float32))
            trans.append(t.astype(np.float32))
            seq.append((start, len(p), di, J))
            start += len(p)
            kept += 1
        print(f"{name}: {kept} sequences, {sum(s[1] for s in seq if s[2] == di) / HZ / 3600:.1f} h", flush=True)
    np.savez(args.out, poses=np.concatenate(poses), trans=np.concatenate(trans),
             seq_start=np.array([s[0] for s in seq], np.int64), seq_len=np.array([s[1] for s in seq], np.int64),
             seq_dataset=np.array([s[2] for s in seq], np.int16), rest=np.stack([s[3] for s in seq]),
             datasets=np.array(names))
    print(f"{len(seq)} sequences, {start / HZ / 3600:.1f} h -> {args.out}")


if __name__ == "__main__":
    main()
