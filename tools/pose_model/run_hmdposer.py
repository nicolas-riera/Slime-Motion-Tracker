"""Runs HMD-Poser (MIT, github.com/Pico-AI-Team/HMD-Poser) offline on a session dump.

  python tools/pose_model/run_hmdposer.py inputs.bin out.npz --repo D:/.../HMD-Poser
         --stand T [--ckpt pretrained_model/pretrained_model_protocol1.pt]

inputs.bin: body_replay --dump. --stand: a body calibration stand capture time
(printed by body_replay as stand_capture), used to calibrate the controllers to the
wrists. Output: t (session seconds), joints (N, 22, 3) in the SteamVR frame.

Joint positions need the SMPL+H body model for the exact skeleton; without it an
approximate SMPL-like skeleton scaled to the user's height is used (legs from our
BodyDimensions ratios), anchored at the head.
"""
import argparse
import os
import sys

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import features  # noqa: E402

PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19]
# Approximate SMPL neutral rest offsets to the parent (m, 1.70 m tall, Y up, facing +Z)
OFFSETS = np.array([
    [0, 0, 0], [0.06, -0.09, 0], [-0.06, -0.09, 0], [0, 0.11, -0.02], [0.04, -0.38, 0], [-0.04, -0.38, 0],
    [0, 0.13, 0.01], [-0.01, -0.40, -0.04], [0.01, -0.40, -0.04], [0, 0.05, 0.03], [0.03, -0.06, 0.12],
    [-0.03, -0.06, 0.12], [0, 0.21, -0.03], [0.08, 0.12, -0.02], [-0.08, 0.12, -0.02], [0, 0.09, 0.05],
    [0.12, 0.04, -0.01], [-0.12, 0.04, -0.01], [0.26, 0, -0.02], [-0.26, 0, -0.02], [0.25, 0.01, 0],
    [-0.25, 0.01, 0]], dtype=np.float64)


def skeleton_smplh(path, height, betas=None):
    """Rest offsets to the parent from the SMPL+H body model, scaled to the user."""
    m = np.load(path, allow_pickle=True)
    v = m["v_template"]
    if betas is not None:
        v = v + m["shapedirs"][:, :, :len(betas)] @ betas
    J = m["J_regressor"] @ v
    off = np.array([J[j] - (J[PARENTS[j]] if PARENTS[j] >= 0 else J[j]) for j in range(22)])
    model_height = v[:, 1].max() - v[:, 1].min()
    return off * (height / model_height)


def skeleton(height):
    off = OFFSETS * (height / 1.70)
    # Legs: our BodyDimensions ratios (hip joint to knee, knee to ankle)
    for j in (4, 5):
        off[j] = [np.sign(off[j][0]) * 0.02 * height / 1.7, -0.245 * height, 0]
    for j in (7, 8):
        off[j] = [0, -0.246 * height, -0.02]
    return off


def sixd_to_mat(x):
    a, b = x[..., :3], x[..., 3:6]
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b - (a * b).sum(-1, keepdims=True) * a
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.stack([a, b, np.cross(a, b)], axis=-1)


def infer(net, X, chunk=1200, warm=300):
    out = np.zeros((len(X), 22 * 6), np.float32)
    with torch.no_grad():
        start = 0
        while start < len(X):
            a = max(0, start - warm)
            b = min(len(X), start + chunk)
            pose, _ = net(torch.from_numpy(X[a:b][None]))
            out[start:b] = pose[0, start - a:].numpy()
            start = b
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("out")
    ap.add_argument("--repo", required=True)
    ap.add_argument("--ckpt", default="pretrained_model/pretrained_model_protocol1.pt")
    ap.add_argument("--stand", type=float, required=True)
    ap.add_argument("--body-model", help="SMPL+H model.npz (neutral) for the exact skeleton")
    ap.add_argument("--chunk", type=int, default=1200, help="frames per inference chunk (the LSTM restarts)")
    ap.add_argument("--warm", type=int, default=300, help="warm-up frames before each chunk")
    ap.add_argument("--reuse", help="previous output: recompute the positions from its rotations, no inference")
    args = ap.parse_args()
    sys.path.insert(0, args.repo)
    from model.network import HMD_imu_HME_Universe

    d = features.load_dump(args.dump)
    f = features.build(d, args.stand)
    height = float(np.median(f["d"]["height"]))
    off = skeleton_smplh(args.body_model, height) if args.body_model else skeleton(height)
    if args.reuse:
        G = np.load(args.reuse)["global_rot"].astype(np.float64)
        n = len(G)
    else:
        net = HMD_imu_HME_Universe(135, 3, 256, 0.05, 8, 2)
        sd = torch.load(os.path.join(args.repo, args.ckpt), map_location="cpu", weights_only=False)
        net.load_state_dict(sd.get("state_dict", sd))
        net.eval()
        X = np.nan_to_num(f["X"])
        pose = infer(net, X, args.chunk, args.warm).reshape(-1, 22, 6)
        local = sixd_to_mat(pose.astype(np.float64))  # (N, 22, 3, 3)
        n = len(local)
        G = np.zeros_like(local)
        for j in range(22):
            p = PARENTS[j]
            G[:, j] = local[:, j] if p < 0 else G[:, p] @ local[:, j]
    P = np.zeros((n, 22, 3))
    for j in range(1, 22):
        p = PARENTS[j]
        P[:, j] = P[:, p] + np.einsum("nij,j->ni", G[:, p], off[j])
    P += (f["head_pos"] - P[:, 15])[:, None]  # anchored at the head (AMASS frame)
    ours = features.RX90.inv().apply(P.reshape(-1, 3)).reshape(n, 22, 3)
    ours[..., 1] += f["floor"][:, None]
    np.savez_compressed(args.out, t=f["t"], joints=ours.astype(np.float32), valid=f["valid"],
                        global_rot=G.astype(np.float32))
    print(f"{n} frames -> {args.out}")


if __name__ == "__main__":
    main()
