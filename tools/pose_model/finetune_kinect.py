"""Fine-tunes a trained pose model on the user's own sessions with Kinect ankle
ground truth, mixed with AMASS batches so it does not forget the rest.

  python tools/pose_model/finetune_kinect.py best.pt out_dir --amass amass.npz --repo HMD-Poser
         --body-models DIR --sessions sessions.json --dumps s0.bin:STAND0 s1.bin:STAND1

Kinect windows come only from the training blocks of tools/tune/pipeline.py (its
test blocks stay unseen, for eval_pose.py). The real inputs go through the same
features as at runtime (features.py); the predicted ankles hang from the head with
the user's skeleton, in the SteamVR frame, against the Kinect ankles (clean,
plausible samples only). The Kinect "ankle" is the depth point cloud's, not the
SMPL joint: the fine-tuning also learns that offset.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tune"))
import features  # noqa: E402
import train_pose as tp  # noqa: E402


def session_data(spec, dump, stand):
    import pipeline as P
    s = P.Session(spec)
    d = features.load_dump(dump)
    f = features.build(d, stand)
    t = f["t"] + s.t0
    n = len(t)
    gt = np.full((n, 2, 3), np.nan, np.float32)
    test = np.zeros(n, bool)
    # GT (30 Hz) onto the 60 Hz feature times: nearest sample within 25 ms
    idx = np.clip(np.searchsorted(s.t, t), 1, len(s.t) - 1)
    near = np.where(np.abs(s.t[idx - 1] - t) < np.abs(s.t[idx] - t), idx - 1, idx)
    close = np.abs(s.t[near] - t) < 0.025
    hmd = np.stack([f["d"]["hx"], f["d"]["hy"], f["d"]["hz"]], 1)
    reach = 1.1 * f["d"]["eye_h"] + 0.1
    for k, leg in enumerate("lr"):
        g = s.g[leg][near]
        ok = close & s.clean[leg][near] & np.all(np.isfinite(g), 1)
        ok &= (np.linalg.norm(g - hmd, axis=1) < reach) & (g[:, 1] < hmd[:, 1] - 0.25)
        gt[ok, k] = g[ok]
    # Test blocks (with their GT samples' neighbourhood) excluded from training
    tb = s.test[near] & (np.abs(s.t[near] - t) < 1.0)
    test |= tb
    in_gt = (t >= s.t[0]) & (t <= s.t[-1])
    return {"X": np.nan_to_num(f["X"]).astype(np.float32), "valid": f["valid"], "gt": gt,
            "train": in_gt & ~test, "head": f["head_pos"].astype(np.float32),
            "floor": f["floor"].astype(np.float32), "height": float(np.median(f["d"]["height"]))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt")
    ap.add_argument("out_dir")
    ap.add_argument("--amass", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--body-models", required=True)
    ap.add_argument("--sessions", required=True)
    ap.add_argument("--dumps", nargs="+", required=True, help="dump.bin:stand_time per session, in order")
    ap.add_argument("--iters", type=int, default=4000)
    ap.add_argument("--lr", type=float, default=5e-5)
    ap.add_argument("--kinect-weight", type=float, default=5.0)
    ap.add_argument("--length", type=int, default=80)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--kbatch", type=int, default=16)
    args = ap.parse_args()
    sys.path.insert(0, args.repo)
    from model.network import HMD_imu_HME_Universe

    os.makedirs(args.out_dir, exist_ok=True)
    log = open(os.path.join(args.out_dir, "finetune.log"), "a")

    def say(s):
        print(s, flush=True)
        log.write(s + "\n")
        log.flush()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    specs = json.load(open(args.sessions))
    sess = []
    for spec, ds in zip(specs, args.dumps):
        path, stand = ds.rsplit(":", 1)
        sd = session_data(spec, path, float(stand))
        L = args.length
        ok = sd["train"] & sd["valid"]
        # window starts: whole window in training frames, with GT somewhere in it
        c = np.concatenate([[0], np.cumsum(~ok)])
        has = np.concatenate([[0], np.cumsum(np.isfinite(sd["gt"][:, :, 0]).any(1))])
        starts = np.arange(len(ok) - L)
        starts = starts[(c[starts + L] - c[starts] == 0) & (has[starts + L] - has[starts] > L // 4)]
        sd["starts"] = starts
        say(f"{os.path.basename(path)}: {len(starts)} training windows, "
            f"{np.isfinite(sd['gt'][sd['train'], :, 0]).sum()} GT ankle samples")
        sess.append(sd)
    m = np.load(os.path.join(args.body_models, "smplh", "neutral", "model.npz"), allow_pickle=True)
    rest_neutral = torch.from_numpy((m["J_regressor"][:22] @ m["v_template"]).astype(np.float32)).to(dev)
    model_height = float(m["v_template"][:, 1].max() - m["v_template"][:, 1].min())

    data = tp.Data(args.amass, args.length)
    J0, Jdirs = (t.to(dev) for t in tp.neutral_shape(args.body_models))
    net = HMD_imu_HME_Universe(135, 3, 256, 0.05, 8, 2).to(dev)
    sd0 = torch.load(args.ckpt, map_location=dev, weights_only=False)
    net.load_state_dict(sd0.get("state_dict", sd0))
    opt = torch.optim.Adam(net.parameters(), lr=args.lr, eps=1e-6)
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda i: min(1.0, (i + 1) / 200) * 0.5 * (1 + math.cos(math.pi * min(1.0, i / args.iters))))
    synth = tp.Synth(dev)
    rng = np.random.default_rng(int(time.time()))
    w = np.array([len(s["starts"]) for s in sess], np.float64)
    t0 = time.time()
    acc = {"amass": 0.0, "kinect": 0.0}
    for it in range(args.iters):
        net.train()
        poses, trans, rest = (t.to(dev) for t in data.batch("train", args.batch, rng))
        with torch.no_grad():
            X, tgt = synth(poses, trans, rest)
        pose, betas = net(X)
        la, _, _, _ = tp.losses(pose, betas, tgt, J0, Jdirs, 2.0)
        # Kinect batch
        Xs, Gs, Hs, Fs, Ss = [], [], [], [], []
        for _ in range(args.kbatch):
            s = sess[rng.choice(len(sess), p=w / w.sum())]
            a = int(rng.choice(s["starts"]))
            sl = slice(a, a + args.length)
            if rng.random() < 0.4:  # ankles-only mode as in the AMASS training
                x = s["X"][sl].copy()
                x[:, list(range(30, 36)) + list(range(66, 72)) + list(range(132, 135))] = 0
            else:
                x = s["X"][sl]
            Xs.append(x)
            Gs.append(s["gt"][sl])
            Hs.append(s["head"][sl])
            Fs.append(s["floor"][sl])
            Ss.append(s["height"] / model_height)
        Xk = torch.from_numpy(np.stack(Xs)).to(dev)
        Gk = torch.from_numpy(np.stack(Gs)).to(dev)
        Hk = torch.from_numpy(np.stack(Hs)).to(dev)
        Fk = torch.from_numpy(np.stack(Fs)).to(dev)
        Sk = torch.tensor(Ss, device=dev, dtype=torch.float32)
        pk, _ = net(Xk)
        B, L, _ = pk.shape
        G = tp.fk_rot(tp.sixd2mat(pk.reshape(B, L, 22, 6)))
        P = tp.fk_pos(G, (rest_neutral[None] * Sk[:, None, None])[:, None])
        P = P - P[:, :, 15:16] + Hk[:, :, None]
        ank = P[:, :, [7, 8]]  # AMASS frame -> ours: (x, z + floor, -y)
        ours = torch.stack([ank[..., 0], ank[..., 2] + Fk[:, :, None], -ank[..., 1]], -1)
        valid = torch.isfinite(Gk[..., 0])
        lk = (ours - torch.nan_to_num(Gk)).abs().sum(-1)[valid].mean() if valid.any() else ours.sum() * 0
        loss = la + args.kinect_weight * lk
        opt.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
        opt.step()
        sched.step()
        acc["amass"] += la.item()
        acc["kinect"] += lk.item()
        if (it + 1) % 100 == 0:
            say(f"it {it + 1} amass {acc['amass'] / 100:.4f} kinect L1 {acc['kinect'] / 100 * 100:.2f} cm "
                f"{(time.time() - t0) / (it + 1):.3f} s/it")
            acc = {"amass": 0.0, "kinect": 0.0}
    torch.save({"state_dict": net.state_dict(), "iter": args.iters}, os.path.join(args.out_dir, "finetuned.pt"))
    say("saved finetuned.pt")


if __name__ == "__main__":
    main()
