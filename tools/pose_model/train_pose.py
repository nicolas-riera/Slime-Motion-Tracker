"""Trains HMD-Poser (MIT, github.com/Pico-AI-Team/HMD-Poser) on AMASS with our sensors.

  python tools/pose_model/train_pose.py amass.npz out_dir --repo HMD-Poser --body-models DIR
         [--init pretrained.pt] [--iters N]

amass.npz: tools/pose_model/amass_prep.py. The sparse inputs are synthesized on the
GPU from the AMASS motion, the way our hardware sees it (features.py layout):
  - shin IMUs at the ankle (on the shin segment, 70-90 % of the knee-ankle length,
    a few cm in front), not at the knee vertex HMD-Poser was trained with;
  - waist IMU on the pelvis segment, front or back of the belt;
  - noise: per-sensor heading and mounting errors (mag drift, body calibration),
    controller-to-wrist calibration errors, HMD-to-head offset errors, accelerometer
    noise and gain;
  - the waist tracker is dropped in a share of the windows (ankles-only mode).
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import torch
import torch.nn.functional as F

PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19]
PAD = 4  # frames of context on each side (velocities, accelerations)


def aa2mat(aa):
    """Axis-angle (..., 3) -> rotation matrices (..., 3, 3)."""
    th = aa.norm(dim=-1, keepdim=True).clamp_min(1e-8)
    k = aa / th
    kx, ky, kz = k.unbind(-1)
    z = torch.zeros_like(kx)
    K = torch.stack([z, -kz, ky, kz, z, -kx, -ky, kx, z], -1).reshape(*aa.shape[:-1], 3, 3)
    s, c = th.sin()[..., None], th.cos()[..., None]
    eye = torch.eye(3, device=aa.device, dtype=aa.dtype).expand_as(K)
    return eye + s * K + (1 - c) * (K @ K)


def sixd(m):
    return torch.cat([m[..., :, 0], m[..., :, 1]], -1)


def sixd2mat(x):
    a = F.normalize(x[..., :3], dim=-1)
    b = x[..., 3:6]
    b = F.normalize(b - (a * b).sum(-1, keepdim=True) * a, dim=-1)
    return torch.stack([a, b, torch.cross(a, b, dim=-1)], -1)


def fk_rot(local):
    """Local rotations (..., 22, 3, 3) -> global."""
    g = [local[..., 0, :, :]]
    for j in range(1, 22):
        g.append(g[PARENTS[j]] @ local[..., j, :, :])
    return torch.stack(g, -3)


def fk_pos(G, rest):
    """Global rotations (..., 22, 3, 3) and rest joints (..., 22, 3) -> positions, root at rest[0]."""
    p = [rest[..., 0, :].expand(G.shape[:-3] + (3,))]
    for j in range(1, 22):
        off = rest[..., j, :] - rest[..., PARENTS[j], :]
        p.append(p[PARENTS[j]] + (G[..., PARENTS[j], :, :] @ off[..., None].expand(G.shape[:-3] + (3, 1)))[..., 0])
    return torch.stack(p, -2)


def rand_rot(n, sigma_deg, device, axis=None):
    """Random small rotations: about a fixed axis (yaw) or a random axis."""
    ang = torch.randn(n, device=device) * math.radians(sigma_deg)
    if axis is None:
        ax = F.normalize(torch.randn(n, 3, device=device), dim=-1)
    else:
        ax = torch.tensor(axis, device=device, dtype=torch.float32).expand(n, 3)
    return aa2mat(ax * ang[:, None])


def rel_vel(m):
    """R(t-1)^T R(t) along dim 1 (first frame uses itself)."""
    prev = torch.cat([m[:, :1], m[:, :-1]], 1)
    return prev.transpose(-1, -2) @ m


def syn_acc(p, n=PAD):
    """Central second difference over +-n frames at 60 Hz (HMD-Poser _syn_acc)."""
    return (p[:, :-2 * n] + p[:, 2 * n:] - 2 * p[:, n:-n]) * 3600.0 / n ** 2


class Synth:
    def __init__(self, device, noise=True, no_waist_share=0.4):
        self.device = device
        self.noise = noise
        self.no_waist_share = no_waist_share

    def __call__(self, poses, trans, rest):
        """poses (B, L+2P, 66), trans (B, L+2P, 3), rest (B, 22, 3) -> features, targets."""
        dev = self.device
        B, T, _ = poses.shape
        L = T - 2 * PAD
        local = aa2mat(poses.reshape(B, T, 22, 3))
        G = fk_rot(local)
        P = fk_pos(G, rest[:, None]) + trans[..., None, :]
        nz = self.noise

        def frac(lo, hi):
            return lo + (hi - lo) * torch.rand(B, 1, device=dev)

        # Shin sensors (rest frame: Y up, +Z forward)
        sens_rot, sens_pos = [], []
        for knee, ankle in ((4, 7), (5, 8)):
            leg = rest[:, ankle] - rest[:, knee]
            off = leg * (frac(0.7, 0.9) if nz else 0.8)
            off = off + torch.cat([torch.zeros(B, 2, device=dev), frac(0.0, 0.06) if nz else torch.full((B, 1), 0.04, device=dev)], 1)
            sens_pos.append(P[:, :, knee] + (G[:, :, knee] @ off[:, None, :, None])[..., 0])
            sens_rot.append(G[:, :, knee])
        side = torch.where(torch.rand(B, 1, device=dev) < 0.5, -1.0, 1.0) if nz else torch.ones(B, 1, device=dev)
        off = torch.cat([torch.zeros(B, 1, device=dev), frac(0.0, 0.08) if nz else torch.full((B, 1), 0.05, device=dev),
                         side * (frac(0.08, 0.14) if nz else 0.11)], 1)
        sens_pos.append(P[:, :, 0] + (G[:, :, 0] @ off[:, None, :, None])[..., 0])
        sens_rot.append(G[:, :, 0])

        head_rot, head_pos = G[:, :, 15], P[:, :, 15]
        hand_rot = [G[:, :, 20], G[:, :, 21]]
        hand_pos = [P[:, :, 20], P[:, :, 21]]
        if nz:
            head_pos = head_pos + (head_rot @ (torch.randn(B, 1, 3, 1, device=dev) * 0.015))[..., 0]
            for k in range(2):
                hand_rot[k] = hand_rot[k] @ rand_rot(B, 8.0, dev)[:, None]
                hand_pos[k] = hand_pos[k] + (hand_rot[k] @ (torch.randn(B, 1, 3, 1, device=dev) * 0.025))[..., 0]
            for k in range(3):
                # heading error (world Z up), slowly drifting across the window, plus mounting error
                yaw0 = torch.randn(B, device=dev) * math.radians(6.0)
                drift = torch.randn(B, device=dev) * math.radians(3.0)
                ramp = torch.linspace(0, 1, T, device=dev)
                yaw = yaw0[:, None] + drift[:, None] * ramp[None]
                Ryaw = aa2mat(torch.stack([torch.zeros_like(yaw), torch.zeros_like(yaw), yaw], -1))
                sens_rot[k] = Ryaw @ sens_rot[k] @ rand_rot(B, 3.0, dev)[:, None]

        acc = [syn_acc(p) for p in sens_pos]
        if nz:
            acc = [a * (1 + 0.05 * torch.randn(B, 1, 1, device=dev)) + 0.3 * torch.randn_like(a) for a in acc]

        c = slice(PAD, PAD + L)
        rots = [head_rot] + hand_rot + sens_rot
        six = torch.cat([sixd(r[:, c]) for r in rots], -1)
        vel = torch.cat([sixd(rel_vel(r)[:, c]) for r in rots], -1)
        pos = torch.cat([head_pos, hand_pos[0], hand_pos[1]], -1)
        dpos = (pos - torch.cat([pos[:, :1], pos[:, :-1]], 1))[:, c]
        hin = [head_rot.transpose(-1, -2) @ h for h in hand_rot]
        hsix = torch.cat([sixd(h[:, c]) for h in hin], -1)
        hvel = torch.cat([sixd(rel_vel(h)[:, c]) for h in hin], -1)
        hp = torch.cat([((p - head_pos)[..., None, :] @ head_rot)[..., 0, :] for p in hand_pos], -1)
        dhp = (hp - torch.cat([hp[:, :1], hp[:, :-1]], 1))[:, c]
        X = torch.cat([six, vel, pos[:, c], dpos, hsix, hvel, hp[:, c], dhp] + acc, -1)
        if nz and self.no_waist_share > 0:
            drop = torch.rand(B, device=dev) < self.no_waist_share
            mask = torch.ones(135, device=dev)
            mask[list(range(30, 36)) + list(range(66, 72)) + list(range(132, 135))] = 0
            X = torch.where(drop[:, None, None], X * mask, X)
        tgt = {"local": sixd(local[:, c]).reshape(B, L, 132), "global": sixd(G[:, c]).reshape(B, L, 132),
               "pos": P[:, c]}
        return X, tgt


class Data:
    def __init__(self, path, L, val_share=0.05, seed=0):
        z = np.load(path)
        self.poses = torch.from_numpy(z["poses"])
        self.trans = torch.from_numpy(z["trans"])
        self.rest = torch.from_numpy(z["rest"])
        start, length = z["seq_start"], z["seq_len"]
        ok = np.where(length >= L + 2 * PAD)[0]
        rng = np.random.default_rng(seed)
        val = rng.random(len(ok)) < val_share
        self.L = L
        self.split = {"train": ok[~val], "val": ok[val]}
        self.start, self.length = start, length
        self.datasets = list(z["datasets"])

    def batch(self, split, B, rng):
        seqs = self.split[split]
        w = (self.length[seqs] - self.L - 2 * PAD + 1).astype(np.float64)
        s = rng.choice(seqs, B, p=w / w.sum())
        o = (rng.random(B) * (self.length[s] - self.L - 2 * PAD + 1)).astype(np.int64)
        idx = torch.from_numpy(self.start[s] + o)[:, None] + torch.arange(self.L + 2 * PAD)[None]
        return self.poses[idx], self.trans[idx], self.rest[torch.from_numpy(s)]


def neutral_shape(body_models):
    m = np.load(os.path.join(body_models, "smplh", "neutral", "model.npz"), allow_pickle=True)
    Jr = m["J_regressor"][:22]
    return (torch.from_numpy((Jr @ m["v_template"]).astype(np.float32)),
            torch.from_numpy(np.einsum("jv,vck->jck", Jr, m["shapedirs"][:, :, :16]).astype(np.float32)))


def losses(pred_pose, pred_betas, tgt, J0, Jdirs, feet_weight):
    B, L, _ = pred_pose.shape
    Gp = fk_rot(sixd2mat(pred_pose.reshape(B, L, 22, 6)))
    rest = J0 + torch.einsum("jck,blk->bljc", Jdirs, pred_betas)
    Pp = fk_pos(Gp, rest)
    Pg = tgt["pos"]
    Pp = Pp - Pp[:, :, 15:16] + Pg[:, :, 15:16]  # head-centred, as HMD-Poser
    l1 = F.l1_loss
    out = {
        "root": l1(pred_pose[..., :6], tgt["local"][..., :6]),
        "local": l1(pred_pose[..., 6:], tgt["local"][..., 6:]),
        "global": l1(sixd(Gp).reshape(B, L, 132), tgt["global"]),
        "joint": l1(Pp, Pg),
        "feet": l1(Pp[:, :, [7, 8, 10, 11]], Pg[:, :, [7, 8, 10, 11]]),
        "smooth": l1(Pp[:, :-2] - 2 * Pp[:, 1:-1] + Pp[:, 2:], Pg[:, :-2] - 2 * Pg[:, 1:-1] + Pg[:, 2:]),
        "shape": l1(pred_betas, pred_betas.mean(1, keepdim=True).expand_as(pred_betas)),
    }
    total = (out["root"] + 5 * out["local"] + out["global"] + out["joint"] + feet_weight * out["feet"]
             + 0.5 * out["smooth"] + 0.1 * out["shape"])
    ank = (Pp[:, :, [7, 8]] - Pg[:, :, [7, 8]]).norm(dim=-1).mean()
    mpjpe = (Pp - Pg).norm(dim=-1).mean()
    return total, out, mpjpe, ank


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("out_dir")
    ap.add_argument("--repo", required=True)
    ap.add_argument("--body-models", required=True)
    ap.add_argument("--init")
    ap.add_argument("--iters", type=int, default=150000)
    ap.add_argument("--batch", type=int, default=192)
    ap.add_argument("--length", type=int, default=90)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--feet-weight", type=float, default=2.0)
    ap.add_argument("--val-every", type=int, default=2000)
    ap.add_argument("--resume")
    args = ap.parse_args()
    sys.path.insert(0, args.repo)
    from model.network import HMD_imu_HME_Universe

    os.makedirs(args.out_dir, exist_ok=True)
    log = open(os.path.join(args.out_dir, "train.log"), "a")

    def say(s):
        print(s, flush=True)
        log.write(s + "\n")
        log.flush()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    data = Data(args.data, args.length)
    say(f"device {dev}, train {len(data.split['train'])} seq, val {len(data.split['val'])} seq, "
        f"{len(data.poses) / 216000:.1f} h, datasets {data.datasets}")
    J0, Jdirs = (t.to(dev) for t in neutral_shape(args.body_models))
    net = HMD_imu_HME_Universe(135, 3, 256, 0.05, 8, 2).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=args.lr, eps=1e-6)
    it0 = 0
    if args.resume:
        ck = torch.load(args.resume, map_location=dev, weights_only=False)
        net.load_state_dict(ck["state_dict"])
        opt.load_state_dict(ck["opt"])
        it0 = ck["iter"]
    elif args.init:
        sd = torch.load(args.init, map_location=dev, weights_only=False)
        net.load_state_dict(sd.get("state_dict", sd))
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda i: min(1.0, (i + 1) / 1000) * 0.5 * (1 + math.cos(math.pi * min(1.0, i / args.iters))),
        last_epoch=it0 - 1 if it0 else -1)
    synth = Synth(dev)
    clean = Synth(dev, noise=False)
    rng = np.random.default_rng(int(time.time()))
    vrng = np.random.default_rng(1)
    val_batches = [data.batch("val", args.batch, vrng) for _ in range(20)]
    best = float("inf")
    t0 = time.time()
    acc = {}
    for it in range(it0, args.iters):
        net.train()
        poses, trans, rest = (t.to(dev, non_blocking=True) for t in data.batch("train", args.batch, rng))
        with torch.no_grad():
            X, tgt = synth(poses, trans, rest)
        pose, betas = net(X)
        total, parts, mpjpe, ank = losses(pose, betas, tgt, J0, Jdirs, args.feet_weight)
        opt.zero_grad(set_to_none=True)
        total.backward()
        torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
        opt.step()
        sched.step()
        for k, v in (("loss", total), ("mpjpe", mpjpe), ("ankle", ank)):
            acc[k] = acc.get(k, 0.0) + v.item()
        if (it + 1) % 200 == 0:
            n = 200
            say(f"it {it + 1} lr {sched.get_last_lr()[0]:.2e} loss {acc['loss'] / n:.4f} "
                f"mpjpe {acc['mpjpe'] / n * 100:.2f} cm ankle {acc['ankle'] / n * 100:.2f} cm "
                f"{(time.time() - t0) / (it + 1 - it0):.3f} s/it")
            acc = {}
        if (it + 1) % args.val_every == 0 or it + 1 == args.iters:
            net.eval()
            res = {"noisy": [0.0, 0.0], "clean": [0.0, 0.0]}
            with torch.no_grad():
                g = torch.Generator(device=dev)
                for name, sy in (("noisy", synth), ("clean", clean)):
                    torch.manual_seed(123)
                    for b in val_batches:
                        X, tgt = sy(*(t.to(dev) for t in b))
                        pose, betas = net(X)
                        _, _, mpjpe, ank = losses(pose, betas, tgt, J0, Jdirs, args.feet_weight)
                        res[name][0] += mpjpe.item() / len(val_batches)
                        res[name][1] += ank.item() / len(val_batches)
            say(f"val it {it + 1}: noisy mpjpe {res['noisy'][0] * 100:.2f} ankle {res['noisy'][1] * 100:.2f} | "
                f"clean mpjpe {res['clean'][0] * 100:.2f} ankle {res['clean'][1] * 100:.2f} cm")
            ck = {"state_dict": net.state_dict(), "opt": opt.state_dict(), "iter": it + 1}
            torch.save(ck, os.path.join(args.out_dir, "last.pt"))
            if res["noisy"][0] < best:
                best = res["noisy"][0]
                torch.save({"state_dict": net.state_dict(), "iter": it + 1}, os.path.join(args.out_dir, "best.pt"))
                say(f"  best -> best.pt")


if __name__ == "__main__":
    main()
