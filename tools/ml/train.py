"""Trains the learned foot model (MLP on the causal feature window, dataset.py)
and scores it on held-out time blocks against the body solver.

Usage: python tools/ml/train.py [--hidden 256 256] [--epochs 60] [--mirror] [--out model.pt]
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import torch
from torch import nn

sys.path.insert(0, os.path.dirname(__file__))
import dataset as ds  # noqa: E402

LIFTED = 0.15  # same as tools/kinect_gt/score.py


def breakdown(P, Y, M, mask):
    """Error stats (cm) as in score.py, per leg: median / p90."""
    res = {}
    for k, leg in enumerate("lr"):
        m = mask & M[:, k]
        e = P[m, 3 * k:3 * k + 3] - Y[m, 3 * k:3 * k + 3]
        n = np.linalg.norm(e, axis=1)
        lifted = Y[m, 3 * k + 1] > LIFTED
        parts = {"all": n, "horizontal": np.hypot(e[:, 0], e[:, 2]), "vertical": np.abs(e[:, 1]),
                 "floor": n[~lifted], "floor_h": np.hypot(e[~lifted, 0], e[~lifted, 2]),
                 "lifted": n[lifted], "lifted_v": np.abs(e[lifted, 1])}
        res[leg] = {k2: (float(np.median(v) * 100), float(np.percentile(v, 90) * 100), int(len(v)))
                    for k2, v in parts.items() if len(v)}
    return res


def fmt(res):
    keys = ("all", "horizontal", "vertical", "floor", "floor_h", "lifted", "lifted_v")
    lines = ["      " + "".join(f"{k:>13}" for k in keys)]
    for leg in "lr":
        lines.append(f"  {leg}   " + "".join(f"{res[leg][k][0]:5.1f}/{res[leg][k][1]:5.1f}  " if k in res[leg]
                                            else f"{'-':>13}" for k in keys))
    return "\n".join(lines)


def summary(P, Y, M, mask):
    """One line: L+R pooled medians (cm): all, all p90, floor horizontal, lifted"""
    r = breakdown(P, Y, M, mask)
    out = []
    for key, q in (("all", 0), ("all", 1), ("floor_h", 0), ("lifted", 0)):
        out.append(np.mean([r[leg][key][q] for leg in "lr"]))
    return "all %.1f p90 %.1f floor_h %.1f lifted %.1f" % tuple(out)


class MLP(nn.Module):
    def __init__(self, n_in, hidden, n_out=6, dropout=0.1, indrop=0.0):
        super().__init__()
        layers, d = ([nn.Dropout(indrop)] if indrop > 0 else []), n_in
        for h in hidden:
            layers += [nn.Linear(d, h), nn.ReLU(), nn.Dropout(dropout)]
            d = h
        layers.append(nn.Linear(d, n_out))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def masked_laplace(pred, logb, y, m):
    """Laplace negative log-likelihood (robust, learns a per-axis scale b)"""
    mm = m.repeat_interleave(3, dim=1).float()
    logb = logb.clamp(-5.5, 1.0)
    l = (pred - y).abs() * torch.exp(-logb) + logb
    return (l * mm).sum() / mm.sum().clamp(min=1)


def masked_huber(pred, y, m, delta):
    mm = m.repeat_interleave(3, dim=1).float()
    l = nn.functional.huber_loss(pred, y, reduction="none", delta=delta)
    return (l * mm).sum() / mm.sum().clamp(min=1)


def yaw_indices(n_lags):
    """x and z columns of every 3-vector of the feature window"""
    base = np.arange(n_lags)[:, None] * ds.FEATS_PER_LAG + np.arange(0, 33, 3)[None, :]
    return torch.tensor(base.ravel()), torch.tensor(base.ravel() + 2)


def rotate_xz(v, ix, iz, c, s):
    x, z = v[:, ix], v[:, iz]
    v = v.clone()
    v[:, ix] = c * x - s * z
    v[:, iz] = s * x + c * z
    return v


def train(X, Y, M, tr, va, hidden, epochs, lr, wd, dropout, noise, mirror, lags, seed=0, verbose=True, cam=None,
          yawaug=0.0, indrop=0.0, hetero=False):
    torch.manual_seed(seed)
    np.random.seed(seed)
    Xtr, Ytr, Mtr = X[tr], Y[tr], M[tr]
    Ctr = cam[tr] if cam is not None else np.zeros_like(Ytr)
    if mirror:
        Xm = ds.mirror_features(Xtr, lags)
        Ym, Mm = ds.mirror_targets(Ytr, Mtr)
        Cm, _ = ds.mirror_targets(Ctr, Mtr)
        Xtr, Ytr, Mtr = np.concatenate([Xtr, Xm]), np.concatenate([Ytr, Ym]), np.concatenate([Mtr, Mm])
        Ctr = np.concatenate([Ctr, Cm])
    mu = Xtr.mean(0)
    sd = Xtr.std(0) + 1e-3
    ymu = np.zeros(6)
    for k in range(2):
        ymu[3 * k:3 * k + 3] = Ytr[Mtr[:, k], 3 * k:3 * k + 3].mean(0)
    net = MLP(X.shape[1], hidden, n_out=12 if hetero else 6, dropout=dropout, indrop=indrop)
    fix, fiz = yaw_indices(len(lags))
    tix, tiz = torch.tensor([0, 3]), torch.tensor([2, 5])
    mu_t, sd_t = torch.tensor(mu, dtype=torch.float32), torch.tensor(sd, dtype=torch.float32)
    ymu_t = torch.tensor(ymu, dtype=torch.float32)
    # Ground-truth bias toward the camera (training only): target = f(x) + b * u_cam
    bias = nn.Parameter(torch.zeros(1))
    opt = torch.optim.AdamW([{"params": net.parameters()}, {"params": [bias], "weight_decay": 0}],
                            lr=lr, weight_decay=wd)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=lr, total_steps=epochs * (len(Xtr) // 512 + 1))
    xt = torch.tensor((Xtr - mu) / sd, dtype=torch.float32)
    yt = torch.tensor(Ytr - ymu, dtype=torch.float32)
    mt = torch.tensor(Mtr)
    ct = torch.tensor(Ctr, dtype=torch.float32)
    xv = torch.tensor((X[va] - mu) / sd, dtype=torch.float32)
    cv = cam[va] if cam is not None else np.zeros((va.sum(), 6))
    best, best_state = np.inf, None
    for ep in range(epochs):
        net.train()
        perm = torch.randperm(len(xt))
        for b in range(0, len(xt), 512):
            i = perm[b:b + 512]
            x, y, cb = xt[i], yt[i], ct[i]
            if yawaug > 0:
                # Heading error augmentation: rotate the whole window and the targets
                a = yawaug * torch.randn(len(i), 1)
                c, sn = torch.cos(a), torch.sin(a)
                x = (rotate_xz(x * sd_t + mu_t, fix, fiz, c, sn) - mu_t) / sd_t
                y = rotate_xz(y + ymu_t, tix, tiz, c, sn) - ymu_t
                cb = rotate_xz(cb, tix, tiz, c, sn)
            if noise > 0:
                x = x + noise * torch.randn_like(x)
            out = net(x)
            if hetero:
                loss = masked_laplace(out[:, :6] + bias * cb, out[:, 6:], y, mt[i])
            else:
                loss = masked_huber(out + bias * cb, y, mt[i], 0.05)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
        net.eval()
        with torch.no_grad():
            pv = net(xv)[:, :6].numpy() + ymu + bias.item() * cv  # ground-truth measurement model
        ev = []
        for k in range(2):
            m = M[va][:, k]
            ev.append(np.linalg.norm(pv[m, 3 * k:3 * k + 3] - Y[va][m, 3 * k:3 * k + 3], axis=1))
        ev = np.concatenate(ev)
        score = np.median(ev) + 0.3 * np.percentile(ev, 90)
        if score < best:
            best = score
            best_state = {k: v.clone() for k, v in net.state_dict().items()}
            best_ep = ep
        if verbose and (ep % 10 == 0 or ep == epochs - 1):
            print(f"  ep {ep:3d} loss {loss.item():.4f} val median {np.median(ev) * 100:.1f} "
                  f"p90 {np.percentile(ev, 90) * 100:.1f}", flush=True)
    net.load_state_dict(best_state)
    net.eval()
    if verbose:
        print(f"  best epoch {best_ep}, camera bias {bias.item() * 100:.1f} cm")
    net.cam_bias = bias.item()
    return net, mu, sd, ymu


def predict(net, mu, sd, ymu, X):
    with torch.no_grad():
        out = net(torch.tensor((X - mu) / sd, dtype=torch.float32)).numpy()
    return out[:, :6] + ymu


def predict_scale(net, mu, sd, X):
    """Laplace scale b per output (m), hetero models only (sigma = sqrt(2) b)"""
    with torch.no_grad():
        out = net(torch.tensor((X - mu) / sd, dtype=torch.float32)).numpy()
    return np.exp(np.clip(out[:, 6:], -5.5, 1.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.join(ds.ML, "ds_v1.npz"))
    ap.add_argument("--hidden", type=int, nargs="+", default=[256, 256])
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--wd", type=float, default=1e-3)
    ap.add_argument("--dropout", type=float, default=0.1)
    ap.add_argument("--noise", type=float, default=0.1)
    ap.add_argument("--nlags", type=int, default=0, help="use only the first N lags")
    ap.add_argument("--mirror", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--clean", action="store_true", help="train on the cleaned ground truth")
    ap.add_argument("--cambias", action="store_true", help="fit the ground-truth camera bias")
    ap.add_argument("--out")
    ap.add_argument("--brief", action="store_true")
    ap.add_argument("--lagidx", help="comma separated lag indices to keep")
    ap.add_argument("--yawaug", type=float, default=0.0, help="heading augmentation sigma (rad)")
    ap.add_argument("--indrop", type=float, default=0.0, help="input dropout")
    ap.add_argument("--hetero", action="store_true", help="also predict a per-axis uncertainty")
    args = ap.parse_args()

    z = np.load(args.data)
    X, Y, M, t = z["X"].astype(np.float64), z["Y"], z["M"], z["t"]
    Mc = z["Mc"]
    Mtrain = Mc if args.clean else M
    lags = list(z["lags"]) if "lags" in z else ds.default_lags()
    if args.nlags:
        lags = lags[:args.nlags]
        X = X[:, :len(lags) * ds.FEATS_PER_LAG]
    if args.lagidx:
        sel = [int(v) for v in args.lagidx.split(",")]
        lags = [lags[i] for i in sel]
        X = X.reshape(len(X), -1, ds.FEATS_PER_LAG)[:, sel].reshape(len(X), -1)
    rec, rep = z["rec"], z["replay"]
    sp = ds.split_of(t, t[0])
    tr, va, te = (sp == 0) & Mtrain.any(1), (sp == 1) & Mc.any(1), sp == 2
    print(f"samples train {tr.sum()} val {va.sum()} test {te.sum()}, features {X.shape[1]}")
    t1 = time.time()
    net, mu, sd, ymu = train(X, Y, Mtrain, tr, va, args.hidden, args.epochs, args.lr, args.wd, args.dropout,
                             args.noise, args.mirror, lags, args.seed, cam=z["cam"] if args.cambias else None,
                             yawaug=args.yawaug, indrop=args.indrop, hetero=args.hetero)
    print(f"trained in {time.time() - t1:.0f} s")
    P = predict(net, mu, sd, ymu, X)
    # Ground truth without its camera bias (as fitted): the fair reference for a
    # camera-independent estimate
    Yd = Y - getattr(net, "cam_bias", 0.0) * z["cam"]
    for name, mask, ref in (("raw", M, Y), ("clean", Mc, Y), ("clean, debiased", Mc, Yd)):
        if args.brief and name != "clean, debiased":
            continue
        print(f"TEST [{name} GT] model (cm, median/p90)\n" + fmt(breakdown(P, ref, mask, te)))
        if not args.brief:
            print(f"TEST [{name} GT] solver live\n" + fmt(breakdown(rec, ref, mask, te)))
            if np.isfinite(rep).all():
                print(f"TEST [{name} GT] solver replay\n" + fmt(breakdown(rep, ref, mask, te)))
    print("SUMMARY", summary(P, Yd, Mc, te), "| val", summary(P, Yd, Mc, va), "| solver live",
          summary(rec, Yd, Mc, te))
    if args.out:
        torch.save({"state": net.state_dict(), "mu": mu, "sd": sd, "ymu": ymu, "hidden": args.hidden,
                    "lags": lags, "args": vars(args), "cam_bias": getattr(net, "cam_bias", 0.0)}, args.out)
        np.save(args.out.replace(".pt", "_pred.npy"), P)


if __name__ == "__main__":
    main()
