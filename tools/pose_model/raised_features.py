"""Raised-foot detector from micro-movements (postural sway), as the solver runs it.

  python tools/pose_model/raised_features.py sessions.json dump0.bin dump1.bin --replays r0.csv r1.csv

Per frame, causal, with one-pole filters that follow the frame time (identical to
core/src/body_solver.cpp, RaisedDetector): per leg, the tremor energy (> 3 Hz) and
the slow sway energy (0.3-3 Hz) of the shin's acceleration, and its coherence
with the HMD's and the waist's sway (a planted leg sways with the body, a raised
one moves on its own); the HMD sway energy (one-leg stance sways more); and the
solver's own ankle height (last frame). A logistic regression on the Kinect
sessions (standing, raised = ankle > 20 cm with the other foot down) gives the
probability that the foot is off the ground. Prints the cross-session AUC and the
C++ coefficients.
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tune"))
import features  # noqa: E402

NAMES = ["self_hf", "self_lf", "self_coh_hmd", "self_coh_waist", "self_acc", "self_h",
         "other_hf", "other_lf", "other_coh_hmd", "other_coh_waist", "other_acc", "other_h", "sway_hmd"]


class OnePole:
    def __init__(self, fc):
        self.tau = 1.0 / (2 * np.pi * fc)
        self.y = None

    def __call__(self, x, dt):
        if self.y is None:
            self.y = np.array(x, dtype=np.float64)
        else:
            self.y = self.y + (x - self.y) * min(1.0, dt / (self.tau + dt))
        return self.y


class Ema:
    def __init__(self, tau):
        self.tau = tau
        self.y = 0.0

    def __call__(self, x, dt):
        self.y += (x - self.y) * min(1.0, dt / self.tau)
        return self.y


def compute(d, solver_h):
    """d: dump columns; solver_h: (N, 2) solver ankle heights above the floor, same rows."""
    t = d["t"]
    n = len(t)
    out = np.zeros((n, 2, len(NAMES)))
    hmd_lp = OnePole(5.0)
    hv_prev, hp_prev = None, None
    hb_lo, hb_hi = OnePole(0.3), OnePole(3.0)
    wb_lo, wb_hi = OnePole(0.3), OnePole(3.0)
    sway = Ema(2.0)
    legs = []
    for _ in range(2):
        legs.append({"hp": OnePole(3.0), "lo": OnePole(0.3), "hi": OnePole(3.0), "hf": Ema(1.0), "lf": Ema(2.0),
                     "xh": Ema(2.0), "xw": Ema(2.0), "ph": Ema(2.0), "pw": Ema(2.0), "acc": Ema(0.1)})
    prev_t = None
    for i in range(n):
        dt = 1 / 90.0 if prev_t is None else max(1e-4, min(0.1, t[i] - prev_t))
        prev_t = t[i]
        hp = hmd_lp(np.array([d["hx"][i], d["hy"][i], d["hz"][i]]), dt)
        if hp_prev is None:
            hv = np.zeros(3)
            ha = np.zeros(3)
        else:
            hv = (hp - hp_prev) / dt
            ha = (hv - hv_prev) / dt if hv_prev is not None else np.zeros(3)
        hp_prev, hv_prev = hp.copy(), hv
        hb = (hb_hi(ha, dt) - hb_lo(ha, dt))[[0, 2]]
        wa = np.array([d["wax"][i], d["way"][i], d["waz"][i]])
        wb = (wb_hi(wa, dt) - wb_lo(wa, dt))[[0, 2]]
        sw = sway(hb @ hb, dt)
        per = []
        for k, p in enumerate("lr"):
            L = legs[k]
            a = np.array([d[p + "ax"][i], d[p + "ay"][i], d[p + "az"][i]])
            hf = a - L["hp"](a, dt)
            ab = (L["hi"](a, dt) - L["lo"](a, dt))[[0, 2]]
            e_hf = L["hf"](hf @ hf, dt)
            e_lf = L["lf"](ab @ ab, dt)
            xh = L["xh"](ab @ hb, dt)
            xw = L["xw"](ab @ wb, dt)
            ph = L["ph"](hb @ hb, dt)
            pw = L["pw"](wb @ wb, dt)
            per.append([np.log(e_hf + 1e-6), np.log(e_lf + 1e-6), xh / np.sqrt(e_lf * ph + 1e-12),
                        xw / np.sqrt(e_lf * pw + 1e-12), L["acc"](np.linalg.norm(a), dt), solver_h[i, k]])
        for k in range(2):
            out[i, k] = per[k] + per[1 - k] + [np.log(sw + 1e-9)]
    return out


def auc(pos, neg):
    x = np.r_[pos, neg]
    y = np.r_[np.ones(len(pos)), np.zeros(len(neg))]
    r = np.argsort(np.argsort(x)) + 1
    return (r[y == 1].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))


def labelled(spec, dump, replay):
    import pipeline as P
    s = P.Session(spec)
    d = features.load_dump(dump)
    rp = pd.read_csv(replay, usecols=["t", "ly", "ry", "floor"])
    k = np.clip(np.searchsorted(rp["t"].values, d["t"]), 0, len(rp) - 1)
    sh = np.stack([rp["ly"].values[k] - rp["floor"].values[k], rp["ry"].values[k] - rp["floor"].values[k]], 1)
    F = compute(d, sh)
    tt = s.t - s.t0
    j = np.clip(np.searchsorted(d["t"], tt), 0, len(d["t"]) - 1)
    floor = d["floor"][j]
    standing = (d["hy"][j] - floor) / d["eye_h"][j] > 0.85
    pos, neg = [], []
    for li, leg in enumerate("lr"):
        o = "r" if leg == "l" else "l"
        gl, go = s.g[leg], s.g[o]
        ok = s.clean[leg] & s.clean[o] & np.isfinite(gl[:, 0]) & np.isfinite(go[:, 0]) & standing
        dist = np.hypot(gl[:, 0] - d["hx"][j], gl[:, 2] - d["hz"][j])
        raised = ok & (gl[:, 1] - floor > 0.20) & (go[:, 1] - floor < 0.12) & (dist > 0.25)
        planted = ok & (gl[:, 1] - floor < 0.10) & (go[:, 1] - floor < 0.10)
        pos.append(F[j[raised], li])
        neg.append(F[j[planted], li])
    return np.concatenate(pos), np.concatenate(neg)


def fit(pos, neg, seed=0):
    from sklearn.linear_model import LogisticRegression
    rng = np.random.default_rng(seed)
    neg = neg[rng.choice(len(neg), min(len(neg), 80000), replace=False)]
    X = np.r_[pos, neg]
    y = np.r_[np.ones(len(pos)), np.zeros(len(neg))]
    mu, sd = X.mean(0), X.std(0) + 1e-9
    m = LogisticRegression(max_iter=3000, class_weight="balanced").fit((X - mu) / sd, y)
    return mu, sd, m.coef_[0], m.intercept_[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions")
    ap.add_argument("dumps", nargs=2)
    ap.add_argument("--replays", nargs=2, required=True)
    args = ap.parse_args()
    specs = json.load(open(args.sessions))
    data = [labelled(s, d, r) for s, d, r in zip(specs, args.dumps, args.replays)]
    for tr, te in ((0, 1), (1, 0)):
        mu, sd, w, b = fit(*data[tr])
        score = lambda X: ((X - mu) / sd) @ w + b
        print(f"train s{tr} test s{te}: AUC {auc(score(data[te][0]), score(data[te][1])):.3f}")
    mu, sd, w, b = fit(np.r_[data[0][0], data[1][0]], np.r_[data[0][1], data[1][1]])
    fmt = lambda v: ", ".join(f"{x:.6g}" for x in v)
    print("// features: " + ", ".join(NAMES))
    print(f"const double kRaisedMu[] = {{{fmt(mu)}}};")
    print(f"const double kRaisedSd[] = {{{fmt(sd)}}};")
    print(f"const double kRaisedW[] = {{{fmt(w)}}};")
    print(f"const double kRaisedB = {b:.6g};")


if __name__ == "__main__":
    main()
