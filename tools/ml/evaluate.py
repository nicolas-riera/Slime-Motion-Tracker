"""Runs an exported foot model (JSON, same math as the C++ module) on every
engine frame of a session and measures what the user feels: jitter while the
feet are still, lag against the ground truth, and the error breakdown on the
held-out ground-truth blocks, next to the live body solver.

Usage: python tools/ml/evaluate.py model.json [--dump s.bin] [--data ds_v1.npz] [--save pred.npy]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import dataset as ds  # noqa: E402
from export import forward  # noqa: E402
from train import breakdown, fmt  # noqa: E402


def predict_all(model, fr, chunk=40000):
    """World ankles (N, 6; y above the floor) and sigmas for every frame (NaN when invalid)."""
    dr = ds.dead_reckoning(fr) if model.get("dr") else None
    P = np.full((fr.n, 6), np.nan)
    S = np.full((fr.n, 6), np.nan)
    for a in range(0, fr.n, chunk):
        idx = np.arange(a, min(a + chunk, fr.n))
        X, ok = ds.features(fr, idx, model["lags"])
        if dr is not None:
            X = np.concatenate([X, dr[idx]], axis=1)
        mean, sig = forward(model, X[ok])
        P[idx[ok]] = ds.to_world(fr, idx[ok], mean)
        S[idx[ok]] = sig
    return P, S


def jitter(P, t, still):
    """RMS of the frame-to-frame acceleration (second difference / dt^2 is
    dominated by noise at 90 Hz; reported as the 2nd difference in mm) and the
    RMS deviation from a 0.3 s centered mean (mm), on still frames."""
    d2 = P[2:] - 2 * P[1:-1] + P[:-2]
    m = still[1:-1] & np.isfinite(d2).all(1)
    rms_d2 = np.sqrt(np.mean(np.sum(d2[m] ** 2, axis=1))) * 1000
    k = 27  # ~0.3 s at 90 Hz
    ker = np.ones(k) / k
    dev = []
    for c in range(P.shape[1]):
        x = np.nan_to_num(P[:, c])
        dev.append(x - np.convolve(x, ker, mode="same"))
    dev = np.stack(dev, 1)
    m2 = still & np.isfinite(P).all(1)
    rms_dev = np.sqrt(np.mean(np.sum(dev[m2] ** 2, axis=1))) * 1000
    return rms_d2, rms_dev


def lag_curve(P, t_frames, gt_t, G, mask, shifts):
    """Median 3D error vs time shift of the estimate (s): a minimum at a positive
    shift means the estimate lags the ground truth."""
    out = []
    for sh in shifts:
        e = []
        for k in range(2):
            m = mask[:, k]
            est = np.stack([np.interp(gt_t[m] + sh, t_frames, np.nan_to_num(P[:, 3 * k + a])) for a in range(3)], 1)
            e.append(np.linalg.norm(est - G[m, 3 * k:3 * k + 3], axis=1))
        out.append(np.median(np.concatenate(e)))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--dump", default=os.path.join(ds.ML, "s232318.bin"))
    ap.add_argument("--data", default=os.path.join(ds.ML, "ds_v1.npz"))
    ap.add_argument("--save")
    args = ap.parse_args()
    model = json.load(open(args.model))
    d, _ = ds.load_dump(args.dump)
    fr = ds.Frames(d)
    P, S = predict_all(model, fr)
    if args.save:
        np.save(args.save, np.concatenate([P, S], 1).astype(np.float32))
    rec = np.stack([d["rec_lx"], d["rec_ly"] - d["floor"], d["rec_lz"], d["rec_rx"], d["rec_ry"] - d["floor"],
                    d["rec_rz"]], 1)
    print(f"valid model output on {np.isfinite(P).all(1).mean() * 100:.2f}% of {fr.n} frames")

    # Jitter on still feet: quiet shin (angular speed, acceleration)
    for k, leg in enumerate("lr"):
        acc = np.linalg.norm(fr.acc[leg], axis=1)
        still = (fr.gyro[leg] < 0.2) & (acc < 0.4)
        jm = jitter(P[:, 3 * k:3 * k + 3], fr.t, still)
        js = jitter(rec[:, 3 * k:3 * k + 3], fr.t, still)
        print(f"jitter {leg} (still {still.mean() * 100:.0f}% of frames): 2nd diff RMS model {jm[0]:.2f} mm, "
              f"solver {js[0]:.2f} mm; deviation from 0.3 s mean model {jm[1]:.1f} mm, solver {js[1]:.1f} mm")

    # Ground-truth frames: lag and test breakdown (debiased GT with the model's fitted camera bias)
    z = np.load(args.data)
    gt_t, Mc, idx = z["t"], z["Mc"], z["idx"]
    b = model.get("camBias", 0.0)
    # ground truth in world (y above floor), debiased
    c, s = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
    off = fr.hmd[idx] * np.array([1, 0, 1])
    Yd = z["Y"] - b * z["cam"]
    G = np.concatenate([ds.from_heading(Yd[:, 0:3], c, s) + off, ds.from_heading(Yd[:, 3:6], c, s) + off], 1)
    sp = ds.split_of(gt_t, gt_t[0])
    te = sp == 2
    shifts = np.arange(-0.15, 0.151, 0.025)
    moving = Mc & te[:, None]
    lm = lag_curve(P, fr.t, gt_t, G, moving, shifts)
    ls = lag_curve(rec, fr.t, gt_t, G, moving, shifts)
    print("lag (test GT): best shift model %+.3f s, solver %+.3f s (positive = lags)" %
          (shifts[np.argmin(lm)], shifts[np.argmin(ls)]))
    Pm = np.concatenate([ds.to_heading(P[idx, 0:3] - off, c, s), ds.to_heading(P[idx, 3:6] - off, c, s)], 1)
    print("TEST [clean, debiased GT] model\n" + fmt(breakdown(Pm, Yd, Mc, te)))
    print("TEST [clean, debiased GT] solver live\n" + fmt(breakdown(z["rec"], Yd, Mc, te)))
    if model.get("hetero"):
        e = np.abs(Pm - Yd)
        Sg = S[idx]
        for k in range(2):
            m = te & Mc[:, k] & np.isfinite(Sg[:, 3 * k])
            r = e[m, 3 * k:3 * k + 3] / Sg[m, 3 * k:3 * k + 3]
            print(f"uncertainty calibration {'lr'[k]}: |error|/sigma median {np.median(r):.2f} (Laplace: 0.49), "
                  f"fraction within 2 sigma {np.mean(r < 2) * 100:.0f}% (Laplace: 94%)")
            for name, sel in (("floor", Yd[m, 3 * k + 1] <= 0.15), ("lifted", Yd[m, 3 * k + 1] > 0.15)):
                sg = Sg[m][sel, 3 * k:3 * k + 3]
                print(f"  {name}: median sigma x/y/z {np.median(sg[:, 0]) * 100:.1f}/{np.median(sg[:, 1]) * 100:.1f}/"
                      f"{np.median(sg[:, 2]) * 100:.1f} cm, within 2 sigma {np.mean(r[sel] < 2) * 100:.0f}%, "
                      f"|err y| median {np.median(e[m][sel, 3 * k + 1]) * 100:.1f} cm")


if __name__ == "__main__":
    main()
