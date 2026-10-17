"""Exports a trained foot model (train.py --out model.pt) to the JSON file read by
core/src/foot_model.cpp, and optionally writes reference outputs computed in
Python from that JSON for slimelite_foot_model_check.

Usage: python tools/ml/export.py model.pt engine/models/foot_model.json [--ref ref.bin] [--dump s.bin]
"""
import argparse
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))
import dataset as ds  # noqa: E402


def g7(a):
    return [float(f"{v:.7g}") for v in np.asarray(a, float).ravel()]


def export(ckpt_path, out_path, note=""):
    ck = torch.load(ckpt_path, weights_only=False)
    state = ck["state"]
    keys = [k for k in state if k.endswith(".weight")]
    layers = []
    for k in keys:
        W = state[k].numpy().astype(float)
        b = state[k.replace(".weight", ".bias")].numpy().astype(float)
        layers.append({"W": [g7(row) for row in W], "b": g7(b)})
    model = {
        "format": "slimelite_foot_model", "version": 1, "note": note,
        "lags": [float(v) for v in ck["lags"]], "accClip": ds.ACC_CLIP, "accTau": ds.ACC_TAU,
        "hetero": bool(ck["args"].get("hetero", False)),
        "dr": bool(ck["args"].get("dr", False)),
        "drParams": {"stillGyro": ds.DR_STILL_GYRO, "stillAcc": ds.DR_STILL_ACC, "zuptAfter": ds.DR_ZUPT_AFTER,
                     "leak": ds.DR_LEAK},
        "mu": g7(ck["mu"]), "sd": g7(ck["sd"]), "ymu": g7(ck["ymu"]), "layers": layers,
        "camBias": float(ck.get("cam_bias", 0.0)), "train": ck["args"],
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as fh:
        json.dump(model, fh, separators=(",", ":"))
    return model


def forward(model, X):
    """Numpy forward pass from the exported JSON (reference for the C++ code)."""
    a = (X - np.array(model["mu"])) / np.array(model["sd"])
    L = model["layers"]
    for i, l in enumerate(L):
        a = a @ np.array(l["W"]).T + np.array(l["b"])
        if i + 1 < len(L):
            a = np.maximum(a, 0)
    mean = a[:, :6] + np.array(model["ymu"])
    sig = np.sqrt(2) * np.exp(np.clip(a[:, 6:12], -5.5, 1.0)) if model["hetero"] else np.zeros_like(mean)
    return mean, sig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt")
    ap.add_argument("out")
    ap.add_argument("--ref")
    ap.add_argument("--dump", default=os.path.join(ds.ML, "s232318.bin"))
    ap.add_argument("--note", default="")
    args = ap.parse_args()
    export(args.ckpt, args.out, args.note)
    model = json.load(open(args.out))  # the rounded values, as C++ reads them
    print(f"{args.out}: {os.path.getsize(args.out) / 1e6:.2f} MB, layers "
          f"{[np.array(l['W']).shape for l in model['layers']]}")
    if args.ref:
        d, _ = ds.load_dump(args.dump)
        fr = ds.Frames(d)
        idx = np.arange(300, fr.n, 97)
        X, ok = ds.features(fr, idx, model["lags"])
        if model.get("dr"):
            X = np.concatenate([X, ds.dead_reckoning(fr)[idx]], axis=1)
        idx, X = idx[ok], X[ok]
        mean, sig = forward(model, X)
        world = ds.to_world(fr, idx, mean)
        world[:, 1] += fr.floor[idx]
        world[:, 4] += fr.floor[idx]
        ref = np.concatenate([idx[:, None].astype(float), world, sig], axis=1)
        ref.astype(np.float64).tofile(args.ref)
        print(f"{len(idx)} reference frames -> {args.ref}")


if __name__ == "__main__":
    main()
