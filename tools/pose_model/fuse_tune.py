"""Tunes the pose model fusion (fusePose in core/src/pose_model.cpp) on the Kinect sessions.

  python tools/pose_model/fuse_tune.py solver.npz model_s0.npz model_s1.npz [--prep prep.npz]

solver.npz: the solver's feet at the GT times for both sessions, 3 trackers and
ankles only (pipeline replays). model_s*.npz: run_hmdposer.py outputs (joints in
the SteamVR frame). The fusion weight ramps with the head height over the
standing eye height from ratioLow to ratioHigh; the model ankles never go below
minAnkleHeight. Grid search on the half A of the test blocks, reported on half B
and in total, per mode, at the pipeline camera bias and at 0 (raw Kinect).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tune"))
import pipeline as P  # noqa: E402


def fuse(sessions, prep, solver, model, lo, hi, hmin):
    out = []
    for i in range(len(sessions)):
        ctx = prep[i]["ctx"]
        sol = solver[i]["feet"]
        ratio = (ctx["hmd_y"] - ctx["floor"]) / ctx["eye_h"]
        w = np.clip((ratio - lo) / max(hi - lo, 1e-6), 0, 1)[:, None]
        f = sol.copy()
        for k in range(2):
            m = model[i][:, 3 * k:3 * k + 3].copy()
            m[:, 1] = np.maximum(m[:, 1], ctx["floor"] + hmin)
            ok = np.isfinite(m).all(1)[:, None]
            f[:, 3 * k:3 * k + 3] = np.where(ok, w * m + (1 - w) * sol[:, 3 * k:3 * k + 3], sol[:, 3 * k:3 * k + 3])
        out.append({"feet": f.astype(np.float32), "waist": solver[i]["waist"], "guard": solver[i]["guard"]})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("solver")
    ap.add_argument("models", nargs=2)
    ap.add_argument("--prep", default="D:/temp-training/pipeline2/prep.npz")
    ap.add_argument("--sessions", default=os.path.join(HERE, "..", "tune", "sessions.json"))
    args = ap.parse_args()
    S = [P.Session(s) for s in json.load(open(args.sessions))]
    prep = list(np.load(args.prep, allow_pickle=True)["prep"])
    sv = np.load(args.solver, allow_pickle=True)
    model = []
    for s, path in zip(S, args.models):
        j = np.load(path)
        t = j["t"] + s.t0
        J = j["joints"]
        model.append(np.stack([P.interp(s.t, t, J[:, k, c]) for k in (7, 8) for c in range(3)], 1))
    cam = json.load(open(P.ENGINE_MODEL)).get("camBias", 0.0)
    grid = [(lo, hi, hm) for lo in (0.5, 0.6, 0.7, 0.8, 0.85, 0.9) for hi in (0.05, 0.1, 0.15, 0.25)
            for hm in (0.05, 0.08, 0.1)]
    for mode in ("three", "ankles"):
        solver = list(sv[mode])
        for bias in (cam, 0.0):
            sc = P.Scorer(S, prep, bias, mode == "ankles")
            best = min(grid, key=lambda g: sc.score(fuse(S, prep, solver, model, g[0], g[0] + g[1], g[2]), "A"))
            lo, hi, hm = best[0], best[0] + best[1], best[2]
            for name, r in (("solver", solver), (f"fused lo {lo} hi {hi:.2f} min {hm}", fuse(S, prep, solver, model, lo, hi, hm))):
                tot, cats = sc.score(r, detail=True)
                print(f"{mode} bias {bias:+.3f} {name:32s} total {tot:6.2f} (A {sc.score(r, 'A'):.2f} B {sc.score(r, 'B'):.2f}) "
                      + " ".join(f"{k} {v:.1f}" for k, v in cats.items()), flush=True)


if __name__ == "__main__":
    main()
