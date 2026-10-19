"""Scores learned-model joints against the Kinect ground truth, per pose category,
next to the solver (the default replay of a pipeline output directory).

  python tools/pose_model/eval_pose.py sessions.json pipeline_dir s0.npz s1.npz
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tune"))
import pipeline as P  # noqa: E402


def main():
    specs = json.load(open(sys.argv[1]))
    out_dir = sys.argv[2]
    joints = sys.argv[3:]
    S = [P.Session(s) for s in specs]
    z = np.load(os.path.join(out_dir, "prep.npz"), allow_pickle=True)
    prep, base = list(z["prep"]), z["results"].tolist()
    res = []
    for s, path, b in zip(S, joints, base):
        j = np.load(path)
        t = j["t"] + s.t0
        J = j["joints"]
        feet = np.stack([P.interp(s.t, t, J[:, k, c]) for k in (7, 8) for c in range(3)], 1).astype(np.float32)
        waist = np.stack([P.interp(s.t, t, J[:, 0, c]) for c in range(3)], 1).astype(np.float32)
        res.append({"feet": feet, "waist": waist, "guard": b["guard"]})
    sc = P.Scorer(S, prep, 0.0, False)
    for name, r in (("solver", base), ("model", res)):
        tot, cats = sc.score(r, detail=True)
        print(f"{name:7s} total {tot:6.2f} (A {sc.score(r, 'A'):.2f} B {sc.score(r, 'B'):.2f}) "
              + " ".join(f"{k} {v:.1f}" for k, v in cats.items()))


if __name__ == "__main__":
    main()
