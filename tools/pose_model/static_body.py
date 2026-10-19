"""How much the solved waist follows the head when only the head/upper body moves.

  python tools/pose_model/static_body.py dump.bin[:stand] [...] [--variants JSON] [--ankles]
         [--pose-model model.bin]

Windows (3 s) standing with both shins and the pelvis still (orientation ranges
< 2 and 3 deg) and the HMD moving > 5 cm horizontally: the body below the chest
did not move, so the waist should not follow the head (SlimeVR without a chest
tracker drags the whole body along). Reports the waist/head displacement ratio
along the head motion; lower is better, the current behaviour is the reference
not to degrade. With --pose-model, dumps need their stand capture time
(dump.bin:T) to calibrate the wrists.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXE = os.path.join(HERE, "..", "..", "build", "Release", "smt_dump_solve.exe")
sys.path.insert(0, HERE)
import features  # noqa: E402


def windows(d, W=3.0):
    t = d["t"]
    rot = {p: features.quat(d, p) for p in ("l", "r", "w")}
    out = []
    k = 0
    while k < len(t):
        e = np.searchsorted(t, t[k] + W)
        if e - k < 60:
            k = max(e, k + 1)
            continue
        sl = slice(k, e)
        ok = all((d[c][sl] > 0).all() for c in ("hmd_valid", "l_valid", "r_valid", "w_valid"))
        if ok and ((d["hy"][sl] - d["floor"][sl]) / d["eye_h"][sl]).min() > 0.85:
            ranges = {p: np.degrees((r[sl][0].inv() * r[sl]).magnitude().max()) for p, r in rot.items()}
            H = np.stack([d["hx"][sl], d["hz"][sl]], 1)
            H = H - H.mean(0)
            u = np.linalg.svd(H, full_matrices=False)[2][0]
            amp = np.ptp(H @ u)
            if ranges["l"] < 2 and ranges["r"] < 2 and ranges["w"] < 3 and amp > 0.05:
                out.append((k, e, u, amp))
        k = e
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dumps", nargs="+")
    ap.add_argument("--variants", default="[{}]")
    ap.add_argument("--ankles", action="store_true")
    ap.add_argument("--pose-model")
    args = ap.parse_args()
    data = []
    for spec in args.dumps:
        path, _, stand = spec.partition(".bin:")
        path = path if path.endswith(".bin") else path + ".bin"
        data.append((path, stand, features.load_dump(path)))
    wins = [windows(d) for _, _, d in data]
    print(f"{sum(len(w) for w in wins)} still-body windows", flush=True)
    variants = json.loads(args.variants)

    def run(job):
        vi, v, (p, stand, d), ws = job
        env = dict(os.environ)
        env["SMT_TUNE"] = ",".join(f"{k}={x}" for k, x in v.items())
        out = p + f".solve{vi}.bin"
        cmd = [EXE, p, out] + (["--ankles"] if args.ankles else [])
        if args.pose_model:
            cmd += ["--pose-model", args.pose_model, "--stand", stand]
        subprocess.run(cmd, env=env, check=True)
        cols = json.load(open(out + ".json"))["columns"]
        o = np.fromfile(out).reshape(-1, len(cols))
        c = {n: o[:, k] for k, n in enumerate(cols)}
        ratios = []
        for k, e, u, amp in ws:
            Wv = np.stack([c["wx"][k:e], c["wz"][k:e]], 1)
            ratios.append(np.ptp((Wv - Wv.mean(0)) @ u) / amp)
        os.remove(out)
        os.remove(out + ".json")
        return vi, ratios

    jobs = [(vi, v, dd, ws) for vi, v in enumerate(variants) for dd, ws in zip(data, wins)]
    by_variant = {}
    with ThreadPoolExecutor(14) as pool:
        for vi, ratios in pool.map(run, jobs):
            by_variant.setdefault(vi, []).extend(ratios)
    for vi, v in enumerate(variants):
        r = np.array(by_variant[vi])
        print(f"{v or 'default'}: waist/head median {np.median(r):.2f}  p75 {np.percentile(r, 75):.2f}  "
              f">0.5: {(r > 0.5).mean() * 100:.0f}%", flush=True)


if __name__ == "__main__":
    main()
