"""Lying: how far sideways the solved pelvis sits from the head-to-feet line.

  python tools/pose_model/lying_lateral.py dump.bin [...] [--variants JSON]

No ground truth lying (the Kinect does not see a body on a bed), but on the back
or on the side the pelvis lies roughly on the line from the head to the middle
of the feet: its horizontal distance from that line (solved feet) is a body bent
sideways. Frames with the pelvis lying (its up axis more than 60 deg from vertical).
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dumps", nargs="+")
    ap.add_argument("--variants", default="[{}]")
    args = ap.parse_args()
    data = [(p, features.load_dump(p)) for p in args.dumps]
    variants = json.loads(args.variants)

    def run(job):
        vi, v, (p, d) = job
        env = dict(os.environ)
        env["SMT_TUNE"] = ",".join(f"{k}={x}" for k, x in v.items())
        out = p + f".lying{vi}.bin"
        subprocess.run([EXE, p, out], env=env, check=True)
        cols = json.load(open(out + ".json"))["columns"]
        o = np.fromfile(out).reshape(-1, len(cols))
        c = {n: o[:, k] for k, n in enumerate(cols)}
        up_y = features.quat(d, "w").apply([0.0, 1.0, 0.0])[:, 1]
        lying = (c["valid"] > 0) & (d["w_valid"] > 0) & (up_y < 0.5) & (d["hmd_valid"] > 0)
        h = np.stack([d["hx"], d["hz"]], 1)[lying]
        f = np.stack([(c["lx"] + c["rx"]) / 2, (c["lz"] + c["rz"]) / 2], 1)[lying]
        w = np.stack([c["wx"], c["wz"]], 1)[lying]
        a = f - h
        n = np.linalg.norm(a, axis=1)
        cross = np.abs(a[:, 0] * (w - h)[:, 1] - a[:, 1] * (w - h)[:, 0]) / np.maximum(n, 1e-6)
        os.remove(out)
        os.remove(out + ".json")
        return vi, cross[n > 0.5]

    by_variant = {}
    with ThreadPoolExecutor(14) as pool:
        for vi, x in pool.map(run, [(vi, v, dd) for vi, v in enumerate(variants) for dd in data]):
            by_variant.setdefault(vi, []).append(x)
    for vi, v in enumerate(variants):
        x = np.concatenate(by_variant[vi]) * 100
        print(f"{v or 'default'}: {len(x) / 90 / 60:.1f} min lying, pelvis off the head-feet line "
              f"median {np.median(x):.1f} cm, p75 {np.percentile(x, 75):.1f}, p90 {np.percentile(x, 90):.1f}", flush=True)


if __name__ == "__main__":
    main()
