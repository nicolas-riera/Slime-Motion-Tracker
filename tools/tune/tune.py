"""Offline tuning of the body solver constants against the Kinect ground truth.

Runs slimelite_body_replay on the Kinect-covered part of a session with solver
constants overridden through SL_TUNE (core/src/body_solver.cpp, tunable()), scores
every run and searches the constants with a simple evolution strategy (CMA-like:
weighted recombination of the best samples, step size adapted to the success rate).
Runs in parallel, with no network and no AI: it can run for hours unattended.

Score (lower is better), cm:
  accuracy: ankle error on the foot model's held-out test blocks (5 min blocks,
    pattern train train val train test, tools/ml/dataset.py), against the
    cleaned ground truth with the camera bias removed: per leg
    median + 0.3 p90 + 0.5 median when lifted, mean of both legs;
  guardrails: plausibility metrics over the whole window (tools/eval_replay.py:
    skating, swing following, jitter, feet in the air...) must not get worse than
    the defaults by more than 10%; beyond, a penalty is added (user perception
    overrides the ground truth: skating or jitter are not traded for accuracy).
The test blocks are reported in two halves (A, B) to see overfitting.

Usage: python tools/tune/tune.py session.jsonl gt.csv out_dir
          [--model engine/models/foot_model.json] [--from S --to S]
          [--workers 14] [--generations 40] [--population 14]
out_dir/trials.jsonl: one line per run; out_dir/best.json: the best constants.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import eval_replay  # noqa: E402

REPLAY = os.path.join(ROOT, "build", "Release", "slimelite_body_replay.exe")
CAMERA = np.array([1.46257241, 1.92838919, 1.48353078])  # tools/ml/dataset.py
LIFTED = 0.15
GUARDRAILS = ("standing_air", "planted_h", "skate", "foot_far", "jit_foot", "jit_knee",
              "jit_waist", "swing_follow")
GUARD_TOLERANCE = 1.10
GUARD_WEIGHT = 10.0

# name: (default, low, high); searched in log space
PARAMS = {
    "kModelRobust": (3.0, 1.5, 6.0),
    "kModelStanceScale": (3.0, 1.0, 10.0),
    "modelScale": (0.6, 0.3, 1.5),
    "modelFloor": (0.03, 0.015, 0.08),
    "kStanceVelLeak": (0.25, 0.1, 0.6),
    "kStanceDispLeak": (0.3, 0.1, 0.8),
    "kLiftOffSpeed": (0.2, 0.1, 0.4),
    "kAnchorLeak": (1.0, 0.3, 3.0),
    "kInsRobust": (5.0, 2.0, 10.0),
    "kInsHipScale": (5.0, 2.0, 15.0),
    "kStraightSigma": (1.2, 0.6, 2.5),
    "kBendCost": (3.0, 1.0, 6.0),
    "kBendSigma": (0.35, 0.2, 0.6),
    "kUnderSigma": (0.08, 0.04, 0.2),
}
NAMES = list(PARAMS)
LO = np.log([PARAMS[n][1] for n in NAMES])
HI = np.log([PARAMS[n][2] for n in NAMES])
DEFAULT = (np.log([PARAMS[n][0] for n in NAMES]) - LO) / (HI - LO)  # unit cube


def to_params(u):
    u = np.clip(u, 0, 1)
    return {n: float(math.exp(lo + v * (hi - lo))) for n, v, lo, hi in zip(NAMES, u, LO, HI)}


class Columns:
    """Dict of columns with len() (what eval_replay.frame_metrics expects)."""

    def __init__(self, cols):
        self.cols = cols
        self.dtype = type("dtype", (), {"names": tuple(cols)})

    def __getitem__(self, k):
        return self.cols[k]

    def __len__(self):
        return len(self.cols["t"])


def load_csv(path):
    with open(path, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return Columns({h: data[:, i] for i, h in enumerate(header)})


def first_time(session):
    with open(session, encoding="utf-8") as fh:
        return json.loads(fh.readline())["t"]


def load_gt(path, cam_bias):
    """Ground truth per leg: times, positions with the camera bias removed, and a
    mask of clean samples (within 15 cm of the track median over +-0.35 s)."""
    with open(path, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    raw = np.genfromtxt(path, delimiter=",", skip_header=1)
    col = {h: raw[:, i] for i, h in enumerate(header)}
    t = col["t"]
    legs = {}
    for leg in "lr":
        g = np.stack([col[f"{leg}_x"], col[f"{leg}_y"], col[f"{leg}_z"]], axis=1)
        valid = (col[f"{leg}_valid"] > 0) & np.all(np.isfinite(g), axis=1)
        u = CAMERA - np.nan_to_num(g)
        u[:, 1] = 0
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-6)
        g = g - cam_bias * u
        clean = valid.copy()
        idx = np.flatnonzero(valid)
        tv, gv = t[idx], g[idx]
        lo = np.searchsorted(tv, tv - 0.35)
        hi = np.searchsorted(tv, tv + 0.35)
        med = np.array([np.median(gv[a:b], axis=0) for a, b in zip(lo, hi)])
        clean[idx] = np.linalg.norm(gv - med, axis=1) < 0.15
        legs[leg] = (g, clean)
    t0 = t[0]
    block = np.floor((t - t0) / 300.0).astype(int)
    edge = ((t - t0) % 300.0 < 3.0) | ((t - t0) % 300.0 > 297.0)
    test = (block % 5 == 4) & ~edge
    half = (block // 5) % 2  # alternate test blocks: A (0), B (1)
    return t, legs, test, half


def accuracy(rp, t_rp, gt, sel):
    t, legs, _, _ = gt
    out = {}
    for leg in "lr":
        g, clean = legs[leg]
        m = sel & clean & (t >= t_rp[0]) & (t <= t_rp[-1])
        e = np.stack([np.interp(t[m], t_rp, rp[f"{leg}{a}"]) for a in "xyz"], axis=1)
        d = np.linalg.norm(e - g[m], axis=1) * 100
        lifted = g[m, 1] > LIFTED
        out[leg] = {
            "median": float(np.median(d)), "p90": float(np.percentile(d, 90)),
            "lifted": float(np.median(d[lifted])) if lifted.any() else 0.0,
            "floor_h": float(np.median(np.hypot(*(e - g[m])[~lifted][:, [0, 2]].T)) * 100),
            "n": int(m.sum()),
        }
    score = np.mean([o["median"] + 0.3 * o["p90"] + 0.5 * o["lifted"] for o in out.values()])
    return float(score), out


def run(args, params, gt, t_first):
    env = dict(os.environ)
    env["SL_TUNE"] = ",".join(f"{k}={v:.6g}" for k, v in params.items())
    fd, csv = tempfile.mkstemp(suffix=".csv", dir=args.out)
    os.close(fd)
    try:
        cmd = [REPLAY, args.session, csv, "--from", str(args.t_from), "--to", str(args.t_to)]
        if args.model:
            cmd += ["--model", args.model]
        subprocess.run(cmd, env=env, check=True, stdout=subprocess.DEVNULL)
        rp = load_csv(csv)
    finally:
        os.remove(csv)
    t_rp = rp["t"] + t_first
    _, _, test, half = gt
    score, legs = accuracy(rp, t_rp, gt, test)
    score_a, _ = accuracy(rp, t_rp, gt, test & (half == 0))
    score_b, _ = accuracy(rp, t_rp, gt, test & (half == 1))
    m = eval_replay.frame_metrics(rp, "")
    allf = np.ones(len(rp), bool)
    guard = {k: v[0] for k, v in eval_replay.summarize(m, allf).items() if k in GUARDRAILS}
    return {"accuracy": score, "A": score_a, "B": score_b, "legs": legs, "guard": guard}


def penalty(guard, base):
    p = 0.0
    for k in GUARDRAILS:
        b, v = base.get(k), guard.get(k)
        if b is None or v is None or not np.isfinite(b) or not np.isfinite(v) or b <= 0:
            continue
        p += GUARD_WEIGHT * max(0.0, v / b - GUARD_TOLERANCE)
    return p


_GT = None
_T_FIRST = None


def init_worker(args):
    global _GT, _T_FIRST
    cam_bias = 0.0
    if args.model:
        with open(args.model, encoding="utf-8") as fh:
            cam_bias = json.load(fh).get("camBias", 0.0)
    _GT = load_gt(args.gt, cam_bias)
    _T_FIRST = first_time(args.session)


def worker(job):
    args, u = job
    params = to_params(u)
    try:
        return u, params, run(args, params, _GT, _T_FIRST)
    except Exception as e:  # a crashed run is just a bad sample
        return u, params, {"error": str(e)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("gt")
    ap.add_argument("out")
    ap.add_argument("--model")
    ap.add_argument("--from", dest="t_from", type=float, default=1750)
    ap.add_argument("--to", dest="t_to", type=float, default=10040)
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--generations", type=int, default=40)
    ap.add_argument("--population", type=int, default=14)
    ap.add_argument("--sigma", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    log = open(os.path.join(args.out, "trials.jsonl"), "a", encoding="utf-8")

    with ProcessPoolExecutor(args.workers, initializer=init_worker, initargs=(args,)) as pool:
        _, _, base = pool.submit(worker, (args, DEFAULT)).result()
        if "error" in base:
            raise SystemExit("default run failed: " + base["error"])
        base_guard = base["guard"]
        base["score"] = base["accuracy"]
        log.write(json.dumps({"gen": 0, "params": to_params(DEFAULT), **base}) + "\n")
        log.flush()
        print(f"defaults: score {base['score']:.2f} (A {base['A']:.2f}, B {base['B']:.2f})", flush=True)
        best = {"score": base["score"], "params": to_params(DEFAULT), "result": base}

        mean, sigma = DEFAULT.copy(), args.sigma
        mu = args.population // 3
        weights = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
        weights /= weights.sum()
        for gen in range(1, args.generations + 1):
            samples = [np.clip(mean + sigma * rng.standard_normal(len(NAMES)), 0, 1)
                       for _ in range(args.population)]
            scored = []
            for u, params, res in pool.map(worker, [(args, s) for s in samples]):
                if "error" in res:
                    res["score"] = 1e9
                else:
                    res["penalty"] = penalty(res["guard"], base_guard)
                    res["score"] = res["accuracy"] + res["penalty"]
                log.write(json.dumps({"gen": gen, "params": params, **res}) + "\n")
                scored.append((res["score"], u, params, res))
            log.flush()
            scored.sort(key=lambda x: x[0])
            if scored[0][0] < best["score"]:
                best = {"score": scored[0][0], "params": scored[0][2], "result": scored[0][3]}
                with open(os.path.join(args.out, "best.json"), "w", encoding="utf-8") as fh:
                    json.dump({"defaults": {"score": base["score"], "A": base["A"], "B": base["B"],
                                            "legs": base["legs"], "guard": base_guard}, **best}, fh, indent=1)
            # recombination and step size from the success rate (1/5th rule)
            elite = np.array([s[1] for s in scored[:mu]])
            mean = weights @ elite
            success = np.mean([s[0] < best["score"] + 0.05 for s in scored])
            sigma = float(np.clip(sigma * math.exp((success - 0.2) / 0.6), 0.02, 0.3))
            r = best["result"]
            print(f"gen {gen}: best {best['score']:.2f} (acc {r['accuracy']:.2f}, A {r['A']:.2f}, "
                  f"B {r['B']:.2f}, penalty {r.get('penalty', 0):.2f}), gen best {scored[0][0]:.2f}, "
                  f"sigma {sigma:.3f}", flush=True)


if __name__ == "__main__":
    main()
