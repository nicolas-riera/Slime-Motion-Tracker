"""Scores body solver outputs against Kinect ground truth (extract.py).

Compares, at each ground-truth frame, the ankle positions of a replay CSV
(slimelite_body_replay: replayed solver, plus the outputs recorded live as rec_*)
with the Kinect ankles. Errors in cm: 3D, horizontal, vertical; split by whether
the ground-truth foot is on the floor or lifted.

Usage: python tools/kinect_gt/score.py gt.csv replay.csv session.jsonl [--from T --to T] [--json out.json]
"""
import argparse
import json

import numpy as np

LIFTED = 0.15  # m, ground-truth ankle above this: foot in the air


def first_time(session):
    with open(session, encoding="utf-8") as fh:
        for line in fh:
            try:
                return json.loads(line)["t"]
            except (json.JSONDecodeError, KeyError):
                continue
    raise ValueError("empty session")


def stats(err):
    if len(err) == 0:
        return None
    return {"n": int(len(err)), "median": float(np.median(err) * 100), "p90": float(np.percentile(err, 90) * 100),
            "mean": float(np.mean(err) * 100)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gt")
    ap.add_argument("replay")
    ap.add_argument("session")
    ap.add_argument("--from", dest="t_from", type=float, default=-np.inf)
    ap.add_argument("--to", dest="t_to", type=float, default=np.inf)
    ap.add_argument("--json")
    args = ap.parse_args()

    gt = np.genfromtxt(args.gt, delimiter=",", names=True)
    rp = np.genfromtxt(args.replay, delimiter=",", names=True)
    t_rp = rp["t"] + first_time(args.session)  # replay times are relative
    gt = gt[(gt["t"] >= max(args.t_from, t_rp[0])) & (gt["t"] <= min(args.t_to, t_rp[-1]))]

    def at(col):
        return np.interp(gt["t"], t_rp, rp[col])

    report = {}
    for source, prefix in (("live", "rec_"), ("replay", "")):
        report[source] = {}
        for leg in ("l", "r"):
            valid = gt[f"{leg}_valid"] > 0
            g = np.stack([gt[f"{leg}_x"], gt[f"{leg}_y"], gt[f"{leg}_z"]], axis=1)[valid]
            e = np.stack([at(f"{prefix}{leg}x"), at(f"{prefix}{leg}y"), at(f"{prefix}{leg}z")], axis=1)[valid]
            d = e - g
            lifted = g[:, 1] > LIFTED
            report[source][leg] = {
                "all": stats(np.linalg.norm(d, axis=1)),
                "horizontal": stats(np.hypot(d[:, 0], d[:, 2])),
                "vertical": stats(np.abs(d[:, 1])),
                "floor": stats(np.linalg.norm(d[~lifted], axis=1)),
                "lifted": stats(np.linalg.norm(d[lifted], axis=1)),
                "lifted_vertical": stats(np.abs(d[lifted, 1])),
            }

    print(f"{len(gt)} ground-truth frames; ankle error in cm, median / p90 (n)")
    keys = ("all", "horizontal", "vertical", "floor", "lifted", "lifted_vertical")
    print(f"{'':12}" + "".join(f"{k:>18}" for k in keys))
    for source in report:
        for leg in ("l", "r"):
            cells = []
            for k in keys:
                s = report[source][leg][k]
                cells.append(f"{s['median']:5.1f} /{s['p90']:5.1f} ({s['n']:5d})" if s else "-")
            print(f"{source + ' ' + leg:12}" + "".join(f"{c:>18}" for c in cells))
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(report, fh, indent=1)


if __name__ == "__main__":
    main()
