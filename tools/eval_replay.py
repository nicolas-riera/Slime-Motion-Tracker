"""Plausibility metrics of a slimelite_body_replay CSV, per time window and overall.

There is no ground truth in most recordings, so these metrics measure physically
implausible outputs (feet in the air while standing, skating, legs bent while the
user stands straight...). Use them as guidance: always look at plots of the worst
windows (tools/plot_replay.py) before changing the solver.

Usage: python tools/eval_replay.py replay.csv [--window S] [--rec] [--worst N]
  --rec: evaluate the recorded engine outputs (rec_* columns) instead of the replay
"""
import argparse

import numpy as np

# Metric name, description, aggregation ("mean" of per-frame values over the frames
# where the metric applies), lower is better for all of them
METRICS = [
    ("standing_air", "standing, both feet > 8 cm above the floor (fraction)"),
    ("planted_h", "planted feet height above the floor while standing (m)"),
    ("skate", "planted feet horizontal speed while standing (m/s)"),
    ("foot_far", "standing, a foot > 0.6 m horizontally from the waist (fraction)"),
    ("knee_limit", "knee angle > 145 deg (fraction)"),
    ("knee_stand", "knee angle when standing tall with vertical shins (deg)"),
    ("jit_foot", "foot position |second difference| (mm)"),
    ("jit_knee", "knee angle |second difference| (deg)"),
    ("jit_waist", "waist position |second difference| (mm)"),
    ("hip_off", "waist horizontal offset from the head-pelvis chain (m)"),
    ("swing_follow", "swing (insSigma < 0.1): |ankle height - dead reckoning| (m)"),
    ("lying", "head low: waist height - head height, above +0.15 (m)"),
]


def frame_metrics(d, prefix):
    """Per-frame metric values (NaN where a metric does not apply)."""
    n = len(d)
    t = d["t"]
    dt = np.maximum(np.diff(t, prepend=t[0] - 1 / 90), 1e-3)
    floor = d["ankle_floor"]
    standing = d["standing"] > 0.8
    m = {}
    nan = np.full(n, np.nan)

    lh = d[prefix + "ly"] - floor
    rh = d[prefix + "ry"] - floor
    pl = d[prefix + "pl"] > 0
    pr = d[prefix + "pr"] > 0

    m["standing_air"] = np.where(standing, ((lh > 0.08) & (rh > 0.08)).astype(float), np.nan)

    ph = np.full(n, np.nan)
    cnt = pl.astype(float) + pr.astype(float)
    s = np.where(pl, lh, 0) + np.where(pr, rh, 0)
    ok = standing & (cnt > 0)
    ph[ok] = s[ok] / cnt[ok]
    m["planted_h"] = ph

    sk = np.full(n, np.nan)
    sp = []
    for side, planted in (("l", pl), ("r", pr)):
        x, z = d[prefix + side + "x"], d[prefix + side + "z"]
        v = np.hypot(np.diff(x, prepend=x[0]), np.diff(z, prepend=z[0])) / dt
        both = planted & np.roll(planted, 1)
        sp.append(np.where(both, v, np.nan))
    sp = np.vstack(sp)
    valid = ~np.all(np.isnan(sp), axis=0) & standing
    sk[valid] = np.nanmean(sp[:, valid], axis=0)
    m["skate"] = sk

    far = np.zeros(n, dtype=bool)
    hip_off = nan.copy()
    for side in ("l", "r"):
        dist = np.hypot(d[prefix + side + "x"] - d[prefix + "wx"], d[prefix + side + "z"] - d[prefix + "wz"])
        far |= dist > 0.6
    m["foot_far"] = np.where(standing, far.astype(float), np.nan)

    kl, kr = d[prefix + "kl"], d[prefix + "kr"]
    m["knee_limit"] = ((kl > 145).astype(float) + (kr > 145).astype(float)) / 2
    tall = standing & (d["hmd_y"] - d["floor"] > 0.93 * d["eye_h"])
    ks = nan.copy()
    for k, up in ((kl, d["lshin_up_y"]), (kr, d["rshin_up_y"])):
        sel = tall & (up > 0.97)
        ks = np.where(sel, np.where(np.isnan(ks), k, (np.nan_to_num(ks) + k) / 2), ks)
    m["knee_stand"] = ks

    def second_diff(a):
        out = np.zeros(n)
        out[1:-1] = np.abs(a[2:] - 2 * a[1:-1] + a[:-2])
        return out

    jf = sum(second_diff(d[prefix + side + c]) for side in ("l", "r") for c in ("x", "y", "z")) / 6
    m["jit_foot"] = jf * 1000
    m["jit_knee"] = (second_diff(kl) + second_diff(kr)) / 2
    m["jit_waist"] = sum(second_diff(d[prefix + "w" + c]) for c in ("x", "y", "z")) / 3 * 1000
    if "hip0_x" in d.dtype.names:
        hip_off = np.hypot(d[prefix + "wx"] - d["hip0_x"], d[prefix + "wz"] - d["hip0_z"])
    m["hip_off"] = hip_off

    sf = []
    for side in ("l", "r"):
        sig = d[f"{side}_ins_sigma"]
        sel = (sig > 0) & (sig < 0.1)
        sf.append(np.where(sel, np.abs(d[prefix + side + "y"] - d[f"{side}_ins_y"]), np.nan))
    sf = np.vstack(sf)
    sw = np.full(n, np.nan)
    has = ~np.all(np.isnan(sf), axis=0)
    sw[has] = np.nanmean(sf[:, has], axis=0)
    m["swing_follow"] = sw

    low = (d["hmd_y"] - d["floor"]) < 0.5 * d["eye_h"]
    m["lying"] = np.where(low, np.maximum(0, d[prefix + "wy"] - d["hmd_y"] - 0.15), np.nan)
    return m


def summarize(m, sel):
    row = {}
    for name, _ in METRICS:
        v = m[name][sel]
        v = v[~np.isnan(v)]
        row[name] = (float(v.mean()) if len(v) else float("nan"), len(v))
    return row


def fmt(v):
    return f"{v:8.4f}" if abs(v) < 10 else f"{v:8.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--window", type=float, default=30)
    ap.add_argument("--rec", action="store_true")
    ap.add_argument("--worst", type=int, default=3)
    ap.add_argument("--quiet", action="store_true", help="overall line only")
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=",", names=True, invalid_raise=False)
    prefix = "rec_" if args.rec else ""
    d = d[(d[prefix + "valid"] > 0)]
    if len(d) == 0:
        print("no valid frames")
        return
    m = frame_metrics(d, prefix)
    t = d["t"]
    names = [n for n, _ in METRICS]

    overall = summarize(m, np.ones(len(d), dtype=bool))
    print(f"{len(d)} frames, {t[-1] - t[0]:.0f} s, {'recorded' if args.rec else 'replayed'} outputs")
    print("overall " + " ".join(f"{n}={overall[n][0]:.4g}" for n in names))
    if args.quiet:
        return

    starts = np.arange(np.floor(t[0] / args.window) * args.window, t[-1], args.window)
    rows = []
    for s in starts:
        sel = (t >= s) & (t < s + args.window)
        if sel.sum() < 50:
            continue
        rows.append((s, summarize(m, sel)))
    print("window " + " ".join(f"{n[:8]:>8}" for n in names))
    for s, r in rows:
        print(f"{s:6.0f} " + " ".join(fmt(r[n][0]) for n in names))
    # Worst windows per metric (relative to the overall value)
    print("worst windows:")
    for n in names:
        vals = [(r[n][0], s) for s, r in rows if not np.isnan(r[n][0])]
        vals.sort(reverse=True)
        print(f"  {n:13s} " + ", ".join(f"{s:.0f}s:{v:.3g}" for v, s in vals[: args.worst]))


if __name__ == "__main__":
    main()
