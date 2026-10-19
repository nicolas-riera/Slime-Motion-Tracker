"""Offline training pipeline (design: tools/tune/PIPELINE.md), all sessions at once.

  prep  default replay of every session (installed foot model): AFK mask, pose
        labels at the ground-truth times, sanity check of each session's GT
  A     foot model retrained on all sessions (AFK excluded, rare poses oversampled)
  B     evolution strategy on the solver constants, three-tracker mode
  C     same, ankles-only mode (SMT_NO_WAIST=1); the live waist output is the pelvis GT
  E     report.md: per-category scores, A/B halves, worst spans with local times

Score (lower is better, cm): per pose category, then the mean over categories
(equal weight, so kneeling or prone count as much as standing). Per category:
mean over legs of median + 0.3 p90 + 0.5 median when lifted, on the test blocks;
lying categories: 0.5 x that (Kinect is unreliable there) + 2 x the median lateral
offset of the pelvis from the head -> feet line. Ankles-only adds a pelvis
category (distance to the live three-tracker waist). Guardrails as in tune.py.

Usage: python tools/tune/pipeline.py sessions.json out_dir [--workers 16]
          [--gens-three 24] [--gens-ankles 16] [--population 16]
sessions.json: [{"session": s.jsonl, "gt": [gt.csv, ...], "calib": calib.npz}, ...]
Each stage writes its result in out_dir and is skipped when rerun.
"""
import argparse
import datetime
import glob
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import pandas as pd
from scipy.ndimage import median_filter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools", "ml"))
sys.path.insert(0, os.path.join(ROOT, "tools", "kinect_gt"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import dataset as ds  # noqa: E402
import eval_replay  # noqa: E402
import preview  # noqa: E402
import tune  # noqa: E402

PY = sys.executable
BIN = os.path.join(ROOT, "build", "Release")
ENGINE_MODEL = os.path.join(ROOT, "engine", "models", "foot_model.json")
LIFTED = 0.15
CATS = ("standing", "walking", "squat", "seated", "kneel", "kneel_trans", "feet_raised",
        "lying_back", "lying_prone", "lying_side")
LYING = ("lying_back", "lying_prone", "lying_side")
MIN_SAMPLES = 100
CTX = ("hmd_x", "hmd_y", "hmd_z", "hmd_pitch", "hmd_yaw", "floor", "eye_h", "lshin_up_y", "rshin_up_y",
       "pelvis_up_y", "pelvis_fwd_y", "l_gyro", "r_gyro", "rec_valid", "rec_wx", "rec_wy", "rec_wz")

# name: (default, low, high, scale); log scale unless "lin"
PARAMS = {n: (*v, "log") for n, v in tune.PARAMS.items()}
PARAMS.update({
    "kChestHead": (0.0, 0.0, 1.0, "lin"),
    "kUpperSpine": (0.5, 0.2, 0.8, "lin"),
    "kBedBelowHip": (0.08, 0.03, 0.2, "log"),
    "kBedSigma": (0.06, 0.03, 0.15, "log"),
    "kSitBack": (0.092, 0.04, 0.15, "log"),
    "kHipLiftSigma": (0.04, 0.02, 0.1, "log"),
    "kUnderHeightSigma": (0.1, 0.05, 0.2, "log"),
    "kFloorSnap": (0.06, 0.03, 0.12, "log"),
    "kSeatFloorSnap": (0.20, 0.06, 0.35, "log"),
    "kSeatSinkTime": (1.0, 0.2, 5.0, "log"),
})
# Mag heading filter constants (body_replay SMT_MAG_REPLAY=1), searched with --mag
MAG_PARAMS = {
    "magSigmaDeg": (8.0, 3.0, 20.0, "log"),
    "magCorrTime": (2.0, 0.5, 10.0, "log"),
    "magHeadingWalkDeg": (0.0645, 0.02, 0.3, "log"),
    "magRateWalkDeg": (0.00215, 0.0003, 0.01, "log"),
    "magNormSoft": (0.04, 0.015, 0.12, "log"),
    "magDipSoftDeg": (2.5, 1.0, 8.0, "log"),
    "magStillGrowth": (5.0, 1.0, 30.0, "log"),
}
MAG = os.environ.get("SMT_PIPE_MAG") == "1"
if MAG:
    PARAMS.update(MAG_PARAMS)
NAMES = list(PARAMS)
# Standing and walking are good already (user): a candidate that degrades them by
# more than STAND_TOL is penalized
STAND_TOL = 0.02


def _fwd(v, lo, hi, sc):
    return (v - lo) / (hi - lo) if sc == "lin" else (math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo))


DEFAULT = np.array([_fwd(*PARAMS[n][:1], *PARAMS[n][1:]) for n in NAMES])


def to_params(u):
    out = {}
    for n, v in zip(NAMES, np.clip(u, 0, 1)):
        _, lo, hi, sc = PARAMS[n]
        out[n] = float(lo + v * (hi - lo) if sc == "lin" else math.exp(math.log(lo) + v * (math.log(hi) - math.log(lo))))
    return out


def log(msg):
    print(f"[{datetime.datetime.now():%H:%M:%S}] {msg}", flush=True)


def interp(tq, t, v):
    r = np.interp(tq, t, v)
    r[(tq < t[0]) | (tq > t[-1])] = np.nan
    return r


# ---------------------------------------------------------------- sessions / GT

class Session:
    def __init__(self, spec):
        self.path = spec["session"]
        self.t0 = ds.first_time(self.path)
        m = re.search(r"(\d{8}_\d{6})", os.path.basename(self.path))
        self.start = datetime.datetime.strptime(m.group(1), "%Y%m%d_%H%M%S")
        self.camera = preview.KinectToWorld(spec["calib"]).points(np.zeros((1, 3)))[0]
        self.gt_files = spec["gt"] if isinstance(spec["gt"], list) else sorted(glob.glob(spec["gt"]))
        g = ds.load_gt(self.gt_files)
        self.t = g["t"]
        self.g, self.clean = {}, {}
        for leg in "lr":
            p = np.stack([g[f"{leg}_x"], g[f"{leg}_y"], g[f"{leg}_z"]], axis=1)
            valid = (g[f"{leg}_valid"] > 0) & np.all(np.isfinite(p), axis=1)
            clean = valid.copy()
            idx = np.flatnonzero(valid)
            tv, pv = self.t[idx], p[idx]
            lo, hi = np.searchsorted(tv, tv - 0.35), np.searchsorted(tv, tv + 0.35)
            med = np.array([np.median(pv[a:b], axis=0) for a, b in zip(lo, hi)])
            clean[idx] = np.linalg.norm(pv - med, axis=1) < 0.15
            self.g[leg], self.clean[leg] = p, clean
        rel = self.t - self.t[0]
        block = np.floor(rel / 300.0).astype(int)
        edge = (rel % 300.0 < 3.0) | (rel % 300.0 > 297.0)
        self.test = (block % 5 == 4) & ~edge
        self.half = (block // 5) % 2
        self.t_from = max(0.0, self.t[0] - self.t0 - 5)
        self.t_to = self.t[-1] - self.t0 + 5
        self.t_to = min(self.t_to, self.t_from + float(os.environ.get("SMT_PIPE_SPAN", 1e12)))  # smoke tests
        self.gt_ok = True

    def gt_debiased(self, leg, cam_bias):
        p = self.g[leg]
        u = self.camera - np.nan_to_num(p)
        u[:, 1] = 0
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-6)
        return p - cam_bias * u

    def local(self, t):
        return self.start + datetime.timedelta(seconds=float(t - self.t0))


_SESS = None


def init_worker(specs):
    global _SESS
    _SESS = [Session(s) for s in specs]


def replay_job(job):
    """One replay of one session. Returns the outputs at the GT times (+ context)."""
    i, params, model, no_waist, want_ctx, out_dir = job
    s = _SESS[i]
    env = dict(os.environ)
    env["SMT_TUNE"] = ",".join(f"{k}={v:.6g}" for k, v in params.items())
    if os.environ.get("SMT_TUNE_EXTRA"):  # first match wins: overrides the candidate
        env["SMT_TUNE"] = os.environ["SMT_TUNE_EXTRA"] + "," + env["SMT_TUNE"]
    env["SMT_NO_WAIST"] = "1" if no_waist else "0"
    env["SMT_MAG_REPLAY"] = "1" if MAG else "0"
    fd, csv = tempfile.mkstemp(suffix=".csv", dir=out_dir)
    os.close(fd)
    try:
        cmd = [os.path.join(BIN, "smt_body_replay.exe"), s.path, csv, "--from", str(s.t_from),
               "--to", str(s.t_to), "--model", model]
        subprocess.run(cmd, env=env, check=True, stdout=subprocess.DEVNULL)
        rp = load_csv(csv)
    except Exception as e:  # a crashed run is just a bad sample
        return {"error": str(e)}
    finally:
        os.remove(csv)
    tr = rp["t"] + s.t0
    out = {"feet": np.stack([interp(s.t, tr, rp[c]) for c in ("lx", "ly", "lz", "rx", "ry", "rz")], 1).astype(np.float32),
           "waist": np.stack([interp(s.t, tr, rp[c]) for c in ("wx", "wy", "wz")], 1).astype(np.float32)}
    m = eval_replay.frame_metrics(rp, "")
    out["guard"] = {k: v[0] for k, v in eval_replay.summarize(m, np.ones(len(rp), bool)).items()
                    if k in tune.GUARDRAILS}
    if want_ctx:
        out["ctx"] = {c: interp(s.t, tr, rp[c]) for c in CTX}
        out["afk"] = afk_mask(rp, s.t, s.t0)
        out["speed"] = hmd_speed(rp, s.t, s.t0)
    return out


def load_csv(path):
    """Replay CSV as float32 columns (t in float64): a 5 h session at full rate in
    16 parallel workers would not fit in memory as float64."""
    df = pd.read_csv(path, dtype=np.float32, engine="c")
    cols = {c: df[c].to_numpy() for c in df.columns}
    cols["t"] = pd.read_csv(path, usecols=["t"], dtype=np.float64)["t"].to_numpy()
    return tune.Columns(cols)


# ---------------------------------------------------------------- AFK and labels

def _grid(rp, t0, hz=10.0):
    tr = rp["t"] + t0
    tg = np.arange(tr[0], tr[-1], 1.0 / hz)
    return tg, {c: np.interp(tg, tr, rp[c]) for c in ("hmd_x", "hmd_y", "hmd_z", "hmd_pitch", "hmd_yaw",
                                                         "l_gyro", "r_gyro")}


def afk_mask(rp, tq, t0):
    """Headset taken off: HMD still (position std < 5 mm, rotation range < 0.5 deg
    over 5 s) while a tracker still moves. Span +-2 s, at the times tq."""
    tg, c = _grid(rp, t0)
    w = 50
    if len(tg) <= w:
        return np.zeros(len(tq), bool)
    sw = np.lib.stride_tricks.sliding_window_view
    pos = np.stack([c["hmd_x"], c["hmd_y"], c["hmd_z"]], 1)
    pstd = np.linalg.norm(sw(pos, w, axis=0).std(axis=2), axis=1)
    yaw = np.unwrap(np.radians(c["hmd_yaw"]))
    rng = lambda v: sw(v, w).max(1) - sw(v, w).min(1)  # noqa: E731
    rot = np.degrees(np.maximum(rng(yaw), rng(np.radians(c["hmd_pitch"]))))
    moving = (sw(np.maximum(c["l_gyro"], c["r_gyro"]), w) > 0.5).mean(1) > 0.1
    still = (pstd < 0.005) & (rot < 0.5) & moving
    afk = np.zeros(len(tg), bool)
    for k in np.flatnonzero(still):
        afk[k:k + w] = True
    pad = 20  # 2 s
    afk = np.convolve(afk, np.ones(2 * pad + 1), "same") > 0
    return np.interp(tq, tg, afk.astype(float), left=0, right=0) > 0.5


def hmd_speed(rp, tq, t0):
    tg, c = _grid(rp, t0)
    v = np.hypot(np.gradient(c["hmd_x"], tg), np.gradient(c["hmd_z"], tg))
    v = np.convolve(v, np.ones(10) / 10, "same")
    return np.interp(tq, tg, v)


def labels(s, ctx, afk, speed):
    """Pose category index per GT sample (-1 = AFK / unusable)."""
    h = (ctx["hmd_y"] - ctx["floor"]) / np.maximum(ctx["eye_h"], 0.5)
    pitch, pu, fwd = ctx["hmd_pitch"], ctx["pelvis_up_y"], ctx["pelvis_fwd_y"]
    su = np.stack([ctx["lshin_up_y"], ctx["rshin_up_y"]], 1)
    lying = (h < 0.55) & ((pu < 0.5) | (np.abs(pitch) > 40))
    ref = fwd[lying & (pitch > 45)]
    # the segment Z axis points backward: down when lying on the back (checked on the data when possible)
    sign = -np.sign(np.nanmedian(ref)) if len(ref) > 50 else 1.0
    gy = np.stack([s.g[leg][:, 1] - ctx["floor"] for leg in "lr"], 1)
    gy = np.where(np.isfinite(gy), gy, 0)
    raised = median_filter(gy.max(1), size=19) > LIFTED
    kneel = ~lying & (h > 0.4) & (h < 0.85) & (su.min(1) < 0.45) & (su.max(1) > 0.6)
    near = np.zeros(len(h), bool)
    kt = s.t[kneel]
    if len(kt):
        j = np.clip(np.searchsorted(kt, s.t), 1, len(kt) - 1)
        near = np.minimum(np.abs(s.t - kt[j - 1]), np.abs(s.t - kt[j])) < 1.5
    squat = ~lying & (h < 0.75) & (su.max(1) < 0.85)
    seated = ~lying & (h < 0.8)
    lab = np.full(len(h), CATS.index("standing"))
    for name, m in (("walking", speed > 0.35), ("seated", seated), ("squat", squat),
                    ("feet_raised", ~lying & raised & (speed < 0.35)), ("kneel_trans", near & ~kneel),
                    ("kneel", kneel), ("lying_side", lying), ("lying_back", lying & (fwd * sign < -0.5)),
                    ("lying_prone", lying & (fwd * sign > 0.5))):
        lab[m] = CATS.index(name)
    lab[afk | ~np.isfinite(h)] = -1
    return lab


# ---------------------------------------------------------------- scoring

class Scorer:
    def __init__(self, sessions, prep, cam_bias, ankles):
        self.S, self.prep, self.ankles = sessions, prep, ankles
        self.gt = [{leg: s.gt_debiased(leg, cam_bias) for leg in "lr"} for s in sessions]

    def per_sample(self, i, res):
        """Foot errors (N, 2) cm (NaN where not usable), lifted (N, 2), lateral and pelvis errors (cm)."""
        s, p = self.S[i], self.prep[i]
        ctx = p["ctx"]
        d, lifted = np.full((len(s.t), 2), np.nan), np.zeros((len(s.t), 2), bool)
        hmd = np.stack([ctx["hmd_x"], ctx["hmd_y"], ctx["hmd_z"]], 1)
        reach = 1.1 * ctx["eye_h"] + 0.1
        for k, leg in enumerate("lr"):
            e = np.linalg.norm(res["feet"][:, 3 * k:3 * k + 3] - self.gt[i][leg], axis=1) * 100
            # A foot farther from the head than the body is long: the Kinect tracked
            # something else (lying on the bed, out of its view)
            # ... or an ankle almost at eye level (the bed or the user's body seen as a foot)
            plausible = (np.linalg.norm(s.g[leg] - hmd, axis=1) < reach) & (s.g[leg][:, 1] < hmd[:, 1] - 0.25)
            ok = s.clean[leg] & s.gt_ok & plausible
            d[ok, k] = e[ok]
            lifted[:, k] = (s.g[leg][:, 1] - ctx["floor"]) > LIFTED
        hx, hz = ctx["hmd_x"], ctx["hmd_z"]
        mid = (self.gt[i]["l"] + self.gt[i]["r"]) / 2
        ax, az = mid[:, 0] - hx, mid[:, 2] - hz
        n = np.hypot(ax, az)
        lat = np.abs((res["waist"][:, 0] - hx) * az - (res["waist"][:, 2] - hz) * ax) / np.maximum(n, 1e-6) * 100
        lat[~(np.isfinite(d[:, 0]) & np.isfinite(d[:, 1])) | (n < 0.3)] = np.nan
        rec = np.stack([ctx["rec_wx"], ctx["rec_wy"], ctx["rec_wz"]], 1)
        pel = np.linalg.norm(res["waist"] - rec, axis=1) * 100
        pel[~(ctx["rec_valid"] > 0)] = np.nan
        return d, lifted, lat, pel

    def score(self, results, which="test", detail=False):
        rows = [self.per_sample(i, r) for i, r in enumerate(results)]
        lab = np.concatenate([p["lab"] for p in self.prep])
        sel = np.concatenate([s.test & ((s.half == 0) if which == "A" else (s.half == 1) if which == "B" else True)
                              for s in self.S])
        d = np.concatenate([r[0] for r in rows])
        lifted = np.concatenate([r[1] for r in rows])
        lat = np.concatenate([r[2] for r in rows])
        pel = np.concatenate([r[3] for r in rows])
        cats = {}
        for c, name in enumerate(CATS):
            m = sel & (lab == c)
            legs = []
            for k in range(2):
                v = m & np.isfinite(d[:, k])
                if v.sum() < MIN_SAMPLES:
                    continue
                e = d[v, k]
                lf = lifted[v, k]
                legs.append(np.median(e) + 0.3 * np.percentile(e, 90) + (0.5 * np.median(e[lf]) if lf.any() else 0))
            if not legs:
                continue
            sc = float(np.mean(legs))
            if name in LYING:
                lv = lat[m & np.isfinite(lat)]
                sc = 0.5 * sc + (2 * float(np.median(lv)) if len(lv) >= MIN_SAMPLES else 0)
            cats[name] = sc
        if self.ankles:
            v = sel & (lab >= 0) & np.isfinite(pel)
            if v.sum() >= MIN_SAMPLES:
                cats["pelvis"] = float(np.median(pel[v]) + 0.3 * np.percentile(pel[v], 90))
        total = float(np.mean(list(cats.values()))) if cats else (1e9 if which == "test" else float("nan"))
        return (total, cats) if detail else total


def stand_penalty(cats, default_cats):
    return float(sum(20 * max(0.0, cats[c] / default_cats[c] - 1 - STAND_TOL)
                     for c in ("standing", "walking") if c in cats and c in default_cats))


def penalty(results, bases):
    return float(np.mean([tune.penalty(r["guard"], b["guard"]) for r, b in zip(results, bases)]))


# ---------------------------------------------------------------- stages

def run_all(pool, specs_n, params, model, no_waist, out, want_ctx=False):
    jobs = [(i, params, model, no_waist, want_ctx, out) for i in range(specs_n)]
    return list(pool.map(replay_job, jobs))


def stage_prep(pool, S, out):
    path = os.path.join(out, "prep.npz")
    if os.path.exists(path):
        z = np.load(path, allow_pickle=True)
        return list(z["prep"]), z["results"].tolist()
    log("prep: default replay of every session (installed model)")
    res = run_all(pool, len(S), to_params(DEFAULT), ENGINE_MODEL, False, out, want_ctx=True)
    prep = []
    for s, r in zip(S, res):
        if "error" in r:
            raise SystemExit(f"default replay failed on {s.path}: {r['error']}")
        lab = labels(s, r["ctx"], r["afk"], r["speed"])
        prep.append({"ctx": r.pop("ctx"), "lab": lab})
        r.pop("afk"), r.pop("speed")
        counts = {CATS[c]: int((lab == c).sum()) for c in range(len(CATS))}
        log(f"  {os.path.basename(s.path)}: {len(lab)} GT samples, AFK {(lab < 0).mean() * 100:.1f}%, {counts}")
    np.savez(path, prep=np.array(prep, dtype=object), results=np.array(res, dtype=object))
    return prep, res


def check_gt(S, prep, res):
    """A session whose Kinect calibration failed would poison the training."""
    for s, p, r in zip(S, prep, res):
        lab = p["lab"]
        e = []
        for k, leg in enumerate("lr"):
            m = s.clean[leg] & np.isin(lab, [CATS.index("standing"), CATS.index("walking")])
            e.append(np.linalg.norm(r["feet"][m, 3 * k:3 * k + 3] - s.g[leg][m], axis=1))
        med = float(np.nanmedian(np.concatenate(e))) if sum(len(x) for x in e) else np.inf
        s.gt_ok = med < 0.35
        log(f"  GT check {os.path.basename(s.path)}: standing/walking median {med * 100:.1f} cm -> "
            f"{'ok' if s.gt_ok else 'REJECTED (Kinect GT unused, ankles-only pelvis only)'}")


def stage_a(S, prep, out):
    model = os.path.join(out, "models", "foot_model.json")
    if os.path.exists(model):
        return model
    os.makedirs(os.path.dirname(model), exist_ok=True)
    parts = []
    for i, (s, p) in enumerate(zip(S, prep)):
        if not s.gt_ok:
            continue
        dump = os.path.join(out, f"s{i}.bin")
        if not os.path.exists(dump):
            log(f"A: ml_dump {os.path.basename(s.path)}")
            subprocess.run([os.path.join(BIN, "smt_ml_dump.exe"), s.path, dump], check=True,
                           stdout=subprocess.DEVNULL)
        log(f"A: dataset {os.path.basename(s.path)}")
        ds.CAMERA = s.camera
        D = ds.build(lags=ds.default_lags(), dump=dump, gt_files=s.gt_files)
        fr, idx = D["fr"], D["idx"]
        c, sn = np.cos(fr.heading[idx]), np.sin(fr.heading[idx])
        off = fr.hmd[idx] * np.array([1, 0, 1])
        rec = np.concatenate([ds.to_heading(D["rec"][leg] - off, c, sn) for leg in "lr"], axis=1)
        j = np.clip(np.searchsorted(s.t, D["gt"]["t"]), 0, len(s.t) - 1)
        lab = p["lab"][j]
        M, Mc = D["M"].copy(), D["Mc"].copy()
        M[lab < 0], Mc[lab < 0] = False, False
        parts.append(dict(X=D["X"].astype(np.float32), Y=D["Y"], M=M, Mc=Mc, t=D["gt"]["t"], rec=rec,
                          replay=rec * np.nan, cam=D["cam"], DR=D["DR"], lab=lab))
        del D
    z = {k: np.concatenate([q[k] for q in parts]) for k in parts[0]}
    lab = z.pop("lab")
    n = np.array([(lab == c).sum() for c in range(len(CATS))], float)
    w = np.sqrt(n.max() / np.maximum(n, 1))
    z["W"] = np.where(lab >= 0, np.clip(np.round(w[np.maximum(lab, 0)]), 1, 8), 1).astype(int)
    z["lags"] = np.array(ds.default_lags())
    data = os.path.join(out, "ds_all.npz")
    np.savez(data, **z)
    del z
    pt = os.path.join(out, "models", "foot_model.pt")
    log("A: training the foot model")
    with open(os.path.join(out, "models", "train.log"), "w") as fh:
        subprocess.run([PY, os.path.join(ROOT, "tools", "ml", "train.py"), "--data", data, "--clean", "--cambias",
                        "--mirror", "--hetero", "--hidden", "128", "128", "--lagidx",
                        "0,1,2,3,4,6,8,10,13,16,20,23,24,25,26,27,28,29", "--trainval", "--out", pt],
                       check=True, stdout=fh, stderr=subprocess.STDOUT)
    first = next(i for i, s in enumerate(S) if s.gt_ok)
    subprocess.run([PY, os.path.join(ROOT, "tools", "ml", "export.py"), pt, model, "--ref",
                    os.path.join(out, "models", "ref.bin"), "--dump", os.path.join(out, f"s{first}.bin"),
                    "--note", "tools/tune/pipeline.py, all sessions"], check=True)
    os.remove(data)
    return model


def evolve(pool, S, prep, out, name, model, no_waist, gens, pop, seed):
    best_path = os.path.join(out, f"best_{name}.json")
    if os.path.exists(best_path):
        with open(best_path) as fh:
            b = json.load(fh)
        if b.get("done"):
            return b
    cam_bias = json.load(open(model)).get("camBias", 0.0)
    scorer = Scorer(S, prep, cam_bias, no_waist)
    trials = open(os.path.join(out, f"trials_{name}.jsonl"), "w")
    rng = np.random.default_rng(seed)
    base = run_all(pool, len(S), to_params(DEFAULT), model, no_waist, out)
    if any("error" in r for r in base):
        raise SystemExit(f"{name}: default run failed")
    acc, cats = scorer.score(base, detail=True)
    log(f"{name}: defaults {acc:.2f} (A {scorer.score(base, 'A'):.2f}, B {scorer.score(base, 'B'):.2f}) {cats}")
    best = {"score": acc, "params": to_params(DEFAULT), "cats": cats, "default_score": acc, "default_cats": cats}
    mean, sigma = DEFAULT.copy(), 0.15
    mu = pop // 3
    wts = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    wts /= wts.sum()
    for gen in range(1, gens + 1):
        U = [np.clip(mean + sigma * rng.standard_normal(len(NAMES)), 0, 1) for _ in range(pop)]
        jobs = [(i, to_params(u), model, no_waist, False, out) for u in U for i in range(len(S))]
        flat = list(pool.map(replay_job, jobs))
        scored = []
        for k, u in enumerate(U):
            res = flat[k * len(S):(k + 1) * len(S)]
            if any("error" in r for r in res):
                sc, rec = 1e9, {"error": True}
            else:
                acc, cats = scorer.score(res, detail=True)
                pen = penalty(res, base) + stand_penalty(cats, best["default_cats"])
                sc = acc + pen
                rec = {"accuracy": acc, "penalty": pen, "A": scorer.score(res, "A"), "B": scorer.score(res, "B"),
                       "cats": cats}
            trials.write(json.dumps({"gen": gen, "score": sc, "params": to_params(u), **rec}) + "\n")
            scored.append((sc, k, rec))
        trials.flush()
        scored.sort(key=lambda x: x[0])
        improved = scored[0][0] < best["score"]
        if improved:
            k = scored[0][1]
            best.update(score=scored[0][0], params=to_params(U[k]), **scored[0][2])
            with open(best_path, "w") as fh:
                json.dump(best, fh, indent=1)
        mean = np.sum([w * U[k] for w, (_, k, _) in zip(wts, scored[:mu])], axis=0)
        sigma = float(np.clip(sigma * (1.15 if improved else 0.92), 0.03, 0.3))
        log(f"{name} gen {gen}/{gens}: gen best {scored[0][0]:.2f}, overall {best['score']:.2f} "
            f"(A {best.get('A', 0):.2f} B {best.get('B', 0):.2f}), sigma {sigma:.3f}")
    best["done"] = True
    with open(best_path, "w") as fh:
        json.dump(best, fh, indent=1)
    return best


def worst_spans(S, prep, scorer, results, n=20, span=10.0):
    rows = []
    for i, (s, r) in enumerate(zip(S, results)):
        d = np.nanmean(scorer.per_sample(i, r)[0], axis=1)
        b = np.floor((s.t - s.t[0]) / span).astype(int)
        for k in np.unique(b):
            m = (b == k) & np.isfinite(d) & (prep[i]["lab"] >= 0)
            if m.sum() < 100:
                continue
            lab = np.bincount(prep[i]["lab"][m], minlength=len(CATS)).argmax()
            rows.append((float(np.median(d[m])), s.local(s.t[m][0]).strftime("%Y-%m-%d %H:%M:%S"), CATS[lab]))
    return sorted(rows, reverse=True)[:n]


def report(pool, S, prep, prep_res, out, model, best3, besta):
    lines = ["# Training report", "", f"{datetime.datetime.now():%Y-%m-%d %H:%M}", ""]
    for s in S:
        lab = prep[S.index(s)]["lab"]
        lines.append(f"- {os.path.basename(s.path)}: GT {'ok' if s.gt_ok else 'REJECTED'}, AFK excluded "
                     f"{(lab < 0).mean() * 100:.1f}% of the GT samples")
    sc_old = Scorer(S, prep, json.load(open(ENGINE_MODEL)).get("camBias", 0.0), False)
    sc_new = Scorer(S, prep, json.load(open(model)).get("camBias", 0.0), False)
    runs = {"installed (old model, defaults)": (sc_old, prep_res),
            "new model, defaults": (sc_new, run_all(pool, len(S), to_params(DEFAULT), model, False, out)),
            "new model, best three": (sc_new, run_all(pool, len(S), best3["params"], model, False, out))}
    sa = Scorer(S, prep, json.load(open(model)).get("camBias", 0.0), True)
    runs["ankles-only, defaults"] = (sa, run_all(pool, len(S), to_params(DEFAULT), model, True, out))
    runs["ankles-only, best"] = (sa, run_all(pool, len(S), besta["params"], model, True, out))
    cols = list(CATS) + ["pelvis"]
    lines += ["", "## Per category (cm, lower is better, test blocks)", "",
              "| run | total | A | B | " + " | ".join(cols) + " |", "|" + "---|" * (len(cols) + 4)]
    for name, (sc, res) in runs.items():
        tot, cats = sc.score(res, detail=True)
        lines.append(f"| {name} | {tot:.2f} | {sc.score(res, 'A'):.2f} | {sc.score(res, 'B'):.2f} | "
                     + " | ".join(f"{cats[c]:.1f}" if c in cats else "-" for c in cols) + " |")
    lines += ["", "## Worst 10 s spans (best three-tracker run)", "", "| median foot error cm | local time | pose |",
              "|---|---|---|"]
    for e, t, c in worst_spans(S, prep, sc_new, runs["new model, best three"][1]):
        lines.append(f"| {e:.1f} | {t} | {c} |")
    for name, b in (("three", best3), ("ankles", besta)):
        lines += ["", f"## Best constants, {name}", "", "| name | default | best |", "|---|---|---|"]
        for n in NAMES:
            lines.append(f"| {n} | {PARAMS[n][0]:.4g} | {b['params'][n]:.4g} |")
    lines += ["", "Not covered yet (need solver code): knee model, chest model, pose classifier as solver prior, "
              "mag trust model / field-anomaly rejection (PIPELINE.md stage D and section 6)."]
    with open(os.path.join(out, "report.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    log("report written")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions")
    ap.add_argument("out")
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--gens-three", type=int, default=24)
    ap.add_argument("--gens-ankles", type=int, default=16)
    ap.add_argument("--population", type=int, default=16)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--eval", action="store_true",
                    help="only score the current build with the defaults (three and ankles-only), per category")
    ap.add_argument("--model", default=ENGINE_MODEL, help="foot model for --eval")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    with open(args.sessions) as fh:
        specs = json.load(fh)
    t_start = time.time()
    S = [Session(s) for s in specs]
    with ProcessPoolExecutor(args.workers, initializer=init_worker, initargs=(specs,)) as pool:
        prep, prep_res = stage_prep(pool, S, args.out)
        check_gt(S, prep, prep_res)
        if not any(s.gt_ok for s in S):
            raise SystemExit("no usable Kinect ground truth")
    if args.eval:
        evaluate(S, prep, specs, args)
        return
    # the workers must see the GT verdicts: restart the pool after stage A
    model = stage_a(S, prep, args.out)
    log(f"A done: {model}")
    with ProcessPoolExecutor(args.workers, initializer=init_worker_ok, initargs=(specs, [s.gt_ok for s in S])) as pool:
        best3 = evolve(pool, S, prep, args.out, "three", model, False, args.gens_three, args.population, args.seed)
        besta = evolve(pool, S, prep, args.out, "ankles", model, True, args.gens_ankles, args.population,
                       args.seed + 1)
        report(pool, S, prep, prep_res, args.out, model, best3, besta)
    log(f"pipeline done in {(time.time() - t_start) / 3600:.1f} h")


def evaluate(S, prep, specs, args):
    cam_bias = json.load(open(args.model)).get("camBias", 0.0)
    with ProcessPoolExecutor(args.workers, initializer=init_worker_ok, initargs=(specs, [s.gt_ok for s in S])) as pool:
        for name, no_waist in (("three", False), ("ankles", True)):
            sc = Scorer(S, prep, cam_bias, no_waist)
            res = run_all(pool, len(S), to_params(DEFAULT), args.model, no_waist, args.out)
            if any("error" in r for r in res):
                log(f"{name}: replay failed {[r.get('error') for r in res]}")
                continue
            tot, cats = sc.score(res, detail=True)
            log(f"{name}: total {tot:.2f} (A {sc.score(res, 'A'):.2f}, B {sc.score(res, 'B'):.2f}) "
                + " ".join(f"{k} {v:.1f}" for k, v in cats.items()))


def init_worker_ok(specs, ok):
    init_worker(specs)
    for s, v in zip(_SESS, ok):
        s.gt_ok = v


if __name__ == "__main__":
    main()
