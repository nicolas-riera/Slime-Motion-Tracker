"""Prototype of the orientation-aided magnetometer calibration, run on a recording.

Model, for every mag sample k with sensor-to-earth rotation R_k (from the 6D fusion):

    R_k^T m_e = K m_raw,k + c

m_e is the earth field in the (yaw-arbitrary) earth frame, K a 3x3 matrix that
absorbs gain, soft iron and the unknown mag-to-IMU axis mapping, c = -K b the
hard-iron term. Everything is linear in (K, c, m_e), so the solution is the
smallest right singular vector of the stacked system, scaled so |m_e| = 1.

Usage: python tools/mag_calib_proto.py recordings/mag_calib_B4E6.jsonl
"""
import json
import math
import sys

import numpy as np


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def load(path):
    rows = [json.loads(l) for l in open(path)]
    t0 = rows[0]["t"]
    quats, mags = [], []
    for r in rows:
        if r["kind"] == "rot":
            quats.append((r["t"] - t0, r["q"]))
        elif r["kind"] == "mag":
            mags.append((r["t"] - t0, r["m"]))
    return quats, mags


def pair(quats, mags):
    """Attach to each mag sample the latest rotation received before it."""
    out, j = [], 0
    for t, m in mags:
        while j + 1 < len(quats) and quats[j + 1][0] <= t:
            j += 1
        if quats and quats[j][0] <= t and t - quats[j][0] < 0.25:
            out.append((t, quat_to_mat(*quats[j][1]), np.array(m, float)))
    return out


def solve(samples):
    rows = []
    for _, R, m in samples:
        Rt = R.T
        for i in range(3):
            row = np.zeros(15)
            row[3 * i : 3 * i + 3] = m
            row[9 + i] = 1.0
            row[12:15] = -Rt[i]
            rows.append(row)
    A = np.array(rows)
    # Scale columns so raw counts (~1e3) and unit terms are comparable
    scale = np.ones(15)
    scale[:9] = 1e-3
    _, s, vt = np.linalg.svd(A * scale, full_matrices=False)
    x = vt[-1] * scale
    K = x[:9].reshape(3, 3)
    c = x[9:12]
    me = x[12:15]
    n = np.linalg.norm(me)
    K, c, me = K / n, c / n, me / n
    if me[2] > 0:  # northern hemisphere: the field points down (earth frame is Z-up)
        K, c, me = -K, -c, -me
    return K, c, me, s


def heading_stats(samples, K, c):
    heads, norms, dips = [], [], []
    for _, R, m in samples:
        v = R @ (K @ m + c)
        norms.append(np.linalg.norm(v))
        dips.append(math.degrees(math.asin(-v[2] / np.linalg.norm(v))))
        heads.append(math.degrees(math.atan2(v[1], v[0])))
    h = np.unwrap(np.radians(heads))
    return np.degrees(h), np.array(norms), np.array(dips)


def main():
    quats, mags = load(sys.argv[1])
    samples = pair(quats, mags)
    print(f"{len(quats)} rotations, {len(mags)} mag samples, {len(samples)} paired")
    raw = np.array([m for _, _, m in samples])
    print("raw min", raw.min(0), "max", raw.max(0))

    cut = float(sys.argv[2]) if len(sys.argv) > 2 else None
    fit = [s for s in samples if cut is None or s[0] < cut]
    K, c, me, sv = solve(fit)
    b = -np.linalg.solve(K, c)
    print("singular values (last 3):", sv[-3:])
    print("hard iron b:", b.round(1))
    print("K * 1000:\n", (K * 1000).round(3))
    U, S, Vt = np.linalg.svd(K)
    print("K gains (counts per unit field):", (1 / S).round(1), " det sign:", np.sign(np.linalg.det(K)))
    print("closest axis mapping (rounded U@Vt):\n", (U @ Vt).round(2))
    print("earth field dip: %.1f deg" % math.degrees(math.asin(-me[2])))

    heads, norms, dips = heading_stats(samples, K, c)
    print("norm   mean %.3f  std %.3f" % (norms.mean(), norms.std()))
    print("dip    mean %.1f  std %.2f deg" % (dips.mean(), dips.std()))
    print("heading std %.2f deg (should be small: yaw drift + calibration error)" % heads.std())
    # Per-second summary to look at the disturbance at the end of the recording
    t = np.array([s[0] for s in samples])
    for sec in range(int(t[-1]) + 1):
        sel = (t >= sec) & (t < sec + 1)
        if sel.any():
            print(f"t={sec:3d}s  heading {heads[sel].mean():8.2f}  norm {norms[sel].mean():.3f}  dip {dips[sel].mean():6.2f}")


if __name__ == "__main__":
    main()
