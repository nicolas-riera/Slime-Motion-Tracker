"""Plots a smt_body_replay CSV: recorded engine outputs against the replayed
solver, with the inputs that matter (segment tilts, accelerations, swing state).

Usage: python tools/plot_replay.py replay.csv out.png [--from S] [--to S]
"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("png")
    ap.add_argument("--from", dest="t_from", type=float, default=-1e18)
    ap.add_argument("--to", dest="t_to", type=float, default=1e18)
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=",", names=True)
    d = d[(d["t"] >= args.t_from) & (d["t"] <= args.t_to)]
    t = d["t"]

    fig, ax = plt.subplots(6, 1, figsize=(20, 22), sharex=True)
    a = ax[0]
    a.set_title("heights (m): recorded dotted, replayed solid")
    a.plot(t, d["hmd_y"], "k", label="hmd")
    a.plot(t, d["floor"], "k:", lw=0.8)
    for key, color in (("w", "C0"), ("l", "C1"), ("r", "C2")):
        a.plot(t, d[f"rec_{key}y"], ":", color=color)
        a.plot(t, d[f"{key}y"], color=color, label={"w": "waist", "l": "left ankle", "r": "right ankle"}[key])
    a.legend(loc="upper left")

    a = ax[1]
    a.set_title("knee angles (deg): recorded dotted, replayed solid; planted as steps")
    a.plot(t, d["rec_kl"], "C1:")
    a.plot(t, d["rec_kr"], "C2:")
    a.plot(t, d["kl"], "C1", label="left")
    a.plot(t, d["kr"], "C2", label="right")
    a.plot(t, d["pl"] * 10 - 20, "C1", lw=0.8)
    a.plot(t, d["pr"] * 10 - 35, "C2", lw=0.8)
    a.legend(loc="upper left")

    a = ax[2]
    a.set_title("segment up axis, vertical component (1 upright, 0 horizontal)")
    a.plot(t, d["pelvis_up_y"], "C0", label="pelvis")
    a.plot(t, d["lshin_up_y"], "C1", label="left shin")
    a.plot(t, d["rshin_up_y"], "C2", label="right shin")
    a.plot(t, d["standing"], "k--", label="standing factor")
    a.legend(loc="upper left")

    a = ax[3]
    a.set_title("ankle linear acceleration (m/s2): vertical solid, norm dotted")
    a.plot(t, d["l_accel_y"], "C1", label="left")
    a.plot(t, d["r_accel_y"], "C2", label="right")
    a.plot(t, d["l_accel"], "C1:", lw=0.8)
    a.plot(t, d["r_accel"], "C2:", lw=0.8)
    a.legend(loc="upper left")

    a = ax[4]
    a.set_title("swing dead reckoning height (m) and its sigma; anchors dashed")
    for side, color in (("l", "C1"), ("r", "C2")):
        sig = d[f"{side}_ins_sigma"]
        y = np.where(sig > 0, d[f"{side}_ins_y"], np.nan)
        a.plot(t, y, color=color, label=f"{side} ins y")
        a.fill_between(t, y - sig, y + sig, color=color, alpha=0.15)
        a.plot(t, np.where(d[f"p{side}"] > 0, d[f"{side}_anchor_y"], np.nan), "--", color=color)
    a.set_ylim(-0.2, 1.2)
    a.legend(loc="upper left")

    a = ax[5]
    a.set_title("horizontal distance from the HMD (m): recorded dotted, replayed solid")
    for key, color in (("w", "C0"), ("l", "C1"), ("r", "C2")):
        for prefix, style in (("rec_", ":"), ("", "-")):
            dist = np.hypot(d[f"{prefix}{key}x"] - d["hmd_x"], d[f"{prefix}{key}z"] - d["hmd_z"])
            a.plot(t, dist, style, color=color)

    for a in ax:
        a.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(args.png, dpi=55)


if __name__ == "__main__":
    main()
