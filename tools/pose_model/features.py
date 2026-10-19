"""Sparse-sensor features for a learned full-body pose model (HMD-Poser input layout).

Reads the per-frame inputs dumped by body_replay (--dump inputs.bin) and builds the
135 features of HMD-Poser (github.com/Pico-AI-Team/HMD-Poser, prepare_data.py) at
60 Hz, in the AMASS world frame (Z up, metres):

  [0:36]    global rotations (6D: first two matrix columns) of head, left wrist,
            right wrist, left shin, right shin, pelvis (SMPL joints 15, 20, 21, 4, 5, 0)
  [36:72]   their rotation velocities R(t-1)^T R(t), 6D
  [72:81]   head, left wrist, right wrist positions
  [81:90]   their position deltas
  [90:102]  wrist rotations in the head frame, 6D
  [102:114] their rotation velocities
  [114:120] wrist positions in the head frame
  [120:126] their deltas
  [126:135] linear accelerations (m/s^2, world, no gravity) of the left shin, right
            shin and pelvis

Conventions. SteamVR world: Y up. Our segment orientations (and the HMD) are the
identity when standing straight facing -Z (body calibration). SMPL joint rotations
are the identity in the rest pose, facing +Z with Y up, and AMASS roots carry the
Y-up to Z-up rotation. Hence  R_amass = Rx(90) * R_ours * Ry(180)  and
p_amass = Rx(90) * p_ours, with the floor moved to z = 0.

Hands: the controller pose is not the wrist pose. At the body calibration stand
capture (arms hanging, palms facing the thighs) each wrist is assumed in that rest
pose; the constant controller-to-wrist rotation is measured there. The position
offset is a fixed guess (the wrist ~8 cm behind the controller origin).
"""
import json

import numpy as np
from scipy.spatial.transform import Rotation as R

RX90 = R.from_euler("x", 90, degrees=True)
RY180 = R.from_euler("y", 180, degrees=True)
HZ = 60.0
WRIST_OFFSET = np.array([0.0, 0.0, 0.08])  # controller frame (+Z = backward)


def load_dump(path):
    names = json.load(open(path + ".json"))["columns"]
    a = np.fromfile(path, dtype=np.float64).reshape(-1, len(names))
    return {n: a[:, k] for k, n in enumerate(names)}


def quat(d, p):
    """(N,) columns pqw, pqx, ... -> scipy Rotation (x, y, z, w order)."""
    return R.from_quat(np.stack([d[p + "qx"], d[p + "qy"], d[p + "qz"], d[p + "qw"]], 1))


def sixd(rot):
    m = rot.as_matrix()
    return np.concatenate([m[:, :, 0], m[:, :, 1]], axis=1)


def to_amass_rot(rot):
    return RX90 * rot * RY180


def to_amass_pos(p, floor):
    q = p.copy()
    q[:, 1] -= floor
    return RX90.apply(q)


def resample(d, hz=HZ):
    """Nearest earlier frame at a fixed rate (the engine frames are ~90 Hz)."""
    t = d["t"]
    tq = np.arange(t[0], t[-1], 1.0 / hz)
    idx = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, len(t) - 1)
    out = {k: v[idx] for k, v in d.items()}
    out["t"] = tq
    return out


def wrist_calibration(d, t_stand):
    """Controller -> wrist rotations from the stand capture (arms down, palms in)."""
    k = int(np.argmin(np.abs(d["t"] - t_stand)))
    head = quat(d, "h")[k]
    yaw = R.from_euler("y", head.as_euler("YXZ")[0])  # body heading (our frame)
    cal = []
    for side, c in ((0, "c0"), (1, "c1")):
        # SMPL rest pose arms point along +-X; hanging down = rotation about the
        # body's forward axis (+Z in the SMPL frame): -90 deg left, +90 deg right
        rest = R.from_euler("z", -90 if side == 0 else 90, degrees=True)
        expected = RX90 * yaw * RY180 * rest
        ctrl = to_amass_rot(quat(d, c)[k:k + 1]) * RY180.inv()  # controller in AMASS world
        cal.append(ctrl.inv() * expected)
    return cal


def build(d, t_stand, left_role="c0"):
    """Features (N, 135) at 60 Hz, plus the head position/rotation (AMASS) and times."""
    d = resample(d)
    n = len(d["t"])
    floor = d["floor"]
    head_rot = to_amass_rot(quat(d, "h"))
    # The HMD sits in front of the eyes; the SMPL head joint is ~10 cm back, ~8 cm down
    head_pos = to_amass_pos(np.stack([d["hx"], d["hy"], d["hz"]], 1)
                            + quat(d, "h").apply(np.array([0.0, -0.08, 0.10])), floor)
    cal = wrist_calibration(d, t_stand)
    rots, pos = [head_rot], [head_pos]
    for side, c in enumerate(("c0", "c1") if left_role == "c0" else ("c1", "c0")):
        ctrl_ours = quat(d, c)
        ctrl = to_amass_rot(ctrl_ours) * RY180.inv()
        rots.append(ctrl * cal[side][0])
        p = np.stack([d[c + "x"], d[c + "y"], d[c + "z"]], 1) + ctrl_ours.apply(WRIST_OFFSET)
        pos.append(to_amass_pos(p, floor))
    for p in ("l", "r", "w"):
        rots.append(to_amass_rot(quat(d, p)))
    six = np.concatenate([sixd(r) for r in rots], 1)  # head, lw, rw, lshin, rshin, pelvis
    vel = []
    for r in rots:
        m = r.as_matrix()
        v = np.einsum("nji,njk->nik", np.concatenate([m[:1], m[:-1]]), m)  # R(t-1)^T R(t)
        vel.append(sixd(R.from_matrix(v)))
    vel = np.concatenate(vel, 1)
    P = np.concatenate(pos, 1)  # (n, 9)
    dP = np.diff(P, axis=0, prepend=P[:1])
    hm = head_rot.as_matrix()
    hand_rot = [R.from_matrix(np.einsum("nji,njk->nik", hm, rots[k].as_matrix())) for k in (1, 2)]
    hand_six = np.concatenate([sixd(r) for r in hand_rot], 1)
    hand_vel = []
    for r in hand_rot:
        m = r.as_matrix()
        hand_vel.append(sixd(R.from_matrix(np.einsum("nji,njk->nik", np.concatenate([m[:1], m[:-1]]), m))))
    hand_vel = np.concatenate(hand_vel, 1)
    hand_pos = np.concatenate([np.einsum("ni,nij->nj", pos[k] - head_pos, hm) for k in (1, 2)], 1)
    dhand = np.diff(hand_pos, axis=0, prepend=hand_pos[:1])
    acc = np.concatenate([RX90.apply(np.stack([d[p + "ax"], d[p + "ay"], d[p + "az"]], 1))
                          for p in ("l", "r", "w")], 1)
    X = np.concatenate([six, vel, P, dP, hand_six, hand_vel, hand_pos, dhand, acc], 1).astype(np.float32)
    assert X.shape[1] == 135
    valid = (d["hmd_valid"] > 0) & (d["l_valid"] > 0) & (d["r_valid"] > 0) & (d["c0_valid"] > 0) & (d["c1_valid"] > 0)
    return {"X": X, "t": d["t"], "valid": valid, "head_pos": head_pos, "head_rot": head_rot, "floor": floor,
            "d": d}
