"""Exports a trained pose model (train_pose.py checkpoint) for the C++ runtime
(core/src/pose_model.cpp).

  python tools/pose_model/export.py best.pt pose_model.bin --body-model smplh/neutral/model.npz

File: "SLPM", uint32 header length, JSON header, then float16 tensors (little
endian, row major). Weight norm is folded (w = g v / |v| per row); the shape head
is dropped (the runtime uses the user's own skeleton).
"""
import argparse
import json
import struct

import numpy as np
import torch

PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt")
    ap.add_argument("out")
    ap.add_argument("--body-model", required=True)
    ap.add_argument("--name", default="")
    args = ap.parse_args()
    sd = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    sd = sd.get("state_dict", sd)
    tensors = {}
    for k, v in sd.items():
        if k.startswith("shape_est"):
            continue
        if k.endswith("_g"):
            continue
        if k.endswith("_v"):
            base = k[:-2]
            g = sd[base + "_g"].double()
            v = v.double()
            v = g * v / v.norm(dim=1, keepdim=True)
            k = base
        tensors[k] = v.detach().double().numpy()
    m = np.load(args.body_model, allow_pickle=True)
    v = m["v_template"]
    J = m["J_regressor"][:22] @ v
    rest = np.array([J[j] - (J[PARENTS[j]] if PARENTS[j] >= 0 else J[j]) for j in range(22)])
    header = {"format": "smt_pose_model", "version": 1, "name": args.name,
              "arch": {"hidden": 256, "heads": 8, "layers": 3, "blocks": 2, "ffn": 2048},
              "hz": 60, "parents": PARENTS, "rest_offsets": rest.round(6).tolist(),
              "model_height": float(v[:, 1].max() - v[:, 1].min()), "tensors": []}
    blob = bytearray()
    for k, a in tensors.items():
        header["tensors"].append({"name": k, "shape": list(a.shape), "offset": len(blob)})
        blob += a.astype("<f2").tobytes()
    h = json.dumps(header).encode()
    with open(args.out, "wb") as f:
        f.write(b"SLPM" + struct.pack("<I", len(h)) + h + bytes(blob))
    n = sum(a.size for a in tensors.values())
    print(f"{len(tensors)} tensors, {n} parameters, {len(blob) / 1e6:.1f} MB -> {args.out}")


if __name__ == "__main__":
    main()
