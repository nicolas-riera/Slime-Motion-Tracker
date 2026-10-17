"""Reads the Kinect depth recordings written by slimelite_kinect_recorder.

As a library:
    rec = DepthRecording("kinect_20261016_235103.depth")
    for t, rt, depth_mm in rec.frames():   # depth_mm: (height, width) uint16
        points = rec.points(depth_mm)       # (N, 3) camera space, meters

As a script, writes a preview image of one frame:
    python tools/kinect_depth.py rec.depth out.png [--index N]
"""
import argparse
import ctypes
import os
import struct

import numpy as np

COMPRESS_ALGORITHM_XPRESS_HUFF = 4


class _Decompressor:
    def __init__(self):
        self.lib = ctypes.WinDLL("cabinet.dll")
        self.handle = ctypes.c_void_p()
        if not self.lib.CreateDecompressor(COMPRESS_ALGORITHM_XPRESS_HUFF, None, ctypes.byref(self.handle)):
            raise OSError("CreateDecompressor failed")

    def decompress(self, data, size):
        out = ctypes.create_string_buffer(size)
        got = ctypes.c_size_t()
        ok = self.lib.Decompress(self.handle, data, ctypes.c_size_t(len(data)), out, ctypes.c_size_t(size),
                                 ctypes.byref(got))
        if not ok or got.value != size:
            raise OSError("Decompress failed")
        return out.raw

    def __del__(self):
        if getattr(self, "handle", None):
            self.lib.CloseDecompressor(self.handle)


class DepthRecording:
    def __init__(self, path):
        self.path = path
        with open(path, "rb") as f:
            magic, self.version, self.width, self.height = struct.unpack("<4sIII", f.read(16))
        if magic != b"SLKD" or self.version not in (1, 2):
            raise ValueError(f"{path}: not a depth recording")
        table_path = os.path.splitext(path)[0] + ".table"
        self.table = None
        if os.path.exists(table_path):
            self.table = np.fromfile(table_path, dtype=np.float32).reshape(self.height, self.width, 2)
        self._dec = _Decompressor()

    def frames(self, start=0.0, end=float("inf")):
        """Yields (t, rt, depth) with t the host time in seconds."""
        size = self.width * self.height * 2
        with open(self.path, "rb") as f:
            f.seek(16)
            while True:
                head = f.read(20)
                if len(head) < 20:
                    return
                t, rt, n = struct.unpack("<ddI", head)
                if t < start:  # skip without reading (parallel extraction of time chunks)
                    f.seek(n, 1)
                    continue
                if t > end:
                    return
                data = f.read(n)
                if len(data) < n:
                    return
                depth = np.frombuffer(self._dec.decompress(data, size), dtype=np.uint16)
                depth = depth.reshape(self.height, self.width)
                if self.version >= 2:
                    depth = np.cumsum(depth, axis=1, dtype=np.uint16)
                yield t, rt, depth

    def points(self, depth, step=1):
        """Camera space points (meters) of the valid pixels, optionally subsampled."""
        if self.table is None:
            raise ValueError("no camera table for this recording")
        d = depth[::step, ::step].astype(np.float32) * 0.001
        ray = self.table[::step, ::step]
        valid = d > 0
        z = d[valid]
        return np.stack([ray[..., 0][valid] * z, ray[..., 1][valid] * z, z], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("depth")
    ap.add_argument("png")
    ap.add_argument("--index", type=int, default=0)
    args = ap.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rec = DepthRecording(args.depth)
    for i, (t, rt, depth) in enumerate(rec.frames()):
        if i == args.index:
            break
    plt.figure(figsize=(10, 8))
    plt.imshow(np.where(depth > 0, depth, np.nan) / 1000.0, cmap="turbo")
    plt.colorbar(label="depth (m)")
    plt.title(f"frame {args.index}, t = {t:.3f}")
    plt.savefig(args.png, dpi=60)


if __name__ == "__main__":
    main()
