# Model archive

Every trained model version is kept here, never overwritten. `engine/models/` (deployed with the driver) holds
the version in use; a new version is copied here before it replaces it.

| File | Date | Training data | Notes |
|---|---|---|---|
| (v1, v2) | 2026-10-16/17 | Kinect GT v1, then 90 Hz features | experimental, never saved: lost |
| foot_model_v3_2026-10-17.json | 2026-10-17 | session_20261016_232318 + Kinect GT v2 | first saved model (commit 6532ff6) |
| pose_model_v4_2026-10-19.bin | 2026-10-19 | AMASS (HMD-Poser architecture, trained from scratch), then fine-tuned on the training blocks of both Kinect sessions | first pose model shipped. Kinect test blocks: 32.1 vs 47.0 for the solver alone (tools/pose_model/eval_pose.py); fused (fuse_tune.py) 32.8 with the waist tracker, 38.0 ankles only |

The pose model weights are derived from AMASS, which is licensed for non-commercial research use only: so are
these weights. No AMASS data is in this repository.
