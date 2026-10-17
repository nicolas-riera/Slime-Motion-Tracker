# Learned foot model log

Goal: estimate the ankle positions from the HMD + 3 IMUs (waist, 2 shins) better
than the hand-written body solver, trained on the user's own sessions with the
Kinect depth ground truth (tools/kinect_gt), and ship it as a C++ module.

## Pipeline

1. `slimelite_ml_dump session.jsonl out.bin` (tools/ml/ml_dump.cpp): one row per
   engine frame with the body solver inputs exactly as the engine builds them
   (HMD pose, floor, pelvis and shin segment orientations, shin world linear
   accelerations and angular speeds, same code as body_replay) plus the live
   solver outputs. Raw float64, columns in out.bin.json.
2. `python tools/ml/dataset.py` -> recordings/ml/ds_v1.npz: one sample per
   Kinect ground-truth frame (session clock), causal features, targets, masks,
   live and replayed solver outputs at the same frames.
3. `python tools/ml/train.py ... --out model.pt`: MLP training and test report.
4. `python tools/ml/export.py model.pt engine/models/foot_model.json --ref ref.bin`
   then `slimelite_foot_model_check dump.bin foot_model.json ref.bin`: C++ vs
   Python on ~10k frames of a session (must be < 1e-4 m; currently ~1e-14).
5. `python tools/ml/evaluate.py foot_model.json`: all 90 Hz frames: jitter,
   lag, held-out breakdown, uncertainty calibration.

## Features and targets

Heading frame: yaw of the pelvis taken from its left-right axis (bending forward
or leaning back does not change it). Origin: HMD x/z now, floor height.

For each lag of a causal window (frame at or before t - lag), 35 values: HMD
position (x/z relative to the HMD now, y above the floor), Y and Z axes of the
HMD, pelvis, left and right shin (6 each), both shins' world linear
accelerations (clipped to +-40 m/s^2 per axis, then a causal 40 ms low-pass,
run on every frame), both angular speeds. Lags kept in the delivered model: 0,
1, 2, 3, 4, 6, 8, 10, 13, 16, 20, 23 frames at 30 Hz, then 0.9, 1.0, 1.2, 1.4,
1.7, 2.0 s (18 lags, 630 inputs).

Targets: both ankles in the heading frame (x/z relative to the HMD, y above
the floor).

## Ground truth handling

- Splits: contiguous 5 min blocks in the pattern train, train, val, train,
  test (3 s guard at block edges): test = 20% of the time, never adjacent to
  the training windows' targets.
- Cleaning (training and the "clean" metrics): samples more than 15 cm from
  the median of the track within +-0.35 s; ankle above 1 m or more than 1.2 m
  horizontally from the HMD; knee (GT ankle + shin length along the IMU shin
  axis) above the highest possible hip (HMD height - standing eye-to-hip
  distance scaled by the pelvis tilt) by > 10 cm; both feet > 20 cm up with the
  head at standing height. About 15-18% of the first extraction's valid samples
  were rejected, mostly "lifted" ones (they were hands picked as shins, see
  commit cc030ce).
- Camera bias: the extraction places the ankle ~7-8 cm too far from the
  Kinect (fitted jointly as target = f(x) + b * u_cam, b = -7.2 cm, u_cam the
  horizontal unit vector toward the camera; consistent with the 5-8 cm bias
  noted in TUNING_LOG.md). The model itself is camera independent; metrics
  are reported against the ground truth with that bias removed ("debiased"),
  for the model and the solver alike. The user mostly faced the camera (80%
  within +-60 deg) so b is partly confounded with a forward offset: the
  delivered model puts the feet where the solver does front/back on average,
  which matches the user's preference (the hip-back change of 00:32 was
  reverted).

## Results log

### 02:30 - first extraction (t 1171544-1175427, 54k GT frames), MLP

Test blocks, cm, median / p90, both legs pooled where one number:

| | all | floor horizontal | lifted |
|---|---|---|---|
| ridge regression (raw GT) | 10.4-10.6 / 32-56 | 7.9-9.1 (median) | 25-37 |
| solver live (raw GT) | 9.3-9.7 / 30-34 | 8.2 | 28-42 |
| MLP 128x128, hetero (clean, debiased GT) | 4.1-4.3 / 8.9-13.3 | 4.0-4.1 | 18.5-20.5 |
| solver live (clean, debiased GT) | 7.0-10.2 / 16.3-16.7 | 6.0-9.7 | 20.6-22.1 |

- Overfitting is the main issue (train floor error 1.6 cm, below the GT noise);
  lag subsets, dropout, weight decay, heading augmentation, mirror
  augmentation give similar test results (+-0.3 cm); mirror helps a bit.
- Lifted feet: ~equal to the solver (18-20 cm). Static raised feet (foot on
  the bed for 15-25 s) cannot be seen in a 2 s window; dynamic lifts are
  under-estimated in height.
- 90 Hz behavior: frame-to-frame noise lower than the live solver (2nd
  difference RMS 1.5 vs 3.6 mm on still feet) but a slow wobble on still feet
  (5 mm RMS around the 0.3 s mean, solver 2.8 mm: the solver locks planted
  feet). Lag vs GT: same as the solver (best alignment +25 ms for both).
- Inference 0.08 ms per frame in C++ (Release).
