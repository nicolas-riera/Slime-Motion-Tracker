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
  axis) more than 25 cm above the highest plausible hip (HMD height - standing
  eye-to-hip distance scaled by the pelvis tilt). On GT v2 this rejects ~3%
  of the valid samples. (On the first extraction a stricter rule, 10 cm and
  both feet up while standing, was used against hands picked as shins; on v2
  it wrongly rejected real feet put on the bed, ankle ~0.55 m.)
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

### 03:20 - GT v2 (117k frames, t 1171544-1179766, 2.3 h), final model

Data: session 232318 dumped to 1.23M engine frames; 117,574 GT frames, valid
L/R 106k/104k, after cleaning 104k/100k. Splits (5 min blocks): train 67.5k,
val 22.5k, test 21.8k frames. Camera bias fitted: -7.8 cm.

Experiments on v2 (test, clean debiased GT, L+R pooled medians, cm):

| model | all | p90 | floor horiz. | lifted |
|---|---|---|---|---|
| solver live (same frames) | 9.0 | 22.0 | 8.0 | 27.1 |
| MLP 128x128 hetero, 18 lags | 4.3 | 16.2 | 3.9 | 28.8 |
| + dead-reckoning inputs (zero-velocity height memory) | 4.2 | 15.8 | 3.8 | 26.0 (val 30.1) |
| 256x256 + dead reckoning | 4.2 | 15.9 | 3.8 | 25.8 (val 28.8) |
| + replayed solver ankles as input (experiment, not exportable) | 3.9 | 15.9 | 3.5 | 25.7 (val 29.1) |
| final: 128x128 hetero, trained on train+val, 30 epochs | 4.2 | 16.1 | 3.8 | 29.7 |

Final model on the test blocks (cm, median / p90 for "all", medians
otherwise; per leg L, R):

| | all | horizontal | floor horiz. | lifted | lifted vertical |
|---|---|---|---|---|---|
| model, raw GT | 9.0/16.0, 9.5/26.4 | 8.7, 8.9 | 8.6, 8.6 | 25.5, 35.1 | 21.8, 31.6 |
| solver live, raw GT | 15.5/24.1, 11.9/29.9 | 14.9, 11.0 | 14.8, 10.5 | 29.0, 29.7 | 16.4, 18.3 |
| solver replay (02:27 build), raw GT | 15.8/25.6, 12.4/31.1 | 15.3, 11.2 | 15.2, 10.7 | 28.8, 29.7 | 16.4, 18.6 |
| model, clean debiased GT | 4.1/10.3, 4.3/22.0 | 4.0, 4.1 | 3.9, 3.8 | 24.6, 34.8 | 20.0, 32.8 |
| solver live, clean debiased GT | 10.5/18.4, 7.6/25.7 | 10.2, 6.6 | 10.1, 6.1 | 27.1, 27.2 | 15.8, 19.3 |
| hybrid: model x/z + solver y, clean debiased | 4.4/10.7, 4.9/19.2 | 4.0, 4.1 | 3.9, 3.8 | 19.6, 24.7 | 15.8, 19.3 |

Horizontal error with the foot lifted (clean debiased): model 9.4 / 7.9 cm
median (p90 25 / 21), solver 14.5 / 10.8 (p90 31 / 27).

90 Hz behavior over the whole session (1.23M frames, model valid on 99.92%):
- jitter on still feet: frame-to-frame 2nd difference RMS 2.0-2.4 mm (solver
  live 7.2 mm); deviation from a 0.3 s centered mean 6.2-6.6 mm (solver
  6.2-6.8 mm): no extra wobble, much less frame noise;
- lag: best alignment with the GT at 0 ms for both the model and the solver
  (the model adds no lag: causal, no smoothing);
- C++ update(): 0.064 ms per frame (Release, feature building included).

Predicted uncertainty (sigma = sqrt(2) * Laplace scale, heading frame):
floor feet median sigma x/y/z 4.1/0.6/4.9 cm, 98-99% of errors within
2 sigma (conservative in the core: |error|/sigma median 0.3 vs 0.49 for a
Laplace). Lifted feet: sigma x/y/z 5.3-5.8 / 3.4-5.0 / 6.9-7.4 cm but only
62-68% within 2 sigma: the vertical sigma is overconfident for raised feet.

Decisions and findings:
- Feet on the floor and horizontal position: the model halves the error
  (3.8-3.9 vs 6-10 cm on debiased GT; 8.6 vs 10.5-15 cm on raw GT). The
  solver's left foot has a ~10 cm systematic offset the model does not have.
- Raised feet: a foot put on the bed (ankle 0.45-0.65 m, held 15-50 s, ~8% of
  the frames) is invisible in a 2 s window: posture alone does not separate it
  from standing (a classifier reaches AUC 0.99 on train but 0.61-0.82 on
  held-out blocks), and the dead-reckoning memory does not capture the slow
  lift (the zero-velocity detector fires during it, the burst is small).
  Both the model and the solver miss it by 40-50 cm. Dynamic lifts: both
  under-estimate the height (median -15 to -19 cm); the solver's vertical is
  better (lifted vertical 16-19 vs 20-33 cm).
- So the model is meant for the horizontal position (hybrid row: best of
  both, all 4.4-4.9 vs 7.6-10.5 cm, lifted 19.6-24.7 vs 27.1-27.2 cm).
- Dead-reckoning inputs and the bigger net: not consistently better on both
  val and test; not used (the C++ path for "dr" models exists and passes the
  check).
- Not a residual on the solver: the solver changes often and the gain was
  small (0.3 cm).

## Delivered

- engine/models/foot_model.json (1.14 MB): MLP 630-128-128-12 (ReLU), outputs
  both ankles (heading frame) + per-axis log Laplace scales.
- core/include/slimelite/foot_model.h, core/src/foot_model.cpp (in
  slimelite_core, std + nlohmann json only): `FootModel::load(path)`,
  `FootModel::update(const BodyInput&, double now) -> FootEstimate` (world
  ankles, per-axis sigma in the heading frame, the heading).
- Checked: `slimelite_foot_model_check dump.bin engine/models/foot_model.json
  ref.bin` -> PASS, max |C++ - Python| 2e-15 m on 12,645 frames (also checked
  for a "dr" model).

## Integration (for the body solver / engine owners)

Inputs per frame: exactly the BodyInput the solver gets (HMD pose, floorY,
hasWaist + waist, both legs valid + shin + accelWorld + angularSpeed) and the
frame time `now` (engine frame clock). Call `update()` on every engine frame,
also when inputs are invalid (the accelerometer low-pass and the 2 s history
must see every frame); it returns valid=false until 2 s of valid history
exist and whenever the HMD, waist or a leg is invalid in the window. Needs
the waist tracker (Mode::Three). Load once at startup / hot reload from
`<engine dir>/models/foot_model.json` (the install/deploy script must copy
engine/models/ next to the engine DLL); if load() fails, skip the model.

Recommended use: a horizontal ankle position measurement in the per-foot
Kalman filter.
- Measurement: (ankle.x, ankle.z) of `FootEstimate::ankle[leg]` (world, the
  same ankle point as the solver's foot output and the Kinect GT).
- Covariance from `sigma` (heading frame: x right, z back) rotated to world.
  With c = cos(heading), s = sin(heading), sx = sigma.x, sz = sigma.z
  (world x = c hx + s hz, world z = -s hx + c hz, see fromHeading):
  Rxx = c^2 sx^2 + s^2 sz^2, Rzz = s^2 sx^2 + c^2 sz^2,
  Rxz = -c s (sx^2 - sz^2).
  Typical sigmas: 4 cm (x) / 5 cm (z) with the foot on the floor, 5-7 cm
  lifted (the predicted sigma is ~1.5x conservative in the core and right in
  the tails). Constant 4 / 5 cm (heading frame) also works.
- Gate the update (skip or down-weight beyond ~3 sigma innovation): rare
  frames are off by 20+ cm.
- Do not use the model's y, or only with sigma_y >= 0.2 m: it pulls raised
  feet to the floor and its vertical sigma is overconfident when the foot is
  up. The solver's vertical (dead reckoning, floor, anchors) is better there.
- During stance the filter's anchor dominates (zero velocity); the model's
  4-5 cm measurement then corrects the anchor's horizontal drift slowly, which
  keeps the model's low frame noise (2 mm vs 7 mm) without adding lag.
- Alternative without the filter: a robust leg-search term
  robust(|ankle_xz - model_xz|^2 / sigma_xz^2) plus a horizontal hip-offset
  target from it (the solver's floor error is mostly a horizontal offset,
  ~10 cm on the left foot, that this removes).

Retraining with new sessions: `slimelite_ml_dump session.jsonl s.bin`, Kinect
GT extraction, `dataset.py --dump s.bin --name vN`, `train.py --data
recordings/ml/ds_vN.npz --clean --cambias --mirror --hetero --hidden 128 128
--lagidx 0,1,2,3,4,6,8,10,13,16,20,23,24,25,26,27,28,29 --trainval --out
m.pt`, `export.py m.pt engine/models/foot_model.json --ref ref.bin --dump
s.bin`, `slimelite_foot_model_check s.bin engine/models/foot_model.json
ref.bin`, `evaluate.py engine/models/foot_model.json --dump s.bin --data
ds_vN.npz`.

Limitations: one user, one 2.3 h session (other days untested; mounting
recalibrations should be fine since the inputs are segment orientations, but
check on a new session before relying on it); the user mostly faced the
camera (the camera-bias correction is partly confounded with a forward
offset: the model places the feet front/back like the solver does on
average); GT accuracy is a few cm, so floor errors below ~3-4 cm cannot be
measured.
