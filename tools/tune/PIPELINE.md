# Offline training pipeline (design)

Runs locally, unattended (overnight), on every recorded session that has Kinect
ground truth. No LLM in the loop: Claude only writes the code and reads the final
report. Nothing is installed automatically.

## 0. Inputs

`tools/tune/sessions.json` lists the training sessions:

```json
[{"session": "D:/temp-training/session_20261016_232318.jsonl", "gt": "D:/temp-training/gt/kinect_20261016_235326.csv"},
 {"session": "D:/temp-training/session_20261017_134038.jsonl", "gt": "D:/temp-training/gt/kinect_20261017_134054.csv"}]
```

GT for a new Kinect recording: `tools/kinect_gt` scene -> calibrate -> extract
(once per recording, cached as CSV).

## 1. Cleaning (per session, cached as .npz)

- **AFK mask**: HMD position std < 5 mm and rotation < 0.5 deg over a 5 s window
  while a tracker or the Kinect body still moves = headset taken off. The span
  plus 2 s on each side is excluded from training and scoring. If the recording
  has a proximity-sensor signal, use it instead.
- **GT validity**: existing cleaning (15 cm from the ±0.35 s track median,
  camera bias). Kinect is unreliable when lying, prone above all: those spans
  are scored on body constraints and the waist tracker, not Kinect feet.
- Never trim sessions (calibration state lives in the early part).

## 2. Pose labels (automatic, from GT + HMD + trackers)

standing, walking, squat, **kneel** (one shin near horizontal, knee near the
floor), **kneel transition** (±1.5 s around entering/leaving kneel), seated,
**lying back**, **lying prone** (waist tracker forward axis pointing down),
lying side, **feet raised** (GT foot > 15 cm above the floor while the body is
supported), AFK. Labels are only for weighting and reporting.

## 3. Splits

Per session, 5 min blocks, pattern T T V T Te. The foot model trains on T and
early-stops on V. The tuner scores Te. Halves A/B of Te are reported separately
(overfitting check). All poses and all sessions are used: nothing is excluded
except AFK and invalid GT.

## 4. Score (lower is better)

Per pose category, then averaged across categories with equal weight, so rare
poses (kneel, prone, feet raised) count as much as standing:

- foot position error: median + 0.3 p90 (Kinect ankles), + 0.5 median when lifted
- knee angle error vs Kinect (catches the full straightening when standing up
  from a kneel: fraction of frames with knee > GT + 10 deg)
- pelvis error vs Kinect spine base; when lying, lateral offset from the
  head->feet line counts double
- kneel: back-foot height error (back leg must not lift)
- guardrails as today (standing_air, planted_h, skate, foot_far, jitter,
  swing_follow): 10 * max(0, ratio - 1.10) penalty vs the defaults

## 5. Stages (one command: `python tools/tune/pipeline.py sessions.json out_dir`)

| Stage | What | Output |
|---|---|---|
| A | Retrain the foot model (tools/ml) on all T blocks, AFK masked | models/foot_model.json |
| B | Evolution strategy on the solver constants (tune.py, multi-session), three-tracker mode | best_three.json |
| C | Same, ankles-only mode: body_replay `--no-waist`, the real waist tracker is the pelvis GT (works on every session, even without Kinect) | best_ankles.json |
| D | Mag robustness: heading_replay with field-anomaly rejection (field norm / dip far from calibration -> lower mag weight); score = ankle/waist yaw error vs Kinect segment yaw, plus heading jumps | best_mag.json |
| E | Report: per-category before/after, A/B halves, worst 20 spans with timestamps | report.md |

B and C have separate parameter sets so ankles-only tuning cannot degrade the
three-tracker mode. Each stage appends to `trials.jsonl` and resumes from it if
interrupted. 16 workers; sessions replay in parallel inside one evaluation.

Parameters searched in B/C: the current 14 plus the bed/chest constants
(kChestHead, kUpperSpine, kReclineStart/Full, kBedBelowHip, kBedSigma) and the
new kneel/prone/feet-air constants once they exist.

## 6. What tuning cannot fix (code first, then rerun the pipeline)

Constants only move behaviour the solver already models. These need code, and
the report's per-category numbers are how they get validated:

1. Feet in the air / feet raised (top priority)
2. Kneeling: back leg lifting on the way down, straightening on the way up
3. Chest emulation (spine between HMD and waist), pelvis too far sideways when lying
4. Prone detection and handling
5. Mag field-anomaly rejection (left ankle drifted on the bed with mag "ok")

Order: run the pipeline once as a baseline (gives the category table), code the
worst categories, rerun, integrate the winners with SteamVR closed.
