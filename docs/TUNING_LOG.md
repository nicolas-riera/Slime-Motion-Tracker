# Body solver live tuning log

Live tuning sessions: recordings replayed with `slimelite_body_replay`, scored
with `tools/eval_replay.py` (plausibility metrics, no ground truth), plotted with
`tools/plot_replay.py`, deployed by hot reload. Metrics are given as
session 224844 (25 min) / live session 232318.

## 2026-10-16 23:40 - straight knees while standing (46e409a)

Observed: while standing tall with vertical shins, knees bent 20-40 deg
(knee_stand 32 deg). The HMD is usually 2-8 cm below the calibrated eye height
(slouching, looking down); the leg search absorbed the gap by bending the knee
(cheap quadratic prior) and the hip solve, run afterwards with the legs fixed,
never rose. The bent thigh put the ankle 15-20 cm behind the hip and the stance
anchors then pulled the hip offset around (user: "the pelvis behaves badly").

Changed: saturating knee-bend prior; while standing a straight leg may leave its
ankle up to 7 cm in the floor in the leg search, the hip solve lifts the hip.

| metric | before | after |
|---|---|---|
| knee_stand (deg) | 31.8 / 32.6 | 3.8 / 5.6 |
| skate (m/s) | 0.122 / 0.133 | 0.077 / 0.094 |
| hip_off (m) | 0.053 / 0.055 | 0.036 / 0.043 |
| jit_waist (mm) | 0.370 / 0.380 | 0.336 / 0.353 |
| planted_h (m) | 0.0089 / 0.0120 | 0.0063 / 0.0124 |
