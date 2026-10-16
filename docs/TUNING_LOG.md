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

## 2026-10-16 23:58 - raised feet stay up, pelvis not dragged by swings (803cb5e)

Observed (user + replay): a foot put on the bed was snapped to the floor (landing
snap up to 40 cm), a held knee lift slid down in ~4 s (air-rest sink), the pelvis
drifted (the swing leg's dead-reckoned target dragged the hip offset along).
Dead-reckoned ankle height at touchdown is within 5 cm of the floor for ~90% of
the steps of both sessions; the zero-velocity drift correction (v * T / 2) did
not improve it (the dead reckoning is already pulled to the solution after 0.4 s).

Changed: anchor at the dead-reckoned height, floor snap within 6 cm only; sink
only within 10 cm; standing with no planted foot on the floor, the lowest one
sinks (0.5 s); swing target pulls the hip with a 5x sigma.

| metric | before | after |
|---|---|---|
| hip_off (m) | 0.036 / 0.041 | 0.020 / 0.023 |
| skate (m/s) | 0.077 / 0.087 | 0.060 / 0.067 |
| jit_waist (mm) | 0.336 / 0.342 | 0.313 / 0.320 |
| standing_air | 0.0023 / 0.0080 | 0.0029 / 0.0081 |
| swing_follow (m) | 0.017 / 0.023 | 0.022 / 0.028 |
| jit_foot (mm) | 0.546 / 0.556 | 0.561 / 0.575 |
