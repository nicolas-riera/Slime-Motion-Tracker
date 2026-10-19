# Body solver live tuning log

Live tuning sessions: recordings replayed with `smt_body_replay`, scored
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

## 2026-10-17 00:32 - hip further back, from Kinect ground truth (see git log)

Kinect ankle ground truth (tools/kinect_gt, session 232318 t=1810-2710 s) is now
the main metric. Signed errors in the pelvis frame: feet on the floor 4-7 cm too
far forward, stance ~7 cm too wide (both feet outward), plus a 5-8 cm bias toward
the camera that is likely in the ground-truth extraction. Sweeps: knee valgus
(thigh tilted medially) fixed the width but not the overall error (shin yaws are
splayed 20-40 deg, so the shin's medial axis is not the body's); headBack and a
horizontal hip back offset fixed the forward bias.

Changed: headBack 0.050 -> 0.065 h, hipBack 0.029 h (horizontal).

| ankle error vs Kinect (cm, L / R) | before | after |
|---|---|---|
| all, median | 15.9 / 14.8 | 11.0 / 12.6 |
| on the floor, median | 14.1 / 11.7 | 9.5 / 10.2 |
| lifted, median | 35.7 / 29.2 | 35.3 / 27.1 |

Remaining: lifted feet 27-35 cm median error (vertical part 17-24 cm): the
solved foot does not go as high as the real one during knee lifts.

## 2026-10-17 00:40 - reverted the hip back offset (e07d725)

The user felt the legs worse after the 00:32 deploy. Reverted for an A/B
comparison. Lesson: user perception overrides the metrics; part of the forward
bias may be the ground truth's camera-facing bias, and a change trading swing
following or jitter for static accuracy should not be deployed.

## 2026-10-17 00:52 - bouncing: feet below the hips (a95e212)

Observed: fast squat bounces / small hops never meet the zero-velocity stance
test; after 1.5 s the legs had no anchor and no dead reckoning, the straight-leg
prior swung the feet ~40 cm backward (shins tilted forward). Changed: in that
case only, a prior keeps the ankle below its hip joint and near the floor.
Bounce window: foot_far 7.6% -> 0.4%, standing_air 1.4% -> 0.7%. Kinect median
L/R 15.9/14.8 -> 15.8/14.7 cm, p90 36.1/40.7 -> 35.2/37.0 cm. Knee jitter +4%.

## 2026-10-17 01:06 - stronger bouncing fallback (ed68029)

User: hops better, legs still go backward. Feet with no anchor and no swing
prediction were still 17-18 cm behind the waist in the bounce window; the
fallback sigma 0.15 -> 0.08 m brings them to 6-9 cm. Kinect error unchanged,
knee jitter +5% overall (knees now follow the squats). Not deployed: pelvic
tilt attenuation in the spine axis (waist jitter -15% but Kinect ankles +2-3 cm
worse). Still open: during short swings with dead reckoning the right foot goes
back up to 30 cm (p10) in bounces: dead reckoning around impacts.

## 2026-10-17 02:50 - seated/lying: hip on the seat (8122535)

User: seated and lying, the torso stays straight. Kinect over 20 min seated: feet
17 cm too far forward (the hip was right below the head). Changed: seated, the
hip sits 0.092 h behind the head chain, then a seat anchor holds it while the
head moves (leaks to the head chain in 8 s, within 30 cm).
Kinect ankle median L/R: seated 19.7/20.8 -> 9.1/10.3 cm; other parts 15.9/14.8
-> 15.3/14.6 and 16.2/12.5 -> 15.0/11.1 cm. Plausibility equal.
Kalman foot filter: implemented (not deployed): neutral on Kinect, but skate
+25% and swing_follow +30% on replays; to be tuned.
