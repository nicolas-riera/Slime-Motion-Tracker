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

## 2026-10-20 - lying pelvis, held leg, couch feet

User: lying (and a bit seated) the pelvis is too far to the side; a leg held up
while moving teleports to the floor; seated, the feet often lift off the floor.
No Kinect data any more: synthetic scenarios in tests/tests.cpp, plus the two
standing recordings of 2026-10-20 as a no-regression check (outputs identical).
- Lying, the head chain runs along an almost horizontal trunk, so a waist
  heading error swings the hip sideways (12 deg -> 9.8 cm). The head's up axis
  lies along the trunk whatever the head turns or nods: its heading corrects the
  chain's by 0.7 (kLieHeadYaw), within 40 deg. 12 deg error -> 3.4 cm; head
  turned 60 deg on the pillow moves the hip 0.2 cm.
- The seat anchor (2 cm, 8 s leak, 30 cm reach) kept the hip where it sat down:
  scooting 25 cm on the bed moved the hip 10 cm. It now follows the head chain
  when the waist tracker accelerates (0.8-2 m/s^2 above its 2 s mean; standing
  recordings show ~1 m/s^2 of sway, so the thresholds stay high) or the pelvis
  turns beyond 29 deg from the anchor's orientation (lying back, rolling). 20 cm.
- Held leg (kHoldTime, was off): the dead reckoning loses its pull within ~1 s
  and the straight-leg prior then dropped a moving raised leg to the floor (5 s
  of a moving knee lift: lowest foot 7 cm -> 30 cm). The hold is decided at 0.6 s
  of swing (foot up, not descending), needs the other foot planted, the head tall
  and quiet, and drops when the raw accelerometer sees the leg lowered. A held
  leg does not pull the hip (a first version bent the standing knee and locked it
  after landing, seen on the high-knee recording).
- Couch: reclined, a planted foot was pulled to the seat's height and only snapped
  to the floor within 6 cm. The touchdown height now decides bed or floor (closer
  of the two); lying flat keeps the bed. Shins at 55 deg: feet 4 cm up -> on the
  floor.

## 2026-10-20 - knee hinge axis learned walking (kHinge*)

The user saw the legs go outward walking (trackers on the outer side of the ankles).
The shin rotation axis while walking, in the calibrated shin frame (twist ignored), is
turned 15-30 deg from the calibrated hinge on all 3 recordings, outward on both legs:
the ski pose measures the ankle's axis, turned outward from the knee's. The axis is now
learned walking (start 0.3 rad, ~17 deg) and the hinge turned to it, standing only
(seated, the full correction brought the knees 13 cm apart instead of 28). Walking thigh
abduction mean 8-11 -> 3-5 deg, p90 28 -> 17-19 deg, walking feet width 21-29 -> 13-24 cm;
pelvis unchanged (GT fore-aft 11.7 -> 11.6 cm on this metric), seated unchanged.

Same day, after the user's VR test: a raised knee with the full correction crossed the
thighs (15-28 deg inward on the 15:04 recording): the correction now fades as the thigh
rises from 0.5 to 0.9 rad (kHingeLift*), walking gains kept, raised legs as before.
Planted feet felt too sticky: kPlantedOutput 1.0 -> 0.6, kAnchorLeak 2 -> 1 s (stance
slide median 1.1 -> 3.9 cm/s, was 8 before the anchor output). Overlay 2 -> 3 m wide.

## 2026-10-23 - mag north kept across automatic calibrations, reconnections

The user reported the trackers' north wandering (legs slowly turned sideways, sometimes
coming back) and sudden turns of the feet while still. On the same samples, the stored
calibrations of consecutive sessions put the corrected north 12-19 deg apart: a replaced
calibration took as its offset the filtered heading of that moment (whatever error the
filter had then), and the fits compared by residual in the drifting device frame (0.5-2
deg/min, bins hours apart) favored fits bent by the drift. Now the replacement keeps the
north (offset from the old calibration averaged over the samples) and must be 30 % more
consistent on norm and dip, which the drift does not touch. Synthetic: a fit bent by 15
min of drift had a better residual (0.049 vs 0.061) and a 3x worse consistency, rejected.
A tracker reconnecting (the 21/10 log: all three dropped 5 s at once) no longer resets
its heading (re-acquired from the local field, learnt drift lost): it is kept unless the
first clean mag samples are more than 25 deg off (a reboot).

Seated thighs spread like the feet's V (thigh abduction followed the shin's turn 1:1, 10-35
deg each): seated or lying, the hinge loses 10 deg of outward turn and is pulled 30 % toward
the pelvis heading (kTibialTurnout 0.17, kTibialSeated 0.3), not crouched. Seated abduction
33/16 -> 15/2 and 9/20 -> 5/14 deg on the 13:40 and 15:50 recordings; standing unchanged.
In VR the knees came too close with the feet parallel (23/10 14:34 recording: -12/-9 deg
instead of -6/-3): only the outward part of the shin's turn is now reduced, by 60 %
(kTibialSeated 0.6, no fixed turnout). Feet in a V 22/24 -> 10/11 deg, parallel feet
unchanged.

Then the screenshot of 14:41: knees spread for real, shown together. The shins' turns there
were 33 and 10 deg out for a symmetric pose: the spread came from the trackers' heading
errors (both off ~12 deg the same way), not from the V. kTibialSeated back to 0.

Same 14:41 recording, the sudden turned feet: the left tracker's own fusion jumped while
still (tilt 16 -> 62 deg, yaw 31 deg in 13 ms) and took 6 s to converge back, its linear
acceleration 10.7 m/s^2 (gravity removed along the wrong vertical). TiltFix (TrackerHub,
body replay): after a rotation jump no limb makes, while still, the orientation is tilted
onto the measured gravity and the acceleration recomputed (fading in 0.5 s when moving),
and the heading re-locks on the mag. Replay: shin tilt 63 -> 10.5 deg (10.3 before), lateral
accel 10.5 -> 0.2 m/s^2, yaw -130 -> -62 deg (-59 before); standing afterwards the left
shin is at the body's heading (-89 vs waist -94) instead of 20 deg out. Only one clean
glitch in all the recordings (seconds-long, gravity-sized); shorter ones are real motion.

## 2026-10-28 - knee twist referenced to the pelvis's left-right axis

A heel kicked back (shin level or above, knee bent 110-140 deg) worked with the right leg
and not the left (28/10 14:13 and 14:16 recordings): the tibial twist (kTibialFree) turned
the shin's front toward the pelvis forward, which lies along the shin or behind its front
in that pose, and came out anywhere: -50 to -80 deg on the left leg, -10 to -60 on the
right, the left thigh thrown 56-70 deg ahead (knee high). The twist now brings the knee's
axis onto the pelvis's left-right axis, the same with the shin hanging: twist -2..-15 /
-10..-18 deg, thigh 13-32 / 12-34 deg, knees at the same height. Raised knees (20/10
15:20-15:31) unchanged (abduction p95 change 2-4 deg).

## 2026-10-28 - slouched seated hip no longer creeps forward

Seated, the pelvis crept forward by itself with the head looking ahead (28/10 16:07): the
waist tracker read the pelvis tilted 37-42 deg back (slouching), and three things followed
that tilt. The neck range was measured from the pelvis, so a level head had already used it
up: the chest swung back with the pelvis, the hip came 10 cm ahead of the head chain and
6 cm up, and went back whenever the head looked down. The standing test ran the rigid
chain along that tilt: the hip read 12-29 % standing, which cut the sit-back shift and
loosened the seat anchor. The sit-back shift faded from 32 deg of tilt on, so the tilt
wandering by 5 deg moved the hip 5 cm. The seat anchor followed all of it in ~8 s.

Now the chest the neck range starts from leans back by at most 25 deg seated
(kSeatChestBackMax), the standing test leaves out a backward tilt (kStandBackTilt), the
sit-back fade starts at 41 deg (kSitBackFade0/1 0.75/0.5), and the seat anchor follows the
head chain horizontally in 20 s (kSeatLeakH; vertically still 8 s, or a hip sitting down
without a waist jolt stays high). 16:07 recording: creep with a still head 2.9 -> 1.0 cm
per 3 s, hip 16.5 -> 29.7 s +14 -> +5 cm, hip 10 -> 3 cm ahead of the head, 5 cm lower.
Seated sessions with 10-27 deg tilts: hip changes 3-5 mm on average; standing unchanged;
reclined frames of 20/10 13:40 p95 4.4 cm. Tried and dropped: a fully upright chest
reference (moved the validated 25 deg seated sessions 6-10 cm back), a wider neck range
(standing changed 2-7 cm; seated only, 3 cm on the other seated sessions for 2 cm less
creep). Test: slouched nod (hip 9.4 -> 4.3 cm).

## 2026-10-28 - crossed leg (ankle on the other knee)

Seated, the right ankle put on the left knee (28/10 18:59 recording, photo: knee high and out,
the shin across, lateral side down) came out low and stretched: the dead reckoning through the
swing over (the shin turning 70 deg) ended 30 cm low, the foot snapped to the floor (touchdown
below kSeatFloorSnap) and, the thigh held in the knee hinge plane, the leg folded (knee 0.29 m,
foot 0.13 m beside the left foot). The hinge plane was 37 deg away from the thigh: the hip
rotates out and the tibia twists on the femur. After kMaxSwing, a still-moving leg was
unconstrained and unfolded to the floor beside the chair (~0.5 s, each crossing).

Now (kCross*), seated on a chair with an upright trunk, a shin 45-65+ deg from vertical, ankle
below the knee, pointing across the body toward the other leg, is a crossed leg: the thigh goes
from the hip toward the other knee's top (the ankle on the sphere the shin leaves it, the
closest to it), the hinge turns to hold that thigh, the foot lands there, and a crossed leg
still moving after the dead reckoning is pulled toward it. 18:59: right knee 0.29 -> 0.64 m
(out 20 cm), foot 0.13 -> 0.48 m beside the left knee; left over right the same (knee 0.65,
foot 0.53, it used to stretch 70 cm to the right). Gated off kneeling (shin level or rising,
pointing back) and lying: on the 20/10 13:40 recording it first moved kneeling and lying legs
by up to 80 cm, now 0.4 % of seated frames by up to 12 cm; 23/10 14:34 141-143 s (left shin
57 deg across, foot on the floor before) now crossed. Standing unchanged. Test: crossed leg.

## 2026-10-28 - thigh over thigh, the crossed leg stretched out, seated restart

Thigh over thigh (19:21 recording, photo) the top foot stood on the floor and its knee sat
inside the other knee at the same height: the shin hangs (25-37 deg), not level, so the
ankle-on-knee rule did not see it. With the thigh in the tracker's hinge plane at a seated
flexion (kCrossPhi) its ankle ends 18-22 cm past the body's middle (knees apart with the feet
together stay near it): then the knee rests kCrossStack above the other knee, the foot hangs
(no snap or sink to the floor while resting on the other leg). Knees 0.57-0.60 m over 0.49.

Then the leg stretched out from there (19:25 recording, video): the shin goes level across
exactly like an ankle on the knee (only the knee flexion differs, which no tracker sees) and
the foot jumped onto the other knee. Thigh over thigh held kCrossLatch s, the knee now stays
on the other knee until the leg uncrosses (foot out 26-36 cm past it); putting the ankle up
is one quick move, the shin never hangs across that long. The foot glides to its rest
(kCrossGlide) instead of being set there, also once the dead reckoning is over: from nothing
to the rest the leg unfolded to the floor and the knee flipped (50 cm in a frame). The other
knee must be up (on one knee, 20/10, it fired).

The solver restarting while seated (replays of seated recordings) put a leg folded in the air
(foot pulled under the hip): the init's horizontal under-hip pull now scales with standing.
Changes elsewhere: 20/10 13:40 seated 3 % of frames up to 12 cm (mostly that init), short
seated crossings on 20/10 15:50 and 23/10. Tests: crossed leg stretched.

## 2026-10-28 - seated foot strikes (tapping, bouncing a leg)

Seated, a leg bounced fast (20 cm at 2.5 Hz, 19:54 recording) showed for ~0.5 s and then
stopped (the user thought the trackers saturated: they did not, 1300 deg/s and 12 g on
4000 deg/s / 32 g sensors, 100 Hz without gaps). The foot never stood still 50 ms between
strikes, so it stayed in one long swing whose dead reckoning loosens with the square of its
age (1 cm, 7 cm at 0.5 s, 26 cm at 1 s); the shin hardly turns, so nothing else moved it.

Now a sharp upward jolt (kImpact*: above 25 m/s^2, risen 22 within ~0.15 s; strikes 27-54,
a lift starts at 10-20) with the dead-reckoned foot within 35 cm of where it last stood is a
foot strike: where the jolt turns the dead-reckoned fall into a rise, the foot is at its
lowest, on that surface, and the dead reckoning restarts there (zeroing the velocity in the
jolt threw the foot 1 m up or under the floor: it bounces, it does not stop). After strikes
the foot comes to rest where it struck (the dead reckoning drifted, both feet stopped 30 cm
up); off the other leg (crossed) the struck surface is the floor. Seated upright only.
19:54: the left foot keeps bouncing 10-30 cm through 4-11 s (it sat on the floor from ~5 s).
All 25 recordings replayed: standing unchanged; seated changes only where feet strike (20/10
13:40 sitting down to the floor then lying with the legs flat: foot 0.18 -> 0.07 m; a raised
foot 0.19 -> 0.29 m for 10 s and 20/10 15:50 2 s, undecided).
