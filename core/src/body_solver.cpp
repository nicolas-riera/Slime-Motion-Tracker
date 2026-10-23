#include "smt/body_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace smt {

double tunable(const char* name, double value) {
	// SMT_TUNE="kName=value,kOther=value"
	const char* env = std::getenv("SMT_TUNE");
	size_t len = std::strlen(name);
	for (const char* p = env; p && *p;) {
		if (std::strncmp(p, name, len) == 0 && p[len] == '=') return std::atof(p + len + 1);
		p = std::strchr(p, ',');
		if (p) p++;
	}
	return value;
}

namespace {

// Knee angle range (straight to fully bent)
constexpr double kPhiMin = -5.0 * kPi / 180;
constexpr double kPhiMax = 150.0 * kPi / 180;
constexpr int kPhiSteps = 78;
// Hip flexion range: thigh direction relative to the pelvis, in its sagittal plane
constexpr double kFlexMin = -35.0 * kPi / 180;
constexpr double kFlexMax = 135.0 * kPi / 180;
// Kneeling (shin flat: the knee is on the floor), the thigh hangs vertical or goes
// forward from the hip, never far back: with a flat shin both knee bends reach the
// floor, and the extended one put the knee ~35 cm behind (AMASS bench). Hip
// extension costs this sigma (rad) as the shin gets flat; 0 disables
const double kKneelExtSigma = tunable("kKneelExtSigma", 0.05);
// Among the allowed poses, prefer straight legs (sigma of the knee angle prior)
const double kStraightSigma = tunable("kStraightSigma", 1.2);
// Bending a knee a little costs about as much as bending it a lot (saturating
// prior): small head drops while standing are slouching, not knee bends. Without
// it the knees absorbed the few cm between the calibrated and the actual head
// height by bending 20-40 deg with vertical shins (the hip sitting back)
const double kBendCost = tunable("kBendCost", 3.0);
const double kBendSigma = tunable("kBendSigma", 0.35);  // rad
// A leg can push the hip up instead of bending: ankle penetration of the floor
// up to kMaxHipLift (m) costs this sigma (m) in the leg search, and the hip solve
// then lifts the hip by as much. Only when the head says standing (the gap is the
// posture of the upper body); deeper penetrations are not allowed (sitting,
// squatting: the knees bend)
const double kHipLiftSigma = tunable("kHipLiftSigma", 0.04);
constexpr double kMaxHipLift = 0.07;
// Clearances above the floor: knee center when kneeling, hip joint when lying
constexpr double kKneeClearance = 0.05;
constexpr double kHipClearance = 0.08;
// Stance anchors follow the solution with this time constant (s), and jump to it
// when farther than kAnchorReset (m). Planted feet are output at their anchor, so this
// is how fast a planted foot drifts: 1 s let it slide ~2 cm/s, 2 s ~1.4 cm/s
const double kAnchorLeak = tunable("kAnchorLeak", 1.0);
const double kAnchorReset = tunable("kAnchorReset", 0.15);
// Swing prediction is dropped after this long without a stance (s)
constexpr double kMaxSwing = 1.5;
// Lift-off detection from the accelerometer integrated during stance
const double kStanceVelLeak = tunable("kStanceVelLeak", 0.25);  // s
const double kStanceDispLeak = tunable("kStanceDispLeak", 0.3);  // s
const double kLiftOffSpeed = tunable("kLiftOffSpeed", 0.2);  // m/s
// A foot that stops within this distance of the floor (m) is on the floor. The
// dead-reckoned height at touchdown is within 5 cm of the floor for ~90% of the
// steps; a foot stopping higher rests on furniture or is held up, and stays there
const double kFloorSnap = tunable("kFloorSnap", 0.06);
// Standing, a planted foot still slightly above the floor (below kFloorSink, m)
// sinks to it with this time constant (s): dead reckoning errors would pile up
constexpr double kFloorSink = 0.10;
constexpr double kFloorSinkTime = 2.0;
// Seated upright (chair, couch), the feet are on the floor unless clearly higher:
// short shuffles leave the dead-reckoned touchdown height 10-20 cm up and the feet
// floated. Planted feet below kSeatFloorSnap (m) above the floor snap to it at
// touchdown and sink to it with kSeatSinkTime (s); higher ones (a coffee table, the
// couch itself) keep their height
const double kSeatFloorSnap = tunable("kSeatFloorSnap", 0.20);
const double kSeatSinkTime = tunable("kSeatSinkTime", 1.0);
// Standing with no planted foot on the floor, the lowest one sinks to it with this
// time constant (s)
constexpr double kNoSupportSinkTime = 0.5;
// Standing, a foot with neither a stance anchor nor a swing prediction (long
// swings: continuous bouncing, hops) stays roughly below its hip joint: without
// it the straight-leg prior swung the feet far backward when the shins tilted
// forward (m)
const double kUnderDeadZone = tunable("kUnderDeadZone", 0.05);
const double kUnderSigma = tunable("kUnderSigma", 0.08);
// ... and near the floor (the knee bending to bring it below the hip must not
// lift it into the air)
const double kUnderHeightSigma = tunable("kUnderHeightSigma", 0.10);
// Seated or lying, the hip rests on the seat (bed, floor): it is placed
// kSitBack * height behind the head chain horizontally (the trunk leans forward
// when seated: fitted on Kinect ground truth, the feet were 17 cm too far forward)
// and then stays where it sits (seat anchor, sigma kSeatSigma m) while the head
// moves, so leaning back or slouching shows in the hip-to-head direction. The seat
// anchor follows the head chain slowly (kSeatLeak s) and stays within kSeatReach
// (m) of it
const double kSitBack = tunable("kSitBack", 0.092);
// Lying (reclined), the sit-back shift goes along the pelvis back axis, which is
// horizontal lying on the side and tilts sideways as the pelvis rolls on the
// back: the hip ended up beside the body. Fraction of it removed when reclined
const double kSitBackRecline = tunable("kSitBackRecline", 1.0);
// Seated or lying, there is no chest tracker and the spine curves (slumped, leaning
// back against the bed head): its upper part follows the head's up axis by this
// fraction, its lower part the pelvis (a straight spine along a reclined pelvis put
// the hip in the belly)
// Lying on the floor with the head raised (chin toward the chest, 2026-10-20), the
// straight trunk held the hip 30 cm above the floor; 0.5 brings it to 21 cm (1: 12 cm,
// but on the bed it then sank 12 cm, and the feet once followed it to the floor)
const double kChestHead = tunable("kChestHead", 0.5);
const double kUpperSpine = tunable("kUpperSpine", 0.5);
// Not reclined (standing, seated upright), the same chest emulation by this
// fraction: leaning the head forward bends the upper back instead of sliding the
// whole rigid trunk, hip included, along with the head
const double kChestStand = tunable("kChestStand", 0.5);
// A knee raised high tilts the pelvis back (30 deg seen) while the trunk stays upright:
// the lumbar spine bends. Taken as a rigid trunk, the hip went 25 cm forward. With a
// leg off the ground and its thigh flexed (kTuckFlex0..1 rad in the pelvis frame), this
// fraction of the pelvis's backward tilt is left out of the head chain
const double kTuck = tunable("kTuck", 1.0);
const double kTuckFlex0 = tunable("kTuckFlex0", 0.4);
const double kTuckFlex1 = tunable("kTuckFlex1", 0.9);
// ...weighted by the foot's height (m, continuous: the planted flag flickers, and the
// pelvis switched with it) and smoothed over this time (s)
const double kTuckFoot0 = tunable("kTuckFoot0", 0.05);
const double kTuckFoot1 = tunable("kTuckFoot1", 0.2);
const double kTuckTime = tunable("kTuckTime", 0.3);
// The neck takes the first kNeckRange (rad) of head tilt away from the trunk: only
// the excess bends the chest, so looking down or around never moves the body
const double kNeckRange = tunable("kNeckRange", 0.6);
// 1: the neck range only applies with the head high (standing, looking around);
// head low (kneeling, squatting, seated) the head follows the back fully
const double kNeckStandOnly = tunable("kNeckStandOnly", 0.0);
// Extra freedom of the hip prior along the pelvis forward axis (m, added in
// quadrature to kHipSigmaH): the upper back bends, so a head moving forward with
// still shins and pelvis leaves the hip where the legs hold it
const double kHipSigmaFwd = tunable("kHipSigmaFwd", 0.0);
// Hip position prior around the head chain (m), with the waist tracker: horizontal
// and vertical sigmas against the legs' anchors. 0.05 let the planted feet pull the
// hip: standing sway came out 40 % larger than the pelvis measured with both
// controllers held on it (2026-10-20, fore-aft error 6.9 -> 5.5 cm with 0.02).
// Seated the seat anchor needs the looser one (the hip stays put as the head leans)
const double kHipSigmaH = tunable("kHipSigmaH", 0.02);
const double kHipSigmaSeated = tunable("kHipSigmaSeated", 0.05);
// Planted on the floor, the foot keeps the yaw it landed with: the shin twists by
// ~10 deg per step and, with the SteamVR foot point ahead of the ankle, that twist
// slid the foot. Beyond kFootYawFree (rad) it is dragged along (pivoting in place)
const double kFootYawFree = tunable("kFootYawFree", 0.26);
// How far the sole may dip below the floor (m)
const double kToeSink = tunable("kToeSink", 0.01);
// The real foot around the ankle (fractions of the height): toe tip ahead and heel
// behind, on the sole kept above the floor (less kToeSink). Only the SteamVR point
// was: avatars' toes went into the floor at toe-off and touchdown (pointing down, the
// ankle a few cm up), their heels on toe-up contacts
const double kToeTip = tunable("kToeTip", 0.11);
const double kHeel = tunable("kHeel", 0.04);
// Planted, how much the output foot is the stance anchor rather than the solved ankle
const double kPlantedOutput = tunable("kPlantedOutput", 0.6);
const double kHipSigmaV = tunable("kHipSigmaV", 0.06);
const double kHipStretch = tunable("kHipStretch", 0.06);  // m, see the hip offset solve
// Reclined (pelvis up axis height from kReclineStart to kReclineFull: leaning back
// on a bed, lying): the chest emulation applies, and the planted feet rest on the
// same surface as the hip, kBedBelowHip under the hip joint, instead of the floor
// (a vertical shin is either a knee up with the foot on the bed or a leg hanging
// from it: lying back, it is the first)
constexpr double kReclineStart = 0.85;
constexpr double kReclineFull = 0.6;
const double kBedBelowHip = tunable("kBedBelowHip", 0.08);
const double kBedSigma = tunable("kBedSigma", 0.06);
// Lying: the head chain's heading follows the head's up axis by this fraction (see the
// solver), within kLieHeadYawMax rad of the waist tracker's
const double kLieHeadYaw = tunable("kLieHeadYaw", 0.7);
const double kLieHeadYawMax = tunable("kLieHeadYawMax", 0.7);
// Pelvis up axis height below which the torso lies flat
const double kLyingFlat = tunable("kLyingFlat", 0.3);
const double kSeatSigma = tunable("kSeatSigma", 0.02);
const double kSeatLeak = tunable("kSeatLeak", 8.0);
constexpr double kSeatReach = 0.30;
// The seat anchor only holds a hip resting on the seat. When the hip itself moves
// (shifting on the chair, scooting up the bed after lying down across it, rolling
// over) it follows the head chain: the waist tracker accelerates (above its slow mean,
// kSeatMoveAccel0..1 m/s^2, then within kSeatMoveTime s) or the pelvis turns more
// than kSeatTurnFree rad away from the orientation the anchor was set with (lying
// back, rolling; leaning forward on a chair stays within it), the excess releasing
// the anchor fully over kSeatTurnFull rad. The anchor used to stay up to 30 cm away
// for many seconds: lying down from the bed edge, the pelvis stayed beside the body
const double kSeatMoveAccel0 = tunable("kSeatMoveAccel0", 0.8);
const double kSeatMoveAccel1 = tunable("kSeatMoveAccel1", 2.0);
const double kSeatMoveTime = tunable("kSeatMoveTime", 0.25);
const double kSeatTurnFree = tunable("kSeatTurnFree", 0.5);
const double kSeatTurnFull = tunable("kSeatTurnFull", 0.5);
// The learned foot model's horizontal ankle estimate loses its pull beyond this
// many sigmas (rare frames are off by 20+ cm)
const double kModelRobust = tunable("kModelRobust", 3.0);
// In stance the anchor dominates: the estimate (sigma scaled by this) only
// corrects its drift slowly (a stronger pull made planted feet skate)
const double kModelStanceScale = tunable("kModelStanceScale", 3.0);
const double kModelSeated = tunable("kModelSeated", 0.0);
// The swing prediction keeps its pull further than the stance anchors (in sigmas)
const double kInsRobust = tunable("kInsRobust", 5.0);
// ... and pulls the hip offset with a sigma this many times larger
const double kInsHipScale = tunable("kInsHipScale", 5.0);
// A leg held up and moving (knee raised, swinging the shin: never still, so never
// planted in the air). The dead reckoning weakens within a second (its sigma grows
// with the swing time) and the straight-leg prior then dropped the foot onto the floor.
// From kHoldStart s of swing on, standing on the other foot with the head quiet (no
// bounce, kHoldBounce m/s), the thigh keeps the direction it had then in the pelvis
// frame (kHoldSigma rad), fading over kHoldTime s past the dead reckoning. 0 disables
const double kHoldTime = tunable("kHoldTime", 8.0);
// Knee smoothness loosening when nothing places the foot (see the leg problem)
const double kFreeSmooth = tunable("kFreeSmooth", 3.0);
// Tibial rotation: the knee bent, the tibia turns on the femur (screw-home, free axial
// rotation in flexion), so the shin's heading no longer gives the thigh's. The hinge
// followed it: a raised knee went inward when the shin turned in (by up to 30 deg, and
// by a different amount at each raise on the 15:20 recording), crouching brought the
// feet in. The hinge heading is pulled toward the pelvis heading by kTibialFree, with the
// knee angle from kTibialFlex0 to kTibialFlex1 rad
const double kTibialFree = tunable("kTibialFree", 0.7);
const double kTibialFlex0 = tunable("kTibialFlex0", 0.5);
const double kTibialFlex1 = tunable("kTibialFlex1", 1.3);
// Seated or lying the feet rest turned out (a V) while the thighs stay near parallel: the
// tibia turns out on the flexed knee. With the hinge following the shins the thighs spread
// 10-35 deg each (13:40 and 15:50 recordings) and crossed legs looked side by side. Seated,
// the outward part of the shin's turn from the pelvis heading can be reduced by
// kTibialSeated (not crouched: the knee ahead of the ankle by more than kTibialSitAhead of
// the shin). Off: knees spread for real were shown together (23/10 14:41, screenshot), and
// the shins' turns there were asymmetric (33 vs 10 deg out for a symmetric pose): the
// spread came from the trackers' heading errors, not from the feet's V.
const double kTibialSeated = tunable("kTibialSeated", 0.0);
const double kTibialSitAhead = tunable("kTibialSitAhead", 0.4);
const double kHoldStart = tunable("kHoldStart", 0.6);
const double kHoldSigma = tunable("kHoldSigma", 0.25);
const double kHoldBounce = tunable("kHoldBounce", 0.25);
const double kHoldLowerDisp = tunable("kHoldLowerDisp", -0.12);  // m
// Raised-foot probability from micro-movements (updateRaised): how much it stops
// pulling a raised foot to the floor and keeps its knee up (kRaisedUse 0..1), and
// how fast a planted foot it says is on the floor sinks there (kRaisedSink, 1/s)
// Standing sway: the body pivots as a block around the ankles, the hip moving with
// the head by the ratio of their heights above the ankles. The head chain alone (down
// the waist tracker's axis) took every sway for a bend at the hips and moved the hip
// against the head. Standing on both planted feet with the head not dropping, the
// hip's horizontal position gets a target of feet + ratio x (head - feet) plus an
// offset that follows the head chain's answer over kSwayTime (a held posture stays
// the solver's): fast sway follows the ankle pivot, slow posture the waist tracker.
const double kSwaySigma = tunable("kSwaySigma", 0.0);  // m, 0 = off (worse in VR, 2026-10-19)
const double kSwayTime = tunable("kSwayTime", 8.0);  // s
const double kSwayDropStart = tunable("kSwayDropStart", 0.04);  // m of head drop: a bend
const double kSwayDropEnd = tunable("kSwayDropEnd", 0.08);
const double kRaisedUse = tunable("kRaisedUse", 0.0);
const double kRaisedSink = tunable("kRaisedSink", 0.0);
// The no-support sink also counts a foot at the floor between stance detections
const double kSupportAtFloor = tunable("kSupportAtFloor", 1.0);
// Logistic regression on the Kinect sessions (tools/pose_model/raised_features.py):
// self_hf, self_lf, self_coh_hmd, self_coh_waist, self_acc, self_h, other_hf,
// other_lf, other_coh_hmd, other_coh_waist, other_acc, other_h, sway_hmd
const double kRaisedMu[] = {-1.96602, -1.99652, 0.0163138, -0.0318284, 1.19918, 0.111609, -2.04239,
							-2.10878, 0.0224781, -0.0214982, 0.886776, 0.0784857, -1.82379};
const double kRaisedSd[] = {3.17906, 3.09844, 0.182323, 0.254436, 2.23811, 0.119208, 3.13544,
							3.03849, 0.179549, 0.248483, 1.71786, 0.024172, 2.22666};
const double kRaisedW[] = {-0.628427, 1.06536, -0.212864, -0.0841964, 0.0249402, 1.83609, 0.137374,
						   -0.244041, -0.144097, -0.182913, 0.163446, -0.0186003, -0.290828};
const double kRaisedB = -0.172468;

double sq(double x) { return x * x; }
double lerp(double a, double b, double t) { return a + (b - a) * t; }

double smoothstep(double e0, double e1, double x) {
	double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
	return t * t * (3 - 2 * t);
}

// Cauchy loss of a squared normalized error: quadratic below k, logarithmic
// beyond, so a constraint that cannot be met stops dominating
double robust(double e2, double k) { return k * k * std::log1p(e2 / (k * k)); }
// Iteratively reweighted least squares weight of the same loss
double robustWeight(double e2, double k) { return 1.0 / (1.0 + e2 / (k * k)); }

double squaredNorm(const Vec3& v) { return v.dot(v); }

// Yaw of a shin from its left-right axis, which stays horizontal-ish for a hinge
// leg even when the shin itself is horizontal (sitting, kneeling, lying)
double shinYaw(const Quat& shin) {
	Vec3 r = shin.rotate({1, 0, 0});
	return std::atan2(-r.z, r.x);
}

}  // namespace

Vec3 BodySolver::OneEuro::filter(const Vec3& v, double dt, double minCutoff, double beta) {
	if (!init || dt <= 0) {
		init = true;
		x = v;
		dx = {};
		return v;
	}
	auto alpha = [dt](double cutoff) {
		double tau = 1.0 / (2 * kPi * cutoff);
		return 1.0 / (1.0 + tau / dt);
	};
	Vec3 d = (v - x) / dt;
	dx += (d - dx) * alpha(1.0);
	double cutoff = minCutoff + beta * dx.norm();
	x += (v - x) * alpha(cutoff);
	return x;
}

// Postural sway tells a raised foot from a planted one: a planted leg sways with
// the body (pelvis, HMD), a raised one trembles and moves on its own, and standing
// on one leg sways more. One-pole filters following the frame time, identical to
// tools/pose_model/raised_features.py
void BodySolver::updateRaised(const BodyInput& in, double dt) {
	Raised& r = m_raised;
	auto pole = [dt](Vec3& y, const Vec3& x, double fc) {
		double tau = 1.0 / (2 * kPi * fc);
		y += (x - y) * std::min(1.0, dt / (tau + dt));
	};
	auto ema = [dt](double& y, double x, double tau) { y += (x - y) * std::min(1.0, dt / tau); };
	auto horiz = [](const Vec3& v) { return Vec3{v.x, 0, v.z}; };
	const Vec3 zero;
	Vec3 ha;
	if (!r.init) {
		r.hmdLp = r.hmdPrev = in.hmd.position;
		r.hmdVel = zero;
		r.hbLo = r.hbHi = r.wbLo = r.wbHi = zero;
		for (int k = 0; k < 2; k++) {
			r.leg[k] = {};
			r.leg[k].hp = r.leg[k].lo = r.leg[k].hi = in.legs[k].accelWorld;
		}
		r.wbLo = r.wbHi = in.waistAccel;
		r.init = true;
	} else {
		pole(r.hmdLp, in.hmd.position, 5.0);
		Vec3 v = (r.hmdLp - r.hmdPrev) / dt;
		ha = (v - r.hmdVel) / dt;
		r.hmdPrev = r.hmdLp;
		r.hmdVel = v;
	}
	pole(r.hbHi, ha, 3.0);
	pole(r.hbLo, ha, 0.3);
	pole(r.wbHi, in.waistAccel, 3.0);
	pole(r.wbLo, in.waistAccel, 0.3);
	const Vec3 hb = horiz(r.hbHi - r.hbLo), wb = horiz(r.wbHi - r.wbLo);
	ema(r.sway, hb.dot(hb), 2.0);
	std::array<std::array<double, 6>, 2> f;
	for (int k = 0; k < 2; k++) {
		Raised::Leg& L = r.leg[k];
		const Vec3& a = in.legs[k].accelWorld;
		pole(L.hp, a, 3.0);
		pole(L.hi, a, 3.0);
		pole(L.lo, a, 0.3);
		Vec3 hf = a - L.hp, ab = horiz(L.hi - L.lo);
		ema(L.hf, hf.dot(hf), 1.0);
		ema(L.lf, ab.dot(ab), 2.0);
		ema(L.xh, ab.dot(hb), 2.0);
		ema(L.xw, ab.dot(wb), 2.0);
		ema(L.ph, hb.dot(hb), 2.0);
		ema(L.pw, wb.dot(wb), 2.0);
		ema(L.acc, a.norm(), 0.1);
		double h = m_initialized ? m_smoothedFeet[k].y - in.floorY : 0.08;
		f[k] = {std::log(L.hf + 1e-6), std::log(L.lf + 1e-6), L.xh / std::sqrt(L.lf * L.ph + 1e-12),
				L.xw / std::sqrt(L.lf * L.pw + 1e-12), L.acc, h};
	}
	for (int k = 0; k < 2; k++) {
		double z = kRaisedB;
		for (int j = 0; j < 6; j++) {
			z += kRaisedW[j] * (f[k][j] - kRaisedMu[j]) / kRaisedSd[j];
			z += kRaisedW[6 + j] * (f[1 - k][j] - kRaisedMu[6 + j]) / kRaisedSd[6 + j];
		}
		z += kRaisedW[12] * (std::log(r.sway + 1e-9) - kRaisedMu[12]) / kRaisedSd[12];
		r.p[k] = 1.0 / (1.0 + std::exp(-z));
	}
}

BodyDimensions BodyDimensions::fromHeight(double h, std::optional<double> eyeHeight) {
	BodyDimensions d;
	d.height = h;
	d.ankleHeight = 0.039 * h;
	d.shin = 0.246 * h;
	d.thigh = 0.245 * h;
	d.hipWidth = 0.095 * h;
	d.headDown = 0.060 * h;  // eyes to the top of the neck, rotates with the head
	d.headBack = 0.050 * h;
	d.neckDown = 0.058 * h;  // neck, along the torso
	d.eyeHeight = eyeHeight.value_or(0.936 * h);
	double neckBase = d.eyeHeight - d.headDown - d.neckDown;
	d.spine = std::clamp(neckBase - d.standingHipHeight(), 0.22 * h, 0.36 * h);
	return d;
}

struct BodySolver::LegProblem {
	Vec3 hipJoint;
	Vec3 down, forward;  // shin axes
	bool pelvisKnown = false;
	Quat pelvis;
	double floorY = 0, ankleY = 0;
	double maxLift = 0;  // how far the hip may rise to keep this leg straight
	double phiPrev = 0, sigmaPhi = 0.1;
	bool planted = false;
	Vec3 anchor;
	double anchorSigma = 0.02;
	double insSigma = 0;  // 0: no swing prediction
	Vec3 ins;
	double bedWeight = 0;  // reclined: weight of "the ankle is at the bed's height"
	double bedY = 0;
	double modelSigma = 0;  // 0: no learned foot estimate
	Vec3 model;
	double support = 0;  // weight of "at least one foot on the floor"
	double under = 0;  // weight of "the foot is below the hip" (no other position cue)
	double otherHeight = 0;  // other ankle above its floor position
	double hold = 0;  // weight of "the thigh keeps holdThigh" (world direction)
	Vec3 holdThigh;
};

void BodySolver::configure(const BodyDimensions& dims, const Settings& settings, Mode mode) {
	m_dims = dims;
	m_settings = settings;
	m_mode = mode;
}

void BodySolver::reset() {
	m_legs = {};
	m_hipOffset = {};
	m_seatValid = false;
	m_waistAccMean = {};
	m_waistActivity = 0;
	m_initialized = false;
	m_filters = {};
	m_smoothedFeet = {};
	m_smoothedWaist = {};
	m_raised = {};
	m_hmdBounce = 0;
	m_tuck = 0;
	m_swayRef = {};
	m_swayGate = m_standHeadY = m_bothPlantedTime = 0;
}

double BodySolver::legCost(const LegProblem& p, double phi) const {
	const auto& d = m_dims;
	Vec3 thighDir = p.down * std::cos(phi) + p.forward * std::sin(phi);
	Vec3 knee = p.hipJoint + thighDir * d.thigh;
	Vec3 ankle = knee + p.down * d.shin;

	double c = sq(phi / kStraightSigma) + kBendCost * (1 - std::exp(-0.5 * sq(phi / kBendSigma)));
	double pen = p.ankleY - ankle.y;
	if (pen > 0) c += sq(std::min(pen, p.maxLift) / kHipLiftSigma) + sq(std::max(0.0, pen - p.maxLift) / 0.01);
	double penKnee = p.floorY + kKneeClearance - knee.y;
	if (penKnee > 0) c += sq(penKnee / 0.01);
	if (p.pelvisKnown) {
		Vec3 t = p.pelvis.conj().rotate(thighDir);
		double flex = std::atan2(-t.z, -t.y);
		if (flex > kFlexMax) c += sq((flex - kFlexMax) / 0.1);
		if (flex < kFlexMin) c += sq((kFlexMin - flex) / 0.1);
		if (kKneelExtSigma > 0 && flex < 0) c += smoothstep(0.5, 0.15, std::fabs(p.down.y)) * sq(flex / kKneelExtSigma);
	}
	if (p.planted) c += robust(squaredNorm(ankle - p.anchor) / sq(p.anchorSigma), 3.0);
	if (p.insSigma > 0) c += robust(squaredNorm(ankle - p.ins) / sq(p.insSigma), kInsRobust);
	if (p.under > 0) {
		Vec3 h = ankle - p.hipJoint;
		double dist = std::sqrt(h.x * h.x + h.z * h.z);
		c += p.under * sq(std::max(0.0, dist - kUnderDeadZone) / kUnderSigma);
		c += p.under * sq(std::max(0.0, ankle.y - p.ankleY) / kUnderHeightSigma);
	}
	if (p.bedWeight > 0) c += p.bedWeight * sq((ankle.y - p.bedY) / kBedSigma);
	if (p.hold > 0) c += p.hold * squaredNorm(thighDir - p.holdThigh) / sq(kHoldSigma);
	if (p.modelSigma > 0) {
		double e2 = (sq(ankle.x - p.model.x) + sq(ankle.z - p.model.z)) / sq(p.modelSigma);
		c += robust(e2, kModelRobust);
	}
	if (p.support > 0) {
		double h = std::min(std::max(0.0, ankle.y - p.ankleY), p.otherHeight);
		c += p.support * sq(h / 0.03);
	}
	c += robust(sq((phi - p.phiPrev) / p.sigmaPhi), 2.0);
	return c;
}

double BodySolver::solveLeg(const LegProblem& p) const {
	// Global grid search (the cost has several minima: leg straight, knee folded
	// up...), then golden-section refinement around the best cell
	double best = std::clamp(p.phiPrev, kPhiMin, kPhiMax);
	double bestCost = legCost(p, best);
	const double step = (kPhiMax - kPhiMin) / kPhiSteps;
	for (int k = 0; k <= kPhiSteps; k++) {
		double phi = kPhiMin + step * k;
		double c = legCost(p, phi);
		if (c < bestCost) {
			bestCost = c;
			best = phi;
		}
	}
	const double g = 0.6180339887498949;
	double a = std::max(kPhiMin, best - step), b = std::min(kPhiMax, best + step);
	double x1 = b - g * (b - a), x2 = a + g * (b - a);
	double f1 = legCost(p, x1), f2 = legCost(p, x2);
	for (int i = 0; i < 16; i++) {
		if (f1 < f2) {
			b = x2;
			x2 = x1;
			f2 = f1;
			x1 = b - g * (b - a);
			f1 = legCost(p, x1);
		} else {
			a = x1;
			x1 = x2;
			f1 = f2;
			x2 = a + g * (b - a);
			f2 = legCost(p, x2);
		}
	}
	double refined = 0.5 * (a + b);
	return legCost(p, refined) < bestCost ? refined : best;
}

BodyOutput BodySolver::update(const BodyInput& rawIn) {
	BodyOutput out;
	if (!rawIn.hmdValid || !rawIn.legs[0].valid || !rawIn.legs[1].valid) return out;
	const double dt = std::clamp(rawIn.dt, 1e-4, 0.1);
	const BodyInput& in = rawIn;
	const auto& d = m_dims;
	const auto& s = m_settings;
	const double ankleY = in.floorY + d.ankleHeight;
	const Vec3 up{0, 1, 0};

	if (m_initialized) {
		double vy = (in.hmd.position.y - m_prevHmdY) / dt;
		m_hmdBounce += (std::fabs(vy) - m_hmdBounce) * std::min(1.0, dt / 0.5);
	}
	m_prevHmdY = in.hmd.position.y;

	updateRaised(in, dt);

	// ---- upper body: head pivot, pelvis, neck base, hip center ----
	const Quat& hq = in.hmd.rotation;
	Vec3 headPivot = in.hmd.position + hq.rotate({0, -d.headDown, d.headBack});
	double eyeRatio = (in.hmd.position.y - in.floorY) / std::max(0.5, d.eyeHeight);

	bool useWaistTracker = m_mode == Mode::Three && in.hasWaist;
	Quat pelvis;
	if (useWaistTracker) {
		pelvis = in.waist;
	} else {
		// Ankles only. Heading from the shins (feet point roughly forward) and the head
		// (unreliable lying down, when the head looks at the ceiling)
		double lying = smoothstep(0.55, 0.3, eyeRatio);
		double sx = 0, sz = 0;
		auto add = [&](double yaw, double w) {
			sx += std::sin(yaw) * w;
			sz += std::cos(yaw) * w;
		};
		add(hq.yawAngle(), 0.6 * (1 - lying));
		add(shinYaw(in.legs[0].shin), 1.0);
		add(shinYaw(in.legs[1].shin), 1.0);
		double pelvisYaw = std::atan2(sx, sz);
		// Upright: the torso leans by a fraction of the head pitch. Head low: the user
		// lies down and the torso follows the head's up axis
		Vec3 hf = hq.rotate({0, 0, -1});
		double headPitch = std::asin(std::clamp(-hf.y, -1.0, 1.0));  // > 0 looking down
		double lean = std::clamp(0.3 * headPitch, deg2rad(-15), deg2rad(30));
		Quat upright = Quat::yaw(pelvisYaw) * Quat::fromAxisAngle({1, 0, 0}, -lean);
		Vec3 torsoUp = (upright.rotate(up) * (1 - lying) + hq.rotate(up) * lying).normalized();
		pelvis = Quat::fromTwoVectors(up, torsoUp) * Quat::yaw(pelvisYaw);
	}
	const Vec3 torsoUp = pelvis.rotate(up);
	Vec3 neck = headPivot - torsoUp * d.neckDown;
	Vec3 hip0 = neck - torsoUp * d.spine;
	const double standing = smoothstep(0.55, 0.8, (hip0.y - in.floorY) / d.standingHipHeight());
	const double seated = 1 - standing;  // seated, kneeling or lying: the hip is supported
	const double reclined = seated * smoothstep(kReclineStart, kReclineFull, torsoUp.y);
	// Lying, the head chain runs along an almost horizontal trunk: a waist tracker
	// heading error of 10 deg put the hip ~10 cm beside the body. The head's up axis
	// lies along the trunk too, whatever the head turns (around that axis) or nods (in
	// the plane of the trunk): its heading corrects the chain's by kLieHeadYaw, unless
	// they differ by more than kLieHeadYawMax rad (the head tilted toward a shoulder)
	Vec3 chainUp = torsoUp;
	if (useWaistTracker && kTuck > 0 && m_initialized) {
		double target = 0;
		for (const LegState& ls : m_legs) {
			const double flex = std::atan2(-ls.thighLocal.z, -ls.thighLocal.y);
			target = std::max(target, smoothstep(kTuckFlex0, kTuckFlex1, flex) *
										  smoothstep(kTuckFoot0, kTuckFoot1, ls.ankle.y - ankleY));
		}
		m_tuck += (target - m_tuck) * std::min(1.0, dt / kTuckTime);
		const double raised = m_tuck;
		Vec3 back = pelvis.rotate({0, 0, 1});
		back.y = 0;
		if (raised > 0.001 && back.norm() > 1e-6) {
			back = back.normalized();
			const double tiltBack = torsoUp.dot(back);
			if (tiltBack > 0) {
				chainUp = (torsoUp - back * (tiltBack * kTuck * raised * standing)).normalized();
				neck = headPivot - chainUp * d.neckDown;
				hip0 = neck - chainUp * d.spine;
			}
		}
	}
	if (useWaistTracker && kLieHeadYaw > 0 && reclined > 0) {
		const Vec3 headUp = hq.rotate(up);
		const double th = std::hypot(torsoUp.x, torsoUp.z), hh = std::hypot(headUp.x, headUp.z);
		if (th > 0.2 && hh > 0.2) {
			const double delta = wrapAngle(std::atan2(headUp.x, headUp.z) - std::atan2(torsoUp.x, torsoUp.z));
			const double w = kLieHeadYaw * reclined * smoothstep(0.2, 0.6, th) * smoothstep(0.2, 0.6, hh) *
							 smoothstep(kLieHeadYawMax, 0.5 * kLieHeadYawMax, std::fabs(delta));
			chainUp = Quat::yaw(delta * w).rotate(torsoUp);
			neck = headPivot - chainUp * d.neckDown;
			hip0 = neck - chainUp * d.spine;
		}
	}
	if (reclined > 0 || kChestStand > 0) {
		const double w = kChestHead * reclined + kChestStand * (1 - reclined);
		Vec3 headUp = hq.rotate(up);
		const double tilt = std::acos(std::clamp(chainUp.dot(headUp), -1.0, 1.0));
		const double neckRange =
			kNeckRange * (kNeckStandOnly > 0 ? lerp(1.0, smoothstep(0.6, 0.8, eyeRatio), kNeckStandOnly) : 1.0);
		if (neckRange > 0 && tilt > 1e-6) {
			// Keep only the tilt beyond the neck's range
			Vec3 axis = chainUp.cross(headUp);
			double excess = std::max(0.0, tilt - neckRange);
			headUp = axis.norm() > 1e-9 ? Quat::fromAxisAngle(axis, excess).rotate(chainUp) : chainUp;
		}
		const Vec3 chestUp = (chainUp * (1 - w) + headUp * w).normalized();
		neck = headPivot - chestUp * d.neckDown;
		hip0 = neck - chestUp * (d.spine * kUpperSpine) - chainUp * (d.spine * (1 - kUpperSpine));
	}
	{
		// Seated, the trunk leans forward: the hip is behind the head chain. Horizontal,
		// fading out as the pelvis tilts (lying down)
		Vec3 pelvisBack = pelvis.rotate({0, 0, 1});
		pelvisBack.y = 0;
		hip0 += pelvisBack * (kSitBack * d.height * seated * (1 - kSitBackRecline * reclined));
	}
	// The user's own offset (calibrated with the controllers): a torso leaning sideways
	// put the whole lower body to one side
	hip0 += pelvis.rotate({d.hipSide, 0, 0});

	// Per leg, how much a planted foot rests on the bed (at the hip's height) rather
	// than on the floor. Reclined with a vertical shin, the leg hangs from the seat
	// (slouching on a couch, sitting back on a bed edge: the foot is on the floor)
	// unless the torso is lying flat (then a vertical shin is a knee up, the foot on
	// the bed)
	std::array<double, 2> bedness{}, floorPref{}, onBedWeight{};
	const double flatLying = smoothstep(kLyingFlat + 0.25, kLyingFlat + 0.1, torsoUp.y);
	for (int i = 0; i < 2; i++) {
		double shinUpY = -in.legs[i].shin.rotate({0, -1, 0}).y;
		double hanging = smoothstep(0.6, 0.85, shinUpY) * smoothstep(kLyingFlat, kLyingFlat + 0.25, torsoUp.y);
		bedness[i] = reclined * (1 - hanging);
		floorPref[i] = seated * (1 - bedness[i]);
		// Lying flat a foot is on the bed whatever its touchdown (see onBed): the anchors
		// may date from standing
		onBedWeight[i] = std::max(m_legs[i].onBed ? 1.0 : 0.0, flatLying);
	}

	// ---- per-leg geometry, stance detection (zero velocity), swing dead reckoning ----
	std::array<Vec3, 2> legOffset, down, forward;
	for (int i = 0; i < 2; i++) {
		const auto& li = in.legs[i];
		LegState& ls = m_legs[i];
		legOffset[i] = pelvis.rotate({(i == 0 ? -0.5 : 0.5) * d.hipWidth, 0, 0});
		down[i] = li.shin.rotate({0, -1, 0});
		{
			// Twist about the shin axis toward the pelvis forward (kTibial*): the hinge
			// plane turns to contain it, unless the shin points along it
			const Vec3 u = -down[i], fs = li.shin.rotate({0, 0, -1}), fp = pelvis.rotate({0, 0, -1});
			const double along = std::fabs(fp.dot(u));
			const Vec3 target = fp - u * fp.dot(u);
			const double twist = std::atan2(u.dot(fs.cross(target)), fs.dot(target));
			const double gate = smoothstep(kTibialFlex0, kTibialFlex1, ls.phi) * smoothstep(0.95, 0.8, along);
			const double sit =
				(1 - standing) * std::max(smoothstep(kTibialSitAhead + 0.1, kTibialSitAhead - 0.1, fp.dot(u)), reclined);
			// Outward turn of the shin (a yaw about +Y turns forward to the left: outward for the
			// left leg, the twist back is then negative)
			const double outward = std::max(0.0, i == 0 ? -twist : twist);
			out.debug.hingeYaw[i] =
				gate * (kTibialFree * standing * twist - kTibialSeated * sit * (i == 0 ? outward : -outward));
			forward[i] = (li.shin * Quat::yaw(out.debug.hingeYaw[i])).rotate({0, 0, -1});
		}

		if (!m_initialized || !ls.initialized) {
			// Start with the foot on the floor below the hip, the knee bent as the shin
			// says. A straight leg along the shin put the feet 75 cm behind after a full
			// reset (the solver restarts while the user still holds the ski pose), where
			// the stance anchors kept them until the user moved
			ls = LegState{};
			ls.initialized = true;
			LegProblem p0;
			p0.hipJoint = hip0 + legOffset[i];
			p0.down = down[i];
			p0.forward = forward[i];
			p0.pelvisKnown = useWaistTracker;
			p0.pelvis = pelvis;
			p0.floorY = in.floorY;
			p0.ankleY = ankleY;
			p0.maxLift = kMaxHipLift * standing;
			p0.sigmaPhi = 10;
			p0.under = 1;
			ls.phi = solveLeg(p0);
			const Vec3 thighDir = down[i] * std::cos(ls.phi) + forward[i] * std::sin(ls.phi);
			ls.ankle = p0.hipJoint + thighDir * d.thigh + down[i] * d.shin;
			ls.ankle.y = std::max(ls.ankle.y, ankleY);
			ls.anchor = ls.insPos = ls.ankle;
		}

		// Shin-mounted IMU: during stance the shin pivots around the ankle (moderate
		// angular rate) but the ankle barely accelerates, so gate mostly on accel
		double a = li.accelWorld.norm();
		bool still = a < 1.2 && li.angularSpeed < 2.5;
		ls.stillTime = still ? ls.stillTime + dt : 0;
		if (ls.planted) {
			// Leaky integration: bounded by the accelerometer bias, but a deliberate
			// lift reaches the threshold within ~0.1 s
			double vLeak = std::exp(-dt / kStanceVelLeak), pLeak = std::exp(-dt / kStanceDispLeak);
			ls.stanceVel = (ls.stanceVel + li.accelWorld * dt) * vLeak;
			ls.stanceDisp = (ls.stanceDisp + ls.stanceVel * dt) * pLeak;
		}
		if (!ls.planted && ls.stillTime >= 0.05) {
			ls.planted = true;
			// Trust the accelerometer: the foot stays where it stopped (on the bed, held
			// up...), at the dead-reckoned height (the solution may lag behind it), unless
			// it is within a few cm of the floor
			ls.anchor = ls.ankle;
			if (ls.swingTime < kMaxSwing) ls.anchor.y = std::max(ankleY, ls.insPos.y);
			// Reclined, the foot lands on the bed (the hip's surface) or on the floor,
			// whichever its touchdown height is closer to: slouched on a couch, the feet
			// on the floor used to be pulled up to the seat and floated
			const double bedY = std::max(ankleY, (hip0 + m_hipOffset).y - kBedBelowHip);
			ls.onBed = bedness[i] > 0.05 && ls.anchor.y - ankleY > 0.5 * (bedY - ankleY);
			onBedWeight[i] = std::max(ls.onBed ? 1.0 : 0.0, flatLying);
			const double floorWeight = lerp(seated, floorPref[i], onBedWeight[i]);
			if (ls.anchor.y - ankleY < lerp(kFloorSnap, kSeatFloorSnap, floorWeight)) ls.anchor.y = ankleY;
			out.debug.touchdownSwing[i] = ls.swingTime;
			out.debug.touchdownHeight[i] = ls.anchor.y - ankleY;
			ls.stanceVel = ls.stanceDisp = {};
		} else if (ls.planted && (a > 2.0 || li.angularSpeed > 4.0 || ls.stanceVel.norm() > kLiftOffSpeed)) {
			// Lift-off: the swing starts with the motion already integrated
			ls.planted = false;
			ls.swingTime = 0;
			ls.insPos = ls.ankle + ls.stanceDisp;
			ls.insVel = ls.stanceVel;
			ls.lowerVel = ls.stanceVel.y;
			ls.lowerAcc = ls.lowerDisp = 0;
		}
		if (!ls.planted) {
			const double prevSwing = ls.swingTime;
			ls.insVel += li.accelWorld * dt;
			ls.insPos += ls.insVel * dt;
			ls.swingTime += dt;
			// Held leg (kHold*): decided while the dead reckoning is still good (the foot
			// up, not descending), dropped when the raw accelerometer sees the leg lowered
			// (the dead reckoning later follows the solution, it cannot tell). Lowered: a
			// leaky, low-passed vertical displacement, so shaking the raised leg (+-15
			// m/s^2 at a few Hz) averages out while putting it down (tens of cm) does not
			ls.lowerAcc += (li.accelWorld.y - ls.lowerAcc) * std::min(1.0, dt / 0.1);
			ls.lowerVel = (ls.lowerVel + ls.lowerAcc * dt) * std::exp(-dt / 0.8);
			ls.lowerDisp = (ls.lowerDisp + ls.lowerVel * dt) * std::exp(-dt / 1.5);
			if (prevSwing < kHoldStart && ls.swingTime >= kHoldStart) {
				ls.holdLatched = ls.insPos.y - ankleY > 0.12 && ls.insVel.y > -0.15;
				ls.lowerDisp = 0;
				// From the dead-reckoned velocity, still good this early: the leaky one had
				// lost part of the lift, so stopping the leg at the top read as lowering it
				// and dropped the hold 0.5 s later, the foot still 90 cm up
				ls.lowerVel = ls.insVel.y;
			}
			if (ls.lowerDisp < kHoldLowerDisp) ls.holdLatched = false;
		} else {
			ls.holdLatched = false;
			ls.lowerVel = 0;
		}
	}

	// ---- standing sway prior (kSway*) ----
	bool swayActive = false;
	Vec3 swayPrior;  // hip target, horizontal
	if (kSwaySigma > 0) {
		const bool both = m_legs[0].planted && m_legs[1].planted;
		m_bothPlantedTime = both ? m_bothPlantedTime + dt : 0;
		// Standing head height: rises at once, sinks slowly (a held lower stance becomes
		// the reference after a while)
		if (!m_initialized || m_standHeadY == 0) m_standHeadY = headPivot.y;
		m_standHeadY = std::max(headPivot.y, m_standHeadY - 0.01 * dt);
		const double drop = m_standHeadY - headPivot.y;
		const double gate = standing * smoothstep(0.3, 0.6, m_bothPlantedTime) *
							smoothstep(kSwayDropEnd, kSwayDropStart, drop);
		m_swayGate += (gate - m_swayGate) * std::min(1.0, dt / 0.3);
		const Vec3 feetMid = (m_legs[0].anchor + m_legs[1].anchor) * 0.5;
		const double headAbove = std::max(0.5, d.eyeHeight - d.headDown - d.ankleHeight);
		const double ratio = std::clamp((d.standingHipHeight() - d.ankleHeight) / headAbove, 0.3, 0.8);
		Vec3 pivotHip = feetMid + (headPivot - feetMid) * ratio;
		Vec3 chain = hip0 - pivotHip;  // the head chain's answer, from the pivot line
		chain.y = 0;
		if (m_swayGate < 0.05) m_swayRef = chain;
		else m_swayRef += (chain - m_swayRef) * std::min(1.0, dt / kSwayTime);
		swayPrior = pivotHip + m_swayRef;
		swayActive = m_swayGate >= 0.05;
	}

	// ---- solve: legs by global search, then the hip offset, alternated ----
	const double plant = std::clamp(s.footPlantStrength, 0.0, 1.0);
	const double inertial = std::clamp(s.inertialWeight, 0.0, 1.0);
	const double anchorSigma = lerp(0.08, 0.01, plant);
	const double insScale = std::pow(4.0, 1.0 - 2.0 * inertial);
	std::array<double, 2> insSigma{};
	for (int i = 0; i < 2; i++) {
		const LegState& ls = m_legs[i];
		if (!ls.planted && inertial > 0.01 && ls.swingTime < kMaxSwing)
			insSigma[i] = (0.01 + 0.25 * sq(ls.swingTime)) * insScale;
	}
	const double hipSigmaH = useWaistTracker ? lerp(kHipSigmaSeated, kHipSigmaH, standing) : 0.15;
	const double hipSigmaV = useWaistTracker ? kHipSigmaV : 0.10;
	const double hipSigmaSmooth = std::max(0.002, 1.5 * dt);
	const double hipLimitH = lerp(useWaistTracker ? 0.25 : 0.4, kSeatReach + 0.1, seated);
	// Seat anchor (see kSeatSigma): follows the hip while standing
	if (useWaistTracker) {
		m_waistAccMean += (in.waistAccel - m_waistAccMean) * std::min(1.0, dt / 2.0);
		const double a = (in.waistAccel - m_waistAccMean).norm();
		m_waistActivity += (a - m_waistActivity) * std::min(1.0, dt / 0.15);
	} else {
		m_waistActivity = 0;
	}
	if (seated < 0.5 || !m_seatValid) {
		m_seat = hip0 + m_hipOffset;
		m_seatPelvis = pelvis;
		m_seatValid = true;
	} else {
		// The hip moves on the seat (kSeatMove*, kSeatTurn*)
		double release = dt / kSeatLeak +
						 dt / kSeatMoveTime * smoothstep(kSeatMoveAccel0, kSeatMoveAccel1, m_waistActivity);
		const double turn = (m_seatPelvis.conj() * pelvis).angle();
		if (turn > kSeatTurnFree) {
			const double excess = turn - kSeatTurnFree;
			m_seatPelvis = slerp(m_seatPelvis, pelvis, excess / turn);
			release += excess / kSeatTurnFull;
		}
		m_seatPelvis = slerp(m_seatPelvis, pelvis, std::min(1.0, dt / kSeatLeak));
		m_seat += (hip0 - m_seat) * std::min(1.0, release);
		Vec3 e = m_seat - hip0;
		if (e.norm() > kSeatReach) m_seat = hip0 + e * (kSeatReach / e.norm());
	}

	auto legVector = [&](int i, double phi) {
		Vec3 thighDir = down[i] * std::cos(phi) + forward[i] * std::sin(phi);
		return legOffset[i] + thighDir * d.thigh + down[i] * d.shin;
	};
	Vec3 hipOffset = m_hipOffset;
	std::array<double, 2> phi{m_legs[0].phi, m_legs[1].phi};
	std::array<double, 2> held{};  // hold weight per leg (kHold*)
	for (int iter = 0; iter < 3; iter++) {
		for (int i = 0; i < 2; i++) {
			const LegState& ls = m_legs[i];
			int o = 1 - i;
			LegProblem p;
			p.hipJoint = hip0 + hipOffset + legOffset[i];
			p.down = down[i];
			p.forward = forward[i];
			p.pelvisKnown = useWaistTracker;
			p.pelvis = pelvis;
			p.floorY = in.floorY;
			p.ankleY = ankleY;
			p.maxLift = std::max(0.0, kMaxHipLift * standing - hipOffset.y);
			p.phiPrev = ls.phi;
			p.sigmaPhi = 0.03 + dt * (3.0 + 2.0 * in.legs[i].angularSpeed);
			p.planted = ls.planted;
			p.anchor = ls.anchor;
			p.anchorSigma = anchorSigma;
			p.insSigma = insSigma[i];
			p.ins = ls.insPos;
			p.support = standing * (ls.planted ? 1.0 : 0.1);
			// Not reclined: the model never saw the user lying back, and there it bent
			// the knees of the legs stretched on the bed
			// Seated, by kModelSeated only: pulling a swinging foot back under the chair
			// (horizontally), it bent the knee, the foot rose 30 cm and stopped there,
			// planted in the air (the leg lifting by itself while seated)
			const double modelUse = lerp(kModelSeated, 1.0, standing);
			if (in.legs[i].hasFootMeasurement && reclined < 0.95 && modelUse > 0.05) {
				p.modelSigma = in.legs[i].footMeasurementSigma * (ls.planted ? kModelStanceScale : 1.0) / (1 - reclined) / modelUse;
				p.model = in.legs[i].footMeasurement;
			}
			p.under = !ls.planted && insSigma[i] <= 0 ? standing : 0.0;
			if (!ls.planted) {
				double hold = 0;
				if (kHoldTime > 0) {
					// Holding a leg up stands on the other one (jogging or dancing in place,
					// neither foot stays planted)
					const LegState& other = m_legs[o];
					const double support = other.planted ? 1.0 : smoothstep(kMaxSwing, 0.5 * kMaxSwing, other.swingTime);
					const double quiet = smoothstep(kHoldBounce, 0.5 * kHoldBounce, m_hmdBounce);
					// Knee lifts stand tall: crouching, a moving foot is on the floor
					const double tall = smoothstep(0.82, 0.9, eyeRatio);
					if (ls.holdLatched)
						hold = support * quiet * tall * smoothstep(kHoldStart, kHoldStart + 0.3, ls.swingTime) *
							   std::exp(-std::max(0.0, ls.swingTime - kMaxSwing) / kHoldTime);
				}
				if (ls.swingTime >= kMaxSwing) hold = std::max(hold, kRaisedUse * m_raised.p[i]);
				// Standing only: seated or squatting, a moving leg rests on the floor or the seat
				p.hold = hold * standing;
				held[i] = p.hold;
				out.debug.hold[i] = p.hold;
				out.debug.holdLatched[i] = ls.holdLatched;
				out.debug.lowerDisp[i] = ls.lowerDisp;
				p.holdThigh = pelvis.rotate(ls.holdThigh);
				// Even partly held, the foot is not pulled under the hip (its horizontal
				// pull alone dragged a raised knee down)
				p.under *= smoothstep(0.3, 0.0, p.hold);
				// Nor by the foot model: it pulled the held foot back toward the standing
				// leg, the knee unfolded and the raised foot went 10-20 cm lower
				if (p.modelSigma > 0) p.modelSigma = p.hold > 0.95 ? 0.0 : p.modelSigma / (1 - p.hold);
			}
			p.under *= 1 - kRaisedUse * m_raised.p[i];
			// Nothing places the foot any more (no anchor, the dead reckoning over, not
			// held): the knee prior must win within a fraction of a second. Against the
			// frame-to-frame smoothness its small pull straightened a bent knee in ~4 s
			// (legs raised lying on the back stayed folded)
			if (!ls.planted && insSigma[i] <= 0 && p.hold < 0.05) p.sigmaPhi *= kFreeSmooth;
			p.bedWeight = ls.planted ? bedness[i] * onBedWeight[i] : 0.0;
			p.bedY = std::max(ankleY, (hip0 + hipOffset).y - kBedBelowHip);
			p.otherHeight = std::max(0.0, (hip0 + hipOffset + legVector(o, phi[o])).y - ankleY);
			phi[i] = solveLeg(p);
		}

		// Hip offset with the legs fixed: per-axis weighted least squares, robust
		// terms reweighted (IRLS), in the pelvis heading frame (x right, z back) so
		// the prior can be looser along the forward axis
		const Quat toHeading = Quat::yaw(-pelvis.yawAngle());
		Vec3 num = toHeading.rotate(m_hipOffset) * (1.0 / sq(hipSigmaSmooth));
		Vec3 den = Vec3{1, 1, 1} * (1.0 / sq(hipSigmaSmooth));
		const double hipSigmaF = std::sqrt(sq(hipSigmaH) + sq(useWaistTracker ? kHipSigmaFwd : 0.0));
		den += Vec3{1 / sq(hipSigmaH), 1 / sq(hipSigmaV), 1 / sq(hipSigmaF)};
		auto addTarget = [&](const Vec3& target, double sigma, double k) {
			double e2 = squaredNorm(hipOffset - target) / sq(sigma);
			double w = robustWeight(e2, k) / sq(sigma);
			num += toHeading.rotate(target) * w;
			den += Vec3{w, w, w};
		};
		if (seated > 0.5) addTarget(m_seat - hip0, kSeatSigma / std::sqrt(2 * seated - 1 + 1e-3), 3.0);
		if (swayActive) {
			Vec3 target = swayPrior - hip0;
			target.y = hipOffset.y;  // horizontal only
			addTarget(target, kSwaySigma / std::sqrt(m_swayGate), 3.0);
		}
		for (int i = 0; i < 2; i++) {
			Vec3 base = hip0 + legVector(i, phi[i]);  // ankle without hip offset
			if (m_legs[i].planted) addTarget(m_legs[i].anchor - base, anchorSigma, 3.0);
			// A swinging leg reaches its dead-reckoned position with its knee: it only
			// pulls the hip when the leg cannot get there (lifting a knee does not lift
			// the hip)
			// (A held leg keeps its knee where it is: it does not pull the hip either)
			if (insSigma[i] > 0 && held[i] < 0.99)
				addTarget(m_legs[i].insPos - base, insSigma[i] * kInsHipScale / (1 - held[i]), kInsRobust);
		}
		hipOffset = toHeading.conj().rotate({num.x / den.x, num.y / den.y, num.z / den.z});
		hipOffset.x = std::clamp(hipOffset.x, -hipLimitH, hipLimitH);
		hipOffset.z = std::clamp(hipOffset.z, -hipLimitH, hipLimitH);
		// Standing, the hip cannot hang much below the head chain (a spine does not
		// stretch): kneeling on a chair (head high, feet up on the seat) the support rule
		// sank the feet anchors to the floor and they pulled the hip 16 cm down
		hipOffset.y = std::clamp(hipOffset.y, -lerp(0.3, useWaistTracker ? kHipStretch : 0.3, standing), 0.3);
		// The hip stays above the floor, and lifts the legs out of it (the leg search
		// may leave an ankle a few cm in the floor rather than bend the knee)
		hipOffset.y = std::max(hipOffset.y, in.floorY + kHipClearance - hip0.y);
		for (int i = 0; i < 2; i++) {
			double lift = ankleY - (hip0 + legVector(i, phi[i])).y;
			hipOffset.y = std::max(hipOffset.y, std::min(lift, kMaxHipLift * standing));
		}
	}
	m_hipOffset = hipOffset;
	const Vec3 hip = hip0 + hipOffset;

	// ---- support: standing, some foot carries the weight. When no planted foot is
	// on the floor (both resting in the air, or one in the air and the other
	// swinging), the lowest planted one is on the floor and its dead reckoning
	// stopped short: sink it. A foot on a bed or held up keeps its height as long
	// as the other one stands on the floor ----
	{
		int lowest = -1;
		bool onFloor = false;
		for (int i = 0; i < 2; i++) {
			const LegState& ls = m_legs[i];
			// A foot at the floor carries the weight even between two stance detections:
			// balancing on one leg (yoga tree pose) its stance flickered, and each time
			// the other foot, resting on the knee, sank a few cm
			if (kSupportAtFloor && ls.ankle.y - ankleY < kFloorSnap) onFloor = true;
			if (!ls.planted) continue;
			if (ls.anchor.y - ankleY < kFloorSnap) onFloor = true;
			if (lowest < 0 || ls.anchor.y < m_legs[lowest].anchor.y) lowest = i;
		}
		if (lowest >= 0 && !onFloor && standing > 0.5) {
			Vec3& a = m_legs[lowest].anchor;
			double keep = 1 - kRaisedUse * m_raised.p[lowest];
			a.y += (ankleY - a.y) * std::min(1.0, keep * standing * dt / kNoSupportSinkTime);
		}
		// A planted foot left in the air that the micro-movements say is on the floor
		if (kRaisedSink > 0 && standing > 0.5)
			for (int i = 0; i < 2; i++) {
				LegState& ls = m_legs[i];
				if (!ls.planted || ls.anchor.y - ankleY < kFloorSnap) continue;
				ls.anchor.y += (ankleY - ls.anchor.y) * std::min(1.0, kRaisedSink * (1 - m_raised.p[i]) * standing * dt);
			}
	}

	// ---- legs: positions, anchors, swing correction, foot orientation ----
	for (int i = 0; i < 2; i++) {
		LegState& ls = m_legs[i];
		ls.phi = phi[i];
		Vec3 thighDir = down[i] * std::cos(ls.phi) + forward[i] * std::sin(ls.phi);
		ls.thighLocal = pelvis.conj().rotate(thighDir);
		if (ls.planted || ls.swingTime < kHoldStart) ls.holdThigh = ls.thighLocal;
		Vec3 knee = hip + legOffset[i] + thighDir * d.thigh;
		Vec3 ankle = knee + down[i] * d.shin;
		ankle.y = std::max(ankle.y, ankleY);

		if (ls.planted) {
			// The anchor follows the solution slowly, and jumps when it was wrong
			// (the foot slid, the body moved away)
			if ((ankle - ls.anchor).norm() > kAnchorReset) {
				// Seated, a planted foot keeps its height: the knee solution flipped (foot
				// flat on the bed vs leg hanging from it), not the foot
				const double y = ls.anchor.y;
				ls.anchor = ankle;
				if (seated > 0.5) ls.anchor.y = y;
			} else {
				// Horizontally only: vertically, the floor decides (below)
				double k = std::min(1.0, dt / kAnchorLeak);
				ls.anchor.x += (ankle.x - ls.anchor.x) * k;
				ls.anchor.z += (ankle.z - ls.anchor.z) * k;
			}
			// Standing, a foot resting a few cm above the floor is on it
			if (ls.anchor.y > ankleY && ls.anchor.y < ankleY + kFloorSink)
				ls.anchor.y += (ankleY - ls.anchor.y) * std::min(1.0, standing * dt / kFloorSinkTime);
			// Seated upright, a foot resting below furniture height is on the floor
			if (ls.anchor.y > ankleY && ls.anchor.y < ankleY + kSeatFloorSnap)
				ls.anchor.y += (ankleY - ls.anchor.y) * std::min(1.0, lerp(seated, floorPref[i], onBedWeight[i]) * dt / kSeatSinkTime);
			ls.insPos = ankle;
			ls.insVel = {};
		} else {
			// Long swings: pull the dead reckoning toward the solution so its drift
			// stays bounded
			double gain = std::min(1.0, dt * 3.0 * smoothstep(0.4, 1.0, ls.swingTime));
			Vec3 kinematicVelocity = (ankle - ls.ankle) / dt;
			ls.insPos += (ankle - ls.insPos) * gain;
			ls.insVel += (kinematicVelocity - ls.insVel) * gain;
			double v = ls.insVel.norm();
			if (v > 8.0) ls.insVel = ls.insVel * (8.0 / v);
		}
		ls.ankle = ankle;

		// Flat on the floor when standing on it; following the shin otherwise (in the
		// air, kneeling, lying down)
		bool onFloor = ls.planted && std::fabs(ankle.y - ankleY) < 0.06 && down[i].y < -0.5;
		double target = onFloor ? 0.0 : 1.0;
		ls.footBlend += (target - ls.footBlend) * std::min(1.0, dt / 0.12);
		const double yawNow = shinYaw(in.legs[i].shin);
		if (!ls.footYawInit) {
			ls.footYawInit = true;
			ls.footYaw = yawNow;
		}
		const double yawDiff = wrapAngle(yawNow - ls.footYaw);
		if (onFloor) {
			if (std::fabs(yawDiff) > kFootYawFree)
				ls.footYaw = wrapAngle(ls.footYaw + yawDiff - std::copysign(kFootYawFree, yawDiff));
		} else {
			ls.footYaw = wrapAngle(ls.footYaw + yawDiff * std::min(1.0, dt / 0.1));
		}
		Quat flat = Quat::yaw(ls.footYaw);
		Quat footRot = slerp(flat, in.legs[i].shin, ls.footBlend);

		// Planted, the output foot is the stance anchor itself: the leg has one degree
		// of freedom (the knee), so the solved ankle went wherever the hip took it (the
		// head chain sways it: 10 cm/s of foot slide while standing). SteamVR only gets
		// the feet and the waist, the game solves the knees. Blended in at touchdown and
		// out at lift-off. Horizontally only: the height stays the solution's (an anchor
		// can keep a stale height, the floor's under a foot now on a bed)
		ls.outBlend += ((ls.planted ? kPlantedOutput : 0.0) - ls.outBlend) * std::min(1.0, dt / (ls.planted ? 0.05 : 0.1));
		Vec3 outAnkle = ankle;
		outAnkle.x += (ls.anchor.x - ankle.x) * ls.outBlend;
		outAnkle.z += (ls.anchor.z - ankle.z) * ls.outBlend;
		out.feet[i] = {outAnkle, footRot};
		out.planted[i] = ls.planted;
		out.knees[i] = knee;
		out.kneeAngles[i] = rad2deg(ls.phi);
		out.debug.anchors[i] = ls.anchor;
		out.debug.ins[i] = ls.insPos;
		out.debug.insSigma[i] = insSigma[i];
	}
	out.debug.neck = neck;
	out.debug.hip0 = hip0;
	for (int i = 0; i < 2; i++) out.debug.hipJoints[i] = hip + legOffset[i];
	out.debug.standing = standing;

	// ---- waist orientation in ankles-only mode: spine from hip to neck ----
	Quat waistRot = pelvis;
	if (!useWaistTracker) waistRot = Quat::fromTwoVectors(torsoUp, (neck - hip).normalized()) * pelvis;
	out.waist = {hip, waistRot};

	// ---- smoothing and velocities ----
	double sm = std::clamp(s.smoothing, 0.0, 1.0);
	double minCutoff = lerp(20.0, 1.5, sm);
	for (int i = 0; i < 2; i++) {
		Vec3 p = m_filters[i].filter(out.feet[i].position, dt, minCutoff, 0.8);
		out.footVelocity[i] = m_initialized ? (p - m_smoothedFeet[i]) / dt : Vec3{};
		m_smoothedFeet[i] = p;
		out.feet[i].position = p;
		// Foot pitch limits keeping the toe tip and the heel (on the sole, ankleHeight below
		// the ankle) above the floor (kToeTip, kHeel), at the smoothed height: following
		// the shin in the air the foot pointed down, into the floor at each touchdown and
		// toe-off (and the smoothed ankle lags below the solution rising)
		Quat& footRot = out.feet[i].rotation;
		const double h = d.ankleHeight, c = in.floorY - kToeSink - p.y;  // c < 0
		const double toe = kToeTip * d.height, heel = kHeel * d.height;
		const double pitch = std::asin(std::clamp(footRot.rotate({0, 0, -1}).y, -1.0, 1.0));
		// toe: toe sin p - h cos p >= c; heel: -heel sin p - h cos p >= c
		const double rt = std::hypot(toe, h), rh = std::hypot(heel, h);
		double lo = -kPi, hi = kPi;
		if (c / rt > -1) lo = std::atan2(h, toe) + std::asin(std::clamp(c / rt, -1.0, 1.0));
		if (-c / rh < 1) hi = std::asin(std::clamp(-c / rh, -1.0, 1.0)) - std::atan2(h, heel);
		const double limited = lo <= hi ? std::clamp(pitch, lo, hi) : 0.5 * (lo + hi);
		if (limited != pitch) footRot = Quat::fromAxisAngle(footRot.rotate({1, 0, 0}), limited - pitch) * footRot;
	}
	Vec3 w = m_filters[2].filter(out.waist.position, dt, minCutoff, 0.8);
	out.waistVelocity = m_initialized ? (w - m_smoothedWaist) / dt : Vec3{};
	m_smoothedWaist = w;
	out.waist.position = w;

	m_initialized = true;
	out.valid = true;
	return out;
}

}  // namespace smt
