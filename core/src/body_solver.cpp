#include "slimelite/body_solver.h"

#include <algorithm>
#include <cmath>

namespace slimelite {

namespace {

// Knee angle range (straight to fully bent)
constexpr double kPhiMin = -5.0 * kPi / 180;
constexpr double kPhiMax = 150.0 * kPi / 180;
constexpr int kPhiSteps = 78;
// Hip flexion range: thigh direction relative to the pelvis, in its sagittal plane
constexpr double kFlexMin = -35.0 * kPi / 180;
constexpr double kFlexMax = 135.0 * kPi / 180;
// Among the allowed poses, prefer straight legs (sigma of the knee angle prior)
constexpr double kStraightSigma = 1.2;
// Bending a knee a little costs about as much as bending it a lot (saturating
// prior): small head drops while standing are slouching, not knee bends. Without
// it the knees absorbed the few cm between the calibrated and the actual head
// height by bending 20-40 deg with vertical shins (the hip sitting back)
constexpr double kBendCost = 3.0;
constexpr double kBendSigma = 0.35;  // rad
// A leg can push the hip up instead of bending: ankle penetration of the floor
// up to kMaxHipLift (m) costs this sigma (m) in the leg search, and the hip solve
// then lifts the hip by as much. Only when the head says standing (the gap is the
// posture of the upper body); deeper penetrations are not allowed (sitting,
// squatting: the knees bend)
constexpr double kHipLiftSigma = 0.04;
constexpr double kMaxHipLift = 0.07;
// Clearances above the floor: knee center when kneeling, hip joint when lying
constexpr double kKneeClearance = 0.05;
constexpr double kHipClearance = 0.08;
// Stance anchors follow the solution with this time constant (s), and jump to it
// when farther than kAnchorReset (m)
constexpr double kAnchorLeak = 1.0;
constexpr double kAnchorReset = 0.15;
// Swing prediction is dropped after this long without a stance (s)
constexpr double kMaxSwing = 1.5;
// Lift-off detection from the accelerometer integrated during stance
constexpr double kStanceVelLeak = 0.25;  // s
constexpr double kStanceDispLeak = 0.3;  // s
constexpr double kLiftOffSpeed = 0.2;  // m/s
// A foot that stops within this distance of the floor (m) is on the floor. The
// dead-reckoned height at touchdown is within 5 cm of the floor for ~90% of the
// steps; a foot stopping higher rests on furniture or is held up, and stays there
constexpr double kFloorSnap = 0.06;
// Standing, a planted foot still slightly above the floor (below kFloorSink, m)
// sinks to it with this time constant (s): dead reckoning errors would pile up
constexpr double kFloorSink = 0.10;
constexpr double kFloorSinkTime = 2.0;
// Standing with no planted foot on the floor, the lowest one sinks to it with this
// time constant (s)
constexpr double kNoSupportSinkTime = 0.5;
// The swing prediction keeps its pull further than the stance anchors (in sigmas)
constexpr double kInsRobust = 5.0;
// ... and pulls the hip offset with a sigma this many times larger
constexpr double kInsHipScale = 5.0;

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
	double support = 0;  // weight of "at least one foot on the floor"
	double otherHeight = 0;  // other ankle above its floor position
};

void BodySolver::configure(const BodyDimensions& dims, const Settings& settings, Mode mode) {
	m_dims = dims;
	m_settings = settings;
	m_mode = mode;
}

void BodySolver::reset() {
	m_legs = {};
	m_hipOffset = {};
	m_initialized = false;
	m_filters = {};
	m_smoothedFeet = {};
	m_smoothedWaist = {};
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
	}
	if (p.planted) c += robust(squaredNorm(ankle - p.anchor) / sq(p.anchorSigma), 3.0);
	if (p.insSigma > 0) c += robust(squaredNorm(ankle - p.ins) / sq(p.insSigma), kInsRobust);
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

BodyOutput BodySolver::update(const BodyInput& in) {
	BodyOutput out;
	if (!in.hmdValid || !in.legs[0].valid || !in.legs[1].valid) return out;
	const double dt = std::clamp(in.dt, 1e-4, 0.1);
	const auto& d = m_dims;
	const auto& s = m_settings;
	const double ankleY = in.floorY + d.ankleHeight;
	const Vec3 up{0, 1, 0};

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
	const Vec3 neck = headPivot - torsoUp * d.neckDown;
	const Vec3 hip0 = neck - torsoUp * d.spine;
	const double standing = smoothstep(0.55, 0.8, (hip0.y - in.floorY) / d.standingHipHeight());

	// ---- per-leg geometry, stance detection (zero velocity), swing dead reckoning ----
	std::array<Vec3, 2> legOffset, down, forward;
	for (int i = 0; i < 2; i++) {
		const auto& li = in.legs[i];
		LegState& ls = m_legs[i];
		legOffset[i] = pelvis.rotate({(i == 0 ? -0.5 : 0.5) * d.hipWidth, 0, 0});
		down[i] = li.shin.rotate({0, -1, 0});
		forward[i] = li.shin.rotate({0, 0, -1});

		if (!m_initialized || !ls.initialized) {
			// Start standing: straight leg below the hip joint
			ls = LegState{};
			ls.initialized = true;
			ls.ankle = hip0 + legOffset[i] + down[i] * (d.thigh + d.shin);
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
			if (ls.anchor.y - ankleY < kFloorSnap) ls.anchor.y = ankleY;
			out.debug.touchdownSwing[i] = ls.swingTime;
			out.debug.touchdownHeight[i] = ls.anchor.y - ankleY;
			ls.stanceVel = ls.stanceDisp = {};
		} else if (ls.planted && (a > 2.0 || li.angularSpeed > 4.0 || ls.stanceVel.norm() > kLiftOffSpeed)) {
			// Lift-off: the swing starts with the motion already integrated
			ls.planted = false;
			ls.swingTime = 0;
			ls.insPos = ls.ankle + ls.stanceDisp;
			ls.insVel = ls.stanceVel;
		}
		if (!ls.planted) {
			ls.insVel += li.accelWorld * dt;
			ls.insPos += ls.insVel * dt;
			ls.swingTime += dt;
		}
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
	const double hipSigmaH = useWaistTracker ? 0.05 : 0.15;
	const double hipSigmaV = useWaistTracker ? 0.06 : 0.10;
	const double hipSigmaSmooth = std::max(0.002, 1.5 * dt);
	const double hipLimitH = useWaistTracker ? 0.25 : 0.4;

	auto legVector = [&](int i, double phi) {
		Vec3 thighDir = down[i] * std::cos(phi) + forward[i] * std::sin(phi);
		return legOffset[i] + thighDir * d.thigh + down[i] * d.shin;
	};
	Vec3 hipOffset = m_hipOffset;
	std::array<double, 2> phi{m_legs[0].phi, m_legs[1].phi};
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
			p.otherHeight = std::max(0.0, (hip0 + hipOffset + legVector(o, phi[o])).y - ankleY);
			phi[i] = solveLeg(p);
		}

		// Hip offset with the legs fixed: per-axis weighted least squares, robust
		// terms reweighted (IRLS)
		Vec3 num = m_hipOffset * (1.0 / sq(hipSigmaSmooth));
		Vec3 den = Vec3{1, 1, 1} * (1.0 / sq(hipSigmaSmooth));
		den += Vec3{1 / sq(hipSigmaH), 1 / sq(hipSigmaV), 1 / sq(hipSigmaH)};
		auto addTarget = [&](const Vec3& target, double sigma, double k) {
			double e2 = squaredNorm(hipOffset - target) / sq(sigma);
			double w = robustWeight(e2, k) / sq(sigma);
			num += target * w;
			den += Vec3{w, w, w};
		};
		for (int i = 0; i < 2; i++) {
			Vec3 base = hip0 + legVector(i, phi[i]);  // ankle without hip offset
			if (m_legs[i].planted) addTarget(m_legs[i].anchor - base, anchorSigma, 3.0);
			// A swinging leg reaches its dead-reckoned position with its knee: it only
			// pulls the hip when the leg cannot get there (lifting a knee does not lift
			// the hip)
			if (insSigma[i] > 0) addTarget(m_legs[i].insPos - base, insSigma[i] * kInsHipScale, kInsRobust);
		}
		hipOffset = {num.x / den.x, num.y / den.y, num.z / den.z};
		hipOffset.x = std::clamp(hipOffset.x, -hipLimitH, hipLimitH);
		hipOffset.z = std::clamp(hipOffset.z, -hipLimitH, hipLimitH);
		hipOffset.y = std::clamp(hipOffset.y, -0.3, 0.3);
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
			if (!ls.planted) continue;
			if (ls.anchor.y - ankleY < kFloorSnap) onFloor = true;
			if (lowest < 0 || ls.anchor.y < m_legs[lowest].anchor.y) lowest = i;
		}
		if (lowest >= 0 && !onFloor && standing > 0.5) {
			Vec3& a = m_legs[lowest].anchor;
			a.y += (ankleY - a.y) * std::min(1.0, standing * dt / kNoSupportSinkTime);
		}
	}

	// ---- legs: positions, anchors, swing correction, foot orientation ----
	for (int i = 0; i < 2; i++) {
		LegState& ls = m_legs[i];
		ls.phi = phi[i];
		Vec3 thighDir = down[i] * std::cos(ls.phi) + forward[i] * std::sin(ls.phi);
		Vec3 knee = hip + legOffset[i] + thighDir * d.thigh;
		Vec3 ankle = knee + down[i] * d.shin;
		ankle.y = std::max(ankle.y, ankleY);

		if (ls.planted) {
			// The anchor follows the solution slowly, and jumps when it was wrong
			// (the foot slid, the body moved away)
			if ((ankle - ls.anchor).norm() > kAnchorReset) {
				ls.anchor = ankle;
			} else {
				// Horizontally only: vertically, the floor decides (below)
				double k = std::min(1.0, dt / kAnchorLeak);
				ls.anchor.x += (ankle.x - ls.anchor.x) * k;
				ls.anchor.z += (ankle.z - ls.anchor.z) * k;
			}
			// Standing, a foot resting a few cm above the floor is on it
			if (ls.anchor.y > ankleY && ls.anchor.y < ankleY + kFloorSink)
				ls.anchor.y += (ankleY - ls.anchor.y) * std::min(1.0, standing * dt / kFloorSinkTime);
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
		Quat flat = Quat::yaw(shinYaw(in.legs[i].shin));
		Quat footRot = slerp(flat, in.legs[i].shin, ls.footBlend);

		out.feet[i] = {ankle, footRot};
		out.planted[i] = ls.planted;
		out.knees[i] = knee;
		out.kneeAngles[i] = rad2deg(ls.phi);
		out.debug.anchors[i] = ls.anchor;
		out.debug.ins[i] = ls.insPos;
		out.debug.insSigma[i] = insSigma[i];
	}
	out.debug.neck = neck;
	out.debug.hip0 = hip0;
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
	}
	Vec3 w = m_filters[2].filter(out.waist.position, dt, minCutoff, 0.8);
	out.waistVelocity = m_initialized ? (w - m_smoothedWaist) / dt : Vec3{};
	m_smoothedWaist = w;
	out.waist.position = w;

	m_initialized = true;
	out.valid = true;
	return out;
}

}  // namespace slimelite
