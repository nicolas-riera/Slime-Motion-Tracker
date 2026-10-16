#include "slimelite/body_solver.h"

#include <algorithm>
#include <vector>

namespace slimelite {

namespace {

double lerp(double a, double b, double t) { return a + (b - a) * t; }

// Yaw of a shin from its left-right axis, which stays horizontal-ish for a hinge
// leg even when the shin itself is horizontal (sitting, kneeling)
double shinYaw(const Quat& shin) {
	Vec3 r = shin.rotate({1, 0, 0});
	return std::atan2(-r.z, r.x);
}

// Solves the n x n system A x = b in place (Gaussian elimination, partial pivot)
bool solveLinear(int n, std::vector<double>& A, std::vector<double>& b) {
	for (int c = 0; c < n; c++) {
		int p = c;
		for (int r = c + 1; r < n; r++)
			if (std::fabs(A[r * n + c]) > std::fabs(A[p * n + c])) p = r;
		if (std::fabs(A[p * n + c]) < 1e-12) return false;
		if (p != c) {
			for (int k = 0; k < n; k++) std::swap(A[c * n + k], A[p * n + k]);
			std::swap(b[c], b[p]);
		}
		for (int r = c + 1; r < n; r++) {
			double f = A[r * n + c] / A[c * n + c];
			for (int k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
			b[r] -= f * b[c];
		}
	}
	for (int c = n - 1; c >= 0; c--) {
		for (int k = c + 1; k < n; k++) b[c] -= A[c * n + k] * b[k];
		b[c] /= A[c * n + c];
	}
	return true;
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
	d.neckDown = 0.058 * h;  // neck, assumed vertical
	double eye = eyeHeight.value_or(0.936 * h);
	double neckBase = eye - d.headDown - d.neckDown;
	double hipCenter = d.ankleHeight + d.shin + d.thigh;
	d.spine = std::clamp(neckBase - hipCenter, 0.22 * h, 0.36 * h);
	return d;
}

struct BodySolver::Frame {
	Vec3 hip0;  // prior hip center
	Quat pelvis;
	struct Leg {
		bool valid = false;
		Vec3 hipOffset;  // hip center -> hip joint
		Vec3 down, forward;  // shin axes
		bool planted = false;
		Vec3 anchor;
		bool predicted = false;
		Vec3 prediction;
		double phiPrev = 0;
	};
	std::array<Leg, 2> legs;
	Vec3 hipOffsetPrev;
	double floorY = 0;
	// 1 / sigma
	double wPlant = 0, wPredict = 0, wPhiSmooth = 0, wHipH = 0, wHipV = 0, wHipSmooth = 0;
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

void BodySolver::residuals(const Frame& f, const double* x, std::vector<double>& r) const {
	r.clear();
	Vec3 hipOffset{x[2], x[3], x[4]};
	Vec3 hip = f.hip0 + hipOffset;
	for (int i = 0; i < 2; i++) {
		const auto& leg = f.legs[i];
		double phi = x[i];
		if (!leg.valid) {
			r.push_back((phi - leg.phiPrev) * 1.0);
			continue;
		}
		Vec3 thighDir = leg.down * std::cos(phi) + leg.forward * std::sin(phi);
		Vec3 ankle = hip + leg.hipOffset + thighDir * m_dims.thigh + leg.down * m_dims.shin;
		if (leg.planted) {
			Vec3 e = (ankle - leg.anchor) * f.wPlant;
			r.insert(r.end(), {e.x, e.y, e.z});
		} else if (leg.predicted) {
			Vec3 e = (ankle - leg.prediction) * f.wPredict;
			r.insert(r.end(), {e.x, e.y, e.z});
		}
		double penetration = f.floorY + m_dims.ankleHeight - ankle.y;
		r.push_back(std::max(0.0, penetration) * 200.0);
		r.push_back((phi - leg.phiPrev) * f.wPhiSmooth);
		r.push_back(std::max(0.0, -0.05 - phi) * 100.0);
		r.push_back(std::max(0.0, phi - 2.6) * 100.0);
	}
	r.push_back(hipOffset.x * f.wHipH);
	r.push_back(hipOffset.y * f.wHipV);
	r.push_back(hipOffset.z * f.wHipH);
	Vec3 d = (hipOffset - f.hipOffsetPrev) * f.wHipSmooth;
	r.insert(r.end(), {d.x, d.y, d.z});
}

BodyOutput BodySolver::update(const BodyInput& in) {
	BodyOutput out;
	if (!in.hmdValid || !in.legs[0].valid || !in.legs[1].valid) return out;
	const double dt = std::clamp(in.dt, 1e-4, 0.1);
	const auto& d = m_dims;
	const auto& s = m_settings;
	const double ankleY = in.floorY + d.ankleHeight;

	// ---- upper body: neck point and pelvis ----
	const Quat& hq = in.hmd.rotation;
	Vec3 neck = in.hmd.position + hq.rotate({0, -d.headDown, d.headBack}) + Vec3{0, -d.neckDown, 0};

	bool useWaistTracker = m_mode == Mode::Three && in.hasWaist;
	double pelvisYaw;
	Quat pelvis;
	if (useWaistTracker) {
		pelvis = in.waist;
		pelvisYaw = pelvis.yawAngle();
	} else {
		// Ankles only: heading from the shins (feet point roughly forward) and the head
		double sx = 0, sz = 0;
		auto add = [&](double yaw, double w) {
			sx += std::sin(yaw) * w;
			sz += std::cos(yaw) * w;
		};
		add(hq.yawAngle(), 0.6);
		add(shinYaw(in.legs[0].shin), 1.0);
		add(shinYaw(in.legs[1].shin), 1.0);
		pelvisYaw = std::atan2(sx, sz);
		// Torso lean follows a fraction of the head pitch
		Vec3 hf = hq.rotate({0, 0, -1});
		double headPitch = std::asin(std::clamp(-hf.y, -1.0, 1.0));  // > 0 looking down
		double lean = std::clamp(0.3 * headPitch, deg2rad(-15), deg2rad(30));
		pelvis = Quat::yaw(pelvisYaw) * Quat::fromAxisAngle({1, 0, 0}, -lean);
	}

	Frame f;
	f.pelvis = pelvis;
	f.hip0 = neck - pelvis.rotate({0, d.spine, 0});
	f.floorY = in.floorY;
	f.hipOffsetPrev = m_hipOffset;
	double plant = std::clamp(s.footPlantStrength, 0.0, 1.0);
	double inertial = std::clamp(s.inertialWeight, 0.0, 1.0);
	f.wPlant = 1.0 / lerp(0.08, 0.004, plant);
	f.wPredict = inertial > 0.01 ? 1.0 / lerp(0.30, 0.01, inertial) : 0.0;
	f.wPhiSmooth = 1.0 / 0.35;
	if (useWaistTracker) {
		f.wHipH = 1.0 / 0.05;
		f.wHipV = 1.0 / 0.06;
	} else {
		f.wHipH = 1.0 / 0.15;
		f.wHipV = 1.0 / 0.10;
	}
	f.wHipSmooth = 1.0 / std::max(0.002, 1.5 * dt);

	// ---- per-leg stance detection (zero velocity) and inertial prediction ----
	for (int i = 0; i < 2; i++) {
		const auto& li = in.legs[i];
		LegState& ls = m_legs[i];
		auto& fl = f.legs[i];
		fl.valid = true;
		double side = i == 0 ? -0.5 : 0.5;
		fl.hipOffset = pelvis.rotate({side * d.hipWidth, 0, 0});
		fl.down = li.shin.rotate({0, -1, 0});
		fl.forward = li.shin.rotate({0, 0, -1});
		fl.phiPrev = ls.phi;

		if (!m_initialized || !ls.initialized) {
			// Start standing: straight leg below the hip joint
			ls.phi = 0.05;
			ls.ankle = f.hip0 + fl.hipOffset + fl.down * (d.thigh + d.shin);
			ls.velocity = {};
			ls.initialized = true;
		}

		// Shin-mounted IMU: during stance the shin pivots around the ankle (moderate
		// angular rate) but the ankle barely accelerates, so gate mostly on accel
		double a = li.accelWorld.norm();
		bool still = a < 1.2 && li.angularSpeed < 2.5;
		ls.stillTime = still ? ls.stillTime + dt : 0;
		if (!ls.planted && ls.stillTime >= 0.05) {
			ls.planted = true;
			ls.anchor = ls.ankle;
			if (std::fabs(ls.anchor.y - ankleY) < 0.10) ls.anchor.y = ankleY;
			ls.velocity = {};
		} else if (ls.planted && (a > 2.5 || li.angularSpeed > 4.0)) {
			ls.planted = false;
			ls.velocity = {};
		}
		fl.planted = ls.planted;
		fl.anchor = ls.anchor;
		if (!ls.planted && f.wPredict > 0) {
			ls.velocity += li.accelWorld * dt;
			fl.predicted = true;
			fl.prediction = ls.ankle + ls.velocity * dt;
		}
	}

	// ---- Gauss-Newton on x = [phiL, phiR, hip offset] ----
	double x[5] = {m_legs[0].phi, m_legs[1].phi, m_hipOffset.x, m_hipOffset.y, m_hipOffset.z};
	std::vector<double> r0, r1;
	for (int iter = 0; iter < 4; iter++) {
		residuals(f, x, r0);
		const int m = int(r0.size());
		std::vector<double> J(m * 5);
		for (int k = 0; k < 5; k++) {
			double xs[5];
			std::copy(x, x + 5, xs);
			const double eps = 1e-5;
			xs[k] += eps;
			residuals(f, xs, r1);
			for (int j = 0; j < m; j++) J[j * 5 + k] = (r1[j] - r0[j]) / eps;
		}
		std::vector<double> A(25, 0.0), b(5, 0.0);
		for (int j = 0; j < m; j++)
			for (int p = 0; p < 5; p++) {
				b[p] -= J[j * 5 + p] * r0[j];
				for (int q = 0; q < 5; q++) A[p * 5 + q] += J[j * 5 + p] * J[j * 5 + q];
			}
		for (int p = 0; p < 5; p++) A[p * 5 + p] += 1e-6;
		if (!solveLinear(5, A, b)) break;
		double step = 0;
		for (int p = 0; p < 5; p++) {
			x[p] += b[p];
			step += b[p] * b[p];
		}
		if (step < 1e-10) break;
	}
	x[0] = std::clamp(x[0], -0.1, 2.7);
	x[1] = std::clamp(x[1], -0.1, 2.7);
	m_hipOffset = {std::clamp(x[2], -0.4, 0.4), std::clamp(x[3], -0.4, 0.4), std::clamp(x[4], -0.4, 0.4)};
	Vec3 hip = f.hip0 + m_hipOffset;

	// ---- legs: positions, velocity fusion, slip handling, foot orientation ----
	for (int i = 0; i < 2; i++) {
		LegState& ls = m_legs[i];
		const auto& fl = f.legs[i];
		ls.phi = x[i];
		Vec3 thighDir = fl.down * std::cos(ls.phi) + fl.forward * std::sin(ls.phi);
		Vec3 knee = hip + fl.hipOffset + thighDir * d.thigh;
		Vec3 ankle = knee + fl.down * d.shin;
		ankle.y = std::max(ankle.y, ankleY);

		if (ls.planted) {
			// The anchor was wrong (or the foot slid): move it
			if ((ankle - ls.anchor).norm() > 0.12) {
				ls.anchor = ankle;
				if (std::fabs(ls.anchor.y - ankleY) < 0.10) ls.anchor.y = ankleY;
			}
			ls.velocity = {};
		} else {
			Vec3 kinematicVelocity = (ankle - ls.ankle) / dt;
			ls.velocity += (kinematicVelocity - ls.velocity) * 0.3;
			double v = ls.velocity.norm();
			if (v > 8.0) ls.velocity = ls.velocity * (8.0 / v);
		}
		ls.ankle = ankle;

		bool onFloor = ls.planted && std::fabs(ankle.y - ankleY) < 0.06;
		double target = onFloor ? 0.0 : 1.0;
		ls.footBlend += (target - ls.footBlend) * std::min(1.0, dt / 0.12);
		Quat flat = Quat::yaw(shinYaw(in.legs[i].shin));
		Quat footRot = slerp(flat, in.legs[i].shin, ls.footBlend);

		out.feet[i] = {ankle, footRot};
		out.planted[i] = ls.planted;
		out.knees[i] = knee;
		out.kneeAngles[i] = rad2deg(ls.phi);
	}

	// ---- waist orientation in ankles-only mode: spine from hip to neck ----
	Quat waistRot = pelvis;
	if (!useWaistTracker) {
		Vec3 up = (neck - hip).normalized();
		Vec3 upLocal = Quat::yaw(-pelvisYaw).rotate(up);
		waistRot = Quat::yaw(pelvisYaw) * Quat::fromTwoVectors({0, 1, 0}, upLocal);
	}
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
