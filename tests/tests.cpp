// Core unit tests on synthetic data. Run: build\Release\slimelite_tests.exe
#include <cstdio>
#include <cstring>
#include <random>

#include "slimelite/body_solver.h"
#include "slimelite/mag.h"
#include "slimelite/mounting_calibration.h"
#include "slimelite/protocol.h"

using namespace slimelite;

static int g_failures = 0;

#define CHECK(cond, ...)                                     \
	do {                                                     \
		if (!(cond)) {                                       \
			g_failures++;                                    \
			std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
			std::printf(__VA_ARGS__);                        \
			std::printf("\n");                               \
		}                                                    \
	} while (0)

static Quat randomQuat(std::mt19937& rng) {
	std::normal_distribution<double> n;
	return Quat{n(rng), n(rng), n(rng), n(rng)}.normalized();
}

static double angleBetween(const Quat& a, const Quat& b) { return rad2deg((a.conj() * b).angle()); }

static void testProtocol() {
	// Bundle with a rotation and a raw mag packet
	std::vector<uint8_t> p = {0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 1};
	auto put32 = [&](uint32_t v) {
		for (int i = 3; i >= 0; i--) p.push_back(uint8_t(v >> (8 * i)));
	};
	auto putF = [&](float f) {
		uint32_t v;
		std::memcpy(&v, &f, 4);
		put32(v);
	};
	p.push_back(0);
	p.push_back(4 + 2 + 16 + 1);
	put32(17);
	p.push_back(0);
	p.push_back(1);
	putF(0.f);
	putF(0.f);
	putF(0.7071068f);
	putF(0.7071068f);
	p.push_back(0);
	p.push_back(0);
	p.push_back(4 + 7);
	put32(210);
	p.push_back(0);
	for (int16_t v : {int16_t(-1234), int16_t(42), int16_t(-4053)}) {
		p.push_back(uint8_t(uint16_t(v) >> 8));
		p.push_back(uint8_t(v & 0xff));
	}
	auto ev = proto::parse(p.data(), p.size());
	CHECK(ev.size() == 2, "bundle: %zu events", ev.size());
	if (ev.size() == 2) {
		CHECK(ev[0].kind == proto::Event::Kind::Rotation, "rotation kind");
		CHECK(std::fabs(ev[0].rotation.z - 0.7071068) < 1e-6, "rotation z");
		CHECK(ev[1].kind == proto::Event::Kind::Mag, "mag kind");
		CHECK(ev[1].mag[0] == -1234 && ev[1].mag[1] == 42 && ev[1].mag[2] == -4053, "mag values");
	}
}

// Synthetic mag: raw = A^-1 * (M^T R^T m_e) + b, with an unknown 90 deg axis mapping
struct SyntheticMag {
	Mat3 sensorToMag;  // rotation: tracker frame -> mag chip frame
	Mat3 softIron;  // gain + soft iron, chip frame
	Vec3 hardIron{496, -4053, 116};
	Vec3 earth;  // world frame, unit
	std::array<int16_t, 3> raw(const Quat& q, std::mt19937& rng) const {
		std::normal_distribution<double> noise(0, 8);
		Vec3 inSensor = q.conj().rotate(earth);
		Vec3 r = softIron * (sensorToMag * inSensor) + hardIron;
		return {int16_t(r.x + noise(rng)), int16_t(r.y + noise(rng)), int16_t(r.z + noise(rng))};
	}
};

static SyntheticMag makeMag() {
	SyntheticMag s;
	s.sensorToMag = Quat::fromAxisAngle({0, 0, 1}, kPi / 2).toMatrix();
	s.softIron.m = {1800, 60, 0, 60, 1550, -40, 0, -40, 1700};
	double dip = deg2rad(60);
	s.earth = {std::cos(dip) * std::sin(0.3), -std::sin(dip), std::cos(dip) * std::cos(0.3)};
	return s;
}

static void testMagCalibration() {
	std::mt19937 rng(1);
	SyntheticMag mag = makeMag();
	MagCalibrator cal;
	double t = 0;
	for (int i = 0; i < 5000 && cal.coverage() < 1.0; i++) {
		Quat q = randomQuat(rng);
		cal.addSample(q, mag.raw(q, rng), 0.5, t);
		t += 0.05;
	}
	CHECK(cal.coverage() >= 0.85, "coverage %.2f", cal.coverage());
	auto res = cal.solve();
	CHECK(res.has_value(), "mag solve failed");
	if (!res) return;
	CHECK(std::fabs(rad2deg(res->dip) - 60) < 1.0, "dip %.2f", rad2deg(res->dip));
	CHECK(res->residual < 0.03, "residual %.3f", res->residual);
	// Heading must be consistent for any orientation
	double maxErr = 0, ref = 0;
	for (int i = 0; i < 500; i++) {
		Quat q = randomQuat(rng);
		Vec3 v = q.rotate(res->apply(mag.raw(q, rng)));
		double h = headingOf(v);
		if (i == 0) ref = h;
		maxErr = std::max(maxErr, std::fabs(rad2deg(wrapAngle(h - ref))));
	}
	CHECK(maxErr < 3.0, "heading spread %.2f deg", maxErr);
	std::printf("mag calibration: dip %.2f, residual %.4f, heading spread %.2f deg\n",
				rad2deg(res->dip), res->residual, maxErr);

	// Degenerate data: only yaw rotations must not produce a calibration
	MagCalibrator flat;
	for (int i = 0; i < 2000; i++) {
		Quat q = Quat::yaw(i * 0.05);
		flat.addSample(q, mag.raw(q, rng), 0.5, i * 0.05);
	}
	CHECK(!flat.solve().has_value(), "yaw-only data should be rejected");
}

static void testHeadingFilter() {
	std::mt19937 rng(2);
	SyntheticMag mag = makeMag();
	MagCalibrator cal;
	for (int i = 0; i < 4000; i++) {
		Quat q = randomQuat(rng);
		cal.addSample(q, mag.raw(q, rng), 0.5, i * 0.05);
	}
	auto c = cal.solve();
	CHECK(c.has_value(), "calibration for heading test");
	if (!c) return;

	HeadingFilter hf;
	hf.setCalibration(*c);
	HeadingFilterSettings s;
	// The device frame has a yaw drift of 20 deg relative to the truth
	Quat drift = Quat::yaw(deg2rad(20));
	Quat truth = Quat::fromAxisAngle({1, 0, 0}, 0.3);
	double dt = 0.02;
	for (int i = 0; i < 1500; i++) hf.update(drift * truth, mag.raw(truth, rng), 0.0, dt, s);
	double h0 = hf.heading();
	CHECK(hf.state() == HeadingFilter::State::Ok, "state ok after convergence");
	// Corrected orientation must not depend on the drift
	Quat corrected = Quat::yaw(-h0) * drift * truth;
	HeadingFilter hf2;
	hf2.setCalibration(*c);
	for (int i = 0; i < 1500; i++) hf2.update(truth, mag.raw(truth, rng), 0.0, dt, s);
	Quat corrected2 = Quat::yaw(-hf2.heading()) * truth;
	CHECK(angleBetween(corrected, corrected2) < 1.0, "drift not removed: %.2f deg",
		  angleBetween(corrected, corrected2));

	// Anti-mag: a magnet adds a big field for 3 s, the tracker does not move
	for (int i = 0; i < 150; i++) {
		auto raw = mag.raw(truth, rng);
		raw[0] = int16_t(raw[0] + 9000);
		raw[2] = int16_t(raw[2] - 6000);
		hf.update(drift * truth, raw, 0.0, dt, s);
	}
	CHECK(hf.state() == HeadingFilter::State::Disturbed, "magnet not detected");
	CHECK(std::fabs(rad2deg(wrapAngle(hf.heading() - h0))) < 0.5, "heading moved during disturbance");
	// Heading-only disturbance: same norm and dip, field rotated by 40 deg
	Quat fake = Quat::yaw(deg2rad(40));
	HeadingFilter hf3 = hf;
	for (int i = 0; i < 100; i++) hf3.update(drift * truth, mag.raw(fake * truth, rng), 0.0, dt, s);
	CHECK(std::fabs(rad2deg(wrapAngle(hf3.heading() - h0))) < 0.5, "heading-only disturbance followed");
	// Back to normal: recovers
	for (int i = 0; i < 200; i++) hf.update(drift * truth, mag.raw(truth, rng), 0.0, dt, s);
	CHECK(hf.state() == HeadingFilter::State::Ok, "not recovered after disturbance");

	// Gyro glitch: the device yaw jumps by 180 deg while the field does not move. The
	// mag must win within a second instead of being rejected as a disturbance
	Quat jumped = Quat::yaw(kPi) * drift;
	hf.gyroGlitch();
	for (int i = 0; i < 50; i++) hf.update(jumped * truth, mag.raw(truth, rng), 0.0, dt, s);
	Quat afterGlitch = Quat::yaw(-hf.heading()) * jumped * truth;
	CHECK(angleBetween(afterGlitch, corrected) < 1.0 && hf.state() == HeadingFilter::State::Ok,
		  "no re-lock after a gyro glitch: %.2f deg", angleBetween(afterGlitch, corrected));
}

static void testMounting() {
	std::mt19937 rng(3);
	for (int trial = 0; trial < 20; trial++) {
		Quat mounting = randomQuat(rng);  // tracker frame -> segment frame (any side of the leg)
		double psiTrue = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
		double bodyYaw = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
		// q_segment = Ry(psi) * q_tracker * M  =>  q_tracker = Ry(-psi) * q_segment * M^-1
		auto tracker = [&](const Quat& seg) { return Quat::yaw(-psiTrue) * seg * mounting.conj(); };
		Quat segStand = Quat::yaw(bodyYaw);
		Quat segSki = Quat::yaw(bodyYaw) * Quat::fromAxisAngle({1, 0, 0}, deg2rad(-35));
		auto r = solveMounting(tracker(segStand), tracker(segSki), bodyYaw, 15);
		CHECK(r.ok, "mounting failed: %s", r.error.c_str());
		if (!r.ok) continue;
		// Any later pose must map back to the true segment orientation
		Quat seg = Quat::yaw(bodyYaw + 0.7) * Quat::fromAxisAngle({1, 0, 0}, -0.9);
		Quat est = Quat::yaw(r.yawAlign) * tracker(seg) * r.mounting;
		CHECK(angleBetween(est, seg) < 0.5, "mounting error %.2f deg", angleBetween(est, seg));

		auto q = solveQuickReset(tracker(segStand), r.mounting, bodyYaw);
		Quat est2 = Quat::yaw(q.yawAlign) * tracker(seg) * q.mounting;
		CHECK(angleBetween(est2, seg) < 0.5, "quick reset error %.2f deg", angleBetween(est2, seg));
	}
}

static BodyOutput runPose(BodySolver& solver, const BodyDimensions& d, double hmdY, double shinPitch,
						  int frames) {
	BodyInput in;
	in.dt = 1.0 / 90;
	in.hmdValid = true;
	in.hmd.position = {0, hmdY, 0};
	in.floorY = 0;
	in.hasWaist = true;
	in.waist = Quat{};
	for (auto& leg : in.legs) {
		leg.valid = true;
		leg.shin = Quat::fromAxisAngle({1, 0, 0}, shinPitch);
	}
	BodyOutput out;
	for (int i = 0; i < frames; i++) out = solver.update(in);
	(void)d;
	return out;
}

static void testBodySolver() {
	BodyDimensions d = BodyDimensions::fromHeight(1.78, std::nullopt);
	BodySolver solver;
	solver.configure(d, Settings{}, Mode::Three);
	double eye = 0.936 * 1.78;
	BodyOutput out = runPose(solver, d, eye, 0, 120);
	CHECK(out.valid, "standing output");
	CHECK(std::fabs(out.feet[0].position.y - d.ankleHeight) < 0.03, "standing ankle height %.3f",
		  out.feet[0].position.y);
	CHECK(out.kneeAngles[0] < 15, "standing knee %.1f", out.kneeAngles[0]);
	CHECK(std::fabs(out.waist.position.y - (d.ankleHeight + d.shin + d.thigh)) < 0.05,
		  "standing hip %.3f", out.waist.position.y);
	std::printf("standing: ankle y %.3f, knee %.1f deg, hip y %.3f\n", out.feet[0].position.y,
				out.kneeAngles[0], out.waist.position.y);

	// Sitting on a chair: head ~0.45 m lower, shins vertical, feet on the floor
	BodySolver sit;
	sit.configure(d, Settings{}, Mode::Three);
	runPose(sit, d, eye, 0, 30);
	out = runPose(sit, d, eye - 0.45, 0, 400);
	CHECK(std::fabs(out.kneeAngles[0] - 90) < 20, "sitting knee %.1f deg", out.kneeAngles[0]);
	CHECK(out.feet[0].position.y < d.ankleHeight + 0.05, "sitting ankle height %.3f",
		  out.feet[0].position.y);
	std::printf("sitting: knee %.1f deg, ankle y %.3f, hip y %.3f\n", out.kneeAngles[0],
				out.feet[0].position.y, out.waist.position.y);

	// Ankles-only mode standing
	BodySolver ankles;
	ankles.configure(d, Settings{}, Mode::Ankles);
	out = runPose(ankles, d, eye, 0, 120);
	CHECK(out.valid && std::fabs(out.feet[1].position.y - d.ankleHeight) < 0.03,
		  "ankles-only standing ankle %.3f", out.feet[1].position.y);
}

int main() {
	testProtocol();
	testMagCalibration();
	testHeadingFilter();
	testMounting();
	testBodySolver();
	if (g_failures) {
		std::printf("%d check(s) failed\n", g_failures);
		return 1;
	}
	std::printf("all tests passed\n");
	return 0;
}
