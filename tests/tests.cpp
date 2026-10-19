// Core unit tests on synthetic data. Run: build\Release\smt_tests.exe
#include <cstdio>
#include <cstring>
#include <random>

#include "battery_estimator.h"
#include "smt/body_solver.h"
#include "smt/dongle_bridge.h"
#include "smt/pose_model.h"
#include "smt/mag.h"
#include "smt/mounting_calibration.h"
#include "smt/protocol.h"

using namespace smt;

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

static void testDongleFrames() {
	using namespace smt::dongle;
	Frame a;
	a.type = kData;
	const uint8_t mac[6] = {0x2C, 0xF4, 0x32, 0x7B, 0xFF, 0xD1};
	std::memcpy(a.mac, mac, 6);
	a.payload = {0, 0, 0, 3, 'H', 'e', 'y', 0xA5, 0x5A};
	Frame b;
	b.type = kInfo;
	b.payload = {'S', 'L', 'D', 'O', 'N', 'G', 'L', 'E', 1, 6};
	std::vector<uint8_t> stream = {0x13, 0xA5, 0x00};  // boot noise
	auto ea = encode(a), eb = encode(b);
	stream.insert(stream.end(), ea.begin(), ea.end());
	std::vector<uint8_t> bad = eb;
	bad.back() ^= 0xFF;  // corrupted checksum: dropped
	stream.insert(stream.end(), bad.begin(), bad.end());
	stream.insert(stream.end(), eb.begin(), eb.end());
	Decoder d;
	std::vector<Frame> got;
	for (size_t i = 0; i < stream.size(); i += 5) {  // arbitrary chunking
		auto f = d.feed(stream.data() + i, std::min<size_t>(5, stream.size() - i));
		got.insert(got.end(), f.begin(), f.end());
	}
	CHECK(got.size() == 2, "dongle frames decoded: %zu", got.size());
	if (got.size() == 2) {
		CHECK(got[0].payload == a.payload && std::memcmp(got[0].mac, mac, 6) == 0, "dongle data frame content");
		CHECK(got[1].type == kInfo && got[1].payload == b.payload, "dongle info frame content");
	}
	CHECK(macString(mac) == "2C:F4:32:7B:FF:D1", "mac string");
}

static void testPoseFusion() {
	// Head high: the model ankles win, never below the floor margin; head low: the solver
	BodyOutput out;
	out.valid = true;
	out.feet[0].position = {0.1, 0.10, 0.0};
	out.feet[1].position = {-0.1, 0.10, 0.0};
	PoseEstimate e;
	e.valid = true;
	e.warmup = 5;
	e.ankle[0] = {0.2, 0.0, 0.1};
	e.ankle[1] = {-0.2, 0.3, 0.1};
	Pose hmd;
	hmd.position = {0, 1.7, 0};
	BodyOutput a = out;
	double w = fusePose(e, hmd, 0.0, 1.6, a);
	CHECK(std::fabs(w - 1) < 1e-9, "fusion weight standing %.3f", w);
	CHECK(std::fabs(a.feet[0].position.y - 0.10) < 1e-9 && std::fabs(a.feet[0].position.x - 0.2) < 1e-9,
		  "model ankle clamped above the floor");
	CHECK(std::fabs(a.feet[1].position.y - 0.3) < 1e-9, "raised model ankle kept");
	hmd.position.y = 0.8;
	BodyOutput b = out;
	CHECK(fusePose(e, hmd, 0.0, 1.6, b) == 0 && b.feet[0].position.x == 0.1, "solver only with the head low");
	// Ankles only, the model already leads at 60 % of the eye height
	hmd.position.y = 0.6 * 1.6;
	BodyOutput b2 = out;
	CHECK(std::fabs(fusePose(e, hmd, 0.0, 1.6, b2, PoseFusionSettings::forWaist(false)) - 1) < 1e-9,
		  "ankles only: model weight at 60 %% of the eye height");
	e.warmup = 0.2;
	hmd.position.y = 1.6;
	BodyOutput c = out;
	CHECK(fusePose(e, hmd, 0.0, 1.6, c) == 0, "no fusion before the warm-up");
	// Wrist calibration round trip through the config
	Config cfg;
	cfg.wristCalibration = std::array<Quat, 2>{Quat::yaw(0.3), Quat::fromAxisAngle({1, 0, 0}, 0.2)};
	Config back;
	back.fromJson(cfg.toJson());
	CHECK(back.wristCalibration && std::fabs((*back.wristCalibration)[0].dot(Quat::yaw(0.3)) - 1) < 1e-9,
		  "wrist calibration saved");
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
	// Visualization accessors (overlay calibration page)
	{
		auto bins = cal.binCounts();
		size_t total = 0;
		int full = 0;
		for (int b : bins) {
			total += size_t(b);
			if (b >= MagCalibrator::binTarget()) full++;
		}
		CHECK(total == cal.sampleCount() && cal.rawSamples().size() == total, "mag view sample count");
		CHECK(std::fabs(cal.coverage() - std::min(1.0, full / 18.0)) < 1e-9, "mag view bin target");
		for (int i = 0; i < MagCalibrator::kBins; i++)
			CHECK(std::fabs(MagCalibrator::binDirection(i).norm() - 1) < 1e-9, "bin direction %d not unit", i);
	}
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

	// The tracker's environment changes (another hard iron): on the new samples the
	// old calibration fits badly, a new solve fits well
	SyntheticMag moved = mag;
	moved.hardIron = mag.hardIron + Vec3{600, -400, 300};
	MagCalibrator later;
	for (int i = 0; i < 5000 && later.coverage() < 1.0; i++) {
		Quat q = randomQuat(rng);
		later.addSample(q, moved.raw(q, rng), 0.5, i * 0.05);
	}
	auto oldOnNew = later.residualOf(*res);
	auto refit = later.solve();
	CHECK(oldOnNew && refit && refit->residual < 0.5 * *oldOnNew,
		  "changed field not detected: old %.3f, new %.3f", oldOnNew.value_or(-1), refit ? refit->residual : -1.0);
	auto sameOnSame = later.residualOf(*refit);
	CHECK(sameOnSame && std::fabs(*sameOnSame - refit->residual) < 0.01, "residualOf mismatch");

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

	// Standing slouched: head 5 cm below the calibrated height, shins vertical. The
	// hip rises a little instead of the knees bending (they used to bend ~25 deg)
	BodySolver slouch;
	slouch.configure(d, Settings{}, Mode::Three);
	runPose(slouch, d, eye, 0, 30);
	out = runPose(slouch, d, eye - 0.05, 0, 200);
	CHECK(out.kneeAngles[0] < 10 && out.kneeAngles[1] < 10, "slouched knees %.1f %.1f deg", out.kneeAngles[0],
		  out.kneeAngles[1]);
	CHECK(std::fabs(out.feet[0].position.y - d.ankleHeight) < 0.03, "slouched ankle height %.3f",
		  out.feet[0].position.y);

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

	// Knee lift held still (or a foot put on a bed): the shin stays vertical, only
	// the accelerometer sees the foot rise 30 cm (and move 40 cm forward with the
	// knee, thigh at ~73 deg). Once still, it must stay up
	{
		BodySolver lift;
		lift.configure(d, Settings{}, Mode::Three);
		runPose(lift, d, eye, 0, 120);
		BodyInput in;
		in.dt = 1.0 / 90;
		in.hmdValid = true;
		in.hmd.position = {0, eye, 0};
		in.hasWaist = true;
		for (auto& leg : in.legs) leg.valid = true;
		const Vec3 accel{0, 4.8, -6.4};  // displacement accel * half^2 = (0, 0.30, -0.40) m
		const double half = 0.25;
		for (int f = 0; f < 90 * 5; f++) {
			double t = f * in.dt;
			in.legs[1].accelWorld = t < half ? accel : t < 2 * half ? accel * -1.0 : Vec3{};
			out = lift.update(in);
			if (f == 90) {
				CHECK(out.feet[1].position.y > d.ankleHeight + 0.2, "lifted foot at %.3f", out.feet[1].position.y);
			}
		}
		CHECK(out.feet[1].position.y > d.ankleHeight + 0.2, "held foot sank to %.3f", out.feet[1].position.y);
		CHECK(std::fabs(out.feet[0].position.y - d.ankleHeight) < 0.03, "standing foot at %.3f",
			  out.feet[0].position.y);
		std::printf("held knee lift: foot y %.3f after 5 s, knee %.1f deg\n", out.feet[1].position.y,
					out.kneeAngles[1]);
	}

	// Seated, leaning forward: the head moves 15 cm forward, the hip stays on the seat
	{
		BodySolver seat;
		seat.configure(d, Settings{}, Mode::Three);
		runPose(seat, d, eye, 0, 30);
		BodyOutput sat = runPose(seat, d, eye - 0.45, 0, 400);
		BodyInput in;
		in.dt = 1.0 / 90;
		in.hmdValid = true;
		in.hmd.position = {0, eye - 0.47, -0.15};
		in.hasWaist = true;
		for (auto& leg : in.legs) leg.valid = true;
		for (int f = 0; f < 90; f++) out = seat.update(in);
		double moved = (out.waist.position - sat.waist.position).norm();
		CHECK(moved < 0.05, "seated lean moved the hip by %.3f", moved);
		std::printf("seated lean: hip moved %.3f m\n", moved);
	}

	// Ankles-only mode standing
	BodySolver ankles;
	ankles.configure(d, Settings{}, Mode::Ankles);
	out = runPose(ankles, d, eye, 0, 120);
	CHECK(out.valid && std::fabs(out.feet[1].position.y - d.ankleHeight) < 0.03,
		  "ankles-only standing ankle %.3f", out.feet[1].position.y);
}

// Overlay battery estimator: USB power readings and single jumps are not charging
static void testBatteryEstimator() {
	using smt::ui::BatteryEstimator;
	const std::string mac = "AA:BB:CC:DD:EE:FF";
	double t = 1000;
	BatteryEstimator e;
	// USB powered: reads 0 % (ignored), then on battery at 41 %, slowly discharging
	for (int i = 0; i < 300; i++, t += 1) e.add(mac, "espnow", 0.0, 0.0, t);
	for (int i = 0; i < 20 * 60; i++, t += 1) e.add(mac, "espnow", 0.41 - (t - 1300) * 1e-6, 3.66, t);
	CHECK(e.estimate(mac).state != BatteryEstimator::State::Charging, "battery: 0 -> 41 %% jump taken as charging");
	CHECK(e.estimate(mac).state == BatteryEstimator::State::Ok, "battery: no estimate after 20 min of discharge");
	// A sustained rise (1 %/min for 5 min) is charging
	const double base = 0.40;
	for (int i = 0; i < 5 * 60; i++, t += 1) e.add(mac, "espnow", base + i / 6000.0, 3.9, t);
	CHECK(e.estimate(mac).state == BatteryEstimator::State::Charging, "battery: sustained rise not seen as charging");
	CHECK(!BatteryEstimator::plausible(0.0, 4.1) && !BatteryEstimator::plausible(0.5, 2.5) &&
			  BatteryEstimator::plausible(0.5, 0.0),
		  "battery: plausibility");
}

int main() {
	testBatteryEstimator();
	testProtocol();
	testMagCalibration();
	testHeadingFilter();
	testDongleFrames();
	testPoseFusion();
	testMounting();
	testBodySolver();
	if (g_failures) {
		std::printf("%d check(s) failed\n", g_failures);
		return 1;
	}
	std::printf("all tests passed\n");
	return 0;
}
