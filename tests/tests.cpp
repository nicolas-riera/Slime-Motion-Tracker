// Core unit tests on synthetic data. Run: build\Release\smt_tests.exe
#include <cstdio>
#include <cstring>
#include <random>

#include "battery_estimator.h"
#include "smt/body_solver.h"
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

static void testPoseFusion() {
	// Standing: the solver alone. Crouched: mostly the model, within the leg's reach and
	// above the floor. A model foot jumping while the solver's stays: ignored
	BodyOutput out;
	out.valid = true;
	out.waist.position = {0, 0.9, 0};
	out.feet[0].position = {0.1, 0.07, 0.0};
	out.feet[1].position = {-0.1, 0.07, 0.0};
	PoseEstimate e;
	e.valid = true;
	e.warmup = 5;
	e.ankle[0] = {0.2, 0.0, 0.1};
	e.ankle[1] = {-0.1, 0.07, 0.0};
	Pose hmd;
	hmd.position = {0, 1.6, 0};
	PoseFuser fuser;
	auto run = [&](double seconds) {
		BodyOutput o = out;
		for (double t = 0; t < seconds; t += 1.0 / 90) {
			o = out;
			fuser.update(e, hmd, 0.0, 1.6, 0.95, 1.0 / 90, o);
		}
		return o;
	};
	BodyOutput a = run(2);
	CHECK(std::fabs(a.feet[0].position.x - 0.1) < 1e-9, "standing: solver only");
	hmd.position.y = 0.6 * 1.6;  // kneeling
	BodyOutput b = run(3);
	PoseFusionSettings fs;
	CHECK(std::fabs(b.feet[0].position.x - (0.1 + fs.crouchWeight * 0.1)) < 1e-3, "kneeling: model weight %.3f",
		  (b.feet[0].position.x - 0.1) / 0.1);
	CHECK(b.feet[0].position.y > 0.07 + fs.crouchWeight * (fs.minAnkleHeight - 0.07) - 1e-3,
		  "model ankle clamped above the floor");
	// Out of reach: brought back to the leg's length
	e.ankle[0] = {0.2, 0.07, 1.5};
	BodyOutput r = run(3);
	CHECK((r.feet[0].position - out.waist.position).norm() < 0.95 + 1e-6, "model ankle within the leg's reach");
	// The model foot jumps 70 cm in 0.4 s while the solver's does not move
	e.ankle[0] = {0.2, 0.07, 0.1};
	run(3);
	BodyOutput g = out;
	for (int k = 0; k < 36; k++) {
		e.ankle[0] = {0.2 + 0.7 * (k + 1) / 36.0, 0.07, 0.1};
		g = out;
		fuser.update(e, hmd, 0.0, 1.6, 0.95, 1.0 / 90, g);
	}
	CHECK(g.feet[0].position.x < 0.1 + 0.15, "jumping model foot ignored (x %.2f)", g.feet[0].position.x);
	e.warmup = 0.2;
	BodyOutput c = out;
	CHECK(fuser.update(e, hmd, 0.0, 1.6, 0.95, 1.0 / 90, c) == 0, "no fusion before the warm-up");
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

// Heading spread of a calibration over random orientations (deg): 0 for a perfect one
static double headingSpread(const MagCalibration& c, const SyntheticMag& mag, std::mt19937& rng) {
	double ref = 0, maxErr = 0;
	for (int i = 0; i < 500; i++) {
		Quat q = randomQuat(rng);
		double h = headingOf(q.rotate(c.apply(mag.raw(q, rng))));
		if (i == 0) ref = h;
		maxErr = std::max(maxErr, std::fabs(rad2deg(wrapAngle(h - ref))));
	}
	return maxErr;
}

static void testMagAutoCalibration() {
	// Automatic calibration while the device yaw drifts by 1 deg/min
	std::mt19937 rng(5);
	SyntheticMag mag = makeMag();
	const double dt = 0.05;
	auto tilted = [&](double maxTilt) {
		std::uniform_real_distribution<double> u(-1, 1);
		return Quat::yaw(u(rng) * kPi) * Quat::fromAxisAngle(Vec3{u(rng), 0, u(rng)}.normalized(), maxTilt * std::fabs(u(rng)));
	};
	auto collect = [&](MagCalibrator& cal, double from, double to, double maxTilt) {
		Quat q = tilted(maxTilt);
		for (double t = from; t < to; t += dt) {
			if (std::fmod(t, 2.0) < dt) q = tilted(maxTilt);  // a new pose every 2 s
			cal.addSample(Quat::yaw(deg2rad(t / 60.0)) * q, mag.raw(q, rng), 0.5, t);
		}
	};

	// A slightly wrong calibration (another hard and soft iron) whose north earlier
	// replacements set (offset 0.4 rad): replaced by the fit of an hour of varied poses,
	// the north kept
	SyntheticMag other = mag;
	other.softIron.m[0] *= 1.12;
	other.hardIron = other.hardIron + Vec3{150, -100, 80};
	MagCalibrator guided;
	for (int i = 0; i < 3000; i++) {
		Quat q = randomQuat(rng);
		guided.addSample(q, other.raw(q, rng), 0.5, i * 0.05);
	}
	auto start = guided.solve();
	CHECK(start.has_value(), "starting calibration");
	if (!start) return;
	start->headingOffset = 0.4;
	MagCalibrator varied;
	collect(varied, 0, 3600, kPi);
	auto rep = varied.replacement(*start);
	CHECK(rep.has_value(), "better fit not taken");
	if (!rep) return;
	double sn = 0, cs = 0;
	for (int i = 0; i < 500; i++) {
		Quat r = randomQuat(rng);
		auto raw = mag.raw(r, rng);
		double d = headingOf(r.rotate(start->apply(raw))) + start->headingOffset - headingOf(r.rotate(rep->apply(raw)))
				 - rep->headingOffset;
		sn += std::sin(d);
		cs += std::cos(d);
	}
	const double northMove = rad2deg(std::atan2(sn, cs)), spread = headingSpread(*rep, mag, rng);
	CHECK(std::fabs(northMove) < 1.5 && spread < 3.0, "replacement: north moved by %.2f deg, spread %.2f deg",
		  northMove, spread);

	// A good calibration, the trackers strapped on (2 min of any orientation) then an hour
	// standing (tilts below 30 deg): the rare orientations' samples are an hour of drift
	// older than the rest, the fit bends. It must not replace the good one.
	MagCalibrator good;
	collect(good, 0, 600, kPi);
	auto exact = good.solve();
	CHECK(exact.has_value(), "reference calibration");
	if (!exact) return;
	std::printf("mag auto calibration: north moved %.2f deg at a replacement\n", northMove);
	for (double minutes : {5.0, 10.0, 15.0, 20.0, 30.0}) {
		MagCalibrator worn;
		collect(worn, 0, 120, kPi);
		collect(worn, 120, 120 + minutes * 60, deg2rad(30));
		auto bent = worn.solve();
		auto spurious = worn.replacement(*exact);
		const double bentSpread = bent ? headingSpread(*bent, mag, rng) : -1.0;
		const bool oldRule = bent && bent->residual < 0.065 && bent->residual < 0.8 * worn.residualOf(*exact).value_or(1);
		CHECK(!spurious.has_value() || bentSpread < 4.0,
			  "%.0f min: a fit bent by the drift replaced a good calibration (spread %.2f vs %.2f deg)", minutes,
			  bentSpread, headingSpread(*exact, mag, rng));
		std::printf("  %2.0f min standing: fit spread %5.2f deg, residual %.3f vs %.3f, consistency %.3f vs %.3f: %s (old rule: %s)\n",
					minutes, bentSpread, bent ? bent->residual : -1.0, worn.residualOf(*exact).value_or(-1),
					bent ? worn.consistency(*bent).value_or(-1) : -1.0, worn.consistency(*exact).value_or(-1),
					spurious ? "taken" : "rejected", oldRule ? "taken" : "rejected");
	}
}

static void testHeadingReconnect() {
	std::mt19937 rng(6);
	SyntheticMag mag = makeMag();
	MagCalibrator cal;
	for (int i = 0; i < 4000; i++) {
		Quat q = randomQuat(rng);
		cal.addSample(q, mag.raw(q, rng), 0.5, i * 0.05);
	}
	auto c = cal.solve();
	if (!c) return;
	HeadingFilter hf;
	hf.setCalibration(*c);
	HeadingFilterSettings s;
	const Quat drift = Quat::yaw(deg2rad(20)), truth = Quat::fromAxisAngle({1, 0, 0}, 0.3);
	const double dt = 0.02;
	for (int i = 0; i < 1500; i++) hf.update(drift * truth, mag.raw(truth, rng), 0.0, dt, s);
	const double h0 = hf.heading();
	const Quat corrected = Quat::yaw(-h0) * drift * truth;

	// WiFi drop: the device fusion kept running, the feet are where the field is 15 deg off.
	// The heading stays (re-acquiring took the local field)
	HeadingFilter wifi = hf;
	wifi.reconnected();
	CHECK(!wifi.hasHeading(), "heading used before the reconnection check");
	const Quat local = Quat::yaw(deg2rad(15));
	for (int i = 0; i < 100; i++) wifi.update(drift * truth, mag.raw(local * truth, rng), 0.0, dt, s);
	CHECK(wifi.hasHeading() && std::fabs(rad2deg(wrapAngle(wifi.heading() - h0))) < 0.5,
		  "heading after a WiFi drop moved by %.2f deg", rad2deg(wrapAngle(wifi.heading() - h0)));

	// Reboot: the device yaw restarted 120 deg away. The mag wins
	HeadingFilter boot = hf;
	boot.reconnected();
	const Quat rebooted = Quat::yaw(deg2rad(120)) * drift;
	for (int i = 0; i < 50; i++) boot.update(rebooted * truth, mag.raw(truth, rng), 0.0, dt, s);
	const double err = angleBetween(Quat::yaw(-boot.heading()) * rebooted * truth, corrected);
	CHECK(boot.hasHeading() && err < 1.5, "heading after a reboot off by %.2f deg", err);
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

// Inputs of a body whose hip joint center is at hip with the trunk along pelvis (the
// head aligned with it, the legs stretched along it), as the solver's head chain sees
// it; accel: linear acceleration of the whole body; head: head rotation in the trunk frame
static BodyInput bodyInput(const BodyDimensions& d, const Vec3& hip, const Quat& pelvis, const Vec3& accel,
						   const Quat& head = Quat{}) {
	BodyInput in;
	in.dt = 1.0 / 90;
	in.hmdValid = true;
	in.hmd.rotation = pelvis * head;
	in.hmd.position = hip + pelvis.rotate({0, d.spine + d.neckDown, 0}) + in.hmd.rotation.rotate({0, d.headDown, -d.headBack});
	in.floorY = 0;
	in.hasWaist = true;
	in.waist = pelvis;
	in.waistAccel = accel;
	for (auto& leg : in.legs) {
		leg.valid = true;
		leg.shin = pelvis;
		leg.accelWorld = accel;
	}
	return in;
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

	// Lying down across the bed from its edge, then scooting 25 cm sideways (toward the
	// middle): the hip moves with the body, the seat anchor must not keep it at the edge
	{
		BodySolver bed;
		bed.configure(d, Settings{}, Mode::Three);
		const Vec3 seat{0, 0.55, 0};
		for (int f = 0; f < 60; f++) out = bed.update(bodyInput(d, {0, d.standingHipHeight(), 0}, Quat{}, {}));
		for (int f = 0; f < 400; f++) out = bed.update(bodyInput(d, seat, Quat{}, {}));
		for (int f = 0; f < 135; f++) {
			const Quat back = Quat::fromAxisAngle({1, 0, 0}, deg2rad(90.0 * f / 134));
			out = bed.update(bodyInput(d, seat, back, {}));
		}
		const Quat lying = Quat::fromAxisAngle({1, 0, 0}, deg2rad(90));
		for (int f = 0; f < 270; f++) out = bed.update(bodyInput(d, seat, lying, {}));
		const double x0 = out.waist.position.x;
		for (int f = 0; f < 63; f++) {
			const double t = f / 63.0;
			const double x = t < 0.5 ? 0.5 * t * t : 0.25 - 0.5 * (1 - t) * (1 - t);
			out = bed.update(bodyInput(d, seat + Vec3{x, 0, 0}, lying, {t < 0.5 ? 2.04 : -2.04, 0, 0}));
		}
		for (int f = 0; f < 90; f++) out = bed.update(bodyInput(d, seat + Vec3{0.25, 0, 0}, lying, {}));
		const double moved = out.waist.position.x - x0;
		CHECK(moved > 0.17, "lying scoot: hip moved %.3f of 0.25 m", moved);
		std::printf("lying scoot: hip moved %.3f of 0.25 m 1 s later\n", moved);
	}

	// Lying on the back with the waist tracker's heading 12 deg off: the hip stays in line
	// with the body (the head's up axis gives the trunk heading); turning the head on the
	// pillow does not move it
	{
		BodySolver lie;
		lie.configure(d, Settings{}, Mode::Three);
		const Vec3 hip{0, 0.55, 0};
		const Quat lying = Quat::fromAxisAngle({1, 0, 0}, deg2rad(90));
		for (int f = 0; f < 60; f++) out = lie.update(bodyInput(d, {0, d.standingHipHeight(), 0}, Quat{}, {}));
		for (int f = 0; f < 400; f++) out = lie.update(bodyInput(d, hip, Quat{}, {}));
		for (int f = 0; f < 135; f++)
			out = lie.update(bodyInput(d, hip, Quat::fromAxisAngle({1, 0, 0}, deg2rad(90.0 * f / 134)), {}));
		BodyInput in = bodyInput(d, hip, lying, {});
		in.waist = Quat::yaw(deg2rad(12)) * lying;
		for (int f = 0; f < 900; f++) out = lie.update(in);
		const double side = std::fabs(out.waist.position.x - hip.x);
		CHECK(side < 0.06, "lying with a 12 deg waist heading error: hip %.3f m to the side", side);
		BodyInput turned = bodyInput(d, hip, lying, {}, Quat::yaw(deg2rad(60)));
		turned.waist = in.waist;
		BodyOutput t = out;
		for (int f = 0; f < 180; f++) t = lie.update(turned);
		const double moved = (t.waist.position - out.waist.position).norm();
		CHECK(moved < 0.03, "lying, head turned 60 deg: hip moved %.3f", moved);
		// Knees up (shins vertical), the feet slide toward the hip on the bed: they do not
		// drop to the floor below it (anchors from standing still pull them part way down)
		BodyInput knees = turned;
		for (auto& leg : knees.legs) leg.shin = Quat{};
		BodyOutput k = t;
		for (int f = 0; f < 270; f++) k = lie.update(knees);
		CHECK(k.feet[0].position.y > 0.18 && k.feet[1].position.y > 0.18, "lying knees up: feet at %.3f %.3f",
			  k.feet[0].position.y, k.feet[1].position.y);
		std::printf("lying, waist heading 12 deg off: hip %.3f m to the side; head turned: hip moved %.3f\n", side,
					moved);
	}

	// Knee raised and held up while the leg keeps moving (the shin swings, never still):
	// the foot must stay up instead of dropping under the hip onto the floor
	{
		BodySolver raise;
		raise.configure(d, Settings{}, Mode::Three);
		runPose(raise, d, eye, 0, 120);
		BodyInput in;
		in.dt = 1.0 / 90;
		in.hmdValid = true;
		in.hmd.position = {0, eye, 0};
		in.hasWaist = true;
		for (auto& leg : in.legs) leg.valid = true;
		const Vec3 accel{0, 4.8, -6.4};
		double minY = 10;
		for (int f = 0; f < 90 * 6; f++) {
			const double t = f * in.dt;
			if (t < 0.5) {
				in.legs[1].accelWorld = t < 0.25 ? accel : accel * -1.0;
			} else {
				const double w = 2 * kPi * 1.5;
				in.legs[1].shin = Quat::fromAxisAngle({1, 0, 0}, 0.4 * std::sin(w * (t - 0.5)));
				// Small circles (1.5 m/s^2, ~2 cm): never still
				in.legs[1].accelWorld = Vec3{1.5 * std::cos(w * (t - 0.5)), 0, 1.5 * std::sin(w * (t - 0.5))};
				in.legs[1].angularSpeed = 1.0;
			}
			out = raise.update(in);
			if (t > 1.0) minY = std::min(minY, out.feet[1].position.y);
		}
		CHECK(minY > d.ankleHeight + 0.15, "raised moving leg dropped to %.3f", minY);
		// Then put down (shin vertical again, the same displacement back) and still
		in.legs[1].shin = Quat{};
		in.legs[1].angularSpeed = 0;
		for (int f = 0; f < 90 * 2; f++) {
			const double t = f * in.dt;
			in.legs[1].accelWorld = t < 0.25 ? accel * -1.0 : t < 0.5 ? accel : Vec3{};
			out = raise.update(in);
		}
		CHECK(out.feet[1].position.y < d.ankleHeight + 0.05, "lowered leg stayed up at %.3f", out.feet[1].position.y);
		CHECK(std::fabs(out.feet[0].position.y - d.ankleHeight) < 0.03, "standing foot at %.3f",
			  out.feet[0].position.y);
		std::printf("raised moving leg: lowest foot y %.3f over 5 s\n", minY);
	}

	// Slouched on a couch (trunk 45 deg back), shins inclined 55 deg with the feet on the
	// floor: the feet stay on the floor, not lifted to the seat
	{
		BodySolver couch;
		couch.configure(d, Settings{}, Mode::Three);
		for (int f = 0; f < 60; f++) out = couch.update(bodyInput(d, {0, d.standingHipHeight(), 0}, Quat{}, {}));
		const Quat recline = Quat::fromAxisAngle({1, 0, 0}, deg2rad(45));
		const Quat shin = Quat::fromAxisAngle({1, 0, 0}, deg2rad(-55));
		BodyInput in = bodyInput(d, {0, 0.5, 0}, recline, {});
		for (auto& leg : in.legs) leg.shin = shin;
		for (int f = 0; f < 450; f++) out = couch.update(in);
		const double y = std::max(out.feet[0].position.y, out.feet[1].position.y);
		CHECK(y < d.ankleHeight + 0.05, "couch feet lifted to %.3f", y);
		std::printf("couch: feet y %.3f %.3f\n", out.feet[0].position.y, out.feet[1].position.y);
	}

	// Full reset: the solver restarts while the user still holds the ski pose (knees
	// bent, shins and trunk leaning forward). The feet start below the hip, not 75 cm
	// behind along straight legs
	{
		BodySolver ski;
		ski.configure(d, Settings{}, Mode::Three);
		BodyInput in = bodyInput(d, {0, 0.8, 0}, Quat::fromAxisAngle({1, 0, 0}, deg2rad(-30)), {});
		for (auto& leg : in.legs) leg.shin = Quat::fromAxisAngle({1, 0, 0}, deg2rad(-25));
		for (int f = 0; f < 30; f++) out = ski.update(in);
		double behind = 0;
		for (int i = 0; i < 2; i++) behind = std::max(behind, out.feet[i].position.z - out.waist.position.z);
		CHECK(behind < 0.2, "ski pose restart: feet %.3f m behind the hip", behind);
		std::printf("ski pose restart: feet %.3f m behind the hip\n", behind);
	}

	// Kneeling on a chair: head high (standing for the solver), knees on the seat, shins
	// flat behind. The torso does not stretch: the hip stays near the head chain
	{
		BodySolver kneel;
		kneel.configure(d, Settings{}, Mode::Three);
		for (int f = 0; f < 60; f++) out = kneel.update(bodyInput(d, {0, d.standingHipHeight(), 0}, Quat{}, {}));
		const Vec3 hip{0, 0.88, 0};
		BodyInput in = bodyInput(d, hip, Quat{}, {});
		for (auto& leg : in.legs) leg.shin = Quat::fromAxisAngle({1, 0, 0}, deg2rad(-90));
		for (int f = 0; f < 450; f++) out = kneel.update(in);
		CHECK(out.waist.position.y > hip.y - 0.07, "kneeling on a chair: hip %.3f below the head chain",
			  hip.y - out.waist.position.y);
		std::printf("kneeling on a chair: hip %.3f m below the head chain\n", hip.y - out.waist.position.y);
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
	for (int i = 0; i < 300; i++, t += 1) e.add(mac, 0.0, 0.0, t);
	for (int i = 0; i < 20 * 60; i++, t += 1) e.add(mac, 0.41 - (t - 1300) * 1e-6, 3.66, t);
	CHECK(e.estimate(mac).state != BatteryEstimator::State::Charging, "battery: 0 -> 41 %% jump taken as charging");
	CHECK(e.estimate(mac).state == BatteryEstimator::State::Ok, "battery: no estimate after 20 min of discharge");
	// A sustained rise (1 %/min for 5 min) is charging
	const double base = 0.40;
	for (int i = 0; i < 5 * 60; i++, t += 1) e.add(mac, base + i / 6000.0, 3.9, t);
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
	testMagAutoCalibration();
	testHeadingReconnect();
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
