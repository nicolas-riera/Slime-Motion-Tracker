#include "slimelite/foot_model.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>

namespace slimelite {

namespace {

constexpr size_t kRing = 2048;  // frames of history (> 2 s at any engine rate)
constexpr int kFeaturesPerLag = 35;

// World vector into the heading frame: v_h = Ry(heading)^T v
Vec3 toHeading(const Vec3& v, double c, double s) { return {c * v.x - s * v.z, v.y, s * v.x + c * v.z}; }
Vec3 fromHeading(const Vec3& v, double c, double s) { return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z}; }

}  // namespace

bool FootModel::load(const std::string& path) {
	m_loaded = false;
	m_layers.clear();
	try {
		std::ifstream f(path);
		if (!f) {
			m_error = "cannot open " + path;
			return false;
		}
		nlohmann::json j = nlohmann::json::parse(f);
		if (j.value("format", "") != "slimelite_foot_model" || j.value("version", 0) != 1) {
			m_error = "not a foot model (version 1)";
			return false;
		}
		m_lags = j.at("lags").get<std::vector<double>>();
		m_accClip = j.at("accClip").get<double>();
		m_accTau = j.at("accTau").get<double>();
		m_mu = j.at("mu").get<std::vector<double>>();
		m_sd = j.at("sd").get<std::vector<double>>();
		m_ymu = j.at("ymu").get<std::vector<double>>();
		m_hetero = j.value("hetero", false);
		m_dr = j.value("dr", false);
		if (m_dr) {
			const auto& d = j.at("drParams");
			m_drStillGyro = d.at("stillGyro").get<double>();
			m_drStillAcc = d.at("stillAcc").get<double>();
			m_drZuptAfter = d.at("zuptAfter").get<double>();
			m_drLeak = d.at("leak").get<double>();
		}
		int n = static_cast<int>(m_lags.size()) * kFeaturesPerLag + (m_dr ? 4 : 0);
		if (static_cast<int>(m_mu.size()) != n || static_cast<int>(m_sd.size()) != n || m_ymu.size() != 6) {
			m_error = "inconsistent normalization sizes";
			return false;
		}
		for (const auto& jl : j.at("layers")) {
			Layer l;
			l.b = jl.at("b").get<std::vector<double>>();
			l.out = static_cast<int>(l.b.size());
			for (const auto& row : jl.at("W")) {
				auto r = row.get<std::vector<double>>();
				if (l.in == 0) l.in = static_cast<int>(r.size());
				if (static_cast<int>(r.size()) != l.in) {
					m_error = "ragged weight matrix";
					return false;
				}
				l.w.insert(l.w.end(), r.begin(), r.end());
			}
			if (static_cast<int>(l.w.size()) != l.in * l.out || l.in != n) {
				m_error = "layer size mismatch";
				return false;
			}
			n = l.out;
			m_layers.push_back(std::move(l));
		}
		if (m_layers.empty() || n != (m_hetero ? 12 : 6)) {
			m_error = "bad output size";
			return false;
		}
	} catch (const std::exception& e) {
		m_error = e.what();
		return false;
	}
	m_loaded = true;
	m_error.clear();
	reset();
	return true;
}

void FootModel::reset() {
	m_ring.assign(kRing, Frame{});
	m_head = 0;
	m_count = 0;
	m_accInit = false;
	m_lastTime = 0;
	m_drVel = {};
	m_drHeight = {};
	m_drStill = {};
}

const FootModel::Frame* FootModel::frameAt(double t) const {
	for (size_t k = 0; k < m_count; k++) {
		const Frame& f = m_ring[(m_head + kRing - 1 - k) % kRing];
		if (f.t <= t) return &f;
	}
	return nullptr;
}

FootEstimate FootModel::update(const BodyInput& in, double now) {
	FootEstimate est;
	if (!m_loaded) return est;

	// Accelerometer: clip per axis, causal low-pass with a dt-dependent gain,
	// every frame (an invalid leg reads zero, as in the training data)
	std::array<Vec3, 2> acc;
	for (int i = 0; i < 2; i++) {
		const Vec3& a = in.legs[i].accelWorld;
		acc[i] = {std::clamp(a.x, -m_accClip, m_accClip), std::clamp(a.y, -m_accClip, m_accClip),
				  std::clamp(a.z, -m_accClip, m_accClip)};
	}
	double dt = 0;
	if (!m_accInit) {
		m_accFiltered = acc;
		m_accInit = true;
	} else {
		dt = std::clamp(now - m_lastTime, 0.0, 0.1);
		double g = 1.0 - std::exp(-dt / m_accTau);
		for (int i = 0; i < 2; i++) m_accFiltered[i] = m_accFiltered[i] + (acc[i] - m_accFiltered[i]) * g;
	}
	m_lastTime = now;
	if (m_dr) {
		for (int i = 0; i < 2; i++) {
			const Vec3& a = m_accFiltered[i];
			if (in.legs[i].angularSpeed < m_drStillGyro && a.norm() < m_drStillAcc) m_drStill[i] += dt;
			else m_drStill[i] = 0;
			if (m_drStill[i] >= m_drZuptAfter) m_drVel[i] = 0;
			else m_drVel[i] += a.y * dt;
			m_drHeight[i] += m_drVel[i] * dt - m_drHeight[i] * dt / m_drLeak;
			m_drHeight[i] = std::clamp(m_drHeight[i], -1.0, 1.0);
		}
	}

	Frame& f = m_ring[m_head];
	f.t = now;
	f.valid = in.hmdValid && in.hasWaist && in.legs[0].valid && in.legs[1].valid;
	f.hmd = {in.hmd.position.x, in.hmd.position.y - in.floorY, in.hmd.position.z};
	const Quat* segs[4] = {&in.hmd.rotation, &in.waist, &in.legs[0].shin, &in.legs[1].shin};
	for (int s = 0; s < 4; s++) {
		f.axes[2 * s] = segs[s]->rotate({0, 1, 0});
		f.axes[2 * s + 1] = segs[s]->rotate({0, 0, 1});
	}
	f.accel = m_accFiltered;
	f.gyro = {in.legs[0].angularSpeed, in.legs[1].angularSpeed};
	const Frame now_ = f;
	m_head = (m_head + 1) % kRing;
	m_count = std::min(m_count + 1, kRing);
	if (!now_.valid) return est;

	// Heading of the pelvis from its left-right axis (bending forward or back
	// does not change it)
	Vec3 right = in.waist.rotate({1, 0, 0});
	double heading = std::atan2(-right.z, right.x);
	double c = std::cos(heading), s = std::sin(heading);

	size_t nIn = m_lags.size() * kFeaturesPerLag + (m_dr ? 4 : 0);
	m_features.resize(nIn);
	double* x = m_features.data();
	for (size_t li = 0; li < m_lags.size(); li++) {
		const Frame* p = m_lags[li] > 0 ? frameAt(now - m_lags[li] + 1e-9) : &now_;
		if (!p || !p->valid) return est;
		double* o = x + li * kFeaturesPerLag;
		auto put = [&o](const Vec3& v) {
			*o++ = v.x;
			*o++ = v.y;
			*o++ = v.z;
		};
		put(toHeading({p->hmd.x - now_.hmd.x, p->hmd.y, p->hmd.z - now_.hmd.z}, c, s));
		for (const Vec3& a : p->axes) put(toHeading(a, c, s));
		put(toHeading(p->accel[0], c, s));
		put(toHeading(p->accel[1], c, s));
		*o++ = p->gyro[0];
		*o++ = p->gyro[1];
	}
	if (m_dr) {
		double* o = x + m_lags.size() * kFeaturesPerLag;
		o[0] = m_drHeight[0];
		o[1] = m_drHeight[1];
		o[2] = std::min(m_drStill[0], 3.0);
		o[3] = std::min(m_drStill[1], 3.0);
	}

	// MLP: standardized input, ReLU hidden layers, linear output
	m_a.resize(nIn);
	for (size_t i = 0; i < nIn; i++) m_a[i] = (x[i] - m_mu[i]) / m_sd[i];
	for (size_t li = 0; li < m_layers.size(); li++) {
		const Layer& l = m_layers[li];
		m_b.assign(l.out, 0.0);
		for (int r = 0; r < l.out; r++) {
			const double* w = &l.w[static_cast<size_t>(r) * l.in];
			double acc0 = l.b[r];
			for (int k = 0; k < l.in; k++) acc0 += w[k] * m_a[k];
			m_b[r] = (li + 1 < m_layers.size()) ? std::max(acc0, 0.0) : acc0;
		}
		std::swap(m_a, m_b);
	}

	Vec3 origin{now_.hmd.x, in.floorY, now_.hmd.z};
	for (int leg = 0; leg < 2; leg++) {
		Vec3 h{m_a[3 * leg] + m_ymu[3 * leg], m_a[3 * leg + 1] + m_ymu[3 * leg + 1],
			   m_a[3 * leg + 2] + m_ymu[3 * leg + 2]};
		est.ankle[leg] = fromHeading(h, c, s) + origin;
		if (m_hetero) {
			auto sig = [&](int k) { return std::sqrt(2.0) * std::exp(std::clamp(m_a[6 + 3 * leg + k], -5.5, 1.0)); };
			est.sigma[leg] = {sig(0), sig(1), sig(2)};
		}
	}
	est.heading = heading;
	est.valid = true;
	return est;
}

}  // namespace slimelite
