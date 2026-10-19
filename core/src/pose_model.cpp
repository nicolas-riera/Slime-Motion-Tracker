#include "slimelite/pose_model.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace slimelite {

namespace {

constexpr int kTokens = 8;
constexpr int kFeatures = 135;
constexpr int kJoints = 22;

// Our frame (Y up, body facing -Z at identity) to the AMASS/SMPL one (Z up, rest
// pose facing +Z): R_amass = Rx90 R_ours Ry180, p_amass = Rx90 p_ours
const Mat3 kRx90 = [] {
	Mat3 m = Mat3::zero();
	m(0, 0) = 1;
	m(1, 2) = -1;
	m(2, 1) = 1;
	return m;
}();
const Mat3 kRy180 = [] {
	Mat3 m = Mat3::zero();
	m(0, 0) = -1;
	m(1, 1) = 1;
	m(2, 2) = -1;
	return m;
}();
const Vec3 kHeadOffset{0.0, -0.08, 0.10};  // HMD frame: SMPL head joint behind/below the HMD
const Vec3 kWristOffset{0.0, 0.0, 0.08};  // controller frame: wrist behind its origin

Mat3 toAmass(const Quat& q) { return kRx90 * q.toMatrix() * kRy180; }
Vec3 posToAmass(const Vec3& p, double floorY) { return kRx90 * Vec3{p.x, p.y - floorY, p.z}; }

Mat3 rotZ(double a) {
	Mat3 m;
	double c = std::cos(a), s = std::sin(a);
	m(0, 0) = c;
	m(0, 1) = -s;
	m(1, 0) = s;
	m(1, 1) = c;
	return m;
}

void putSixd(const Mat3& r, float* x) {
	for (int c = 0; c < 2; c++)
		for (int i = 0; i < 3; i++) *x++ = static_cast<float>(r(i, c));
}
void putVec(const Vec3& v, float* x) {
	x[0] = static_cast<float>(v.x);
	x[1] = static_cast<float>(v.y);
	x[2] = static_cast<float>(v.z);
}

Mat3 sixdToMat(const float* x) {
	Vec3 a{x[0], x[1], x[2]}, b{x[3], x[4], x[5]};
	a = a.normalized();
	b = (b - a * a.dot(b)).normalized();
	Vec3 c = a.cross(b);
	Mat3 m;
	for (int i = 0; i < 3; i++) {
		m(i, 0) = a[i];
		m(i, 1) = b[i];
		m(i, 2) = c[i];
	}
	return m;
}

float leaky(float v) { return v > 0 ? v : 0.01f * v; }
float sigmoid(float v) { return 1.0f / (1.0f + std::exp(-v)); }

void layerNorm(float* x, int n, const std::vector<float>& w, const std::vector<float>& b, float eps) {
	double mean = 0, var = 0;
	for (int i = 0; i < n; i++) mean += x[i];
	mean /= n;
	for (int i = 0; i < n; i++) var += (x[i] - mean) * (x[i] - mean);
	var /= n;
	float inv = static_cast<float>(1.0 / std::sqrt(var + eps));
	for (int i = 0; i < n; i++) x[i] = (x[i] - static_cast<float>(mean)) * inv * w[i] + b[i];
}

float halfToFloat(uint16_t h) {
	uint32_t s = (h >> 15) & 1, e = (h >> 10) & 0x1f, m = h & 0x3ff, f;
	if (e == 0) {
		if (m == 0) {
			f = s << 31;
		} else {  // subnormal
			e = 127 - 15 + 1;
			while (!(m & 0x400)) {
				m <<= 1;
				e--;
			}
			f = (s << 31) | (e << 23) | ((m & 0x3ff) << 13);
		}
	} else if (e == 31) {
		f = (s << 31) | 0x7f800000 | (m << 13);
	} else {
		f = (s << 31) | ((e - 15 + 127) << 23) | (m << 13);
	}
	float out;
	std::memcpy(&out, &f, 4);
	return out;
}

}  // namespace

namespace {

// y[t][o] = W[o] . x[t] + b[o] for NT inputs at once (weights read once)
template <int NT>
void rowsKernel(const uint16_t* w, int in, int out, const float* x, const float* bias, float* y) {
	for (int o = 0; o < out; o++) {
		const uint16_t* r = w + static_cast<size_t>(o) * in;
		float sum[NT] = {};
		int i = 0;
#if defined(__AVX2__)
		__m256 acc[NT];
		for (int t = 0; t < NT; t++) acc[t] = _mm256_setzero_ps();
		for (; i + 8 <= in; i += 8) {
			__m256 wv = _mm256_cvtph_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(r + i)));
			for (int t = 0; t < NT; t++) acc[t] = _mm256_fmadd_ps(wv, _mm256_loadu_ps(x + static_cast<size_t>(t) * in + i), acc[t]);
		}
		for (int t = 0; t < NT; t++) {
			__m128 h = _mm_add_ps(_mm256_castps256_ps128(acc[t]), _mm256_extractf128_ps(acc[t], 1));
			h = _mm_add_ps(h, _mm_movehl_ps(h, h));
			h = _mm_add_ss(h, _mm_shuffle_ps(h, h, 1));
			sum[t] = _mm_cvtss_f32(h);
		}
#endif
		for (; i < in; i++) {
			float wf = halfToFloat(r[i]);
			for (int t = 0; t < NT; t++) sum[t] += wf * x[static_cast<size_t>(t) * in + i];
		}
		for (int t = 0; t < NT; t++) y[static_cast<size_t>(t) * out + o] = sum[t] + bias[o];
	}
}

}  // namespace

void PoseModel::Linear::applyBatch(const float* x, int n, float* y) const {
	int t = 0;
	for (; t + kTokens <= n; t += kTokens)
		rowsKernel<kTokens>(w.data(), in, out, x + static_cast<size_t>(t) * in, b.data(), y + static_cast<size_t>(t) * out);
	for (; t < n; t++)
		rowsKernel<1>(w.data(), in, out, x + static_cast<size_t>(t) * in, b.data(), y + static_cast<size_t>(t) * out);
}

bool PoseModel::load(const std::string& path) {
	m_loaded = false;
	try {
		std::ifstream f(path, std::ios::binary);
		if (!f) {
			m_error = "cannot open " + path;
			return false;
		}
		char magic[4];
		uint32_t hlen = 0;
		f.read(magic, 4);
		f.read(reinterpret_cast<char*>(&hlen), 4);
		if (!f || std::memcmp(magic, "SLPM", 4) != 0) {
			m_error = "not a pose model";
			return false;
		}
		std::string hs(hlen, '\0');
		f.read(hs.data(), hlen);
		std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		nlohmann::json j = nlohmann::json::parse(hs);
		if (j.value("format", "") != "slimelite_pose_model" || j.value("version", 0) != 1) {
			m_error = "unsupported pose model format";
			return false;
		}
		m_name = j.value("name", "");
		const auto& a = j.at("arch");
		m_hidden = a.at("hidden");
		m_heads = a.at("heads");
		m_layers = a.at("layers");
		m_blocks = a.at("blocks");
		m_ffn = a.at("ffn");
		m_hz = j.value("hz", 60.0);
		m_modelHeight = j.at("model_height");
		auto rest = j.at("rest_offsets");
		auto parents = j.at("parents");
		for (int k = 0; k < kJoints; k++) {
			m_rest[k] = {rest[k][0], rest[k][1], rest[k][2]};
			m_parents[k] = parents[k];
		}
		std::map<std::string, std::pair<std::vector<int>, size_t>> index;
		for (const auto& t : j.at("tensors")) index[t.at("name")] = {t.at("shape").get<std::vector<int>>(), t.at("offset")};
		auto tensor = [&](const std::string& name, size_t expect) {
			auto it = index.find(name);
			if (it == index.end()) throw std::runtime_error("missing tensor " + name);
			size_t n = 1;
			for (int d : it->second.first) n *= d;
			if (n != expect) throw std::runtime_error("bad size for " + name);
			if (it->second.second + 2 * n > blob.size()) throw std::runtime_error("truncated " + name);
			std::vector<float> v(n);
			const char* p = blob.data() + it->second.second;
			for (size_t i = 0; i < n; i++) {
				uint16_t h;
				std::memcpy(&h, p + 2 * i, 2);
				v[i] = halfToFloat(h);
			}
			return v;
		};
		auto halfTensor = [&](const std::string& name, size_t expect) {
			auto it = index.find(name);
			if (it == index.end()) throw std::runtime_error("missing tensor " + name);
			size_t n = 1;
			for (int d : it->second.first) n *= d;
			if (n != expect) throw std::runtime_error("bad size for " + name);
			if (it->second.second + 2 * n > blob.size()) throw std::runtime_error("truncated " + name);
			std::vector<uint16_t> v(n);
			std::memcpy(v.data(), blob.data() + it->second.second, 2 * n);
			return v;
		};
		auto linear = [&](const std::string& name, int in, int out) {
			Linear l;
			l.in = in;
			l.out = out;
			l.w = halfTensor(name + ".weight", static_cast<size_t>(in) * out);
			l.b = tensor(name + ".bias", out);
			return l;
		};
		const int H = m_hidden, q = H / 4, big = (H - 16) / 2;
		// Token groups: embedding sizes and feature slices (network.py forward)
		struct Spec {
			const char* name;
			std::vector<std::pair<int, int>> parts;  // (feature start, width)
			std::vector<int> outs;
		};
		const std::array<Spec, kTokens> specs{{
			{"head", {{0, 6}, {36, 6}, {72, 3}, {81, 3}}, {q, q, q, q}},
			{"lhand", {{6, 6}, {42, 6}, {75, 3}, {84, 3}}, {q, q, q, q}},
			{"rhand", {{12, 6}, {48, 6}, {78, 3}, {87, 3}}, {q, q, q, q}},
			{"lfoot", {{18, 6}, {54, 6}, {126, 3}}, {big, big, 16}},
			{"rfoot", {{24, 6}, {60, 6}, {129, 3}}, {big, big, 16}},
			{"pelvis", {{30, 6}, {66, 6}, {132, 3}}, {big, big, 16}},
			{"lhand_inhead", {{90, 6}, {102, 6}, {114, 3}, {120, 3}}, {q, q, q, q}},
			{"rhand_inhead", {{96, 6}, {108, 6}, {117, 3}, {123, 3}}, {q, q, q, q}},
		}};
		for (int t = 0; t < kTokens; t++) {
			Group g;
			for (size_t k = 0; k < specs[t].parts.size(); k++) {
				auto [start, width] = specs[t].parts[k];
				g.emb.push_back(linear("backbone." + std::string(specs[t].name) + "_linear_embeddings." +
										   std::to_string(k) + ".0",
									   width, specs[t].outs[k]));
				std::vector<int> idx;
				for (int i = 0; i < width; i++) idx.push_back(start + i);
				g.slices.push_back(idx);
			}
			m_groups[t] = std::move(g);
		}
		m_normW = tensor("backbone.norm.weight", H);
		m_normB = tensor("backbone.norm.bias", H);
		m_lstm.assign(m_blocks, {});
		m_spatial.assign(m_blocks, std::vector<EncoderLayer>(m_layers));
		for (int b = 0; b < m_blocks; b++) {
			for (int t = 0; t < kTokens; t++) {
				std::string p = "backbone.time_encoder." + std::to_string(b) + "." + std::to_string(t);
				Lstm& l = m_lstm[b][t];
				l.ih = {H, 4 * H, halfTensor(p + ".weight_ih_l0", 4 * H * H), tensor(p + ".bias_ih_l0", 4 * H)};
				l.hh = {H, 4 * H, halfTensor(p + ".weight_hh_l0", 4 * H * H), tensor(p + ".bias_hh_l0", 4 * H)};
			}
			for (int k = 0; k < m_layers; k++) {
				std::string p = "backbone.spatial_encoder." + std::to_string(b) + ".layers." + std::to_string(k);
				EncoderLayer& e = m_spatial[b][k];
				e.inProj = {H, 3 * H, halfTensor(p + ".self_attn.in_proj_weight", 3 * H * H),
							tensor(p + ".self_attn.in_proj_bias", 3 * H)};
				e.outProj = linear(p + ".self_attn.out_proj", H, H);
				e.ff1 = linear(p + ".linear1", H, m_ffn);
				e.ff2 = linear(p + ".linear2", m_ffn, H);
				e.n1w = tensor(p + ".norm1.weight", H);
				e.n1b = tensor(p + ".norm1.bias", H);
				e.n2w = tensor(p + ".norm2.weight", H);
				e.n2b = tensor(p + ".norm2.bias", H);
			}
		}
		m_pose1 = linear("pose_est.0", kTokens * H, 256);
		m_pose2 = linear("pose_est.2", 256, kJoints * 6);
	} catch (const std::exception& e) {
		m_error = e.what();
		return false;
	}
	m_x.assign(kFeatures, 0.0f);
	m_tokens.assign(kTokens * m_hidden, 0.0f);
	m_buf.assign(std::max(4 * m_hidden, m_ffn) * kTokens, 0.0f);
	m_buf2.assign(std::max(4 * m_hidden, m_ffn) * kTokens, 0.0f);
	m_pose.assign(kJoints * 6, 0.0f);
	m_loaded = true;
	reset();
	return true;
}

void PoseModel::reset() {
	m_h.assign(static_cast<size_t>(m_blocks) * kTokens * m_hidden, 0.0f);
	m_c.assign(m_h.size(), 0.0f);
	m_havePrev = false;
	m_warm = 0;
	m_lastStep = -1;
	m_last = {};
}

std::array<Quat, 2> PoseModel::wristCalibrationFor(const Pose& hmd, const std::array<Pose, 2>& hands) {
	// The SMPL rest pose holds the arms along -/+X; hanging down is a rotation
	// about the body's forward axis: -90 deg left, +90 deg right
	std::array<Quat, 2> cal;
	Mat3 yaw = Quat::yaw(hmd.rotation.yawAngle()).toMatrix();
	for (int s = 0; s < 2; s++) {
		Mat3 expected = kRx90 * yaw * kRy180 * rotZ(s == 0 ? -kPi / 2 : kPi / 2);
		Mat3 ctrl = kRx90 * hands[s].rotation.toMatrix();
		cal[s] = Quat::fromMatrix(ctrl.transposed() * expected).normalized();
	}
	return cal;
}

void PoseModel::calibrateWrists(const Pose& hmd, const std::array<Pose, 2>& hands) {
	setWristCalibration(wristCalibrationFor(hmd, hands));
}

void PoseModel::setWristCalibration(const std::array<Quat, 2>& cal) {
	m_wristCal = cal;
	m_wristsCalibrated = true;
}

PoseEstimate PoseModel::update(const PoseModelInput& in, double now) {
	if (!m_loaded) return {};
	const double period = 1.0 / m_hz;
	if (m_lastStep >= 0 && now - m_lastStep > 0.5) reset();  // inputs missing too long
	if (m_lastStep < 0 || now >= m_nextTick) {
		m_nextTick = (m_lastStep < 0 || now - m_nextTick > 0.1) ? now + period : m_nextTick + period;
		PoseEstimate e = step(in);
		if (e.valid) m_lastStep = now;
		return e;
	}
	return m_last;
}

PoseEstimate PoseModel::step(const PoseModelInput& in) {
	if (!m_loaded || !m_wristsCalibrated || !in.hmdValid || !in.legValid[0] || !in.legValid[1] ||
		!in.handValid[0] || !in.handValid[1]) {
		m_last.valid = false;
		return m_last;
	}
	const double floor = in.floorY;
	std::array<Mat3, 6> rot;  // head, left wrist, right wrist, left shin, right shin, pelvis
	std::array<Vec3, 3> pos;  // head, left wrist, right wrist
	rot[0] = toAmass(in.hmd.rotation);
	pos[0] = posToAmass(in.hmd.position + in.hmd.rotation.rotate(kHeadOffset), floor);
	for (int s = 0; s < 2; s++) {
		rot[1 + s] = kRx90 * in.hands[s].rotation.toMatrix() * m_wristCal[s].toMatrix();
		pos[1 + s] = posToAmass(in.hands[s].position + in.hands[s].rotation.rotate(kWristOffset), floor);
	}
	rot[3] = toAmass(in.shin[0]);
	rot[4] = toAmass(in.shin[1]);
	rot[5] = in.hasWaist ? toAmass(in.waist) : Mat3{};
	std::array<Mat3, 2> hin;
	std::array<Vec3, 2> hpos;
	Mat3 headT = rot[0].transposed();
	for (int s = 0; s < 2; s++) {
		hin[s] = headT * rot[1 + s];
		hpos[s] = headT * (pos[1 + s] - pos[0]);
	}
	if (!m_havePrev) {
		m_prevRot = rot;
		m_prevPos = pos;
		m_prevHandInHead = hin;
		m_prevHandPos = hpos;
		m_havePrev = true;
	}
	float* x = m_x.data();
	for (int k = 0; k < 6; k++) {
		putSixd(rot[k], x + 6 * k);
		putSixd(m_prevRot[k].transposed() * rot[k], x + 36 + 6 * k);
	}
	for (int k = 0; k < 3; k++) {
		putVec(pos[k], x + 72 + 3 * k);
		putVec(pos[k] - m_prevPos[k], x + 81 + 3 * k);
	}
	for (int s = 0; s < 2; s++) {
		putSixd(hin[s], x + 90 + 6 * s);
		putSixd(m_prevHandInHead[s].transposed() * hin[s], x + 102 + 6 * s);
		putVec(hpos[s], x + 114 + 3 * s);
		putVec(hpos[s] - m_prevHandPos[s], x + 120 + 3 * s);
	}
	putVec(kRx90 * in.legAccel[0], x + 126);
	putVec(kRx90 * in.legAccel[1], x + 129);
	putVec(kRx90 * in.waistAccel, x + 132);
	if (!in.hasWaist) {
		for (int i = 30; i < 36; i++) x[i] = 0;
		for (int i = 66; i < 72; i++) x[i] = 0;
		for (int i = 132; i < 135; i++) x[i] = 0;
	}
	m_prevRot = rot;
	m_prevPos = pos;
	m_prevHandInHead = hin;
	m_prevHandPos = hpos;

	forward(m_pose.data());

	// Forward kinematics of the head and leg chains with the user's skeleton
	const double scale = m_height / m_modelHeight;
	std::array<Mat3, kJoints> g;
	std::array<Vec3, kJoints> p;
	std::array<bool, kJoints> need{};
	for (int k : {7, 8, 15}) {
		for (int j = k; j >= 0; j = m_parents[j]) need[j] = true;
	}
	for (int j = 0; j < kJoints; j++) {
		if (!need[j]) continue;
		Mat3 local = sixdToMat(m_pose.data() + 6 * j);
		int par = m_parents[j];
		g[j] = par < 0 ? local : g[par] * local;
		p[j] = par < 0 ? Vec3{} : p[par] + g[par] * (m_rest[j] * scale);
	}
	Vec3 shift = pos[0] - p[15];  // hanging from the head
	Mat3 back = kRx90.transposed();
	auto toWorld = [&](const Vec3& v) {
		Vec3 w = back * (v + shift);
		w.y += floor;
		return w;
	};
	m_warm += 1.0 / m_hz;
	m_last.valid = true;
	m_last.ankle[0] = toWorld(p[7]);
	m_last.ankle[1] = toWorld(p[8]);
	m_last.pelvis = toWorld(p[0]);
	m_last.warmup = m_warm;
	return m_last;
}

void PoseModel::encoder(const EncoderLayer& l, float* x) {
	const int H = m_hidden, nh = m_heads, dh = H / nh;
	float* qkv = m_buf.data();  // [token][3H]
	l.inProj.applyBatch(x, kTokens, qkv);
	float* att = m_buf2.data();  // [token][H]
	const float sc = 1.0f / std::sqrt(static_cast<float>(dh));
	for (int h = 0; h < nh; h++) {
		for (int i = 0; i < kTokens; i++) {
			float w[kTokens], mx = -1e30f, sum = 0;
			const float* qi = qkv + i * 3 * H + h * dh;
			for (int j = 0; j < kTokens; j++) {
				const float* kj = qkv + j * 3 * H + H + h * dh;
				float d = 0;
				for (int c = 0; c < dh; c++) d += qi[c] * kj[c];
				w[j] = d * sc;
				mx = std::max(mx, w[j]);
			}
			for (int j = 0; j < kTokens; j++) sum += (w[j] = std::exp(w[j] - mx));
			float* o = att + i * H + h * dh;
			for (int c = 0; c < dh; c++) o[c] = 0;
			for (int j = 0; j < kTokens; j++) {
				const float* vj = qkv + j * 3 * H + 2 * H + h * dh;
				float a = w[j] / sum;
				for (int c = 0; c < dh; c++) o[c] += a * vj[c];
			}
		}
	}
	float* tmp = m_buf.data();  // qkv no longer needed
	l.outProj.applyBatch(att, kTokens, tmp);
	for (int t = 0; t < kTokens; t++) {
		for (int i = 0; i < H; i++) x[t * H + i] += tmp[t * H + i];
		layerNorm(x + t * H, H, l.n1w, l.n1b, 1e-5f);
	}
	float* hid = m_buf2.data();  // attention output no longer needed
	l.ff1.applyBatch(x, kTokens, hid);
	for (int i = 0; i < kTokens * m_ffn; i++) hid[i] = hid[i] > 0 ? hid[i] : 0;
	l.ff2.applyBatch(hid, kTokens, tmp);
	for (int t = 0; t < kTokens; t++) {
		for (int i = 0; i < H; i++) x[t * H + i] += tmp[t * H + i];
		layerNorm(x + t * H, H, l.n2w, l.n2b, 1e-5f);
	}
}

void PoseModel::forward(float* pose) {
	const int H = m_hidden;
	float* tok = m_tokens.data();
	for (int t = 0; t < kTokens; t++) {
		const Group& g = m_groups[t];
		float* o = tok + t * H;
		int at = 0;
		for (size_t k = 0; k < g.emb.size(); k++) {
			float in[6];
			for (size_t i = 0; i < g.slices[k].size(); i++) in[i] = m_x[g.slices[k][i]];
			g.emb[k].apply(in, o + at);
			for (int i = 0; i < g.emb[k].out; i++) o[at + i] = leaky(o[at + i]);
			at += g.emb[k].out;
		}
		layerNorm(o, H, m_normW, m_normB, 1e-6f);
	}
	std::vector<float> gates(4 * H), gh(4 * H);
	for (int b = 0; b < m_blocks; b++) {
		for (int t = 0; t < kTokens; t++) {
			const Lstm& l = m_lstm[b][t];
			float* h = m_h.data() + (static_cast<size_t>(b) * kTokens + t) * H;
			float* c = m_c.data() + (static_cast<size_t>(b) * kTokens + t) * H;
			l.ih.apply(tok + t * H, gates.data());
			l.hh.apply(h, gh.data());
			for (int i = 0; i < H; i++) {
				float ig = sigmoid(gates[i] + gh[i]), fg = sigmoid(gates[H + i] + gh[H + i]);
				float gg = std::tanh(gates[2 * H + i] + gh[2 * H + i]), og = sigmoid(gates[3 * H + i] + gh[3 * H + i]);
				c[i] = fg * c[i] + ig * gg;
				h[i] = og * std::tanh(c[i]);
			}
			std::memcpy(tok + t * H, h, sizeof(float) * H);
		}
		for (const auto& l : m_spatial[b]) encoder(l, tok);
	}
	std::vector<float> hid(256);
	m_pose1.apply(tok, hid.data());
	for (auto& v : hid) v = leaky(v);
	m_pose2.apply(hid.data(), pose);
}

double fusePose(const PoseEstimate& e, const Pose& hmd, double floorY, double eyeHeight, BodyOutput& out,
				const PoseFusionSettings& s) {
	if (!e.valid || !out.valid || e.warmup < s.minWarmup || eyeHeight <= 0) return 0;
	double ratio = (hmd.position.y - floorY) / eyeHeight;
	double w = std::clamp((ratio - s.ratioLow) / std::max(s.ratioHigh - s.ratioLow, 1e-6), 0.0, 1.0);
	if (w <= 0) return 0;
	for (int i = 0; i < 2; i++) {
		Vec3 m = e.ankle[i];
		m.y = std::max(m.y, floorY + s.minAnkleHeight);
		out.feet[i].position = out.feet[i].position * (1 - w) + m * w;
	}
	return w;
}

}  // namespace slimelite
