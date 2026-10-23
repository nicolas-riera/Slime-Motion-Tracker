#include <algorithm>

#include "smt/mag.h"

namespace smt {

namespace {

// 26 directions: cube faces, edges and corners
const std::array<Vec3, MagCalibrator::kBins>& binDirections() {
	static const auto dirs = [] {
		std::array<Vec3, MagCalibrator::kBins> d{};
		int n = 0;
		for (int x = -1; x <= 1; x++)
			for (int y = -1; y <= 1; y++)
				for (int z = -1; z <= 1; z++)
					if (x || y || z) d[n++] = Vec3{double(x), double(y), double(z)}.normalized();
		return d;
	}();
	return dirs;
}

int nearestBin(const Vec3& v) {
	const auto& dirs = binDirections();
	int best = 0;
	double bestDot = -2;
	for (int i = 0; i < MagCalibrator::kBins; i++) {
		double d = dirs[i].dot(v);
		if (d > bestDot) {
			bestDot = d;
			best = i;
		}
	}
	return best;
}

constexpr int kMinSamplesPerBin = 3;
constexpr double kMinCoverage = 0.45;
// Raw counts are ~1e3, the other unknowns ~1: scale the K columns
constexpr double kRawScale = 1e-3;
// Residual (field norm units) a new automatic fit needs to replace a valid
// calibration (good ones: 0.045-0.055), and by how much it must beat the current one
// on the same samples
constexpr double kMaxReplacingResidual = 0.065;
constexpr double kReplacingRatio = 0.7;

}  // namespace

void MagCalibrator::addSample(const Quat& qWorld, const std::array<int16_t, 3>& raw,
							  double angularSpeed, double time) {
	if (angularSpeed > 3.0) return;
	// Bin by where "up" is in the sensor frame: the fit needs tilt diversity
	Vec3 upSensor = qWorld.conj().rotate({0, 1, 0});
	int bin = nearestBin(upSensor);
	if (time - m_lastAdded[bin] < 0.05) return;
	m_lastAdded[bin] = time;
	auto& b = m_bins[bin];
	b.push_back({qWorld.toMatrix(), Vec3{double(raw[0]), double(raw[1]), double(raw[2])}});
	if (b.size() > kPerBin) b.pop_front();
}

std::optional<double> MagCalibrator::consistency(const MagCalibration& cal) const {
	if (!cal.valid || sampleCount() < 30) return std::nullopt;
	double sq = 0;
	for (const auto& b : m_bins)
		for (const auto& s : b) {
			const Vec3 v = cal.K * s.m + cal.c;
			const double n = v.norm();
			if (n < 1e-9) return std::nullopt;
			// Gravity in the sensor frame: R is sensor -> world
			const Vec3 up = s.R.transposed() * Vec3{0, 1, 0};
			const double dip = std::asin(std::clamp(-up.dot(v) / n, -1.0, 1.0));
			sq += (n - 1) * (n - 1) + (dip - cal.dip) * (dip - cal.dip);
		}
	return std::sqrt(sq / double(sampleCount()));
}

std::optional<double> MagCalibrator::headingOffsetFor(const MagCalibration& from, const MagCalibration& to) const {
	if (!from.valid || !to.valid || sampleCount() < 30) return std::nullopt;
	double sn = 0, cs = 0;
	for (const auto& b : m_bins)
		for (const auto& s : b) {
			const double d = headingOf(s.R * (from.K * s.m + from.c)) + from.headingOffset
						   - headingOf(s.R * (to.K * s.m + to.c));
			sn += std::sin(d);
			cs += std::cos(d);
		}
	return std::atan2(sn, cs);
}

std::optional<MagCalibration> MagCalibrator::replacement(const MagCalibration& current) const {
	auto cal = solve();
	if (!cal) return std::nullopt;
	if (!current.valid) return cal;
	// Both measured on the same samples. Only a good fit replaces a valid calibration:
	// near a local disturbance (a bed frame) the recent samples favor any new fit, and
	// poor fits (residual 0.07-0.11 vs ~0.05) replaced each other
	auto now = consistency(current), next = consistency(*cal);
	auto offset = headingOffsetFor(current, *cal);
	if (!now || !next || !offset || cal->residual >= kMaxReplacingResidual || *next >= kReplacingRatio * *now)
		return std::nullopt;
	cal->headingOffset = *offset;
	return cal;
}

double MagCalibrator::coverage() const {
	int full = 0;
	for (const auto& b : m_bins)
		if (b.size() >= kMinSamplesPerBin) full++;
	// 18 of the 26 bins is plenty: some orientations are awkward to reach
	return std::min(1.0, full / 18.0);
}

std::vector<Vec3> MagCalibrator::rawSamples() const {
	std::vector<Vec3> out;
	for (const auto& b : m_bins)
		for (const auto& s : b) out.push_back(s.m);
	return out;
}

std::array<int, MagCalibrator::kBins> MagCalibrator::binCounts() const {
	std::array<int, kBins> out{};
	for (int i = 0; i < kBins; i++) out[i] = int(m_bins[i].size());
	return out;
}

Vec3 MagCalibrator::binDirection(int i) { return binDirections()[std::clamp(i, 0, kBins - 1)]; }

int MagCalibrator::binTarget() { return kMinSamplesPerBin; }

size_t MagCalibrator::sampleCount() const {
	size_t n = 0;
	for (const auto& b : m_bins) n += b.size();
	return n;
}

std::optional<double> MagCalibrator::residualOf(const MagCalibration& cal) const {
	if (!cal.valid || sampleCount() < 30) return std::nullopt;
	// |K m + c - R^T e| = |R (K m + c) - e|: e is the mean of R (K m + c)
	std::vector<Vec3> v;
	v.reserve(sampleCount());
	Vec3 mean;
	for (const auto& b : m_bins)
		for (const auto& s : b) {
			v.push_back(s.R * (cal.K * s.m + cal.c));
			mean += v.back();
		}
	mean = mean / double(v.size());
	double sq = 0;
	for (const auto& x : v) sq += (x - mean).dot(x - mean);
	return std::sqrt(sq / v.size());
}

void MagCalibrator::reset() {
	for (auto& b : m_bins) b.clear();
	m_lastAdded.fill(-1e9);
}

std::optional<MagCalibration> MagCalibrator::solve() const {
	if (coverage() < kMinCoverage) return std::nullopt;
	std::vector<Sample> all;
	for (const auto& b : m_bins) all.insert(all.end(), b.begin(), b.end());
	return solveSamples(all);
}

std::optional<MagCalibration> MagCalibrator::solveSamples(const std::vector<Sample>& samples) {
	if (samples.size() < 30) return std::nullopt;

	// Unknowns x = [K row-major (9), c (3), m_e (3)]; per sample 3 rows:
	//   K_i . m + c_i - (R^T)_i . m_e = 0
	constexpr int N = 15;
	std::vector<double> ata(N * N, 0.0);
	for (const auto& s : samples) {
		Mat3 Rt = s.R.transposed();
		for (int i = 0; i < 3; i++) {
			double row[N] = {};
			for (int j = 0; j < 3; j++) row[3 * i + j] = s.m[j] * kRawScale;
			row[9 + i] = 1.0;
			for (int j = 0; j < 3; j++) row[12 + j] = -Rt(i, j);
			for (int a = 0; a < N; a++)
				for (int b = 0; b < N; b++) ata[a * N + b] += row[a] * row[b];
		}
	}
	std::vector<double> values, vectors;
	symmetricEigen(N, ata, values, vectors);
	// A well-posed problem has a single near-zero eigenvalue
	if (values[1] <= 0 || values[0] / values[1] > 0.25) return std::nullopt;

	const double* x = &vectors[0];
	MagCalibration cal;
	for (int i = 0; i < 9; i++) cal.K.m[i] = x[i] * kRawScale;
	cal.c = {x[9], x[10], x[11]};
	Vec3 me{x[12], x[13], x[14]};
	double n = me.norm();
	if (n < 1e-9) return std::nullopt;
	cal.K = cal.K * (1.0 / n);
	cal.c = cal.c / n;
	me = me / n;
	// Both chips are right-handed: K must preserve handedness
	if (cal.K.det() < 0) {
		cal.K = cal.K * -1.0;
		cal.c = -cal.c;
		me = -me;
	}
	cal.dip = std::asin(std::clamp(-me.y, -1.0, 1.0));

	double sq = 0;
	for (const auto& s : samples) {
		Vec3 r = cal.K * s.m + cal.c - s.R.transposed() * me;
		sq += r.dot(r);
	}
	cal.residual = std::sqrt(sq / samples.size());

	// Sanity checks: plausible residual, soft iron and dip
	Mat3 KtK = cal.K.transposed() * cal.K;
	std::vector<double> ev, evec;
	symmetricEigen(3, std::vector<double>(KtK.m.begin(), KtK.m.end()), ev, evec);
	double anisotropy = std::sqrt(ev[2] / std::max(ev[0], 1e-30));
	double dipDeg = rad2deg(std::fabs(cal.dip));
	if (cal.residual > 0.15 || anisotropy > 2.5 || dipDeg < 5 || dipDeg > 85) return std::nullopt;

	cal.valid = true;
	return cal;
}

}  // namespace smt
