// Magnetometer calibration and heading fusion.
//
// The tracker runs a 6D (gyro + accel) fusion on device; its yaw drifts slowly
// and starts at an arbitrary value at every boot. The host uses the raw mag to
// make the yaw absolute (magnetic north) and drift free:
//
//  * MagCalibrator: orientation-aided calibration. For each sample k with the
//    device rotation R_k (sensor -> world), R_k^T m_e = K m_raw,k + c. Linear in
//    (K, c, m_e): 15 unknowns, solved as the smallest eigenvector of the normal
//    matrix. K absorbs gain, soft iron and the mag-to-IMU axis mapping, c the hard
//    iron. No need to know how the mag chip is oriented on the PCB.
//
//  * HeadingFilter: low-pass of the field heading seen in the drifting device
//    frame, with disturbance rejection ("anti-mag"): when the field changes in a
//    way the gyro does not explain (norm, dip or heading jump, e.g. a controller
//    brought close to the tracker), the mag is ignored and the gyro alone keeps
//    the orientation until the field is consistent again.
#pragma once

#include <array>
#include <deque>
#include <optional>
#include <vector>

#include "slimelite/linalg.h"

namespace slimelite {

struct MagCalibration {
	Mat3 K = Mat3::zero();
	Vec3 c;
	double dip = 0;  // radians, positive when the field points down
	double residual = 1;  // RMS fit residual, in units of the field norm
	// Added to the measured heading so the corrected frame stays continuous when a
	// new calibration replaces an older one
	double headingOffset = 0;
	bool valid = false;

	Vec3 apply(const std::array<int16_t, 3>& raw) const {
		return K * Vec3{double(raw[0]), double(raw[1]), double(raw[2])} + c;
	}
};

class MagCalibrator {
public:
	static constexpr int kBins = 26;
	static constexpr int kPerBin = 20;

	MagCalibrator() { reset(); }

	// qWorld: device orientation (sensor -> world, Y up). angularSpeed in rad/s.
	void addSample(const Quat& qWorld, const std::array<int16_t, 3>& raw, double angularSpeed,
				   double time);
	// Fraction of orientation bins with enough samples, 0..1
	double coverage() const;
	size_t sampleCount() const;
	void reset();

	// Solves with the collected samples. Returns nullopt if the data is degenerate
	// or the fit does not pass the sanity checks.
	std::optional<MagCalibration> solve() const;

	// Exposed for tests and the replay tool
	struct Sample {
		Mat3 R;  // sensor -> world
		Vec3 m;  // raw counts
	};
	static std::optional<MagCalibration> solveSamples(const std::vector<Sample>& samples);

private:
	std::array<std::deque<Sample>, kBins> m_bins;
	std::array<double, kBins> m_lastAdded{};
};

struct HeadingFilterSettings {
	double timeConstant = 6.0;  // seconds, heading low-pass
	double sensitivity = 0.5;  // 0 = tolerant, 1 = rejects small disturbances
	double clearTime = 1.0;  // seconds of consistent field before trusting it again
	double reacquireTime = 10.0;  // persistent heading-only offset: slowly re-acquire
	double reacquireTimeConstant = 20.0;
	double maxAngularSpeed = 6.0;  // rad/s, above this the mag is skipped (latency)
	// q and m are sampled up to ~20 ms apart: widen the heading tolerance with speed
	double headingTolPerRadPerSec = deg2rad(3.0);
};

class HeadingFilter {
public:
	enum class State { Uncalibrated, Acquiring, Ok, Disturbed };

	void setCalibration(const MagCalibration& cal);
	const MagCalibration& calibration() const { return m_cal; }
	// Replaces the calibration keeping the corrected heading continuous
	void updateCalibration(MagCalibration cal, const Quat& qWorld,
						   const std::array<int16_t, 3>& raw);
	void reset();
	// Hot reload: resume with a known heading instead of re-acquiring
	void restore(double heading) {
		if (!m_cal.valid) return;
		m_heading = heading;
		m_initialized = true;
		m_state = State::Ok;
	}

	void update(const Quat& qWorld, const std::array<int16_t, 3>& raw, double angularSpeed,
				double dt, const HeadingFilterSettings& s);

	// Yaw to apply on the left of the device orientation: q_corrected = Ry(-h) * q
	bool hasHeading() const { return m_initialized; }
	double heading() const { return m_heading; }
	State state() const { return m_state; }

	// Last measurement, for diagnostics
	double lastNorm() const { return m_lastNorm; }
	double lastDip() const { return m_lastDip; }

private:
	MagCalibration m_cal;
	State m_state = State::Uncalibrated;
	bool m_initialized = false;
	double m_heading = 0;
	double m_cleanTime = 0;
	double m_headingOnlyTime = 0;
	double m_rejectRatio = 0;  // recent fraction of heading-inconsistent samples
	double m_lastNorm = 0, m_lastDip = 0;
	// Initial acquisition: circular mean of the first consistent samples
	double m_acqSin = 0, m_acqCos = 0, m_acqTime = 0;
};

// Heading of the horizontal part of v, increasing with rotations around +Y
inline double headingOf(const Vec3& v) { return std::atan2(v.x, v.z); }

}  // namespace slimelite
