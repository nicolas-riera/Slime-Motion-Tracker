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
//  * HeadingFilter: Kalman filter on the heading correction h and its drift rate
//    (the residual gyro bias, optionally linear in the IMU temperature), measured
//    by the field heading seen in the drifting device frame. The drift rate is
//    learnt while the field is clean, so when the mag is ignored the gyro keeps
//    drifting much less. The mag weight drops with field anomalies (norm, dip far
//    from the calibration: metal furniture, radiators, bed frames) and with time
//    spent still (a still tracker keeps seeing the same local disturbance, so its
//    repeated samples carry no new information). Outliers are gated against the
//    predicted uncertainty instead of being followed after a delay.
#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <vector>

#include "smt/linalg.h"

namespace smt {

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
	// Residual of a given calibration on the current samples (same scale as
	// MagCalibration::residual), so an old fit and a new one compare on the same
	// data. The earth field direction is re-fitted: the device's world yaw drifts.
	std::optional<double> residualOf(const MagCalibration& cal) const;

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
	double sensitivity = 0.5;  // 0 = tolerant, 1 = rejects small disturbances
	double clearTime = 1.0;  // seconds of consistent field before trusting it again
	double maxAngularSpeed = 6.0;  // rad/s, above this the mag is skipped (latency)
	// q and m are sampled up to ~20 ms apart: widen the heading tolerance with speed
	double headingTolPerRadPerSec = deg2rad(3.0);
	// Kalman model. Indoor fields vary by tens of degrees across a room while the
	// gyro drifts ~1 deg/min: the mag heading is a noisy, slowly decorrelating
	// measurement (magSigma per magCorrTime seconds of samples)
	double magSigma = deg2rad(8.0);
	double magCorrTime = 2.0;
	double headingWalk = deg2rad(0.5) / std::sqrt(60.0);  // rad / sqrt(s)
	double rateInit = deg2rad(2.0) / 60.0;  // rad/s, initial drift rate uncertainty
	// The drift rate is not constant: up to 8 deg/min right after boot, decaying as
	// the IMU warms up (seen on the waist tracker)
	double rateWalk = deg2rad(1.0) / 60.0 / std::sqrt(60.0);  // rad/s / sqrt(s)
	double tempRateInit = deg2rad(0.2) / 60.0;  // rad/s per deg C
	// Field anomaly: soft down-weighting scale (hard rejection stays at the
	// sensitivity-derived tolerances)
	double normSoft = 0.04;
	double dipSoft = deg2rad(2.5);
	// Still tracker: the mag variance grows by 1x per stillGrowth seconds of rest
	double stillSpeed = 0.35;  // rad/s
	double stillGrowth = 5.0;
	double stillMaxFactor = 100.0;
	// Local field reference: the expected norm and dip follow the field where the
	// player is (another room, the building's steel: uniformly different from where
	// the calibration was done) over refTime seconds, only while the mag heading
	// agrees with the gyro. A field changing as the tracker moves (metal nearby) or
	// a heading the gyro contradicts stays a disturbance. Bounded around the
	// calibration's norm and dip.
	bool adaptiveRef = true;
	double refTime = 20.0;
	double refMaxNorm = 0.4;  // relative
	double refMaxDip = deg2rad(25.0);
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
	void restore(double heading, double rate = 0) {
		if (!m_cal.valid) return;
		m_heading = heading;
		m_rate = rate;
		HeadingFilterSettings d;
		m_P[0][0] = deg2rad(2.0) * deg2rad(2.0);
		m_P[1][1] = d.rateInit * d.rateInit;
		m_P[2][2] = d.tempRateInit * d.tempRateInit;
		m_initialized = true;
		m_state = State::Ok;
	}

	// temperature: IMU temperature in deg C, NaN when unknown
	void update(const Quat& qWorld, const std::array<int16_t, 3>& raw, double angularSpeed,
				double dt, const HeadingFilterSettings& s, double temperature = std::nan(""));
	// The device orientation jumped (sensor glitch, fusion reset): the gyro is wrong
	// this time, so re-lock onto the next clean mag samples instead of rejecting them
	void gyroGlitch() {
		if (!m_initialized) return;
		m_relock = true;
		m_relockSin = m_relockCos = m_relockTime = 0;
	}

	// Yaw to apply on the left of the device orientation: q_corrected = Ry(-h) * q
	bool hasHeading() const { return m_initialized; }
	double heading() const { return m_heading; }
	// Learnt drift rate of the heading correction (rad/s) and its temperature slope
	double rate() const { return m_rate; }
	double tempRate() const { return m_tempRate; }
	double headingSigma() const { return std::sqrt(m_P[0][0]); }
	State state() const { return m_state; }

	// Last measurement, for diagnostics
	double lastNorm() const { return m_lastNorm; }
	double lastDip() const { return m_lastDip; }
	double refNorm() const { return m_refNorm; }
	double refDip() const { return m_refDip; }

private:
	MagCalibration m_cal;
	State m_state = State::Uncalibrated;
	bool m_initialized = false;
	double m_heading = 0;
	double m_rate = 0, m_tempRate = 0;  // drift: dh/dt = rate + tempRate * (T - Tref)
	double m_P[3][3] = {};  // covariance of (heading, rate, tempRate)
	double m_tempRef = std::nan("");
	double m_stillTime = 0;
	double m_cleanTime = 0;
	double m_rejectRatio = 0;  // recent fraction of heading-inconsistent samples
	double m_lastNorm = 0, m_lastDip = 0;
	double m_refNorm = 1, m_refDip = 0;  // local field reference (adaptiveRef)
	// Initial acquisition: circular mean of the first consistent samples
	double m_acqSin = 0, m_acqCos = 0, m_acqTime = 0;
	// Re-lock after a gyro glitch: circular mean of the innovation
	bool m_relock = false;
	double m_relockSin = 0, m_relockCos = 0, m_relockTime = 0;
};

// Heading of the horizontal part of v, increasing with rotations around +Y
inline double headingOf(const Vec3& v) { return std::atan2(v.x, v.z); }

}  // namespace smt
