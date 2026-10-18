#include <algorithm>

#include "slimelite/mag.h"

namespace slimelite {

namespace {

double lerp(double a, double b, double t) { return a + (b - a) * t; }

}  // namespace

void HeadingFilter::setCalibration(const MagCalibration& cal) {
	m_cal = cal;
	reset();
}

void HeadingFilter::reset() {
	m_state = m_cal.valid ? State::Acquiring : State::Uncalibrated;
	m_initialized = false;
	m_rate = m_tempRate = 0;
	for (auto& row : m_P)
		for (double& x : row) x = 0;
	m_tempRef = std::nan("");
	m_stillTime = m_cleanTime = m_rejectRatio = 0;
	m_acqSin = m_acqCos = m_acqTime = 0;
	m_relock = false;
}

void HeadingFilter::updateCalibration(MagCalibration cal, const Quat& qWorld,
									  const std::array<int16_t, 3>& raw) {
	if (m_initialized && m_cal.valid) {
		// Keep the corrected frame continuous: whatever heading the new calibration
		// measures right now, map it onto the current filtered heading
		double hNew = headingOf(qWorld.rotate(cal.apply(raw)));
		cal.headingOffset = wrapAngle(m_heading - hNew);
		m_cal = cal;
		return;
	}
	setCalibration(cal);
}

void HeadingFilter::update(const Quat& qWorld, const std::array<int16_t, 3>& raw,
						   double angularSpeed, double dt, const HeadingFilterSettings& s,
						   double temperature) {
	if (!m_cal.valid) {
		m_state = State::Uncalibrated;
		return;
	}
	Vec3 v = qWorld.rotate(m_cal.apply(raw));
	double norm = v.norm();
	if (norm < 1e-6) return;
	double dip = std::asin(std::clamp(-v.y / norm, -1.0, 1.0));
	double h = wrapAngle(headingOf(v) + m_cal.headingOffset);
	m_lastNorm = norm;
	m_lastDip = dip;

	double sens = std::clamp(s.sensitivity, 0.0, 1.0);
	double normTol = lerp(0.35, 0.08, sens);
	double dipTol = deg2rad(lerp(16.0, 4.0, sens));
	double headTol = deg2rad(lerp(25.0, 6.0, sens));
	double dNorm = std::fabs(norm - 1.0), dDip = std::fabs(dip - m_cal.dip);
	bool fieldOk = dNorm < normTol && dDip < dipTol;

	if (!m_initialized) {
		m_state = State::Acquiring;
		if (angularSpeed > s.maxAngularSpeed || !fieldOk) {
			m_acqSin = m_acqCos = m_acqTime = 0;
			return;
		}
		m_acqSin += std::sin(h);
		m_acqCos += std::cos(h);
		m_acqTime += dt;
		if (m_acqTime >= 0.4) {
			m_heading = std::atan2(m_acqSin, m_acqCos);
			m_initialized = true;
			m_state = State::Ok;
			m_P[0][0] = s.magSigma * s.magSigma;
			m_P[1][1] = s.rateInit * s.rateInit;
			m_P[2][2] = s.tempRateInit * s.tempRateInit;
		}
		return;
	}

	// Predict: the device yaw drifts at rate + tempRate * (T - Tref)
	double u = 0;
	if (std::isfinite(temperature)) {
		if (!std::isfinite(m_tempRef)) m_tempRef = temperature;
		u = temperature - m_tempRef;
	}
	m_heading = wrapAngle(m_heading + (m_rate + m_tempRate * u) * dt);
	{
		// P = F P F^T + Q with F = [[1, dt, dt u], [0, 1, 0], [0, 0, 1]]
		double f[3] = {1, dt, dt * u};
		double row[3];
		for (int j = 0; j < 3; j++) row[j] = f[0] * m_P[0][j] + f[1] * m_P[1][j] + f[2] * m_P[2][j];
		double p00 = row[0] * f[0] + row[1] * f[1] + row[2] * f[2];
		m_P[0][0] = p00;
		m_P[0][1] = m_P[1][0] = row[1];
		m_P[0][2] = m_P[2][0] = row[2];
		m_P[0][0] += s.headingWalk * s.headingWalk * dt;
		m_P[1][1] += s.rateWalk * s.rateWalk * dt;
	}

	if (angularSpeed < s.stillSpeed) m_stillTime += dt;
	else m_stillTime = 0;

	// Fast rotations: q and m are not perfectly simultaneous, skip
	if (angularSpeed > s.maxAngularSpeed) return;

	double innovation = wrapAngle(h - m_heading);
	if (m_relock) {
		// After a gyro glitch, any clean field is right whatever the heading says
		if (!fieldOk) {
			m_relockSin = m_relockCos = m_relockTime = 0;
			m_state = State::Disturbed;
			return;
		}
		m_relockSin += std::sin(innovation);
		m_relockCos += std::cos(innovation);
		m_relockTime += dt;
		if (m_relockTime >= 0.3) {
			m_heading = wrapAngle(m_heading + std::atan2(m_relockSin, m_relockCos));
			m_P[0][0] = s.magSigma * s.magSigma;
			m_P[0][1] = m_P[1][0] = m_P[0][2] = m_P[2][0] = 0;
			m_relock = false;
			m_rejectRatio = m_cleanTime = 0;
			m_state = State::Ok;
		}
		return;
	}

	// Gate against the predicted uncertainty: a heading the gyro cannot explain is a
	// disturbance, unless the prediction itself has become uncertain enough
	headTol += s.headingTolPerRadPerSec * angularSpeed;
	bool headingOk = std::fabs(innovation) < headTol + 3.0 * std::sqrt(m_P[0][0]);
	double k = std::min(1.0, dt / 0.5);
	m_rejectRatio += ((headingOk && fieldOk ? 0.0 : 1.0) - m_rejectRatio) * k;

	if (!fieldOk || !headingOk) {
		// Anti-mag: something distorts the field, trust the gyro (and the learnt drift)
		m_state = State::Disturbed;
		m_cleanTime = 0;
		return;
	}
	if (m_state == State::Disturbed) {
		m_cleanTime += dt;
		if (m_cleanTime < s.clearTime || m_rejectRatio > 0.3) return;
	}
	m_state = State::Ok;

	// Measurement variance: one magSigma per magCorrTime seconds of samples, inflated
	// by field anomalies and by time spent still
	double r = s.magSigma * s.magSigma * s.magCorrTime / std::max(dt, 1e-3);
	double a = dNorm / s.normSoft, b = dDip / s.dipSoft;
	r *= 1.0 + a * a + b * b;
	r *= std::min(s.stillMaxFactor, 1.0 + m_stillTime / s.stillGrowth);

	// Kalman update with H = [1, 0, 0]
	double sInv = 1.0 / (m_P[0][0] + r);
	double g[3] = {m_P[0][0] * sInv, m_P[1][0] * sInv, m_P[2][0] * sInv};
	m_heading = wrapAngle(m_heading + g[0] * innovation);
	m_rate += g[1] * innovation;
	m_tempRate += g[2] * innovation;
	double p0[3] = {m_P[0][0], m_P[0][1], m_P[0][2]};
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) m_P[i][j] -= g[i] * p0[j];
	// Keep the drift physically plausible (the firmware already removes most of it)
	double maxRate = deg2rad(20.0) / 60.0;
	m_rate = std::clamp(m_rate, -maxRate, maxRate);
}

}  // namespace slimelite
