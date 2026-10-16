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
	m_cleanTime = m_headingOnlyTime = m_rejectRatio = 0;
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
						   double angularSpeed, double dt, const HeadingFilterSettings& s) {
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

	// Fast rotations: q and m are not perfectly simultaneous, skip
	if (angularSpeed > s.maxAngularSpeed) return;

	double sens = std::clamp(s.sensitivity, 0.0, 1.0);
	double normTol = lerp(0.35, 0.08, sens);
	double dipTol = deg2rad(lerp(16.0, 4.0, sens));
	double headTol = deg2rad(lerp(25.0, 6.0, sens));

	bool fieldOk = std::fabs(norm - 1.0) < normTol && std::fabs(dip - m_cal.dip) < dipTol;

	if (!m_initialized) {
		m_state = State::Acquiring;
		if (!fieldOk) {
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
		}
		return;
	}

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
			m_relock = false;
			m_rejectRatio = m_cleanTime = m_headingOnlyTime = 0;
			m_state = State::Ok;
		}
		return;
	}
	headTol += s.headingTolPerRadPerSec * angularSpeed;
	bool headingOk = std::fabs(innovation) < headTol;
	// Recent fraction of heading outliers (time constant 0.5 s)
	double k = std::min(1.0, dt / 0.5);
	m_rejectRatio += ((headingOk ? 0.0 : 1.0) - m_rejectRatio) * k;

	if (!fieldOk) {
		// Anti-mag: something distorts the field (norm or dip), trust the gyro only
		m_state = State::Disturbed;
		m_cleanTime = 0;
		m_headingOnlyTime = 0;
		return;
	}

	if (!headingOk) {
		// Norm and dip look normal but the heading disagrees with the gyro. Isolated
		// outliers (motion, noise) are just skipped. Many in a row: a disturbance that
		// keeps the norm, or the field really differs here (or the gyro drifted during
		// a long disturbance). Reject it, but if it persists, slowly follow it.
		m_cleanTime = 0;
		if (m_rejectRatio < 0.6 && m_state != State::Disturbed) return;
		m_headingOnlyTime += dt;
		if (m_headingOnlyTime < s.reacquireTime) {
			m_state = State::Disturbed;
			return;
		}
		m_heading = wrapAngle(m_heading + innovation * std::min(1.0, dt / s.reacquireTimeConstant));
		m_state = State::Ok;
		return;
	}

	if (m_state == State::Disturbed) {
		m_cleanTime += dt;
		if (m_cleanTime < s.clearTime || m_rejectRatio > 0.3) return;
	}
	m_headingOnlyTime = 0;
	m_state = State::Ok;
	m_heading = wrapAngle(m_heading + innovation * std::min(1.0, dt / s.timeConstant));
}

}  // namespace slimelite
