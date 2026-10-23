// Recovery from a firmware orientation glitch.
//
// Now and then the tracker's own fusion jumps while the tracker is still (23/10 14:41
// recording: tilt 16 -> 62 deg and yaw 31 deg in 13 ms) and takes seconds to converge
// back (6 s). Meanwhile the leg pointed sideways in the air, and the linear acceleration
// the firmware sends (gravity removed along its wrong vertical: 10.7 m/s^2) moved the
// feet. The glitch starts with a rotation jump no limb makes (TrackerHub's glitch rule);
// from there, while still, the measured gravity (linear acceleration plus the firmware's
// gravity) gives the true vertical: the orientation is tilted onto it and the
// acceleration recomputed, until the firmware's own tilt agrees again.
#pragma once

#include <algorithm>
#include <cmath>

#include "smt/linalg.h"

namespace smt {

class TiltFix {
public:
	static constexpr double kGravity = 9.81;

	// A rotation jump no limb makes: the device fusion glitched
	void glitch(double t) {
		m_active = true;
		m_start = m_lastLarge = t;
	}
	bool active() const { return m_active; }
	// Recovering and still: the corrected orientation is good enough for the mag heading
	bool still(double angularSpeed) const { return m_active && angularSpeed < kStillSpeed; }

	// raw: device orientation (sensor -> world, Y up), rawLinear: the firmware's linear
	// acceleration (sensor frame), angularSpeed rad/s
	void update(const Quat& raw, const Vec3& rawLinear, double angularSpeed, double t, double dt) {
		if (!m_active) return;
		const Vec3 measured = rawLinear + raw.conj().rotate({0, 1, 0}) * kGravity;
		const double n = measured.norm();
		// Still and only gravity measured: the vertical is known
		if (angularSpeed < kStillSpeed && std::fabs(n - kGravity) < kGravityTol * kGravity) {
			const Quat target = Quat::fromTwoVectors(raw.rotate(measured * (1.0 / n)), {0, 1, 0});
			m_fix = slerp(m_fix, target, std::min(1.0, dt / kSmoothTime));
		} else if (angularSpeed >= kStillSpeed) {
			// Moving: the vertical is unknown and the firmware converges faster (it did within
			// 0.6 s once the user stood up): a held correction tilted the leg the other way
			m_fix = slerp(m_fix, Quat{}, std::min(1.0, dt / kMovingFade));
		}
		if (m_fix.angle() > kDoneAngle) m_lastLarge = t;
		// The firmware converged again (or it was a real fast motion after all)
		if (t - m_lastLarge > kDoneTime || t - m_start > kMaxTime) {
			m_active = false;
			m_fix = Quat{};
		}
	}

	Quat apply(const Quat& raw) const { return m_active ? m_fix * raw : raw; }
	// The linear acceleration with gravity removed along the corrected vertical
	Vec3 linear(const Quat& raw, const Vec3& rawLinear) const {
		if (!m_active) return rawLinear;
		const Vec3 up{0, 1, 0};
		return rawLinear + (raw.conj().rotate(up) - apply(raw).conj().rotate(up)) * kGravity;
	}

private:
	static constexpr double kStillSpeed = 0.5;  // rad/s
	static constexpr double kGravityTol = 0.1;
	static constexpr double kSmoothTime = 0.15;  // s
	static constexpr double kMovingFade = 0.5;  // s
	static constexpr double kDoneAngle = 0.05;  // rad (3 deg)
	static constexpr double kDoneTime = 0.5;  // s below kDoneAngle
	static constexpr double kMaxTime = 20.0;  // s
	bool m_active = false;
	double m_start = 0, m_lastLarge = 0;
	Quat m_fix;
};

}  // namespace smt
