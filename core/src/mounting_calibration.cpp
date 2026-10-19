#include "smt/mounting_calibration.h"

#include <cstdio>

namespace smt {

MountingResult solveMounting(const Quat& qStand, const Quat& qSki, double bodyYaw,
							 double minBendDegrees) {
	MountingResult r;
	// World-frame rotation from standing to ski pose, independent of the mounting
	Vec3 rv = (qSki * qStand.conj()).toRotationVector();
	double bend = rv.norm();
	r.bendDegrees = rad2deg(bend);
	if (r.bendDegrees < minBendDegrees) {
		char msg[128];
		std::snprintf(msg, sizeof(msg), "not enough bend in the ski pose (%.0f deg, need %.0f)",
					  r.bendDegrees, minBendDegrees);
		r.error = msg;
		return r;
	}
	Vec3 axis = rv / bend;
	if (std::fabs(axis.y) > 0.6) {
		r.error = "the ski pose rotated the tracker around the vertical axis: bend forward, do not twist";
		return r;
	}
	// Leaning forward rotates around axis = up x forward, so forward = axis x up
	Vec3 forward = axis.cross({0, 1, 0});
	forward.y = 0;
	forward = forward.normalized();
	double forwardYawInTrackerWorld = std::atan2(-forward.x, -forward.z);

	r.yawAlign = wrapAngle(bodyYaw - forwardYawInTrackerWorld);
	Quat aligned = Quat::yaw(r.yawAlign) * qStand;
	r.mounting = (aligned.conj() * Quat::yaw(bodyYaw)).normalized();
	r.ok = true;
	return r;
}

MountingResult solveQuickReset(const Quat& qStand, const Quat& mounting, double bodyYaw) {
	MountingResult r;
	// Heading of the segment's forward axis with the old mounting
	double segYaw = (qStand * mounting).yawAngle();
	r.yawAlign = wrapAngle(bodyYaw - segYaw);
	// Refresh the tilt part (strap may have moved) while keeping that heading
	r.mounting = (qStand.conj() * Quat::yaw(segYaw)).normalized();
	r.ok = true;
	return r;
}

}  // namespace smt
