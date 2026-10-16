// Mounting calibration ("stand straight" then "ski pose", like SlimeVR).
//
// Unknowns per tracker: the mounting rotation M (tracker frame -> body segment
// frame) and the yaw alignment psi between the tracker's world frame (magnetic
// north if the mag works, else the device's boot frame) and SteamVR's world:
//
//     q_segment = Ry(psi) * q_tracker * M
//
// Standing straight gives q_segment = Ry(bodyYaw) (segment upright, facing where
// the HMD faces) but cannot separate psi from the mounting yaw (a tracker on the
// side of the leg looks like a tracker on the front with a different psi).
// The ski pose (knees bent, leaning forward) rotates the shins and the pelvis
// around the body's left-right axis: the rotation axis between the two captures,
// seen in the tracker's world frame, tells where "forward" is, which fixes psi.
// Segment frame: +Y up along the segment, -Z forward, +X right.
#pragma once

#include <string>

#include "slimelite/linalg.h"

namespace slimelite {

struct MountingResult {
	bool ok = false;
	std::string error;
	Quat mounting;
	double yawAlign = 0;
	double bendDegrees = 0;
};

// qStand / qSki: tracker orientations in its own world frame. bodyYaw: SteamVR
// yaw the user faces (from the HMD at the stand capture).
MountingResult solveMounting(const Quat& qStand, const Quat& qSki, double bodyYaw,
							 double minBendDegrees);

// Quick reset: keeps the mounting heading, re-solves psi and the mounting tilt
// from a standing capture.
MountingResult solveQuickReset(const Quat& qStand, const Quat& mounting, double bodyYaw);

}  // namespace slimelite
