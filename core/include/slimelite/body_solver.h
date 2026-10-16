// Lower-body pose estimation from the HMD plus 2 or 3 IMUs (ankles, optional waist).
//
// IMUs only give orientations; positions come from a kinematic model anchored to
// the HMD, solved every frame:
//
//  * Neck point from the HMD pose, hip center = neck - spine along the pelvis up
//    axis (pelvis from the waist tracker, or estimated in ankles-only mode).
//  * Each knee is a hinge around the shin's left-right axis, so with the shin
//    orientation known, a leg has a single unknown: the knee angle phi.
//      thigh direction t = cos(phi) * shinDown + sin(phi) * shinForward
//      ankle = hipJoint + thigh * t + shin * shinDown
//  * Unknowns x = [phiL, phiR, hip offset (3)] minimize, by Gauss-Newton:
//      - planted foot: when the ankle IMU is still (zero velocity, ZUPT), the ankle
//        stays where it was when it landed, snapped to the floor when close;
//      - swing foot: stays close to the position predicted by integrating the
//        ankle's linear acceleration from its last state;
//      - never below the floor; knee angle limits;
//      - hip offset prior (small in 3-tracker mode, looser with ankles only) and
//        temporal smoothness.
//  * Outputs: waist pose, foot poses (flat on the floor when planted, following
//    the shin in the air).
#pragma once

#include <array>
#include <optional>
#include <vector>

#include "slimelite/config.h"
#include "slimelite/linalg.h"

namespace slimelite {

struct Pose {
	Vec3 position;
	Quat rotation;
};

struct BodyDimensions {
	double height = 1.75;
	double shin = 0, thigh = 0, ankleHeight = 0, hipWidth = 0, spine = 0;
	double headDown = 0, headBack = 0, neckDown = 0;

	// Anthropometric ratios (Drillis & Contini); eyeHeight, when measured,
	// fixes the spine length so the chain matches the real standing HMD height
	static BodyDimensions fromHeight(double height, std::optional<double> eyeHeight);
};

struct BodyInput {
	double dt = 0;
	bool hmdValid = false;
	Pose hmd;
	double floorY = 0;

	bool hasWaist = false;
	Quat waist;  // pelvis segment orientation
	struct Leg {
		bool valid = false;
		Quat shin;  // shin segment orientation
		Vec3 accelWorld;  // linear acceleration, world frame
		double angularSpeed = 0;
	};
	std::array<Leg, 2> legs;  // 0 left, 1 right
};

struct BodyOutput {
	bool valid = false;
	Pose waist;
	std::array<Pose, 2> feet;
	std::array<Vec3, 2> footVelocity;
	Vec3 waistVelocity;
	std::array<bool, 2> planted{};
	std::array<Vec3, 2> knees;  // for debugging/visualization
	std::array<double, 2> kneeAngles{};
};

class BodySolver {
public:
	void configure(const BodyDimensions& dims, const Settings& settings, Mode mode);
	void reset();
	BodyOutput update(const BodyInput& in);

private:
	struct LegState {
		double phi = 0.1;
		bool planted = false;
		double stillTime = 0;
		Vec3 anchor;
		Vec3 ankle;
		Vec3 velocity;
		bool initialized = false;
		double footBlend = 0;  // 0 flat on the floor, 1 follows the shin
	};

	// One Euro filter (Casiez et al.) on a 3D point
	struct OneEuro {
		bool init = false;
		Vec3 x, dx;
		Vec3 filter(const Vec3& v, double dt, double minCutoff, double beta);
	};

	struct Frame;  // per-update precomputed geometry
	void residuals(const Frame& f, const double* x, std::vector<double>& r) const;

	BodyDimensions m_dims;
	Settings m_settings;
	Mode m_mode = Mode::Three;
	std::array<LegState, 2> m_legs;
	Vec3 m_hipOffset;
	bool m_initialized = false;
	std::array<OneEuro, 3> m_filters;  // left foot, right foot, waist
	std::array<Vec3, 2> m_smoothedFeet;
	Vec3 m_smoothedWaist;
};

}  // namespace slimelite
