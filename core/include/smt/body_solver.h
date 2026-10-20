// Lower-body pose estimation from the HMD plus 2 or 3 IMUs (ankles, optional waist).
//
// IMUs only give orientations; positions come from a kinematic model anchored to
// the HMD, solved every frame:
//
//  * Upper body: head pivot from the HMD pose, neck base below it along the torso
//    axis (not assumed vertical, so lying down works), hip center = neck base -
//    spine along the pelvis up axis (pelvis from the waist tracker, or estimated
//    from the head and the shins in ankles-only mode). A small free hip offset
//    absorbs the spine curvature. Seated or lying, the hip rests on the seat: it
//    sits behind the head chain (the trunk leans forward) and stays there while
//    the head moves, so the hip-to-head direction shows the trunk lean.
//  * Each knee is a hinge around the shin's left-right axis, so with the shin
//    orientation known, a leg has a single unknown: the knee angle phi.
//      thigh direction t = cos(phi) * shinDown + sin(phi) * shinForward
//      ankle = hipJoint + thigh * t + shin * shinDown
//  * phi is found per leg by a global search (no local minima: a wrong knee
//    solution cannot get stuck) over a cost made of:
//      - floor: ankle and knee never below the floor;
//      - straight-leg prior: among the poses the other terms allow, the leg is as
//        straight as possible (standing, sitting with feet on the floor and lying
//        down all follow from this plus the floor). It saturates: a small bend
//        costs nearly as much as a large one, so while standing a head a few cm
//        lower than calibrated (slouching, looking down) lifts the hip offset
//        (a straight leg may push the hip up by a few cm) instead of bending
//        the knees with vertical shins;
//      - hip flexion limits relative to the pelvis;
//      - stance: when the ankle IMU is still (zero velocity), the ankle stays where
//        it landed. Robust cost, so an anchor that the body cannot reach any more
//        (after lying down, a slip...) loses its pull instead of folding the leg.
//        The anchor height is the dead-reckoned one at touchdown: a foot put on a
//        bed or held up stays there. It snaps to the floor only within a few cm
//        of it, and when standing with no planted foot on the floor the lowest
//        one sinks to it (dead reckoning that stopped short);
//      - swing: the ankle follows the position dead-reckoned from its linear
//        acceleration since it left the ground, with an uncertainty growing with
//        time. This is the main term in the air: lifting a knee keeps the shin
//        vertical, only the accelerometer sees it. Lift-off is detected from the
//        accelerometer too (velocity integrated during stance), and the motion
//        already integrated seeds the swing;
//      - learned foot model (optional): the ankle stays horizontally close to the
//        position a learned model estimates from the recent motion of all the
//        trackers (robust, so its rare outliers lose their pull); stance anchors
//        follow it through the solution;
//      - support: when the head says the user is standing, at least one foot is on
//        the floor;
//      - fallback: standing, a foot with neither a stance anchor nor a swing
//        prediction (continuous bouncing or hops never pass the zero-velocity
//        test) stays near the floor below its hip joint, so the knees bend with
//        the shins instead of swinging straight legs backward;
//      - temporal smoothness (robust as well).
//    The hip offset is then solved with the legs fixed, and both are alternated.
//    Stance anchors pull the hip; a swinging leg reaches its dead-reckoned
//    position with its knee and pulls the hip only weakly (lifting a knee does
//    not lift or shift the pelvis).
//  * Outputs: waist pose, foot poses (flat on the floor when planted, following
//    the shin otherwise).
#pragma once

#include <array>
#include <optional>
#include <vector>

#include "smt/config.h"
#include "smt/linalg.h"

namespace smt {

// Offline tuning (tools/tune): a solver constant overridden by the SMT_TUNE
// environment variable ("kName=value,..."), else its default. Read once at
// startup; the engine runs without SMT_TUNE.
double tunable(const char* name, double value);

struct Pose {
	Vec3 position;
	Quat rotation;
};

struct BodyDimensions {
	double height = 1.75;
	double eyeHeight = 0;  // standing eye height above the floor
	double shin = 0, thigh = 0, ankleHeight = 0, hipWidth = 0, spine = 0;
	double headDown = 0, headBack = 0, neckDown = 0;
	double hipSide = 0;  // hip center to the right of the head chain, pelvis frame (m)

	// Hip joint height above the floor when standing straight
	double standingHipHeight() const { return ankleHeight + shin + thigh; }

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
	Vec3 waistAccel;  // waist tracker linear acceleration, world frame
	struct Leg {
		bool valid = false;
		Quat shin;  // shin segment orientation
		Vec3 accelWorld;  // linear acceleration, world frame
		double angularSpeed = 0;
		// Optional horizontal ankle position from another estimator (the learned
		// foot model), with its standard deviation (m); its height is not used
		bool hasFootMeasurement = false;
		Vec3 footMeasurement;
		double footMeasurementSigma = 0;
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

	// Solver internals, for the recorder and the offline replay
	struct Debug {
		Vec3 neck, hip0;
		std::array<Vec3, 2> hipJoints;  // unsmoothed
		double standing = 0;  // 0..1, how much the head height says "standing"
		std::array<Vec3, 2> anchors, ins;
		std::array<double, 2> insSigma{};  // 0 when the swing prediction is off
		// Touchdown this frame: swing duration (0: none), stance anchor height
		// above the floor height
		std::array<double, 2> touchdownSwing{}, touchdownHeight{};
		std::array<double, 2> hingeYaw{};  // hinge turn from the shin heading (kTibial*), rad
		std::array<double, 2> hold{};  // held raised leg weight (kHold*), and its latch
		std::array<bool, 2> holdLatched{};
		std::array<double, 2> lowerDisp{};
	} debug;
};

class BodySolver {
public:
	void configure(const BodyDimensions& dims, const Settings& settings, Mode mode);
	void reset();
	BodyOutput update(const BodyInput& rawIn);

private:
	struct LegState {
		bool initialized = false;
		double phi = 0.05;
		bool planted = false;
		double stillTime = 0;
		Vec3 anchor;  // stance position, leaks toward the solution
		Vec3 ankle;  // last solved ankle
		// Accelerometer integrated during stance (leaky): a slow knee lift keeps the
		// shin vertical and accelerates gently, the velocity still shows it
		Vec3 stanceVel, stanceDisp;
		// Swing dead reckoning from the last stance
		Vec3 insPos, insVel;
		double swingTime = 0;
		double footBlend = 0;  // 0 flat on the floor, 1 follows the shin
		double outBlend = 0;  // output foot: 0 solved ankle, 1 stance anchor
		bool footYawInit = false;
		double footYaw = 0;  // flat foot yaw, held while planted
		// Thigh direction (pelvis frame) of the last solve, and the one held when a
		// swing outlasts the dead reckoning (knee kept up, see kHoldTime)
		Vec3 thighLocal{0, -1, 0}, holdThigh{0, -1, 0};
		bool holdLatched = false;  // the leg is held up (see kHold*)
		bool onBed = false;  // reclined, planted on the hip's surface rather than the floor
		// Leg being lowered (kHold*): low-passed vertical acceleration, its leaky velocity
		// and displacement from the raw accelerometer
		double lowerAcc = 0, lowerVel = 0, lowerDisp = 0;
	};

	// One Euro filter (Casiez et al.) on a 3D point
	struct OneEuro {
		bool init = false;
		Vec3 x, dx;
		Vec3 filter(const Vec3& v, double dt, double minCutoff, double beta);
	};

	struct LegProblem;  // per-frame, per-leg cost inputs
	double legCost(const LegProblem& p, double phi) const;
	double solveLeg(const LegProblem& p) const;

	BodyDimensions m_dims;
	Settings m_settings;
	Mode m_mode = Mode::Three;
	std::array<LegState, 2> m_legs;
	Vec3 m_hipOffset;
	// Standing sway (kSway*): hip offset from the ankle-pivot line, standing head height
	Vec3 m_swayRef;
	double m_swayGate = 0;  // 0..1, smoothed
	double m_standHeadY = 0;
	double m_bothPlantedTime = 0;
	// HMD vertical speed (low-passed |vy|): walking and running bounce, holding a
	// leg up in place does not
	double m_prevHmdY = 0, m_hmdBounce = 0;
	// Raised-foot detector from micro-movements (see updateRaised): one-pole filter
	// and energy states, probability per leg that the foot is off the ground
	struct Raised {
		bool init = false, haveVel = false;
		Vec3 hmdLp, hmdPrev, hmdVel, hbLo, hbHi, wbLo, wbHi;
		double sway = 0;
		struct Leg {
			Vec3 hp, lo, hi;
			double hf = 0, lf = 0, xh = 0, xw = 0, ph = 0, pw = 0, acc = 0;
		};
		std::array<Leg, 2> leg;
		std::array<double, 2> p{};
	} m_raised;
	void updateRaised(const BodyInput& in, double dt);
	Vec3 m_seat;  // where the hip rests when seated or lying
	bool m_seatValid = false;
	// Hip moving on the seat (see kSeatMove*): waist tracker acceleration above its slow
	// mean, and the pelvis orientation the seat anchor goes with
	Vec3 m_waistAccMean;
	double m_waistActivity = 0;
	Quat m_seatPelvis;
	double m_tuck = 0;  // smoothed raised-knee pelvis tuck weight (kTuck*)
	bool m_initialized = false;
	std::array<OneEuro, 3> m_filters;  // left foot, right foot, waist
	std::array<Vec3, 2> m_smoothedFeet;
	Vec3 m_smoothedWaist;
};

}  // namespace smt
