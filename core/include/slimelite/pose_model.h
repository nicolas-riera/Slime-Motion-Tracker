// Learned full-body pose model (tools/pose_model): HMD-Poser (MIT,
// github.com/Pico-AI-Team/HMD-Poser) retrained on AMASS with our sensor layout,
// run causally at 60 Hz on the HMD, both controllers and the trackers.
//
// Inputs (identical to tools/pose_model/features.py): global rotations, rotation
// velocities and positions of the head, both wrists (controller poses through a
// rotation calibrated at the body calibration stand pose), the shin and pelvis
// segment rotations and world linear accelerations of their trackers, in the
// AMASS frame (Z up, floor at 0). Without a waist tracker its inputs are zeroed
// (the model was trained for it). Output: the SMPL body pose; the runtime only
// needs both ankles, placed with the user's skeleton (SMPL+H neutral offsets
// scaled to the user's height) hanging from the head.
//
// Usage: load() once, calibrateWrists() at the stand pose, then update() every
// engine frame. The network steps at 60 Hz (sample and hold), ~70M multiply-adds
// per step.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "slimelite/body_solver.h"
#include "slimelite/linalg.h"

namespace slimelite {

struct PoseModelInput {
	double floorY = 0;
	bool hmdValid = false;
	Pose hmd;
	std::array<bool, 2> handValid{};  // controllers, 0 left, 1 right
	std::array<Pose, 2> hands;
	bool hasWaist = false;
	Quat waist;
	Vec3 waistAccel;  // world linear acceleration (m/s^2, no gravity)
	std::array<bool, 2> legValid{};
	std::array<Quat, 2> shin;
	std::array<Vec3, 2> legAccel;
};

struct PoseEstimate {
	bool valid = false;
	std::array<Vec3, 2> ankle;  // world (raw tracking space), 0 left, 1 right
	Vec3 pelvis;
	double warmup = 0;  // s of network history since the last reset
};

// Fusion with the body solver (tools/pose_model, Kinect-tuned): the model places
// the feet better while the head is high (standing, walking, getting up), the
// solver while it is low (seated, squatting, kneeling: it knows the planted feet).
// The weight of the model ramps with the head height over the standing eye
// height; the model ankles never go below minAnkleHeight above the floor.
struct PoseFusionSettings {
	double ratioLow = 0.80, ratioHigh = 1.05;
	double minAnkleHeight = 0.10;  // m
	double minWarmup = 1.0;  // s of network history before it is trusted

	// Tuned on the Kinect sessions (tools/pose_model/fuse_tune.py): with the waist
	// tracker the solver is better as soon as the head drops a little; ankles only
	// it knows less and the model takes over from half the eye height
	static PoseFusionSettings forWaist(bool hasWaist) {
		PoseFusionSettings s;
		if (!hasWaist) {
			s.ratioLow = 0.50;
			s.ratioHigh = 0.60;
		}
		return s;
	}
};

// Blends the estimate into out.feet positions; returns the model weight (0..1)
double fusePose(const PoseEstimate& e, const Pose& hmd, double floorY, double eyeHeight, BodyOutput& out,
				const PoseFusionSettings& s = {});

class PoseModel {
public:
	bool load(const std::string& path);
	bool loaded() const { return m_loaded; }
	const std::string& error() const { return m_error; }
	const std::string& name() const { return m_name; }

	// User height (m), for the skeleton
	void setHeight(double height) { m_height = height; }
	// Controller-to-wrist rotations, with the user standing straight, arms
	// hanging, palms against the thighs (the body calibration stand pose)
	void calibrateWrists(const Pose& hmd, const std::array<Pose, 2>& hands);
	static std::array<Quat, 2> wristCalibrationFor(const Pose& hmd, const std::array<Pose, 2>& hands);
	void setWristCalibration(const std::array<Quat, 2>& cal);
	bool wristsCalibrated() const { return m_wristsCalibrated; }
	const std::array<Quat, 2>& wristCalibration() const { return m_wristCal; }

	void reset();
	// Every engine frame; steps the network when a 60 Hz tick has passed
	PoseEstimate update(const PoseModelInput& in, double now);
	// One network step on this input (the 60 Hz sampling done by the caller)
	PoseEstimate step(const PoseModelInput& in);

	const std::vector<float>& lastFeatures() const { return m_x; }

private:
	struct Linear {
		int in = 0, out = 0;
		std::vector<uint16_t> w;  // out x in, float16 (half the memory traffic)
		std::vector<float> b;
		void apply(const float* x, float* y) const { applyBatch(x, 1, y); }
		// n inputs of size in (contiguous) -> n outputs of size out; the
		// weights are read once for all of them
		void applyBatch(const float* x, int n, float* y) const;
	};
	struct Lstm {
		Linear ih, hh;  // 4*hidden rows, gates i f g o
	};
	struct EncoderLayer {
		Linear inProj, outProj, ff1, ff2;
		std::vector<float> n1w, n1b, n2w, n2b;
	};
	struct Group {
		std::vector<Linear> emb;
		std::vector<std::vector<int>> slices;  // feature indices per embedding
	};

	void forward(float* pose);
	void encoder(const EncoderLayer& l, float* x);

	bool m_loaded = false;
	std::string m_error, m_name;
	int m_hidden = 256, m_heads = 8, m_ffn = 2048, m_blocks = 2, m_layers = 3;
	std::array<Group, 8> m_groups;
	std::vector<float> m_normW, m_normB;
	std::vector<std::array<Lstm, 8>> m_lstm;  // per block
	std::vector<std::vector<EncoderLayer>> m_spatial;  // per block
	Linear m_pose1, m_pose2;
	std::array<Vec3, 22> m_rest;  // offsets to the parent at model height
	std::array<int, 22> m_parents{};
	double m_modelHeight = 1.7, m_height = 1.75, m_hz = 60;

	// Runtime state
	std::vector<float> m_h, m_c;  // [block][token][hidden]
	bool m_havePrev = false;
	std::array<Mat3, 6> m_prevRot;
	std::array<Vec3, 3> m_prevPos;
	std::array<Mat3, 2> m_prevHandInHead;
	std::array<Vec3, 2> m_prevHandPos;
	std::array<Quat, 2> m_wristCal;
	bool m_wristsCalibrated = false;
	double m_nextTick = 0, m_lastStep = -1, m_warm = 0;
	PoseEstimate m_last;
	std::vector<float> m_x, m_tokens, m_buf, m_buf2, m_pose;
};

}  // namespace slimelite
