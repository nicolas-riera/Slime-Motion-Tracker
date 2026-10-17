// Learned ankle position estimate (tools/ml): a small MLP trained on Kinect
// ground truth of the user's own sessions, run once per engine frame on the body
// solver inputs.
//
// Features (identical to tools/ml/dataset.py): over a causal window of past
// frames (lags up to ~2 s), the HMD position (horizontal relative to the HMD
// now, height above the floor), the up (Y) and back (Z) axes of the HMD, pelvis
// and both shins, both shins' world linear accelerations (clipped per axis,
// causal low-pass) and angular speeds, all expressed in the heading frame of the
// pelvis now (yaw of its left-right axis). The network outputs both ankles in
// that frame (x/z relative to the HMD, y above the floor), optionally with a
// per-axis Laplace scale (uncertainty).
//
// Usage: load() once, then update() every engine frame, valid or not (the
// accelerometer low-pass and the history must see every frame). Inference
// costs ~0.05 ms (about 100k multiply-adds).
#pragma once

#include <array>
#include <string>
#include <vector>

#include "slimelite/body_solver.h"
#include "slimelite/linalg.h"

namespace slimelite {

struct FootEstimate {
	bool valid = false;
	std::array<Vec3, 2> ankle;  // world (raw tracking space), 0 left, 1 right
	// Per-axis standard deviation (m) in the heading frame (x right, y up,
	// z back), when the model predicts its uncertainty; 0 otherwise
	std::array<Vec3, 2> sigma;
	double heading = 0;  // rad, heading frame yaw (Quat::yaw(heading))
};

class FootModel {
public:
	// Loads a model exported by tools/ml/export.py; false (and loaded() false)
	// on any error, with a message in error()
	bool load(const std::string& path);
	bool loaded() const { return m_loaded; }
	const std::string& error() const { return m_error; }
	void reset();

	// now: frame time (s, any monotonic clock, e.g. the engine's frame time)
	FootEstimate update(const BodyInput& in, double now);

	// Raw network input of the last valid update (for tests)
	const std::vector<double>& lastFeatures() const { return m_features; }

private:
	struct Frame {
		double t = 0;
		bool valid = false;
		Vec3 hmd;  // y above the floor
		std::array<Vec3, 8> axes;  // Y, Z axes of hmd, pelvis, left shin, right shin
		std::array<Vec3, 2> accel;  // low-passed
		std::array<double, 2> gyro{};
	};
	struct Layer {
		int in = 0, out = 0;
		std::vector<double> w;  // out x in, row major
		std::vector<double> b;
	};

	const Frame* frameAt(double t) const;  // last frame at or before t

	bool m_loaded = false;
	std::string m_error;
	std::vector<double> m_lags;
	double m_accClip = 40, m_accTau = 0.04;
	std::vector<double> m_mu, m_sd, m_ymu;
	std::vector<Layer> m_layers;
	bool m_hetero = false;

	std::vector<Frame> m_ring;  // circular history
	size_t m_head = 0, m_count = 0;
	bool m_accInit = false;
	std::array<Vec3, 2> m_accFiltered;
	double m_lastTime = 0;
	std::vector<double> m_features, m_a, m_b;
};

}  // namespace slimelite
