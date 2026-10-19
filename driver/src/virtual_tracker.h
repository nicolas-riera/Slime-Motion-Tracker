// A SteamVR generic tracker whose pose is computed by the body solver
#pragma once

#include <openvr_driver.h>

#include <string>

#include "smt/engine_api.h"

namespace smt {

class VirtualTracker : public vr::ITrackedDeviceServerDriver {
public:
	// role: SteamVR tracker role, e.g. "TrackerRole_Waist"
	VirtualTracker(std::string serial, std::string role);

	const std::string& serial() const { return m_serial; }
	const std::string& role() const { return m_role; }
	bool isActive() const { return m_id != vr::k_unTrackedDeviceIndexInvalid; }

	void updatePose(const SmtPose& pose, bool sendVelocity);
	void updateBattery(const SmtBattery& battery);

	vr::EVRInitError Activate(uint32_t unObjectId) override;
	void Deactivate() override;
	void EnterStandby() override {}
	void* GetComponent(const char*) override { return nullptr; }
	void DebugRequest(const char*, char* response, uint32_t size) override {
		if (size) response[0] = 0;
	}
	vr::DriverPose_t GetPose() override { return m_pose; }

private:
	std::string m_serial;
	std::string m_role;
	uint32_t m_id = vr::k_unTrackedDeviceIndexInvalid;
	vr::DriverPose_t m_pose{};
};

}  // namespace smt
