// SteamVR server driver host. Registers the virtual trackers (once the trackers
// behind them connect) and forwards every frame to the hot-reloadable engine (see
// engine_api.h). Kept minimal on purpose: changing this file requires restarting
// SteamVR, changing the engine does not.
#pragma once

#include <openvr_driver.h>

#include <array>
#include <memory>

#include "engine_loader.h"
#include "virtual_tracker.h"

namespace smt {

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
	vr::EVRInitError Init(vr::IVRDriverContext* ctx) override;
	void Cleanup() override;
	const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
	void RunFrame() override;
	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}

private:
	void addDevice(int slot);

	std::array<std::unique_ptr<VirtualTracker>, SMT_DEVICE_COUNT> m_devices;
	std::array<bool, SMT_DEVICE_COUNT> m_connected{};  // last state reported, for the log
	std::unique_ptr<EngineLoader> m_engine;
	// Device class/role/serial, refreshed every few seconds (property reads are not free)
	struct DeviceInfo {
		int deviceClass = 0;
		int role = 0;
		char serial[48] = {};
	};
	std::array<DeviceInfo, SMT_MAX_DEVICES> m_deviceInfo{};
	double m_lastInfoRefresh = 0;
};

}  // namespace smt
