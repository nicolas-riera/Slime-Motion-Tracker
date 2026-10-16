#include "device_provider.h"

#include <windows.h>

#include <chrono>
#include <cstring>

namespace slimelite {

namespace {

const char* kSerials[SL_DEVICE_COUNT] = {"SLV-WAIST", "SLV-LFOOT", "SLV-RFOOT"};
const char* kRoles[SL_DEVICE_COUNT] = {"TrackerRole_Waist", "TrackerRole_LeftFoot",
									   "TrackerRole_RightFoot"};

std::filesystem::path moduleDirectory() {
	HMODULE module = nullptr;
	GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
					   reinterpret_cast<LPCWSTR>(&moduleDirectory), &module);
	wchar_t path[MAX_PATH];
	GetModuleFileNameW(module, path, MAX_PATH);
	return std::filesystem::path(path).parent_path();
}

void driverLog(const std::string& msg) {
	if (auto* l = vr::VRDriverLog()) l->Log(("[slimevr_lite] " + msg + "\n").c_str());
}

double nowSeconds() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* ctx) {
	VR_INIT_SERVER_DRIVER_CONTEXT(ctx);

	for (int i = 0; i < SL_DEVICE_COUNT; i++) {
		m_devices[i] = std::make_unique<VirtualTracker>(kSerials[i], kRoles[i]);
		// Assign the SteamVR tracker role so games pick them up without manual setup
		std::string key = std::string("/devices/slimevr_lite/") + kSerials[i];
		vr::VRSettings()->SetString("trackers", key.c_str(), kRoles[i]);
		vr::VRServerDriverHost()->TrackedDeviceAdded(kSerials[i], vr::TrackedDeviceClass_GenericTracker,
													  m_devices[i].get());
	}

	auto dir = moduleDirectory();
	std::u8string u8 = dir.u8string();
	std::string dirUtf8(u8.begin(), u8.end());
	m_engine = std::make_unique<EngineLoader>(dir / "slimelite_engine.dll", dirUtf8, driverLog);
	return vr::VRInitError_None;
}

void DeviceProvider::Cleanup() {
	m_engine.reset();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void DeviceProvider::RunFrame() {
	vr::VREvent_t ev;
	while (vr::VRServerDriverHost()->PollNextEvent(&ev, sizeof(ev))) {}

	double now = nowSeconds();
	m_engine->poll(now);

	static SlFrameInput in;  // large, keep it off the stack
	in = SlFrameInput{};
	in.now = now;
	vr::TrackedDevicePose_t poses[SL_MAX_DEVICES];
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, poses, SL_MAX_DEVICES);
	in.hmdValid = poses[0].bPoseIsValid ? 1 : 0;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 4; c++) in.hmd[r][c] = poses[0].mDeviceToAbsoluteTracking.m[r][c];

	if (now - m_lastInfoRefresh > 3.0) {
		m_lastInfoRefresh = now;
		auto props = vr::VRProperties();
		for (uint32_t i = 0; i < SL_MAX_DEVICES; i++) {
			auto& info = m_deviceInfo[i];
			info = DeviceInfo{};
			if (!poses[i].bDeviceIsConnected) continue;
			auto c = props->TrackedDeviceToPropertyContainer(i);
			vr::ETrackedPropertyError err;
			info.deviceClass = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &err);
			info.role = props->GetInt32Property(c, vr::Prop_ControllerRoleHint_Int32, &err);
			props->GetStringProperty(c, vr::Prop_SerialNumber_String, info.serial, sizeof(info.serial), &err);
		}
	}
	in.deviceCount = SL_MAX_DEVICES;
	for (int i = 0; i < SL_MAX_DEVICES; i++) {
		auto& d = in.devices[i];
		const auto& p = poses[i];
		d.connected = p.bDeviceIsConnected ? 1 : 0;
		d.valid = p.bPoseIsValid ? 1 : 0;
		d.deviceClass = m_deviceInfo[i].deviceClass;
		d.controllerRole = m_deviceInfo[i].role;
		std::memcpy(d.serial, m_deviceInfo[i].serial, sizeof(d.serial));
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 4; c++) d.pose[r][c] = p.mDeviceToAbsoluteTracking.m[r][c];
		for (int k = 0; k < 3; k++) {
			d.velocity[k] = p.vVelocity.v[k];
			d.angularVelocity[k] = p.vAngularVelocity.v[k];
		}
	}

	SlFrameOutput out{};
	m_engine->frame(in, out);
	for (int i = 0; i < SL_DEVICE_COUNT; i++) m_devices[i]->updatePose(out.devices[i], out.sendVelocity != 0);
}

}  // namespace slimelite
