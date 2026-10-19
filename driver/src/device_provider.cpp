#include "device_provider.h"

#include <windows.h>

#include <chrono>
#include <cstring>

namespace smt {

namespace {

const char* kSerials[SMT_DEVICE_COUNT] = {"SMT-WAIST", "SMT-LFOOT", "SMT-RFOOT"};
const char* kRoles[SMT_DEVICE_COUNT] = {"TrackerRole_Waist", "TrackerRole_LeftFoot",
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
	if (auto* l = vr::VRDriverLog()) l->Log(("[slime_motion_tracker] " + msg + "\n").c_str());
}

double nowSeconds() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* ctx) {
	VR_INIT_SERVER_DRIVER_CONTEXT(ctx);

	// Registered with SteamVR in RunFrame, once the trackers behind them connect
	for (int i = 0; i < SMT_DEVICE_COUNT; i++)
		m_devices[i] = std::make_unique<VirtualTracker>(kSerials[i], kRoles[i]);

	auto dir = moduleDirectory();
	std::u8string u8 = dir.u8string();
	std::string dirUtf8(u8.begin(), u8.end());
	m_engine = std::make_unique<EngineLoader>(dir / "smt_engine.dll", dirUtf8, driverLog);
	return vr::VRInitError_None;
}

void DeviceProvider::addDevice(int slot) {
	auto& dev = *m_devices[slot];
	dev.setAdded();  // even on failure: do not retry every frame
	// Assign the SteamVR tracker role so games pick them up without manual setup
	std::string key = "/devices/slime_motion_tracker/" + dev.serial();
	vr::VRSettings()->SetString("trackers", key.c_str(), dev.role().c_str());
	if (vr::VRServerDriverHost()->TrackedDeviceAdded(dev.serial().c_str(), vr::TrackedDeviceClass_GenericTracker,
													  &dev))
		driverLog(dev.serial() + " added");
	else
		driverLog(dev.serial() + ": TrackedDeviceAdded failed");
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

	static SmtFrameInput in;  // large, keep it off the stack
	in = SmtFrameInput{};
	in.now = now;
	vr::TrackedDevicePose_t poses[SMT_MAX_DEVICES];
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, poses, SMT_MAX_DEVICES);
	in.hmdValid = poses[0].bPoseIsValid ? 1 : 0;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 4; c++) in.hmd[r][c] = poses[0].mDeviceToAbsoluteTracking.m[r][c];

	if (now - m_lastInfoRefresh > 3.0) {
		m_lastInfoRefresh = now;
		SmtBattery batteries[SMT_DEVICE_COUNT] = {};
		if (m_engine->batteries(batteries))
			for (int i = 0; i < SMT_DEVICE_COUNT; i++) m_devices[i]->updateBattery(batteries[i]);
		auto props = vr::VRProperties();
		for (uint32_t i = 0; i < SMT_MAX_DEVICES; i++) {
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
	in.deviceCount = SMT_MAX_DEVICES;
	for (int i = 0; i < SMT_MAX_DEVICES; i++) {
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

	SmtFrameOutput out{};
	m_engine->frame(in, out);
	SmtDeviceStatus status[SMT_DEVICE_COUNT] = {};
	bool haveStatus = m_engine->deviceStatus(status);
	for (int i = 0; i < SMT_DEVICE_COUNT; i++) {
		auto& dev = *m_devices[i];
		SmtDeviceStatus s = status[i];
		// Older engine without the status: register on the first valid pose, then
		// never report it disconnected
		if (!haveStatus) s.connected = s.calibrated = (out.devices[i].valid || dev.isAdded()) ? 1 : 0;
		if (!dev.isAdded()) {
			if (!s.connected) continue;
			addDevice(i);
		}
		if ((s.connected != 0) != m_connected[i]) {
			m_connected[i] = s.connected != 0;
			driverLog(dev.serial() + (m_connected[i] ? " connected" : " disconnected"));
		}
		dev.updatePose(out.devices[i], s, out.sendVelocity != 0);
	}
}

}  // namespace smt
