// SteamVR entry point
#include <openvr_driver.h>

#include <cstring>

#include "device_provider.h"

static smt::DeviceProvider g_provider;

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* interfaceName, int* returnCode) {
	if (std::strcmp(interfaceName, vr::IServerTrackedDeviceProvider_Version) == 0) return &g_provider;
	if (returnCode) *returnCode = vr::VRInitError_Init_InterfaceNotFound;
	return nullptr;
}
