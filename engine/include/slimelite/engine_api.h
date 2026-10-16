/* C ABI between the SteamVR driver host (driver_slimevr_lite.dll, never reloaded)
 * and the engine (slimelite_engine.dll, hot-reloaded when the file changes).
 *
 * The host only talks to SteamVR: it registers the virtual trackers, reads the HMD
 * pose and publishes the poses the engine computes. Everything else (network,
 * mag fusion, calibration, body solver, overlay link) lives in the engine.
 *
 * Bump SL_ENGINE_API_VERSION on any change to this file: the host refuses to load
 * an engine with a different version and keeps the current one.
 */
#ifndef SLIMELITE_ENGINE_API_H
#define SLIMELITE_ENGINE_API_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SL_ENGINE_API_VERSION 1

/* Virtual tracker slots, in the order the host registers them */
enum { SL_DEVICE_WAIST = 0, SL_DEVICE_LEFT_FOOT = 1, SL_DEVICE_RIGHT_FOOT = 2, SL_DEVICE_COUNT = 3 };

typedef struct SlEngineHost {
	void (*log)(const char* message);
	const char* moduleDirectory; /* UTF-8, folder of the driver DLL (bin/win64) */
} SlEngineHost;

typedef struct SlPose {
	int valid;
	double position[3];
	double rotation[4]; /* w, x, y, z */
	double velocity[3];
} SlPose;

#define SL_MAX_DEVICES 16

/* Every SteamVR device (HMD, controllers, other trackers), for recording */
typedef struct SlDevicePose {
	int connected;
	int valid;
	int deviceClass; /* vr::ETrackedDeviceClass */
	int controllerRole; /* vr::ETrackedControllerRole */
	char serial[48];
	double pose[3][4]; /* raw tracking space, row-major 3x4 */
	double velocity[3];
	double angularVelocity[3];
} SlDevicePose;

typedef struct SlFrameInput {
	double now; /* seconds, monotonic (steady clock) */
	int hmdValid;
	double hmd[3][4]; /* raw tracking space, row-major 3x4 */
	int deviceCount;
	SlDevicePose devices[SL_MAX_DEVICES];
} SlFrameInput;

typedef struct SlFrameOutput {
	SlPose devices[SL_DEVICE_COUNT];
	int sendVelocity;
} SlFrameOutput;

/* Exported by the engine */
typedef int (*SlEngineApiVersionFn)(void);
/* handover: JSON written by the previous engine's destroy, or NULL on first start */
typedef void* (*SlEngineCreateFn)(const SlEngineHost* host, const char* handover);
typedef void (*SlEngineFrameFn)(void* engine, const SlFrameInput* in, SlFrameOutput* out);
/* Writes the handover JSON (NUL-terminated, truncated to capacity) and frees the engine.
 * Returns the full JSON length. */
typedef size_t (*SlEngineDestroyFn)(void* engine, char* handover, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
