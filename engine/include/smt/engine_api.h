/* C ABI between the SteamVR driver host (driver_slime_motion_tracker.dll, never reloaded)
 * and the engine (smt_engine.dll, hot-reloaded when the file changes).
 *
 * The host only talks to SteamVR: it registers the virtual trackers, reads the HMD
 * pose and publishes the poses the engine computes. Everything else (network,
 * mag fusion, calibration, body solver, overlay link) lives in the engine.
 *
 * Bump SMT_ENGINE_API_VERSION on any change to this file, except new optional
 * exports: the host refuses to load an engine with a different version and keeps
 * the current one. Optional exports are looked up by name; a host or an engine
 * without them keeps working, so they can be added without a SteamVR restart.
 */
#ifndef SMT_ENGINE_API_H
#define SMT_ENGINE_API_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SMT_ENGINE_API_VERSION 1

/* Virtual tracker slots, in the order the host registers them */
enum { SMT_DEVICE_WAIST = 0, SMT_DEVICE_LEFT_FOOT = 1, SMT_DEVICE_RIGHT_FOOT = 2, SMT_DEVICE_COUNT = 3 };

typedef struct SmtEngineHost {
	void (*log)(const char* message);
	const char* moduleDirectory; /* UTF-8, folder of the driver DLL (bin/win64) */
} SmtEngineHost;

typedef struct SmtPose {
	int valid;
	double position[3];
	double rotation[4]; /* w, x, y, z */
	double velocity[3];
} SmtPose;

#define SMT_MAX_DEVICES 16

/* Every SteamVR device (HMD, controllers, other trackers), for recording */
typedef struct SmtDevicePose {
	int connected;
	int valid;
	int deviceClass; /* vr::ETrackedDeviceClass */
	int controllerRole; /* vr::ETrackedControllerRole */
	char serial[48];
	double pose[3][4]; /* raw tracking space, row-major 3x4 */
	double velocity[3];
	double angularVelocity[3];
} SmtDevicePose;

typedef struct SmtFrameInput {
	double now; /* seconds, monotonic (steady clock) */
	int hmdValid;
	double hmd[3][4]; /* raw tracking space, row-major 3x4 */
	int deviceCount;
	SmtDevicePose devices[SMT_MAX_DEVICES];
} SmtFrameInput;

typedef struct SmtFrameOutput {
	SmtPose devices[SMT_DEVICE_COUNT];
	int sendVelocity;
} SmtFrameOutput;

/* Exported by the engine */
typedef int (*SmtEngineApiVersionFn)(void);
/* handover: JSON written by the previous engine's destroy, or NULL on first start */
typedef void* (*SmtEngineCreateFn)(const SmtEngineHost* host, const char* handover);
typedef void (*SmtEngineFrameFn)(void* engine, const SmtFrameInput* in, SmtFrameOutput* out);
/* Writes the handover JSON (NUL-terminated, truncated to capacity) and frees the engine.
 * Returns the full JSON length. */
typedef size_t (*SmtEngineDestroyFn)(void* engine, char* handover, size_t capacity);

/* Optional export "sl_engine_batteries": battery of the tracker behind each
 * virtual tracker slot (SMT_DEVICE_*) */
typedef struct SmtBattery {
	int known;  /* 0: no battery information for this slot */
	float level;  /* 0..1 */
	int charging;
} SmtBattery;
typedef void (*SmtEngineBatteriesFn)(void* engine, SmtBattery out[SMT_DEVICE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
