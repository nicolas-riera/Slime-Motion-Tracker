// Loads smt_engine.dll and reloads it when the file changes.
// The DLL is copied to %TEMP% before loading so the build can overwrite it.
#pragma once

#include <filesystem>
#include <functional>
#include <string>

#include "smt/engine_api.h"

namespace smt {

class EngineLoader {
public:
	using Logger = std::function<void(const std::string&)>;

	EngineLoader(std::filesystem::path enginePath, std::string moduleDirectory, Logger log);
	~EngineLoader();

	bool loaded() const { return m_engine != nullptr; }
	// Checks the file (at most twice a second) and hot-reloads it when it changed
	void poll(double now);
	void frame(const SmtFrameInput& in, SmtFrameOutput& out);
	// False when the engine does not export it (older engine)
	bool batteries(SmtBattery out[SMT_DEVICE_COUNT]);
	// All disconnected when no engine is loaded; false when the engine does not
	// export it (older engine)
	bool deviceStatus(SmtDeviceStatus out[SMT_DEVICE_COUNT]);
	void unload();

private:
	struct Module {
		void* dll = nullptr;
		std::filesystem::path copy;
		SmtEngineCreateFn create = nullptr;
		SmtEngineFrameFn frame = nullptr;
		SmtEngineDestroyFn destroy = nullptr;
		SmtEngineBatteriesFn batteries = nullptr;  // optional
		SmtEngineDeviceStatusFn deviceStatus = nullptr;  // optional
	};
	bool loadModule(Module& m);
	void freeModule(Module& m);
	std::string destroyEngine();

	std::filesystem::path m_path;
	std::string m_moduleDirectory;
	Logger m_log;
	SmtEngineHost m_host{};
	Module m_module;
	void* m_engine = nullptr;
	std::filesystem::file_time_type m_loadedTime{};
	double m_lastCheck = 0;
	int m_generation = 0;
};

}  // namespace smt
