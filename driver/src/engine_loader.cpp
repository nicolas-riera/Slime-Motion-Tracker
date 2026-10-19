#include "engine_loader.h"

#include <windows.h>

#include <vector>

namespace smt {

namespace {

EngineLoader::Logger* g_logger = nullptr;

void hostLog(const char* msg) {
	if (g_logger) (*g_logger)(msg);
}

std::filesystem::path tempDirectory() {
	std::error_code ec;
	auto dir = std::filesystem::temp_directory_path(ec) / "slime_motion_tracker_engine";
	std::filesystem::create_directories(dir, ec);
	return dir;
}

}  // namespace

EngineLoader::EngineLoader(std::filesystem::path enginePath, std::string moduleDirectory, Logger log)
	: m_path(std::move(enginePath)), m_moduleDirectory(std::move(moduleDirectory)), m_log(std::move(log)) {
	g_logger = &m_log;
	m_host.log = hostLog;
	m_host.moduleDirectory = m_moduleDirectory.c_str();

	// Leftover copies from previous sessions
	std::error_code ec;
	for (const auto& e : std::filesystem::directory_iterator(tempDirectory(), ec))
		std::filesystem::remove(e.path(), ec);

	Module m;
	if (loadModule(m)) {
		m_engine = m.create(&m_host, nullptr);
		if (m_engine) {
			m_module = m;
			m_log("engine loaded: " + m_path.string());
		} else {
			m_log("engine create failed");
			freeModule(m);
		}
	}
}

EngineLoader::~EngineLoader() {
	unload();
	g_logger = nullptr;
}

bool EngineLoader::loadModule(Module& m) {
	std::error_code ec;
	if (!std::filesystem::exists(m_path, ec)) {
		m_log("engine not found: " + m_path.string());
		return false;
	}
	m.copy = tempDirectory() / ("engine_" + std::to_string(GetCurrentProcessId()) + "_"
								+ std::to_string(++m_generation) + ".dll");
	if (!std::filesystem::copy_file(m_path, m.copy, std::filesystem::copy_options::overwrite_existing, ec)) {
		m_log("cannot copy the engine: " + ec.message());
		return false;
	}
	HMODULE dll = LoadLibraryW(m.copy.c_str());
	if (!dll) {
		m_log("LoadLibrary failed for the engine, error " + std::to_string(GetLastError()));
		std::filesystem::remove(m.copy, ec);
		return false;
	}
	auto version = reinterpret_cast<SmtEngineApiVersionFn>(GetProcAddress(dll, "sl_engine_api_version"));
	m.create = reinterpret_cast<SmtEngineCreateFn>(GetProcAddress(dll, "sl_engine_create"));
	m.frame = reinterpret_cast<SmtEngineFrameFn>(GetProcAddress(dll, "sl_engine_frame"));
	m.destroy = reinterpret_cast<SmtEngineDestroyFn>(GetProcAddress(dll, "sl_engine_destroy"));
	m.batteries = reinterpret_cast<SmtEngineBatteriesFn>(GetProcAddress(dll, "sl_engine_batteries"));
	m.deviceStatus = reinterpret_cast<SmtEngineDeviceStatusFn>(GetProcAddress(dll, "sl_engine_device_status"));
	m.dll = dll;
	if (!version || !m.create || !m.frame || !m.destroy) {
		m_log("engine is missing exports");
		freeModule(m);
		return false;
	}
	if (version() != SMT_ENGINE_API_VERSION) {
		m_log("engine API version " + std::to_string(version()) + " != host "
			  + std::to_string(SMT_ENGINE_API_VERSION) + ": restart SteamVR to update the host");
		freeModule(m);
		return false;
	}
	m_loadedTime = std::filesystem::last_write_time(m_path, ec);
	return true;
}

void EngineLoader::freeModule(Module& m) {
	if (m.dll) FreeLibrary(static_cast<HMODULE>(m.dll));
	std::error_code ec;
	if (!m.copy.empty()) std::filesystem::remove(m.copy, ec);
	m = Module{};
}

std::string EngineLoader::destroyEngine() {
	std::string handover;
	if (m_engine && m_module.destroy) {
		std::vector<char> buf(1 << 16);
		size_t n = m_module.destroy(m_engine, buf.data(), buf.size());
		if (n >= buf.size()) {
			buf.resize(n + 1);
			// Too big for the buffer: the engine is already gone, keep what we got
			m_log("engine handover truncated");
		}
		handover = buf.data();
	}
	m_engine = nullptr;
	return handover;
}

void EngineLoader::unload() {
	destroyEngine();
	freeModule(m_module);
}

void EngineLoader::poll(double now) {
	if (now - m_lastCheck < 0.5) return;
	m_lastCheck = now;
	std::error_code ec;
	auto t = std::filesystem::last_write_time(m_path, ec);
	if (ec || t == m_loadedTime) return;
	// Wait until the build has finished writing the file
	auto age = std::filesystem::file_time_type::clock::now() - t;
	if (age < std::chrono::milliseconds(1000)) return;

	m_log("engine changed on disk, hot reloading");
	Module next;
	if (!loadModule(next)) {
		m_loadedTime = t;  // do not retry this build
		return;
	}
	std::string handover = destroyEngine();
	void* engine = next.create(&m_host, handover.empty() ? nullptr : handover.c_str());
	if (engine) {
		freeModule(m_module);
		m_module = next;
		m_engine = engine;
		m_log("engine reloaded");
	} else {
		m_log("new engine failed to start, keeping the previous one");
		freeModule(next);
		m_engine = m_module.create(&m_host, handover.empty() ? nullptr : handover.c_str());
	}
}

bool EngineLoader::batteries(SmtBattery out[SMT_DEVICE_COUNT]) {
	if (!m_engine || !m_module.batteries) return false;
	m_module.batteries(m_engine, out);
	return true;
}

bool EngineLoader::deviceStatus(SmtDeviceStatus out[SMT_DEVICE_COUNT]) {
	if (!m_engine) {
		for (int i = 0; i < SMT_DEVICE_COUNT; i++) out[i] = SmtDeviceStatus{};
		return true;
	}
	if (!m_module.deviceStatus) return false;
	m_module.deviceStatus(m_engine, out);
	return true;
}

void EngineLoader::frame(const SmtFrameInput& in, SmtFrameOutput& out) {
	if (!m_engine) {
		out = SmtFrameOutput{};
		return;
	}
	m_module.frame(m_engine, &in, &out);
}

}  // namespace smt
