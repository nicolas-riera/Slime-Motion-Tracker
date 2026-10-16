#ifndef LOGGING_LOGGER_H
#define LOGGING_LOGGER_H

#include <Arduino.h>

#include "Level.h"
#include "debug.h"

namespace SlimeVR::Logging {
class Logger {
public:
	Logger(const char* prefix)
		: m_Prefix(prefix)
		, m_Tag(nullptr){};
	Logger(const char* prefix, const char* tag)
		: m_Prefix(prefix)
		, m_Tag(nullptr) {
		setTag(tag);
	};

	~Logger() {
		if (m_Tag != nullptr) {
			free(m_Tag);
		}
	}

	void setTag(const char* tag);

	void trace(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void debug(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void info(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void warn(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void error(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void fatal(const char* str, ...) const __attribute__((format(printf, 2, 3)));

	static void tick();

private:
	void log(Level level, const char* str, va_list args) const;

	const char* const m_Prefix;
	char* m_Tag;
};
}  // namespace SlimeVR::Logging

#endif
