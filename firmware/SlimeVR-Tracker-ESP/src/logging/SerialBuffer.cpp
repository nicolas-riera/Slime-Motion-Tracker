#include "SerialBuffer.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace SlimeVR::Logging {

SerialBuffer::SerialBuffer() { buffer.resize(BufferSize); }

SerialBuffer& SerialBuffer::getInstance() { return instance; }

void SerialBuffer::printf(const char* fmt, ...) {
	va_list lst;
	va_start(lst, fmt);

	if (immediateMode) {
		vprintf(fmt, lst);
		va_end(lst);
		return;
	}

	auto result = vsnprintf(printfBuffer, sizeof(printfBuffer), fmt, lst);
	va_end(lst);
	if (result < 0) {
		return;
	}

	auto written = static_cast<size_t>(result);

	if (written > sizeof(printfBuffer)) {
		written = sizeof(printfBuffer) - 1;
	}

	size_t beforeEnd = BufferSize - tail;
	if (beforeEnd >= written) {
		memcpy(buffer.data() + tail, printfBuffer, written);
	} else {
		memcpy(buffer.data() + tail, printfBuffer, beforeEnd);
		memcpy(buffer.data(), printfBuffer + beforeEnd, written - beforeEnd);
	}

	tail = (tail + written) % BufferSize;
	if (count + written > BufferSize) {
		head = tail;
		count = BufferSize;
	} else {
		count += written;
	}
}

void SerialBuffer::tick() {
	if (count == 0) {
		return;
	}

	size_t maxWrite
		= std::min(PerTickWriteSize, static_cast<size_t>(Serial.availableForWrite()));
	size_t toWrite = std::min(maxWrite, count);
	size_t beforeEnd = BufferSize - head;
	if (beforeEnd >= toWrite) {
		Serial.printf("%.*s", static_cast<int>(toWrite), buffer.data() + head);
		head = (head + toWrite) % BufferSize;
	} else {
		Serial.printf(
			"%.*s%.*s",
			static_cast<int>(beforeEnd),
			buffer.data() + head,
			static_cast<int>(toWrite - beforeEnd),
			buffer.data()
		);
		head = toWrite - beforeEnd;
	}
	count -= toWrite;
}

void SerialBuffer::enableImmediateMode(bool enable) { immediateMode = enable; }

SerialBuffer SerialBuffer::instance;

}  // namespace SlimeVR::Logging
