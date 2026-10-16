/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 Gorbit99 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <Arduino.h>

#include <cmath>
#include <vector>

namespace SlimeVR::Logging {

class SerialBuffer {
public:
	void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
	void tick();
	void enableImmediateMode(bool enable = true);

	static SerialBuffer& getInstance();

private:
	SerialBuffer();
	static constexpr size_t BufferSize = 8192;
	static constexpr size_t PerTickWriteSize = 128;
	std::vector<char> buffer;
	char printfBuffer[512 + 1];
	size_t head = 0;
	size_t tail = 0;
	size_t count = 0;
	bool immediateMode = false;

	static SerialBuffer instance;

	static_assert(sizeof(printfBuffer) < BufferSize);
};

}  // namespace SlimeVR::Logging
