// Records raw Kinect v2 depth frames next to the SteamVR session recordings, as
// ground truth for tuning and training the body solver: the point cloud shows
// where the legs really are (foot on a bed, leg held up, crossed legs) from any
// viewing angle. No Kinect skeleton is used (it needs the user facing the sensor).
//
// Output, in %LOCALAPPDATA%\SlimeVR-lite\recordings (or --dir):
//   kinect_YYYYMMDD_HHMMSS.depth   frames
//     header: "SLKD" u32 version=2, u32 width, u32 height
//     frame:  f64 t (host steady clock, s, same clock as the session recordings),
//             f64 rt (Kinect frame time, s), u32 size, then `size` bytes:
//             width*height u16 depth in mm (0 = no data), each row delta coded
//             (d[x] - d[x-1], wrapping u16; the first pixel of a row as is), then
//             compressed with the Windows Compression API, XPRESS_HUFF
//   kinect_YYYYMMDD_HHMMSS.table   depth pixel -> camera space ray
//     width*height pairs of f32 (x, y): point = (x * z, y * z, z), z in meters
// The Kinect -> SteamVR transform and the time offset are solved offline.
//
// Usage: slimelite_kinect_recorder [--seconds N] [--dir PATH]
#include <Windows.h>
#include <Kinect.h>
#include <compressapi.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <thread>
#include <vector>

namespace {

double hostNow() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

template <class T>
void release(T*& p) {
	if (p) {
		p->Release();
		p = nullptr;
	}
}

volatile bool g_stop = false;

BOOL WINAPI onConsoleEvent(DWORD) {
	g_stop = true;
	return TRUE;
}

}  // namespace

int main(int argc, char** argv) {
	double seconds = 1e18;
	std::filesystem::path dir;
	for (int i = 1; i + 1 < argc; i++) {
		if (!std::strcmp(argv[i], "--seconds")) seconds = std::atof(argv[i + 1]);
		if (!std::strcmp(argv[i], "--dir")) dir = argv[i + 1];
	}
	if (dir.empty()) {
		const char* base = std::getenv("LOCALAPPDATA");
		dir = std::filesystem::path(base ? base : ".") / "SlimeVR-lite" / "recordings";
	}
	std::filesystem::create_directories(dir);
	SetConsoleCtrlHandler(onConsoleEvent, TRUE);

	IKinectSensor* sensor = nullptr;
	IDepthFrameSource* source = nullptr;
	IDepthFrameReader* reader = nullptr;
	ICoordinateMapper* mapper = nullptr;
	if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor || FAILED(sensor->Open()) ||
		FAILED(sensor->get_DepthFrameSource(&source)) || FAILED(source->OpenReader(&reader)) ||
		FAILED(sensor->get_CoordinateMapper(&mapper))) {
		std::printf("cannot open the Kinect\n");
		return 1;
	}
	COMPRESSOR_HANDLE compressor = nullptr;
	if (!CreateCompressor(COMPRESS_ALGORITHM_XPRESS_HUFF, nullptr, &compressor)) {
		std::printf("cannot create the compressor\n");
		return 1;
	}

	char stem[64];
	std::time_t now = std::time(nullptr);
	std::strftime(stem, sizeof(stem), "kinect_%Y%m%d_%H%M%S", std::localtime(&now));
	std::filesystem::path depthPath = dir / (std::string(stem) + ".depth");
	std::filesystem::path tablePath = dir / (std::string(stem) + ".table");
	FILE* out = _wfopen(depthPath.wstring().c_str(), L"wb");
	if (!out) {
		std::printf("cannot write %s\n", depthPath.string().c_str());
		return 1;
	}
	std::printf("recording to %s\n", depthPath.string().c_str());
	std::fflush(stdout);

	const double start = hostNow();
	double lastFlush = start, lastStatus = start;
	long long frames = 0;
	uint64_t rawBytes = 0, storedBytes = 0;
	bool headerWritten = false, tableWritten = false;
	INT64 lastRelTime = -1;
	std::vector<uint16_t> depth;
	std::vector<uint8_t> packed;
	while (!g_stop && hostNow() - start < seconds) {
		IDepthFrame* frame = nullptr;
		if (FAILED(reader->AcquireLatestFrame(&frame)) || !frame) {
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			continue;
		}
		double t = hostNow();
		INT64 relTime = 0;
		frame->get_RelativeTime(&relTime);
		IFrameDescription* desc = nullptr;
		int width = 0, height = 0;
		if (SUCCEEDED(frame->get_FrameDescription(&desc)) && desc) {
			desc->get_Width(&width);
			desc->get_Height(&height);
			release(desc);
		}
		if (relTime == lastRelTime || width <= 0 || height <= 0) {
			release(frame);
			continue;
		}
		lastRelTime = relTime;
		depth.resize(size_t(width) * height);
		HRESULT hr = frame->CopyFrameDataToArray(UINT(depth.size()), depth.data());
		release(frame);
		if (FAILED(hr)) continue;

		if (!headerWritten) {
			uint32_t header[4];
			std::memcpy(header, "SLKD", 4);
			header[1] = 2;
			header[2] = uint32_t(width);
			header[3] = uint32_t(height);
			std::fwrite(header, sizeof(header), 1, out);
			headerWritten = true;
		}
		if (!tableWritten) {
			// Available once the sensor has streamed for a moment
			PointF* table = nullptr;
			UINT32 count = 0;
			if (SUCCEEDED(mapper->GetDepthFrameToCameraSpaceTable(&count, &table)) && table &&
				count == depth.size() && (table[0].X != 0 || table[0].Y != 0)) {
				if (FILE* tf = _wfopen(tablePath.wstring().c_str(), L"wb")) {
					std::fwrite(table, sizeof(PointF), count, tf);
					std::fclose(tf);
					tableWritten = true;
				}
			}
			if (table) CoTaskMemFree(table);
		}

		// Neighbor differences are small: compresses about twice as well
		for (int y = 0; y < height; y++) {
			uint16_t* row = depth.data() + size_t(y) * width;
			for (int x = width - 1; x > 0; x--) row[x] = uint16_t(row[x] - row[x - 1]);
		}
		const size_t rawSize = depth.size() * sizeof(uint16_t);
		packed.resize(rawSize + 4096);
		SIZE_T packedSize = 0;
		if (!Compress(compressor, depth.data(), rawSize, packed.data(), packed.size(), &packedSize)) continue;
		double rt = relTime * 1e-7;
		uint32_t size = uint32_t(packedSize);
		std::fwrite(&t, sizeof(t), 1, out);
		std::fwrite(&rt, sizeof(rt), 1, out);
		std::fwrite(&size, sizeof(size), 1, out);
		std::fwrite(packed.data(), 1, packedSize, out);
		frames++;
		rawBytes += rawSize;
		storedBytes += packedSize;

		if (t - lastFlush > 1.0) {
			std::fflush(out);
			lastFlush = t;
		}
		if (t - lastStatus > 10.0) {
			std::printf("%.0f s: %lld frames, %.0f MB stored (ratio %.1f)%s\n", t - start, frames,
						storedBytes / 1e6, storedBytes ? double(rawBytes) / storedBytes : 0.0,
						tableWritten ? "" : ", waiting for the camera table");
			std::fflush(stdout);
			lastStatus = t;
		}
	}
	std::fclose(out);
	CloseCompressor(compressor);
	release(mapper);
	release(reader);
	release(source);
	sensor->Close();
	release(sensor);
	std::printf("done: %lld frames, %.0f MB\n", frames, storedBytes / 1e6);
	return 0;
}
