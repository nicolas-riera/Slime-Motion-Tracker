#include "sounds.h"

#include <windows.h>
#include <mmsystem.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace smt::sounds {

namespace {

constexpr int kRate = 44100;
constexpr double kPi = 3.14159265358979323846;

struct Note {
    double start;  // s
    double freq;  // Hz
    double length;  // s
    double gain;
};

// Soft bell-like tone: sine + a little second harmonic, fast attack, exponential decay
std::vector<int16_t> synth(const std::vector<Note>& notes, float volume) {
    double total = 0;
    for (const Note& n : notes) total = std::max(total, n.start + n.length);
    std::vector<double> mix(static_cast<size_t>(total * kRate) + 1, 0.0);
    for (const Note& n : notes) {
        const size_t first = static_cast<size_t>(n.start * kRate);
        const size_t count = static_cast<size_t>(n.length * kRate);
        for (size_t i = 0; i < count && first + i < mix.size(); ++i) {
            const double t = static_cast<double>(i) / kRate;
            const double attack = std::min(1.0, t / 0.004);
            const double release = std::min(1.0, (n.length - t) / 0.01);
            const double env = attack * release * std::exp(-t * 5.0 / n.length);
            const double s = std::sin(2 * kPi * n.freq * t) + 0.25 * std::sin(4 * kPi * n.freq * t);
            mix[first + i] += n.gain * env * s / 1.25;
        }
    }
    // Perceived loudness is roughly logarithmic: square the slider value
    const double scale = 32767.0 * 0.8 * static_cast<double>(volume) * static_cast<double>(volume);
    std::vector<int16_t> out(mix.size());
    for (size_t i = 0; i < mix.size(); ++i)
        out[i] = static_cast<int16_t>(std::clamp(mix[i] * scale, -32767.0, 32767.0));
    return out;
}

void writeWav(std::vector<uint8_t>& wav, const std::vector<int16_t>& samples) {
    const uint32_t dataBytes = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
    wav.resize(44 + dataBytes);
    uint8_t* p = wav.data();
    const auto u32 = [&](uint32_t v) {
        std::memcpy(p, &v, 4);
        p += 4;
    };
    const auto u16 = [&](uint16_t v) {
        std::memcpy(p, &v, 2);
        p += 2;
    };
    const auto tag = [&](const char* t) {
        std::memcpy(p, t, 4);
        p += 4;
    };
    tag("RIFF");
    u32(36 + dataBytes);
    tag("WAVE");
    tag("fmt ");
    u32(16);
    u16(1);  // PCM
    u16(1);  // mono
    u32(kRate);
    u32(kRate * 2);
    u16(2);
    u16(16);
    tag("data");
    u32(dataBytes);
    std::memcpy(p, samples.data(), dataBytes);
}

// PlaySound reads the buffer while it plays: it is only rewritten after stopping
std::vector<uint8_t> g_wav;

}  // namespace

void play(Sound sound, float volume) {
    volume = std::clamp(volume, 0.0f, 1.0f);
    std::vector<Note> notes;
    switch (sound) {
    case Sound::Tick:
        notes = {{0.0, 880.0, 0.12, 0.8}};
        break;
    case Sound::Done:
        notes = {{0.0, 784.0, 0.16, 0.7}, {0.09, 1175.0, 0.30, 0.8}};
        break;
    case Sound::Error:
        notes = {{0.0, 330.0, 0.18, 0.8}, {0.16, 247.0, 0.30, 0.8}};
        break;
    }
    PlaySoundW(nullptr, nullptr, 0);
    if (volume <= 0.0f) return;
    writeWav(g_wav, synth(notes, volume));
    PlaySoundW(reinterpret_cast<LPCWSTR>(g_wav.data()), nullptr, SND_MEMORY | SND_ASYNC | SND_NODEFAULT);
}

void stop() { PlaySoundW(nullptr, nullptr, 0); }

}  // namespace smt::sounds
