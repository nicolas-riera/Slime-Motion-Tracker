// Calibration countdown sounds, synthesized in memory (no sound files to ship).
#pragma once

namespace smt::sounds {

enum class Sound {
    Tick,  // one per countdown second
    Done,  // calibration done
    Error,  // calibration failed
};

// Plays asynchronously, replacing the sound playing. volume 0..1.
void play(Sound sound, float volume);

// Stops the sound playing (it reads a static buffer): call before exiting.
void stop();

}  // namespace smt::sounds
