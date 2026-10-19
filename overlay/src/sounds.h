// Calibration countdown sounds, synthesized in memory (no sound files to ship).
#pragma once

namespace smt::sounds {

enum class Sound {
    Tick,  // one per countdown second
    Done,  // pose captured / calibration done
    Error,  // calibration failed
};

// Plays asynchronously, replacing the sound playing. volume 0..1.
void play(Sound sound, float volume);

}  // namespace smt::sounds
