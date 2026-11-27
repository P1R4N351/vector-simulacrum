// Emit a bounded sequence of stock fixed-duration targets from rest, in radians.
#include "velocityProfileGenerator.h"
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    if (argc != 4) return 1;
    const float start = std::strtof(argv[1], nullptr);
    const float end = std::strtof(argv[2], nullptr);
    const float duration = std::strtof(argv[3], nullptr);
    Anki::VelocityProfileGenerator profile;
    if (!profile.StartProfile_fixedDuration(start, 0, duration * .1f, end,
            duration * .1f, 10, 10000, duration, .005f)) return 2;
    float velocity = 0, position = start;
    for (int tick = 0; tick < 400 && !profile.TargetReached(); ++tick) {
        profile.Step(velocity, position);
        std::printf("%.9g\n", position);
    }
    return profile.TargetReached() ? 0 : 3;
}
