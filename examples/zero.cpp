#include "f32c/gimbal.hpp"

#include <cstdio>
#include <iostream>
#include <string>

int main() {
    f32c::Gimbal2Dof g;
    if (!g.open()) {
        std::fprintf(stderr, "open failed\n");
        return 1;
    }
    if (!g.ping()) {
        std::fprintf(stderr, "ping failed\n");
        return 1;
    }

    f32c::Gimbal2Dof::State before;
    g.read(&before);
    std::printf("now  yaw=%.1f  pitch=%.1f\n", before.yaw_deg, before.pitch_deg);
    std::printf("will ENABLE and SET ZERO (persisted). Enter to continue.\n");
    std::string line;
    if (!std::getline(std::cin, line)) return 0;

    if (!g.enable()) {
        std::fprintf(stderr, "enable failed\n");
        g.disable();
        return 1;
    }
    if (!g.set_zero()) {
        std::fprintf(stderr, "set_zero failed\n");
        g.disable();
        return 1;
    }

    f32c::Gimbal2Dof::State after;
    g.read(&after);
    std::printf("after zero  yaw=%.1f  pitch=%.1f\n", after.yaw_deg, after.pitch_deg);
    if (!g.close_checked()) {
        std::fprintf(stderr, "cleanup failed: %s\n", g.last_error().c_str());
        return 1;
    }
    return after.ok ? 0 : 1;
}
