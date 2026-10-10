#include "f32c/gimbal.hpp"

#include <cstdio>

int main() {
    f32c::Gimbal2Dof g;
    if (!g.open()) {
        std::fprintf(stderr, "open failed\n");
        return 1;
    }
    if (!g.ping()) {
        std::fprintf(stderr, "ping failed (yaw/pitch not both answering)\n");
        return 1;
    }
    f32c::Gimbal2Dof::State s;
    if (!g.read(&s)) {
        std::fprintf(stderr, "read failed\n");
        return 1;
    }
    std::printf("online  yaw=%.1f deg  pitch=%.1f deg\n", s.yaw_deg, s.pitch_deg);
    return 0;
}
