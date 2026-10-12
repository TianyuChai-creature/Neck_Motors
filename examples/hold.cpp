#include "f32c/gimbal.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_run{true};

static void on_sig(int) { g_run = false; }

int main() {
    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM, on_sig);

    f32c::Gimbal2Dof g;
    if (!g.open()) {
        std::fprintf(stderr, "open failed\n");
        return 1;
    }
    if (!g.ping()) {
        std::fprintf(stderr, "ping failed\n");
        return 1;
    }

    std::printf("will ENABLE yaw+pitch. Enter to continue, Ctrl-C to abort.\n");
    std::string line;
    if (!std::getline(std::cin, line)) return 0;
    if (!g_run) return 0;

    if (!g.enable()) {
        std::fprintf(stderr, "enable failed\n");
        g.disable();
        return 1;
    }
    std::printf("enabled. Ctrl-C to disable and exit.\n");

    while (g_run) {
        f32c::Gimbal2Dof::State s;
        if (g.read(&s)) {
            std::printf("yaw=%.1f  pitch=%.1f\n", s.yaw_deg, s.pitch_deg);
        } else {
            std::printf("read failed\n");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!g.close_checked()) {
        std::fprintf(stderr, "cleanup failed: %s\n", g.last_error().c_str());
        return 1;
    }
    std::printf("disable requests sent; motor state unconfirmed\n");
    return 0;
}
