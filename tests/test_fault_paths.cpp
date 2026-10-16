#include "f32c/c_api.h"
#include "f32c/gimbal.hpp"
#include "serial_port.hpp"

#include <pty.h>
#include <poll.h>
#include <unistd.h>

#include <atomic>
#include "check.hpp"
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

enum class WriteMode { Normal, Again, Interrupted, Zero, Partial, EnableFails };
static WriteMode mode = WriteMode::Normal;
static unsigned calls = 0;
static unsigned disable_attempts[3] = {};
static bool enable_failed = false;

extern "C" ssize_t __real_write(int fd, const void* data, size_t size);
extern "C" ssize_t __wrap_write(int fd, const void* data, size_t size) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    ++calls;
    if (size >= 3 && bytes[0] == 0x7A && bytes[1] <= 2 && bytes[2] == 5)
        ++disable_attempts[bytes[1]];
    if (mode == WriteMode::EnableFails && size >= 3 && bytes[2] == 6) {
        enable_failed = true;
        errno = EIO;
        return -1;
    }
    if (mode == WriteMode::Zero) return 0;
    if (mode == WriteMode::Partial && calls == 1) return 1;
    if (mode == WriteMode::Again || mode == WriteMode::Interrupted || mode == WriteMode::Partial ||
            (mode == WriteMode::EnableFails && enable_failed)) {
        errno = mode == WriteMode::Interrupted ? EINTR : EAGAIN;
        return -1;
    }
    return __real_write(fd, data, size);
}

template<class F> double timed(F operation) {
    const auto start = std::chrono::steady_clock::now();
    operation();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

int main() {
    int master, slave;
    char device[128] = {};
    CHECK(openpty(&master, &slave, device, nullptr, nullptr) == 0);

    f32c::SerialPort port;
    CHECK(port.open(device));
    char log_path[] = "/tmp/f32c_fault_paths_XXXXXX";
    const int log_fd = mkstemp(log_path);
    CHECK(log_fd >= 0);
    close(log_fd);
    CHECK(port.start_log(log_path));
    const uint8_t frame[] = {0x7A, 1, 5, 0x7E, 0x7B};
    for (const auto failure : {WriteMode::Again, WriteMode::Interrupted, WriteMode::Zero, WriteMode::Partial}) {
        mode = failure;
        calls = 0;
        const auto elapsed = timed([&] { CHECK(!port.write_all(frame, sizeof(frame))); });
        CHECK(elapsed >= f32c::SerialPort::write_timeout_ms - 10 && elapsed < 300);
        CHECK(calls > 1);
    }
    port.close();
    std::ifstream log(log_path);
    const std::string text((std::istreambuf_iterator<char>(log)), std::istreambuf_iterator<char>());
    CHECK(text.find("TX_ERROR write timeout") != std::string::npos);
    CHECK(text.find("sent 1/5 bytes") != std::string::npos);
    unlink(log_path);

    std::atomic<bool> running{true};
    std::thread emulator([&] {
        std::vector<uint8_t> buffered;
        while (running) {
            pollfd descriptor{master, POLLIN, 0};
            if (poll(&descriptor, 1, 20) <= 0) continue;
            uint8_t bytes[128];
            const auto count = read(master, bytes, sizeof(bytes));
            CHECK(count > 0);
            buffered.insert(buffered.end(), bytes, bytes + count);
            while (buffered.size() >= 3) {
                const auto function = buffered[2];
                const size_t size = function == 14 ? 6 :
                    (function == 5 || function == 6 || function == 8 || function == 10) ? 5 : 7;
                if (buffered.size() < size) break;
                CHECK(buffered[0] == 0x7A && buffered[size - 1] == 0x7B);
                if (function == 14) {
                    const auto address = buffered[1];
                    const uint8_t reply[] = {0x7A, address, 2, 0, 0, 0, 0,
                        static_cast<uint8_t>(0x7A ^ address ^ 2), 0x7B};
                    CHECK(__real_write(master, reply, sizeof(reply)) == sizeof(reply));
                }
                buffered.erase(buffered.begin(), buffered.begin() + size);
            }
        }
    });
    f32c::Gimbal2Dof::Config config;
    config.device = device;
    {
        mode = WriteMode::Normal;
        f32c::Gimbal2Dof g(config);
        CHECK(g.open() && g.enable_at(0, 0));
        mode = WriteMode::Again;
        CHECK(timed([&] { CHECK(!g.set_yaw_deg(1)); }) < 300);
        CHECK(g.last_error().find("write failed") != std::string::npos);
        CHECK(timed([&] { CHECK(!g.disable()); }) < 600);
        CHECK(g.last_error().find("hardware state unknown") != std::string::npos);
        const auto yaw_attempts = disable_attempts[1], pitch_attempts = disable_attempts[2];
        CHECK(!g.set_deg(0, 0) && !g.set_pi(0, 50, 16, 63, 0));
        CHECK(g.last_error().find("hardware state unknown") != std::string::npos);
        CHECK(timed([&] { CHECK(!g.close_checked()); }) < 600);
        CHECK(disable_attempts[1] > yaw_attempts && disable_attempts[2] > pitch_attempts);
        CHECK(!g.close_checked() && !g.open());
    }
    {
        mode = WriteMode::Normal;
        f32c::Gimbal2Dof g(config);
        CHECK(g.open() && g.enable_at(0, 0));
        mode = WriteMode::Zero;
        CHECK(!g.disable());
        mode = WriteMode::Normal;
        CHECK(g.ping());  // Successful feedback must not erase outstanding cleanup.
        CHECK(!g.set_deg(0, 0));
        CHECK(g.last_error().find("hardware state unknown") != std::string::npos);
        CHECK(g.close_checked() && g.close_checked());
    }
    {
        mode = WriteMode::Normal;
        f32c::Gimbal2Dof g(config);
        CHECK(g.open());
        mode = WriteMode::EnableFails;
        enable_failed = false;
        CHECK(timed([&] { CHECK(!g.enable_at(0, 0)); }) < 600);
        CHECK(g.last_error().find("hardware state unknown") != std::string::npos);
        const auto yaw_attempts = disable_attempts[1], pitch_attempts = disable_attempts[2];
        mode = WriteMode::Normal;
        CHECK(g.close_checked());
        CHECK(disable_attempts[1] > yaw_attempts && disable_attempts[2] > pitch_attempts);
    }
    mode = WriteMode::Normal;
    auto* handle = f32c_open(device, 1, 2);
    CHECK(handle && f32c_enable_at(handle, 0, 0));
    mode = WriteMode::Interrupted;
    CHECK(timed([&] { CHECK(!f32c_close_checked(handle)); }) < 600);
    CHECK(std::string(f32c_last_error(nullptr)).find("hardware state unknown") != std::string::npos);
    CHECK(f32c_close_checked(nullptr));
    const auto yaw_attempts = disable_attempts[1], pitch_attempts = disable_attempts[2];
    CHECK(timed([&] {
        mode = WriteMode::Normal;
        f32c::Gimbal2Dof g(config);
        CHECK(g.open() && g.enable_at(0, 0));
        mode = WriteMode::Again;  // Destructor also attempts both axes with bounded waits.
    }) < 1000);
    CHECK(disable_attempts[1] > yaw_attempts && disable_attempts[2] > pitch_attempts);
    running = false;
    emulator.join();
    close(master);
    close(slave);
    std::puts("bounded writes and failed-disable cleanup passed");
}
