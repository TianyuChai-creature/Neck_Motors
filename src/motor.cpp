#include "motor.hpp"

#include "f32c/protocol.hpp"

#include <chrono>
#include <algorithm>
#include <thread>
#include <vector>

namespace f32c {
namespace {

constexpr int kGapMs = 8;
constexpr int kQueryMs = 25;
constexpr int kRetries = 2;

}  // namespace

Motor::Motor(SerialPort& port, uint8_t id) : port_(port), id_(id) {}

bool Motor::send(const std::vector<uint8_t>& pkt) {
    if (!port_.write_all(pkt)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(kGapMs));
    return true;
}

bool Motor::enable() { return send(cmd_enable(id_)); }
bool Motor::disable() { return send(cmd_disable(id_)); }
bool Motor::save() { return send(cmd_save(id_)); }
bool Motor::set_zero() { return send(cmd_set_zero(id_)); }
bool Motor::set_deg(double deg) { return send(cmd_single_turn_deg(id_, deg)); }

bool Motor::setup_position(int travel_rpm, uint16_t accel, double initial_deg) {
    if (!send(cmd_mode_single_turn_direct(id_))) return false;
    if (!send(cmd_speed_rpm(id_, static_cast<int16_t>(travel_rpm)))) return false;
    if (!set_deg(initial_deg)) return false;
    return send(cmd_accel(id_, accel));
}

bool Motor::set_pi(uint16_t speed_kp, uint16_t speed_ki, uint16_t pos_kp, uint16_t pos_ki) {
    if (!send(cmd_speed_kp(id_, speed_kp))) return false;
    if (!send(cmd_speed_ki(id_, speed_ki))) return false;
    if (!send(cmd_pos_kp(id_, pos_kp))) return false;
    if (!send(cmd_pos_ki(id_, pos_ki))) return false;
    return true;
}

bool Motor::query_mech(double* deg, int timeout_ms) {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    auto remaining = [&] { return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count()); };
    port_.flush_input();
    if (!port_.write_all(cmd_feedback_mech(id_), remaining())) return false;
    std::vector<uint8_t> raw;
    while (remaining() > 0) {
        const auto chunk = port_.read_some(remaining());
        raw.insert(raw.end(), chunk.begin(), chunk.end());
        if (const auto fr = find_mech_feedback(raw.data(), raw.size(), id_)) {
            *deg = static_cast<double>(*feedback_i32(*fr)) / 10.0;
            // Retain the former receive-idle gap before the next query on the bus.
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            return true;
        }
        // Keep enough suffix for a frame split across reads, bounding noise storage.
        if (raw.size() > 256) raw.erase(raw.begin(), raw.end() - 8);
    }
    return false;
}

bool Motor::read_deg(double* deg, int timeout_ms) {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    for (int i = 0; i < kRetries; ++i) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count();
        if (remaining <= 0) break;
        if (query_mech(deg, std::min(kQueryMs, static_cast<int>(remaining)))) return true;
    }
    return false;
}

}  // namespace f32c
