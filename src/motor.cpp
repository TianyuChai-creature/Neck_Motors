#include "motor.hpp"

#include "f32c/protocol.hpp"

#include <chrono>
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

bool Motor::setup_position(int travel_rpm, uint16_t accel,
                           uint16_t speed_kp, uint16_t speed_ki,
                           uint16_t pos_kp, uint16_t pos_ki, double initial_deg) {
    if (!send(cmd_mode_single_turn_direct(id_))) return false;
    if (!send(cmd_speed_rpm(id_, static_cast<int16_t>(travel_rpm)))) return false;
    // Latch the hold target before PI writes can drive a stale target.
    if (!set_deg(initial_deg)) return false;
    if (!send(cmd_accel(id_, accel))) return false;
    return set_pi(speed_kp, speed_ki, pos_kp, pos_ki);
}

bool Motor::set_pi(uint16_t speed_kp, uint16_t speed_ki, uint16_t pos_kp, uint16_t pos_ki) {
    if (!send(cmd_speed_kp(id_, speed_kp))) return false;
    if (!send(cmd_speed_ki(id_, speed_ki))) return false;
    if (!send(cmd_pos_kp(id_, pos_kp))) return false;
    if (!send(cmd_pos_ki(id_, pos_ki))) return false;
    return true;
}

bool Motor::query_mech(double* deg) {
    port_.flush_input();
    if (!port_.write_all(cmd_feedback_mech(id_))) return false;
    auto raw = port_.read_for(kQueryMs);
    auto fr = find_frame(raw.data(), raw.size());
    if (!fr) return false;
    if (fr->addr != id_ || fr->func != static_cast<uint8_t>(Feedback::MechAngleX10)) return false;
    auto v = feedback_i32(*fr);
    if (!v) return false;
    *deg = static_cast<double>(*v) / 10.0;
    return true;
}

bool Motor::read_deg(double* deg) {
    for (int i = 0; i < kRetries; ++i) {
        if (query_mech(deg)) return true;
    }
    return false;
}

}  // namespace f32c
